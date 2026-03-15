#include "audio/audio.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "audio/drivers/audio_i2s.h"
#include "cs47l63.h"
#include "audio/drivers/cs47l63_comm.h"
#include "cs47l63_reg_conf.h"

#if !DT_HAS_COMPAT_STATUS_OKAY(cirrus_cs47l63)
#error "No cirrus,cs47l63 node available. Use nrf5340_audio_dk/nrf5340/cpuapp."
#endif

BUILD_ASSERT(AUDIO_SAMPLE_RATE_HZ == AUDIO_I2S_SAMPLE_RATE_HZ,
	     "audio and audio_i2s sample rate must match");
BUILD_ASSERT(AUDIO_SAMPLES_PER_FRAME == AUDIO_I2S_SAMPLES_PER_BLOCK,
	     "audio and audio_i2s frame size must match");
BUILD_ASSERT(AUDIO_BLOCK_BYTES == AUDIO_I2S_BLOCK_BYTES,
	     "audio and audio_i2s block size must match");

#define MIC_PEAK_DETECT_THRESHOLD 64

static audio_rx_cb_t rx_cb;
static struct k_msgq *playback_q;
static bool is_initialized;
static bool is_started;

/* I2S RX/TX buffers (double-buffered as in nrf5340_audio). */
static uint32_t i2s_rx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_rx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];

static cs47l63_t codec_driver;
static int selected_mic_channel = -1; /* -1=auto, 0=left, 1=right */

static int codec_reg_conf_write(const uint32_t config[][2], size_t num_of_regs)
{
	for (size_t i = 0; i < num_of_regs; i++) {
		const uint32_t reg = config[i][0];
		const uint32_t value = config[i][1];
		uint32_t ret;

		if (reg == SPI_BUSY_WAIT) {
			k_busy_wait(value);
			continue;
		}

		ret = cs47l63_write_reg(&codec_driver, reg, value);
		if (ret != CS47L63_STATUS_OK) {
			printk("CS47L63 write failed: reg=0x%08x val=0x%08x ret=%u\n",
			       reg, value, ret);
			return -EIO;
		}
	}

	return 0;
}

static int codec_mic_path_prepare(void)
{
	int err;

	err = cs47l63_comm_init(&codec_driver);
	if (err) {
		printk("cs47l63_comm_init failed: %d\n", err);
		return err;
	}

	err = codec_reg_conf_write(soft_reset, ARRAY_SIZE(soft_reset));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(clock_configuration, ARRAY_SIZE(clock_configuration));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(GPIO_configuration, ARRAY_SIZE(GPIO_configuration));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(asp1_enable, ARRAY_SIZE(asp1_enable));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(pdm_mic_enable_configure,
				   ARRAY_SIZE(pdm_mic_enable_configure));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(output_enable, ARRAY_SIZE(output_enable));
	if (err) {
		return err;
	}

	err = cs47l63_write_reg(&codec_driver, CS47L63_OUT1L_VOLUME_1,
				OUT_VOLUME_DEFAULT | VOLUME_UPDATE_BIT);
	if (err != CS47L63_STATUS_OK) {
		printk("CS47L63 output volume set failed: %d\n", err);
		return -EIO;
	}

	return 0;
}

static inline int32_t abs_s16(int16_t s)
{
	return (s < 0) ? -(int32_t)s : (int32_t)s;
}

static inline uint8_t mic_channel_pick(int32_t left_peak, int32_t right_peak)
{
	if (selected_mic_channel >= 0) {
		return (uint8_t)selected_mic_channel;
	}

	return (right_peak > left_peak) ? 1U : 0U;
}

static inline void i2s_word_unpack(uint32_t word, int16_t *left, int16_t *right)
{
	/* NRF_I2S_ALIGN_LEFT with NRF_I2S_SWIDTH_16BIT: the 16-bit sample
	 * occupies the upper half of the 32-bit word (bits [31:16] = left
	 * channel, bits [15:0] = right channel). */
	*left  = (int16_t)(word >> 16);
	*right = (int16_t)(word & 0xFFFF);
}

static void stereo_peak_analyze_words(const uint32_t *rx_words,
				      int32_t *left_peak, int32_t *right_peak)
{
	int32_t l_peak = 0;
	int32_t r_peak = 0;

	for (size_t i = 0; i < AUDIO_I2S_SAMPLES_PER_BLOCK; i++) {
		int16_t left;
		int16_t right;
		i2s_word_unpack(rx_words[i], &left, &right);

		const int32_t la = abs_s16(left);
		const int32_t ra = abs_s16(right);

		if (la > l_peak) {
			l_peak = la;
		}
		if (ra > r_peak) {
			r_peak = ra;
		}
	}

	*left_peak = l_peak;
	*right_peak = r_peak;
}

static void extract_selected_channel_to_mono(const uint32_t *rx_words, int16_t *mono,
					     uint8_t channel)
{
	for (size_t i = 0; i < AUDIO_I2S_SAMPLES_PER_BLOCK; i++) {
		int16_t left;
		int16_t right;
		i2s_word_unpack(rx_words[i], &left, &right);
		mono[i] = (channel == 0U) ? left : right;
	}
}

static void i2s_process_rx_block(const uint32_t *rx_words)
{
	int16_t mono_frame[AUDIO_SAMPLES_PER_FRAME];
	int32_t left_peak;
	int32_t right_peak;
	uint8_t ch;

	stereo_peak_analyze_words(rx_words, &left_peak, &right_peak);

	if (selected_mic_channel < 0 &&
	    (left_peak > MIC_PEAK_DETECT_THRESHOLD ||
	     right_peak > MIC_PEAK_DETECT_THRESHOLD)) {
		selected_mic_channel = (right_peak > left_peak) ? 1 : 0;
		printk("MIC channel auto-selected: %s\n",
		       selected_mic_channel == 0 ? "LEFT" : "RIGHT");
	}

	ch = mic_channel_pick(left_peak, right_peak);
	extract_selected_channel_to_mono(rx_words, mono_frame, ch);

	if (rx_cb != NULL) {
		rx_cb(mono_frame);
	}
}

/* Last successfully decoded frame, replayed on underrun to avoid silence
 * clicks caused by the broadcaster ACLK / receiver BLE clock drift. */
static uint32_t tx_last_frame[AUDIO_I2S_WORDS_PER_BLOCK];

static void tx_buffer_fill(uint32_t *tx_words)
{
	static uint32_t underrun_cnt;

	if (playback_q != NULL &&
	    k_msgq_get(playback_q, tx_words, K_NO_WAIT) == 0) {
		/* Got a real frame — save it for potential replay. */
		memcpy(tx_last_frame, tx_words, AUDIO_I2S_BLOCK_BYTES);
	} else {
		/* Queue empty: replay the last frame instead of silence to
		 * avoid audible clicks from the ACLK/BLE clock drift. */
		memcpy(tx_words, tx_last_frame, AUDIO_I2S_BLOCK_BYTES);
		if ((underrun_cnt++ % 200U) == 0U) {
			printk("Uplink RX playback: replaying last frame (cnt=%u)\n",
			       underrun_cnt);
		}
	}
}

static void i2s_block_complete(uint32_t *rx_buf_released, const uint32_t *tx_buf_released)
{
	static uint32_t i2s_requeue_err_cnt;
	static uint8_t tx_buf_sel;
	uint32_t *next_tx_buf;
	int err;

	ARG_UNUSED(tx_buf_released);

	if (rx_buf_released != NULL) {
		i2s_process_rx_block(rx_buf_released);
	}

	next_tx_buf = (tx_buf_sel == 0U) ? i2s_tx_buf_a : i2s_tx_buf_b;
	tx_buf_sel ^= 1U;
	tx_buffer_fill(next_tx_buf);

	err = audio_i2s_set_next_buf(rx_buf_released, next_tx_buf);
	if (err != 0) {
		if ((i2s_requeue_err_cnt++ % 50U) == 0U) {
			printk("audio_i2s_set_next_buf failed: %d (cnt=%u)\n",
			       err, i2s_requeue_err_cnt);
		}
	}
}

int audio_volume_adjust(int8_t step_db)
{
	uint32_t vol;
	int ret;

	ret = cs47l63_read_reg(&codec_driver, CS47L63_OUT1L_VOLUME_1, &vol);
	if (ret != CS47L63_STATUS_OK) {
		return -EIO;
	}

	int32_t new_vol = (int32_t)(vol & 0xFFU) + (int32_t)(step_db * 2);

	if (new_vol < 0) {
		new_vol = 0;
	} else if (new_vol > MAX_VOLUME_REG_VAL) {
		new_vol = MAX_VOLUME_REG_VAL;
	}

	ret = cs47l63_write_reg(&codec_driver, CS47L63_OUT1L_VOLUME_1,
				(uint32_t)new_vol | VOLUME_UPDATE_BIT);
	if (ret != CS47L63_STATUS_OK) {
		return -EIO;
	}

	printk("[VOL] %+d dB -> reg=0x%02x (%d dB)\n",
	       step_db, (uint8_t)new_vol, (int)(new_vol / 2) - MAX_VOLUME_DB);
	return 0;
}

int audio_init(struct k_msgq *tx_q, audio_rx_cb_t mono_rx_cb)
{
	int err;

	if (tx_q == NULL || mono_rx_cb == NULL) {
		return -EINVAL;
	}

	if (is_initialized) {
		return 0;
	}

	playback_q = tx_q;
	rx_cb = mono_rx_cb;
	selected_mic_channel = -1;

	err = codec_mic_path_prepare();
	if (err) {
		return err;
	}

	is_initialized = true;
	return 0;
}

int audio_start(void)
{
	int err;

	if (!is_initialized) {
		return -EPERM;
	}

	if (is_started) {
		return 0;
	}

	err = codec_reg_conf_write(FLL_toggle, ARRAY_SIZE(FLL_toggle));
	if (err) {
		printk("Codec FLL toggle failed: %d\n", err);
		return err;
	}

	audio_i2s_blk_cb_register(i2s_block_complete);

	err = audio_i2s_init();
	if (err) {
		printk("audio_i2s_init failed: %d\n", err);
		return err;
	}

	memset(i2s_tx_buf_a, 0, sizeof(i2s_tx_buf_a));
	memset(i2s_tx_buf_b, 0, sizeof(i2s_tx_buf_b));

	/* Pre-fill the playback queue with silence so tx_buffer_fill() always
	 * finds a frame ready, even before the uplink decoder produces its first
	 * real frame.  Without this the nrfx I2S ISR fires before the decoder
	 * thread has run, finds tx_msgq empty, and injects a silence burst that
	 * the codec reproduces as a click or crackle.  Two frames (20 ms) is
	 * enough to absorb one full decoder wake-up cycle. */
	if (playback_q != NULL) {
		static const uint8_t silence[AUDIO_I2S_BLOCK_BYTES];

		for (int i = 0; i < 2; i++) {
			(void)k_msgq_put(playback_q, silence, K_NO_WAIT);
		}
	}

	err = audio_i2s_start(i2s_rx_buf_a, i2s_tx_buf_a);
	if (err) {
		printk("audio_i2s_start failed: %d\n", err);
		return err;
	}

	err = audio_i2s_set_next_buf(i2s_rx_buf_b, i2s_tx_buf_b);
	if (err) {
		printk("audio_i2s_set_next_buf failed: %d\n", err);
		return err;
	}

	printk("I2S RX/TX started via NRFX API (ACLK + DTS pinctrl)\n");
	is_started = true;
	return 0;
}
