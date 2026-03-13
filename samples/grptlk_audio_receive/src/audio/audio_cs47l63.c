#include "audio.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(audio_cs47l63);

#include "audio_i2s.h"
#include "cs47l63.h"
#include "cs47l63_comm.h"
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

static struct audio_drift_ctx *rx_drift;
static struct audio_drift_ctx *tx_drift;
static bool is_initialized;
static bool is_started;

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
			LOG_ERR("CS47L63 write failed: reg=0x%08x val=0x%08x ret=%u", reg, value,
				ret);
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
		LOG_ERR("cs47l63_comm_init failed: %d", err);
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

	err = codec_reg_conf_write(pdm_mic_enable_configure, ARRAY_SIZE(pdm_mic_enable_configure));
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
		LOG_ERR("CS47L63 output volume set failed: %d", err);
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
	*left = (int16_t)(word >> 16);
	*right = (int16_t)(word & 0xFFFF);
}

static void stereo_peak_analyze_words(const uint32_t *rx_words, int32_t *left_peak,
				      int32_t *right_peak)
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
	    (left_peak > MIC_PEAK_DETECT_THRESHOLD || right_peak > MIC_PEAK_DETECT_THRESHOLD)) {
		selected_mic_channel = (right_peak > left_peak) ? 1 : 0;
		LOG_INF("MIC channel auto-selected: %s",
			selected_mic_channel == 0 ? "LEFT" : "RIGHT");
	}

	ch = mic_channel_pick(left_peak, right_peak);
	extract_selected_channel_to_mono(rx_words, mono_frame, ch);

	if (rx_drift != NULL) {
		audio_drift_write(rx_drift, mono_frame, 1);
	}
}

static void tx_buffer_fill(uint32_t *tx_words)
{
	if (tx_drift) {
		audio_drift_read(tx_drift, tx_words, 1);
	} else {
		memset(tx_words, 0, AUDIO_I2S_BLOCK_BYTES);
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
			LOG_ERR("audio_i2s_set_next_buf failed: %d (cnt=%u)", err,
				i2s_requeue_err_cnt);
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

	LOG_INF("[VOL] %+d dB -> reg=0x%02x (%d dB)", step_db, (uint8_t)new_vol,
		(int)(new_vol / 2) - MAX_VOLUME_DB);
	return 0;
}

int audio_init(struct audio_drift_ctx *dl_drift, struct audio_drift_ctx *ul_drift)
{
	int err;

	if (dl_drift == NULL || ul_drift == NULL) {
		return -EINVAL;
	}

	if (is_initialized) {
		return 0;
	}

	tx_drift = dl_drift;
	rx_drift = ul_drift;
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
		LOG_ERR("Codec FLL toggle failed: %d", err);
		return err;
	}

	audio_i2s_blk_cb_register(i2s_block_complete);

	err = audio_i2s_init();
	if (err) {
		LOG_ERR("audio_i2s_init failed: %d", err);
		return err;
	}

	memset(i2s_tx_buf_a, 0, sizeof(i2s_tx_buf_a));
	memset(i2s_tx_buf_b, 0, sizeof(i2s_tx_buf_b));

	err = audio_i2s_start(i2s_rx_buf_a, i2s_tx_buf_a);
	if (err) {
		LOG_ERR("audio_i2s_start failed: %d", err);
		return err;
	}

	err = audio_i2s_set_next_buf(i2s_rx_buf_b, i2s_tx_buf_b);
	if (err) {
		LOG_ERR("audio_i2s_set_next_buf failed: %d", err);
		return err;
	}

	LOG_INF("I2S RX/TX started (ACLK + DTS pinctrl)");
	is_started = true;
	return 0;
}
