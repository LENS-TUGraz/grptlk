/*
 * Audio backend for nRF5340 Audio DK using the Cirrus Logic CS47L63 codec.
 *
 * Phase 1: lock-free ring buffers replace K_MSGQ between the DMA ISR and the
 * LC3 codec threads.  The ISR itself stays lean — only mic channel extraction
 * and stereo packing, no codec calls.
 *
 * Why LC3 does NOT run in the ISR
 * --------------------------------
 * LC3 encode/decode needs ~3–4 KB of stack (internal scratch + FPU state).
 * The Cortex-M ISR stack (CONFIG_ISR_STACK_SIZE, default 2 KB) is shared
 * across ALL interrupt handlers on the system — overflowing it causes the
 * exact "Stack overflow (context area not valid)" fault seen in testing.
 * LC3 runs safely in dedicated threads (decoder: 6144 B, encoder: 4096 B).
 *
 * DMA ISR responsibilities (cheap, ISR-safe)
 * ------------------------------------------
 * Uplink:   extract one mic channel from the 80-word stereo RX block → write
 *           mono PCM into the uplink ring (atomic slot index + memcpy).
 *           The encoder thread wakes via a semaphore and calls lc3_encode().
 *
 * Downlink: drain the playback queue (K_MSGQ, K_NO_WAIT) for stereo PCM →
 *           write into the next I2S TX DMA buffer.  The decoder thread
 *           calls lc3_decode() and puts the result into the playback queue.
 *
 * Clock note
 * ----------
 * The CS47L63 FLL slaves the codec SYSCLK to the I2S BCLK, which is derived
 * from the nRF5340 HFCLKAUDIO (12.288 MHz fractional PLL).  This makes the
 * DMA tick deterministic: exactly 80 stereo samples @ 16 kHz every 5 ms.
 *
 * I2S transfer: 80 samples × 2 ch × 2 bytes = 320 bytes per 5 ms block.
 * Double-buffering: buf_a is in flight while buf_b is being prepared, then swap.
 */

#include "audio/audio.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "audio/drivers/audio_i2s.h"
#include "audio/drivers/cs47l63_comm.h"
#include "cs47l63_reg_conf.h"
#include "cs47l63.h"

#if !DT_HAS_COMPAT_STATUS_OKAY(cirrus_cs47l63)
#error "No cirrus,cs47l63 node available. Use nrf5340_audio_dk/nrf5340/cpuapp."
#endif

BUILD_ASSERT(AUDIO_SAMPLE_RATE_HZ == AUDIO_I2S_SAMPLE_RATE_HZ,
	     "audio and audio_i2s sample rate must match");
BUILD_ASSERT(AUDIO_SAMPLES_PER_FRAME == AUDIO_I2S_SAMPLES_PER_BLOCK,
	     "audio and audio_i2s frame size must match");
BUILD_ASSERT(AUDIO_BLOCK_BYTES == AUDIO_I2S_BLOCK_BYTES,
	     "audio and audio_i2s block size must match");

/* Threshold (absolute sample value) above which a mic channel is considered
 * active for auto-selection. Keeps silent channels from being chosen.
 * 512 ≈ 1.5% FS — rejects background hiss (was 64 = 0.2% FS, too easily
 * triggered by ambient noise causing wrong/noisy channel selection). */
#define MIC_PEAK_DETECT_THRESHOLD 512

/* --- State set by audio_init() ------------------------------------------- */

/* Called from the DMA ISR with each captured mono mic frame.
 * Must write into a lock-free ring — no blocking allowed. */
static audio_rx_cb_t rx_cb;

/* Stereo PCM frames produced by the decoder thread, consumed by the DMA ISR. */
static struct k_msgq *playback_q;

static bool is_initialized;
static bool is_started;

/* --- I2S DMA buffers (double-buffered) ------------------------------------ */
/* buf_a is submitted first; buf_b is queued as "next". After each DMA
 * completion the callback swaps them so one is always in flight. */
static uint32_t i2s_rx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_rx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];

/* Last successfully played stereo PCM block.
 * Used as freeze-frame on tx_msgq underrun instead of hard-zero silence.
 * Written and read only from tx_buffer_fill() which runs in DMA ISR context
 * (single-threaded) — no locking needed.
 * Initialised to zero (BSS), so the very first underrun still produces silence. */
static uint32_t tx_last_valid_buf[AUDIO_I2S_WORDS_PER_BLOCK];

static cs47l63_t codec_driver;

/* -1 = not yet determined (auto-pick on first speech burst above threshold).
 *  0 = left channel forced.   1 = right channel forced. */
static int selected_mic_channel = -1;

/* Work item to log mic channel selection outside of ISR context. */
static struct k_work mic_ch_log_work;

static void mic_ch_log_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	printk("MIC channel auto-selected: %s\n",
	       selected_mic_channel == 0 ? "LEFT" : "RIGHT");
}

/* --- Codec register helpers ----------------------------------------------- */

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

/* Initialise the codec: reset, configure clocks, GPIO, ASP1 I2S port,
 * PDM microphone input, and speaker/headphone output. */
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

/* --- Uplink: microphone capture helpers (called from DMA ISR) ------------- */

static inline int32_t abs_s16(int16_t s)
{
	return (s < 0) ? -(int32_t)s : (int32_t)s;
}

/* Unpack one nrfx I2S word into left/right 16-bit samples.
 * nRF I2S left-aligned 16-bit: bits[31:16] = left, bits[15:0] = right. */
static inline void i2s_word_unpack(uint32_t word, int16_t *left, int16_t *right)
{
	*left  = (int16_t)(word >> 16);
	*right = (int16_t)(word & 0xFFFF);
}

/* Find the peak absolute value of each channel across one DMA block. */
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

/* Copy one channel out of a stereo I2S word buffer into a mono int16 buffer. */
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

/* Called from the DMA ISR with one RX block (80 stereo words).
 *
 * Auto-selects the louder mic channel on first speech, then extracts it to
 * mono and calls rx_cb().  rx_cb() writes into the uplink ring (lock-free,
 * atomic slot index + memcpy) — safe to call from ISR context.
 *
 * Note: printk is NOT called here even for the channel-select message,
 * because printk from ISR context can trigger deferred logging which
 * is not always safe at high interrupt priorities.  The channel selection
 * is logged once from the encoder thread instead. */
static void i2s_process_rx_block(const uint32_t *rx_words)
{
	int16_t mono_frame[AUDIO_SAMPLES_PER_FRAME];
	int32_t left_peak;
	int32_t right_peak;

	stereo_peak_analyze_words(rx_words, &left_peak, &right_peak);

	/* Latch mic channel on first block that exceeds the activity threshold.
	 * selected_mic_channel is only written here (single ISR context) so no
	 * atomic needed — the read in extract_selected_channel_to_mono runs in
	 * the same ISR. */
	if (selected_mic_channel < 0 &&
	    (left_peak > MIC_PEAK_DETECT_THRESHOLD ||
	     right_peak > MIC_PEAK_DETECT_THRESHOLD)) {
		selected_mic_channel = (right_peak > left_peak) ? 1 : 0;
		/* Log from system workqueue — printk is not safe in ISR context. */
		k_work_submit(&mic_ch_log_work);
	}

	uint8_t ch = (selected_mic_channel >= 0) ? (uint8_t)selected_mic_channel : 0U;

	extract_selected_channel_to_mono(rx_words, mono_frame, ch);

	if (rx_cb != NULL) {
		rx_cb(mono_frame);
	}
}

/* --- Downlink: speaker playback (called from DMA ISR) --------------------- */

/* Fill the next TX DMA buffer from the playback queue.
 * On success: save a copy as the freeze-frame reference.
 * On underrun: repeat the last valid frame instead of injecting silence.
 *
 * Why freeze-frame instead of silence:
 *   Silence (hard zero) causes a 5 ms amplitude step from live audio → 0 → live
 *   audio, which is audible as a click/knock.  Repeating the last frame is
 *   perceptually inaudible for a single 5 ms gap and avoids the discontinuity.
 *   The DMA must never stall — either path always writes a full block. */
static void tx_buffer_fill(uint32_t *tx_words)
{
	static uint32_t underrun_cnt;

	if (playback_q != NULL &&
	    k_msgq_get(playback_q, tx_words, K_NO_WAIT) == 0) {
		/* Fresh frame — save as freeze-frame reference. */
		memcpy(tx_last_valid_buf, tx_words, AUDIO_I2S_BLOCK_BYTES);
		return;
	}

	/* Queue empty: repeat last valid frame to avoid hard zero-crossing. */
	memcpy(tx_words, tx_last_valid_buf, AUDIO_I2S_BLOCK_BYTES);
	if ((underrun_cnt++ % 200U) == 0U) {
		printk("Speaker: freeze-frame injected (cnt=%u)\n", underrun_cnt);
	}
}

/* --- DMA completion callback (runs at interrupt level) --------------------
 *
 * Called by audio_i2s every 5 ms when one DMA block completes.
 *
 * rx_buf_released: the RX buffer the DMA just finished writing into.
 * tx_buf_released: the TX buffer the DMA just finished reading (ignored here,
 *                  because we always overwrite the next buffer before queuing).
 *
 * Sequence each call:
 *   1. i2s_process_rx_block(): extract mono mic frame → rx_cb() → uplink ring
 *   2. tx_buffer_fill():        drain playback_q → next I2S TX DMA buffer
 *   3. audio_i2s_set_next_buf(): re-arm DMA for the next 5 ms block
 *
 * ISR time budget: mic extract (~0.05 ms) + queue get (~0.01 ms) << 5 ms tick.
 * LC3 encode/decode happen in threads with their own stacks — not here.
 */
static void i2s_block_complete(uint32_t *rx_buf_released, const uint32_t *tx_buf_released)
{
	static uint32_t i2s_requeue_err_cnt;
	static uint8_t  tx_buf_sel;
	uint32_t       *next_tx_buf;
	int             err;

	ARG_UNUSED(tx_buf_released);

	if (rx_buf_released != NULL) {
		i2s_process_rx_block(rx_buf_released);
	}

	/* Alternate between tx_buf_a and tx_buf_b so one is always in flight. */
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

/* --- Public API ----------------------------------------------------------- */

/* Adjust output volume by step_db dB.  Reads the current OUT1L register,
 * clamps to [0, MAX_VOLUME_REG_VAL], and writes back with the update bit. */
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

/* Initialise the codec and store the playback queue and mic callback.
 * Must be called before audio_start(). */
int audio_init(struct k_msgq *tx_q, audio_rx_cb_t mono_rx_cb)
{
	int err;

	if (tx_q == NULL || mono_rx_cb == NULL) {
		return -EINVAL;
	}

	if (is_initialized) {
		return 0;
	}

	playback_q           = tx_q;
	rx_cb                = mono_rx_cb;
	selected_mic_channel = -1;

	k_work_init(&mic_ch_log_work, mic_ch_log_work_handler);

	err = codec_mic_path_prepare();
	if (err) {
		return err;
	}

	is_initialized = true;
	return 0;
}

/* Start the I2S DMA engine and the FLL.  After this returns the
 * i2s_block_complete callback fires every 5 ms driving both audio paths. */
int audio_start(void)
{
	int err;

	if (!is_initialized) {
		return -EPERM;
	}

	if (is_started) {
		return 0;
	}

	/* Toggle the CS47L63 FLL to lock the audio clock before starting I2S.
	 * The FLL slaves the codec SYSCLK to the I2S BCLK (12.288 MHz HFCLKAUDIO
	 * → BCLK 512 kHz → 16 kHz sample rate), making the DMA interval exact. */
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

	/* Prime both TX buffers with silence so the DMA has valid data before
	 * the decoder thread produces its first frame. */
	memset(i2s_tx_buf_a, 0, sizeof(i2s_tx_buf_a));
	memset(i2s_tx_buf_b, 0, sizeof(i2s_tx_buf_b));

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

	printk("I2S RX/TX started (CS47L63, ACLK 12.288 MHz, 16 kHz 5 ms blocks)\n");
	is_started = true;
	return 0;
}
