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
 * Uplink:   extract one mic channel from the 16-word stereo RX block,
 *           accumulate 5 × 1ms blocks into one 80-sample mono frame,
 *           then call rx_cb() → uplink ring → encoder thread → lc3_encode().
 *
 * Downlink: read one 1ms stereo block from the ring buffer (volatile index
 *           check + memcpy) → write into the next I2S TX DMA buffer.
 *           The decoder thread calls lc3_decode() and splits the 5ms frame
 *           into 5 × 1ms blocks in the ring buffer.
 *
 * Clock note
 * ----------
 * The CS47L63 FLL slaves the codec SYSCLK to the I2S BCLK, which is derived
 * from the nRF5340 HFCLKAUDIO (12.288 MHz fractional PLL).  This makes the
 * DMA tick deterministic: exactly 80 stereo samples @ 16 kHz every 5 ms.
 *
 * I2S transfer: 16 samples × 1 word (stereo packed) = 64 bytes per 1 ms block.
 * Double-buffering: buf_a is in flight while buf_b is being prepared, then swap.
 */

#include "audio/audio.h"

#ifndef CONFIG_GRPTLK_RELAY_ONLY
#include "audio/sync/clk_sync.h"
#endif

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
/* I2S block size (1ms = 16 samples) is now decoupled from the LC3 frame size
 * (5ms = 80 samples).  The ring buffer bridges the two cadences. */
BUILD_ASSERT(AUDIO_I2S_SAMPLES_PER_BLOCK == AUDIO_BLK_SAMPLES_MONO,
	     "I2S block size must equal 1ms block size");

/* Threshold (absolute sample value) above which a mic channel is considered
 * active for auto-selection. Keeps silent channels from being chosen.
 * 512 ≈ 1.5% FS — rejects background hiss (was 64 = 0.2% FS, too easily
 * triggered by ambient noise causing wrong/noisy channel selection). */
#define MIC_PEAK_DETECT_THRESHOLD 512

/* --- State set by audio_init() ------------------------------------------- */

/* Called from the DMA ISR with each captured mono mic frame.
 * Must write into a lock-free ring — no blocking allowed. */
static audio_rx_cb_t rx_cb;

/* Ring buffer: stereo 1ms I2S blocks, produced by decoder thread, consumed by DMA ISR. */
static struct audio_ring *ring;

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
 * Used as freeze-frame on ring buffer underrun instead of hard-zero silence.
 * Written and read only from tx_buffer_fill() which runs in DMA ISR context
 * (single-threaded) — no locking needed.
 * Initialised to zero (BSS), so the very first underrun still produces silence. */
static uint32_t tx_last_valid_buf[AUDIO_I2S_WORDS_PER_BLOCK];

static cs47l63_t codec_driver;

/* -1 = not yet determined (auto-pick on first speech burst above threshold).
 *  0 = left channel forced.   1 = right channel forced. */
static int selected_mic_channel = -1;

/* Debug counters (reset on audio_init). */
static uint32_t i2s_rx_block_cnt;
static uint32_t rx_cb_call_cnt;

/* Work item to log mic channel selection outside of ISR context. */
static struct k_work mic_ch_log_work;
static struct k_work debug_log_work;

static void mic_ch_log_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	printk("[MIC] channel auto-selected: %s (L=%d,R=%d at sel)\n",
	       selected_mic_channel == 0 ? "LEFT" : "RIGHT",
	       selected_mic_channel);
}

static void debug_log_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	printk("[I2S] rx_blocks=%u, rx_cb_calls=%u, mic_ch=%d\n",
	       i2s_rx_block_cnt, rx_cb_call_cnt, selected_mic_channel);
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

/* --- Mic accumulator: 5 × 1ms blocks → 1 × 5ms frame for uplink ---------- */
static int16_t mic_accum[AUDIO_SAMPLES_PER_FRAME]; /* 80 samples */
static uint8_t mic_accum_count;                      /* 0..AUDIO_BLKS_PER_FRAME-1 */

/* Called from the DMA ISR with one RX block (16 stereo words, 1ms).
 *
 * Auto-selects the louder mic channel on first speech, then extracts it to
 * mono and accumulates into mic_accum[].  Every AUDIO_BLKS_PER_FRAME (5)
 * blocks, calls rx_cb() with a full 80-sample mono frame — keeping the
 * entire uplink chain (encoder, iso_tx) completely unchanged.
 *
 * Note: printk is NOT called here even for the channel-select message,
 * because printk from ISR context can trigger deferred logging which
 * is not always safe at high interrupt priorities.  The channel selection
 * is logged once from the encoder thread instead. */
static void i2s_process_rx_block(const uint32_t *rx_words)
{
	int16_t mono_1ms[AUDIO_BLK_SAMPLES_MONO]; /* 16 samples */
	int32_t left_peak;
	int32_t right_peak;

	i2s_rx_block_cnt++;

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

	extract_selected_channel_to_mono(rx_words, mono_1ms, ch);

	/* Accumulate 1ms blocks into a 5ms frame for the uplink encoder. */
	memcpy(&mic_accum[mic_accum_count * AUDIO_BLK_SAMPLES_MONO],
	       mono_1ms, AUDIO_BLK_SAMPLES_MONO * sizeof(int16_t));
	mic_accum_count++;

	if (mic_accum_count >= AUDIO_BLKS_PER_FRAME) {
		mic_accum_count = 0;
		rx_cb_call_cnt++;
		if (rx_cb != NULL) {
			rx_cb(mic_accum); /* 80-sample frame, same as before */
		}
		/* Periodic debug logging every 200 frames (1 second). */
		if ((rx_cb_call_cnt % 200U) == 0U) {
			k_work_submit(&debug_log_work);
		}
	}
}

/* --- Downlink: speaker playback (called from DMA ISR) --------------------- */

/* Fill the next TX DMA buffer from the ring buffer.
 * On success: save a copy as the freeze-frame reference.
 * On underrun: repeat the last valid block instead of injecting silence.
 *
 * Why freeze-frame instead of silence:
 *   Silence (hard zero) causes a 1 ms amplitude step from live audio → 0 → live
 *   audio, which is audible as a click/knock.  Repeating the last block is
 *   perceptually inaudible for a single 1 ms gap and avoids the discontinuity.
 *   The DMA must never stall — either path always writes a full block. */
static uint32_t underrun_window_cnt; /* per-window counter, reset by audio_underrun_count_reset() */

static void tx_buffer_fill(uint32_t *tx_words)
{
	if (ring != NULL) {
		uint16_t ci = *ring->cons_idx;
		uint16_t pi = *ring->prod_idx;

		if (ci != pi) {
			memcpy(tx_words, ring->fifo[ci],
			       AUDIO_I2S_SAMPLES_PER_BLOCK * sizeof(uint32_t));
			memcpy(tx_last_valid_buf, ring->fifo[ci],
			       AUDIO_I2S_SAMPLES_PER_BLOCK * sizeof(uint32_t));
			*ring->cons_idx = (ci + 1U) % ring->num_blks;
			return;
		}
	}

	/* Ring empty: repeat last valid block to avoid hard zero-crossing. */
	memcpy(tx_words, tx_last_valid_buf,
	       AUDIO_I2S_SAMPLES_PER_BLOCK * sizeof(uint32_t));
	underrun_window_cnt++;
}

uint32_t audio_underrun_count_reset(void)
{
	uint32_t n = underrun_window_cnt;

	underrun_window_cnt = 0;
	return n;
}

/* --- DMA completion callback (runs at interrupt level) --------------------
 *
 * Called by audio_i2s every 1 ms when one DMA block completes.
 *
 * rx_buf_released: the RX buffer the DMA just finished writing into.
 * tx_buf_released: the TX buffer the DMA just finished reading (ignored here,
 *                  because we always overwrite the next buffer before queuing).
 *
 * Sequence each call:
 *   1. i2s_process_rx_block(): extract 16 mono samples, accumulate into 5ms frame
 *   2. tx_buffer_fill():        read 1 block from ring buffer → I2S TX DMA
 *   3. audio_i2s_set_next_buf(): re-arm DMA for the next 1 ms block
 *
 * ISR time budget: mic extract (~0.01 ms) + ring memcpy (~0.01 ms) << 1 ms tick.
 * LC3 encode/decode happen in threads with their own stacks — not here.
 */
static void i2s_block_complete(uint32_t *rx_buf_released,
			      const uint32_t *tx_buf_released)
{
	static uint32_t i2s_requeue_err_cnt;
	static uint8_t  tx_buf_sel;
	uint32_t       *next_tx_buf;
	int             err;

	ARG_UNUSED(tx_buf_released);

	if (rx_buf_released != NULL) {
		i2s_process_rx_block(rx_buf_released);
	}

	/* I2S timestamp drift measurement removed — use ring buffer level
	 * in clk_sync module instead for more reliable drift detection. */

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

/* Initialise the codec and store the playback ring buffer and mic callback.
 * Must be called before audio_start(). */
int audio_init(struct audio_ring *playback_ring, audio_rx_cb_t mono_rx_cb)
{
	int err;

	if (playback_ring == NULL || mono_rx_cb == NULL) {
		return -EINVAL;
	}

	if (is_initialized) {
		return 0;
	}

	ring                 = playback_ring;
	rx_cb                = mono_rx_cb;
	selected_mic_channel = -1;
	mic_accum_count      = 0;

	k_work_init(&mic_ch_log_work, mic_ch_log_work_handler);
	k_work_init(&debug_log_work, debug_log_work_handler);

	/* Reset debug counters. */
	i2s_rx_block_cnt = 0;
	rx_cb_call_cnt = 0;

	err = codec_mic_path_prepare();
	if (err) {
		return err;
	}

	is_initialized = true;
	return 0;
}

/* Start the I2S DMA engine and the FLL.  After this returns the
 * i2s_block_complete callback fires every 1 ms driving both audio paths. */
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

	printk("I2S RX/TX started (CS47L63, ACLK 12.288 MHz, 16 kHz 1 ms blocks)\n");
	is_started = true;
	return 0;
}
