#include "audio/audio.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "audio/backend.h"
#include "audio/drivers/audio_i2s.h"

BUILD_ASSERT(AUDIO_SAMPLE_RATE_HZ == AUDIO_I2S_SAMPLE_RATE_HZ,
	     "audio and audio_i2s sample rate must match");
BUILD_ASSERT(AUDIO_SAMPLES_PER_FRAME == AUDIO_I2S_SAMPLES_PER_FRAME,
	     "audio and audio_i2s LC3 frame size must match");
BUILD_ASSERT(AUDIO_BLOCK_BYTES == AUDIO_I2S_BLOCK_BYTES,
	     "audio and audio_i2s block size must match");

#define MIC_PEAK_DETECT_THRESHOLD 64

static audio_rx_cb_t rx_cb;
static struct audio_ring *ring;
static bool is_initialized;
static bool is_started;
static bool capture_enabled;
static int selected_capture_channel = AUDIO_BACKEND_CAPTURE_CHANNEL_AUTO;

/* I2S RX/TX buffers (double-buffered as in nrf5340_audio). */
static uint32_t i2s_rx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_rx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];

/* Mic accumulator: AUDIO_I2S_BLKS_PER_FRAME x 1 ms blocks -> 1 LC3 frame. */
static int16_t mic_accum[AUDIO_I2S_SAMPLES_PER_FRAME];
static uint8_t mic_accum_blk;

/* Underrun counter - incremented by tx_buffer_fill() on every DMA miss.
 * Read and atomically cleared by audio_underrun_count_reset(). */
static atomic_t g_underrun_cnt = ATOMIC_INIT(0);

/* Served counter - incremented by tx_buffer_fill() each time a real block is
 * read from the ring (not an underrun zero-fill).
 * Read and atomically cleared by audio_served_count_reset(). */
static atomic_t g_served_cnt = ATOMIC_INIT(0);

/* Consecutive underrun tracking - written only from the DMA callback context
 * so g_consec_underrun needs no atomics. g_max_consec_underrun is the
 * per-window peak, read from the stats thread via audio_max_consec_underrun_reset(). */
static uint32_t g_consec_underrun;
static atomic_t g_max_consec_underrun = ATOMIC_INIT(0);

enum playback_state {
	PLAYBACK_STATE_DISARMED = 0,
	PLAYBACK_STATE_ARMED = 1,
};

static enum playback_state g_playback_state = PLAYBACK_STATE_DISARMED;

static int capture_channel_sanitize(int capture_channel_hint)
{
	switch (capture_channel_hint) {
	case AUDIO_BACKEND_CAPTURE_CHANNEL_LEFT:
	case AUDIO_BACKEND_CAPTURE_CHANNEL_RIGHT:
	case AUDIO_BACKEND_CAPTURE_CHANNEL_AUTO:
		return capture_channel_hint;
	default:
		return AUDIO_BACKEND_CAPTURE_CHANNEL_AUTO;
	}
}

static inline int32_t abs_s16(int16_t s)
{
	return (s < 0) ? -(int32_t)s : (int32_t)s;
}

static inline uint8_t mic_channel_pick(int32_t left_peak, int32_t right_peak)
{
	if (selected_capture_channel >= 0) {
		return (uint8_t)selected_capture_channel;
	}

	return (right_peak > left_peak) ? 1U : 0U;
}

static inline void i2s_word_unpack(uint32_t word, int16_t *left, int16_t *right)
{
#if defined(CONFIG_GRPTLK_AUDIO_CODEC_MAX9867)
	/* MAX9867 on the BlueBite path presents MICRP on the right slot, and the
	 * working hello_world sample observes the raw stream as low-halfword=left,
	 * high-halfword=right in its int16_t interleaved view. Use that ordering
	 * here so right-channel capture selects the live analog mic. */
	*left = (int16_t)(word & 0xFFFF);
	*right = (int16_t)(word >> 16);
#else
	/* Default nRF5340 Audio DK/Cirrus path. */
	*left = (int16_t)(word >> 16);
	*right = (int16_t)(word & 0xFFFF);
#endif
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
	int16_t *dst = &mic_accum[mic_accum_blk * AUDIO_I2S_SAMPLES_PER_BLOCK];
	int32_t left_peak;
	int32_t right_peak;
	uint8_t ch;

	stereo_peak_analyze_words(rx_words, &left_peak, &right_peak);

	if (selected_capture_channel < 0 &&
	    (left_peak > MIC_PEAK_DETECT_THRESHOLD || right_peak > MIC_PEAK_DETECT_THRESHOLD)) {
		selected_capture_channel = (right_peak > left_peak) ? 1 : 0;
		printk("[mic] auto-selected channel %d (L_peak=%d R_peak=%d)\n",
		       selected_capture_channel, (int)left_peak, (int)right_peak);
	}

	ch = mic_channel_pick(left_peak, right_peak);
	extract_selected_channel_to_mono(rx_words, dst, ch);

	if (++mic_accum_blk >= AUDIO_I2S_BLKS_PER_FRAME) {
		mic_accum_blk = 0;
		if (rx_cb != NULL) {
			rx_cb(mic_accum); /* fires every 5th call = every 5 ms */
		}
	}
}

static uint16_t playback_ring_fill_get(uint16_t cons_idx, uint16_t prod_idx, uint16_t num_blks)
{
	return (prod_idx >= cons_idx) ? (uint16_t)(prod_idx - cons_idx)
				      : (uint16_t)(num_blks - cons_idx + prod_idx);
}

static void tx_buffer_fill(uint32_t *tx_words)
{
	if (ring != NULL) {
		uint16_t ci = *ring->cons_idx;
		uint16_t pi = *ring->prod_idx;
		uint16_t fill = playback_ring_fill_get(ci, pi, ring->num_blks);

		if (g_playback_state == PLAYBACK_STATE_DISARMED) {
			if (fill < AUDIO_BLKS_PER_FRAME) {
				memset(tx_words, 0, AUDIO_I2S_SAMPLES_PER_BLOCK * sizeof(uint32_t));
				return;
			}

			g_playback_state = PLAYBACK_STATE_ARMED;
			printk("[ring] playback armed (fill %u/%u)\n", fill, ring->num_blks);
		}

		if (ci != pi) {
			if (g_consec_underrun > 0U) {
				printk("[ring] gap_end: %u ms silence (fill now %u/%u)\n",
				       g_consec_underrun, fill, ring->num_blks);
				g_consec_underrun = 0U;
			}
			memcpy(tx_words, ring->fifo[ci],
			       AUDIO_I2S_SAMPLES_PER_BLOCK * sizeof(uint32_t));
			memset(ring->fifo[ci], 0, AUDIO_I2S_SAMPLES_PER_BLOCK * sizeof(uint32_t));
			*ring->cons_idx = (ci + 1U) % ring->num_blks;
			atomic_inc(&g_served_cnt);
			return;
		}
	}

	memset(tx_words, 0, AUDIO_I2S_SAMPLES_PER_BLOCK * sizeof(uint32_t));
	atomic_inc(&g_underrun_cnt);

	g_consec_underrun++;
	uint32_t cur_max = (uint32_t)atomic_get(&g_max_consec_underrun);

	if (g_consec_underrun > cur_max) {
		atomic_set(&g_max_consec_underrun, (atomic_val_t)g_consec_underrun);
	}
}

static void i2s_block_complete(uint32_t *rx_buf_released, const uint32_t *tx_buf_released)
{
	static uint32_t i2s_requeue_err_cnt;
	static uint8_t tx_buf_sel;
	uint32_t *next_tx_buf;
	int err;

	ARG_UNUSED(tx_buf_released);

	if (capture_enabled && rx_buf_released != NULL) {
		i2s_process_rx_block(rx_buf_released);
	}

	next_tx_buf = (tx_buf_sel == 0U) ? i2s_tx_buf_a : i2s_tx_buf_b;
	tx_buf_sel ^= 1U;
	tx_buffer_fill(next_tx_buf);

	err = audio_i2s_set_next_buf(capture_enabled ? rx_buf_released : NULL, next_tx_buf);
	if (err != 0) {
		i2s_requeue_err_cnt++;
		if (i2s_requeue_err_cnt <= 5U || (i2s_requeue_err_cnt % 50U) == 0U) {
			printk("[i2s] REQUEUE FAIL: err=%d cnt=%u - DMA may glitch!\n", err,
			       i2s_requeue_err_cnt);
		}
	}
}

uint32_t audio_underrun_count_reset(void)
{
	return (uint32_t)atomic_set(&g_underrun_cnt, 0);
}

uint32_t audio_served_count_reset(void)
{
	return (uint32_t)atomic_set(&g_served_cnt, 0);
}

uint32_t audio_max_consec_underrun_reset(void)
{
	return (uint32_t)atomic_set(&g_max_consec_underrun, 0);
}

void audio_playback_reset(void)
{
	g_playback_state = PLAYBACK_STATE_DISARMED;
	g_consec_underrun = 0U;
	atomic_set(&g_underrun_cnt, 0);
	atomic_set(&g_served_cnt, 0);
	atomic_set(&g_max_consec_underrun, 0);
}

int audio_volume_adjust(int8_t step_db)
{
	return audio_backend_volume_adjust(step_db);
}

int audio_input_source_switch(bool use_line_in)
{
	int capture_channel_hint = AUDIO_BACKEND_CAPTURE_CHANNEL_AUTO;
	int err;

	err = audio_backend_input_source_switch(use_line_in, &capture_channel_hint);
	if (err) {
		return err;
	}

	selected_capture_channel = capture_channel_sanitize(capture_channel_hint);
	return 0;
}

int audio_init(struct audio_ring *playback_ring, audio_rx_cb_t mono_rx_cb)
{
	struct audio_backend_config backend_cfg = {
		.capture_enabled = true,
		.capture_channel_hint = AUDIO_BACKEND_CAPTURE_CHANNEL_AUTO,
	};
	int err;

	if (playback_ring == NULL || mono_rx_cb == NULL) {
		return -EINVAL;
	}

	if (is_initialized) {
		return 0;
	}

	ring = playback_ring;
	rx_cb = mono_rx_cb;

	err = audio_backend_init(&backend_cfg);
	if (err) {
		return err;
	}

	capture_enabled = backend_cfg.capture_enabled;
	selected_capture_channel = capture_channel_sanitize(backend_cfg.capture_channel_hint);
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

	audio_i2s_blk_cb_register(i2s_block_complete);

	err = audio_i2s_init();
	if (err) {
		printk("audio_i2s_init failed: %d\n", err);
		return err;
	}

	memset(i2s_tx_buf_a, 0, sizeof(i2s_tx_buf_a));
	memset(i2s_tx_buf_b, 0, sizeof(i2s_tx_buf_b));
	audio_playback_reset();

	err = audio_i2s_start(capture_enabled ? i2s_rx_buf_a : NULL, i2s_tx_buf_a);
	if (err) {
		printk("audio_i2s_start failed: %d\n", err);
		return err;
	}

	err = audio_i2s_set_next_buf(capture_enabled ? i2s_rx_buf_b : NULL, i2s_tx_buf_b);
	if (err) {
		printk("audio_i2s_set_next_buf failed: %d\n", err);
		return err;
	}

	err = audio_backend_prepare_stream_start();
	if (err) {
		printk("audio_backend_prepare_stream_start failed: %d\n", err);
		audio_i2s_stop();
		return err;
	}

	printk("I2S %s started via NRFX API (ACLK + DTS pinctrl)\n",
	       capture_enabled ? "RX/TX" : "TX");
	is_started = true;
	return 0;
}
