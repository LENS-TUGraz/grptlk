/*
 * clk_sync.c - controller-agnostic HFCLKAUDIO drift compensation
 *
 * The receiver uses playback ring fill, measured just before each decoded LC3
 * frame is written into the ring, as the drift error signal. The target is a
 * small pre-write headroom so the ring has room for a full frame while keeping
 * latency low.
 */

#include "clk_sync.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <nrfx_clock.h>

#include "../audio.h"

/* APLL frequency value encoding for 12.165 / 12.288 / 12.411 MHz. */
#define APLL_FREQ_MIN    0x8FD8u
#define APLL_FREQ_CENTER 0x9BA6u
#define APLL_FREQ_MAX    0xA774u

#define APLL_FREQ_ADJ(t_us) (-(((int32_t)(t_us)) * 1000) / 331)

#define DRIFT_MEAS_WINDOW_MS		    100U
#define CALIB_PACKET_COUNT		    MAX(1U, (DRIFT_MEAS_WINDOW_MS / AUDIO_FRAME_DURATION_MS))
#define DRIFT_ERR_THRESH_LOCK		    16
#define DRIFT_ERR_THRESH_UNLOCK		    32
#define DRIFT_REGULATOR_DIV_FACTOR	    2
#define DRIFT_UNLOCK_CONSECUTIVE_LIMIT	    3U
#define DRIFT_MAX_FREQ_STEP		    24
#define DRIFT_ERR_DEADBAND_US		    8
#define DRIFT_OFFSET_RECALIB_MEASURE_LIMIT 10U
#define DRIFT_RING_AVG_SAMPLES		    4U

#define RING_TARGET_LEVEL CONFIG_GRPTLK_RX_TARGET_PREFILL_BLKS
#define RING_SCALE_US	   10

#define UPLINK_Q_DEPTH 4

BUILD_ASSERT(RING_TARGET_LEVEL <= (AUDIO_RING_NUM_BLKS - 1U - AUDIO_BLKS_PER_FRAME),
	     "Target prefill must leave room for one decoded frame");

K_MSGQ_DEFINE(uplink_q, sizeof(struct audio_encoded_frame), UPLINK_Q_DEPTH, 4);

static struct {
	atomic_t ring_level;
	atomic_t new_data;
} g_rx = {
	.ring_level = ATOMIC_INIT(0),
	.new_data = ATOMIC_INIT(0),
};

static atomic_t g_reset = ATOMIC_INIT(1);
static atomic_t g_locked = ATOMIC_INIT(0);
static atomic_t g_state = ATOMIC_INIT(DRIFT_STATE_INIT);

static K_SEM_DEFINE(clk_sync_sem, 0, 1);

/* Preserved across resets so re-sync can keep the last converged APLL value. */
static uint16_t g_freq_value = APLL_FREQ_CENTER;

#define CLK_SYNC_STACK_SIZE 1024
#define CLK_SYNC_PRIORITY   1

K_THREAD_STACK_DEFINE(clk_sync_stack, CLK_SYNC_STACK_SIZE);
static struct k_thread clk_sync_thread_data;

struct ring_avg_state {
	uint16_t samples[DRIFT_RING_AVG_SAMPLES];
	uint32_t sum;
	uint8_t idx;
	uint8_t count;
};

struct drift_state_data {
	enum clk_sync_drift_state state;
	uint32_t center_freq;
	int32_t calib_ring_sum;
	uint32_t calib_ring_count;
	uint8_t unlock_bad_count;
	uint8_t offset_recalib_count;
	struct ring_avg_state ring_avg;
};

static inline int32_t abs_s32(int32_t value)
{
	return (value < 0) ? -value : value;
}

static void ring_avg_reset(struct ring_avg_state *avg)
{
	memset(avg->samples, 0, sizeof(avg->samples));
	avg->sum = 0U;
	avg->idx = 0U;
	avg->count = 0U;
}

static uint32_t ring_avg_update(struct ring_avg_state *avg, uint32_t sample)
{
	if (avg->count < DRIFT_RING_AVG_SAMPLES) {
		avg->samples[avg->idx] = (uint16_t)sample;
		avg->sum += sample;
		avg->count++;
	} else {
		avg->sum -= avg->samples[avg->idx];
		avg->samples[avg->idx] = (uint16_t)sample;
		avg->sum += sample;
	}

	avg->idx = (uint8_t)((avg->idx + 1U) % DRIFT_RING_AVG_SAMPLES);

	return avg->sum / MAX(avg->count, 1U);
}

static int32_t err_us_from_ring(uint32_t ring_level)
{
	return ((int32_t)RING_TARGET_LEVEL - (int32_t)ring_level) * RING_SCALE_US;
}

static int32_t err_us_condition(int32_t err_us)
{
	err_us /= DRIFT_REGULATOR_DIV_FACTOR;

	if (abs_s32(err_us) <= DRIFT_ERR_DEADBAND_US) {
		return 0;
	}

	return err_us;
}

static int32_t freq_adj_condition(int32_t freq_adj)
{
	return CLAMP(freq_adj, -DRIFT_MAX_FREQ_STEP, DRIFT_MAX_FREQ_STEP);
}

static void hfclkaudio_set(uint32_t freq_value)
{
	uint16_t clamped = (uint16_t)CLAMP((int32_t)freq_value, (int32_t)APLL_FREQ_MIN,
					   (int32_t)APLL_FREQ_MAX);

	if (clamped != g_freq_value) {
		g_freq_value = clamped;
		nrfx_clock_hfclkaudio_config_set(g_freq_value);
	}
}

static void drift_state_transition(struct drift_state_data *dc, enum clk_sync_drift_state new_state)
{
	dc->state = new_state;
	atomic_set(&g_state, (atomic_val_t)new_state);
	ring_avg_reset(&dc->ring_avg);

	switch (new_state) {
	case DRIFT_STATE_INIT:
		atomic_set(&g_locked, 0);
		dc->calib_ring_sum = 0;
		dc->calib_ring_count = 0;
		dc->unlock_bad_count = 0U;
		dc->offset_recalib_count = 0U;
		break;

	case DRIFT_STATE_CALIB:
		atomic_set(&g_locked, 0);
		dc->calib_ring_sum = 0;
		dc->calib_ring_count = 0;
		dc->unlock_bad_count = 0U;
		dc->offset_recalib_count = 0U;
		printk("[drift] CALIB\n");
		break;

	case DRIFT_STATE_OFFSET:
		atomic_set(&g_locked, 0);
		dc->unlock_bad_count = 0U;
		dc->offset_recalib_count = 0U;
		printk("[drift] OFFSET center=0x%04x\n", dc->center_freq);
		break;

	case DRIFT_STATE_LOCKED:
		atomic_set(&g_locked, 1);
		dc->unlock_bad_count = 0U;
		dc->offset_recalib_count = 0U;
		printk("[drift] LOCKED freq=0x%04x\n", g_freq_value);
		break;
	}
}

static void reset_internal(struct drift_state_data *dc)
{
	dc->center_freq = APLL_FREQ_CENTER;
	atomic_set(&g_rx.new_data, 0);
	drift_state_transition(dc, DRIFT_STATE_INIT);
	/* g_freq_value intentionally not reset to avoid a harsh APLL jump. */
}

static void clk_sync_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct drift_state_data dc = {
		.state = DRIFT_STATE_INIT,
		.center_freq = APLL_FREQ_CENTER,
	};

	g_freq_value = nrfx_clock_hfclkaudio_config_get();
	if (g_freq_value == 0U) {
		g_freq_value = APLL_FREQ_CENTER;
	}

	while (1) {
		k_sem_take(&clk_sync_sem, K_FOREVER);

		if (atomic_set(&g_reset, 0) != 0) {
			reset_internal(&dc);
			continue;
		}

		if (!atomic_cas(&g_rx.new_data, 1, 0)) {
			continue;
		}

		const uint32_t ring_level_raw = (uint32_t)atomic_get(&g_rx.ring_level);
		const uint32_t ring_level_filtered = ring_avg_update(&dc.ring_avg, ring_level_raw);

		switch (dc.state) {
		case DRIFT_STATE_INIT:
			drift_state_transition(&dc, DRIFT_STATE_CALIB);
			break;

		case DRIFT_STATE_CALIB: {
			dc.calib_ring_sum += (int32_t)ring_level_raw;
			dc.calib_ring_count++;

			if (dc.calib_ring_count < CALIB_PACKET_COUNT) {
				break;
			}

			const int32_t avg_ring = dc.calib_ring_sum / (int32_t)dc.calib_ring_count;
			const int32_t avg_err_us = err_us_condition(err_us_from_ring((uint32_t)avg_ring));
			const int32_t freq_adj = freq_adj_condition(APLL_FREQ_ADJ(avg_err_us));
			const uint32_t cf = (uint32_t)((int32_t)APLL_FREQ_CENTER + freq_adj);

			if ((cf > APLL_FREQ_MAX) || (cf < APLL_FREQ_MIN)) {
				printk("[drift] CALIB invalid center (avg_ring=%d), retry\n", avg_ring);
				drift_state_transition(&dc, DRIFT_STATE_INIT);
				break;
			}

			dc.center_freq = cf;
			hfclkaudio_set(dc.center_freq);
			drift_state_transition(&dc, DRIFT_STATE_OFFSET);
			break;
		}

		case DRIFT_STATE_OFFSET: {
			const int32_t err_us =
				err_us_condition(err_us_from_ring(ring_level_filtered));
			const int32_t freq_adj = freq_adj_condition(APLL_FREQ_ADJ(err_us));

			hfclkaudio_set((uint32_t)((int32_t)dc.center_freq + freq_adj));
			printk("[drift] OFFSET err=%d freq=0x%04x\n", err_us, g_freq_value);

			if (abs_s32(err_us) < DRIFT_ERR_THRESH_LOCK) {
				drift_state_transition(&dc, DRIFT_STATE_LOCKED);
				break;
			}

			if (++dc.offset_recalib_count >= DRIFT_OFFSET_RECALIB_MEASURE_LIMIT) {
				printk("[drift] OFFSET unstable err=%d, recalibrating\n", err_us);
				drift_state_transition(&dc, DRIFT_STATE_INIT);
			}
			break;
		}

		case DRIFT_STATE_LOCKED: {
			const int32_t err_us =
				err_us_condition(err_us_from_ring(ring_level_filtered));
			const int32_t freq_adj = freq_adj_condition(APLL_FREQ_ADJ(err_us));

			hfclkaudio_set((uint32_t)((int32_t)dc.center_freq + freq_adj));

			if (abs_s32(err_us) > DRIFT_ERR_THRESH_UNLOCK) {
				if (++dc.unlock_bad_count >= DRIFT_UNLOCK_CONSECUTIVE_LIMIT) {
					printk("[drift] UNLOCK err=%d, fallback to OFFSET\n", err_us);
					drift_state_transition(&dc, DRIFT_STATE_OFFSET);
				}
			} else {
				dc.unlock_bad_count = 0U;
			}
			break;
		}
		}
	}
}

void clk_sync_init(void)
{
	k_thread_create(&clk_sync_thread_data, clk_sync_stack, K_THREAD_STACK_SIZEOF(clk_sync_stack),
			clk_sync_thread_fn, NULL, NULL, NULL, CLK_SYNC_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&clk_sync_thread_data, "clk_sync");
}

void clk_sync_rx_notify(uint16_t ring_level)
{
	atomic_set(&g_rx.ring_level, (atomic_val_t)ring_level);
	atomic_set(&g_rx.new_data, 1);
	k_sem_give(&clk_sync_sem);
}

void clk_sync_reset(void)
{
	atomic_set(&g_reset, 1);
	k_sem_give(&clk_sync_sem);
}

bool clk_sync_is_locked(void)
{
	return atomic_get(&g_locked) != 0;
}

uint16_t clk_sync_freq_get(void)
{
	return g_freq_value;
}

enum clk_sync_drift_state clk_sync_state_get(void)
{
	return (enum clk_sync_drift_state)atomic_get(&g_state);
}

struct k_msgq *clk_sync_get_uplink_q(void)
{
	return &uplink_q;
}
