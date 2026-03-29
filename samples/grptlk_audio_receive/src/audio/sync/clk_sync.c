/*
 * clk_sync.c — BLE BIG anchor → HFCLKAUDIO drift compensation
 *
 * 4-state machine:  INIT → CALIB → OFFSET → LOCKED
 *
 * Error signal: audio ring buffer fill level.
 *   ring > RING_TARGET_LEVEL → I2S consuming too slowly → HFCLKAUDIO slow
 *                            → increase APLL freq
 *   ring < RING_TARGET_LEVEL → I2S consuming too fast  → HFCLKAUDIO fast
 *                            → decrease APLL freq
 *
 * err_us = (RING_TARGET_LEVEL − ring_level) × RING_SCALE_US
 *   ring=7 (too full):  err_us = (4−7)×10 = −30 → APLL_FREQ_ADJ(−15) = +45 → freq up ✓
 *   ring=1 (too empty): err_us = (4−1)×10 = +30 → APLL_FREQ_ADJ(+15) = −45 → freq down ✓
 *
 * CALIB: Accumulate ring levels over 20 packets (~100 ms at 5 ms BIS interval).
 * Compute avg_ring_err = avg_ring − RING_TARGET_LEVEL, then set center_freq
 * to compensate for observed steady-state offset.
 *
 * FREQ_VALUE encoding (nRF5340 PS §5.4.2):
 *   FREQ_VALUE = round(65536 × ((12 × f_out / 32 MHz) − 4))
 *   Center: 12.288 MHz → 0x9BA6   Step ≈ 40.7 Hz per LSB
 *
 * APLL_FREQ_ADJ formula (from nrf5340_audio, uses ns to reduce rounding):
 *   APLL_FREQ_ADJ(t_us) = -(t_us * 1000) / 331
 */

#include "clk_sync.h"

#include <zephyr/kernel.h>
#include <nrfx_clock.h>

/* -------------------------------------------------------------------------
 * APLL frequency constants
 * Values from nrf5340_audio/src/modules/audio_i2s.h
 * ------------------------------------------------------------------------- */
#define APLL_FREQ_CENTER 0x9BA6u  /* 12.288 MHz — nominal for 48 kHz codec  */
#define APLL_FREQ_MIN    0x8FD8u  /* 12.165 MHz                              */
#define APLL_FREQ_MAX    0xA774u  /* 12.411 MHz                              */

/* Convert µs timing error to FREQ_VALUE delta.
 * Ported from nrf5340_audio audio_datapath.c — uses ns internally. */
#define APLL_FREQ_ADJ(t) (-(((int32_t)(t)) * 1000) / 331)

/* -------------------------------------------------------------------------
 * Drift compensation constants
 * Ported from nrf5340_audio audio_datapath.c
 * ------------------------------------------------------------------------- */
#define DRIFT_ERR_THRESH_LOCK        16  /* µs: enter LOCKED if |err| < 16   */
#define DRIFT_ERR_THRESH_UNLOCK      32  /* µs: exit  LOCKED if |err| > 32   */
#define DRIFT_REGULATOR_DIV_FACTOR    2  /* proportional gain divisor        */

/* Ring buffer target level and scale factor for error signal. */
#define RING_TARGET_LEVEL  4   /* half of AUDIO_RING_NUM_BLKS = 8         */
#define RING_SCALE_US     10   /* µs per block of deviation from target   */
/* Number of packets in CALIB window (5 ms BIS interval × 20 = 100 ms). */
#define CALIB_PACKET_COUNT 20

/* -------------------------------------------------------------------------
 * Uplink FIFO (shared with main.c via clk_sync_get_uplink_q)
 * ------------------------------------------------------------------------- */
#define UPLINK_Q_DEPTH 4

struct encoded_frame {
	uint8_t data[CONFIG_BT_ISO_TX_MTU];
	size_t  len;
};

K_MSGQ_DEFINE(uplink_q, sizeof(struct encoded_frame), UPLINK_Q_DEPTH, 4);

/* -------------------------------------------------------------------------
 * Shared state — written from iso_recv ISR context, read by sync thread
 * ------------------------------------------------------------------------- */
static struct {
	atomic_t sdu_ref_us;
	atomic_t ring_level;
	atomic_t new_data;        /* 1 after each rx_notify, cleared by thread */
} g_rx = {
	.sdu_ref_us  = ATOMIC_INIT(0),
	.ring_level  = ATOMIC_INIT(0),
	.new_data    = ATOMIC_INIT(0),
};

static atomic_t g_reset  = ATOMIC_INIT(1);
static atomic_t g_locked = ATOMIC_INIT(0);
static atomic_t g_state  = ATOMIC_INIT(DRIFT_STATE_INIT);

static K_SEM_DEFINE(clk_sync_sem, 0, 1);

/* Current FREQ_VALUE — preserved across resets so re-sync uses the last
 * converged value rather than always starting cold from nominal. */
static uint16_t g_freq_value = APLL_FREQ_CENTER;

/* -------------------------------------------------------------------------
 * Thread
 * ------------------------------------------------------------------------- */
#define CLK_SYNC_STACK_SIZE 1024
#define CLK_SYNC_PRIORITY   1

K_THREAD_STACK_DEFINE(clk_sync_stack, CLK_SYNC_STACK_SIZE);
static struct k_thread clk_sync_thread_data;

struct drift_state_data {
	enum clk_sync_drift_state state;
	uint32_t center_freq;

	/* CALIB: ring level accumulator over CALIB_PACKET_COUNT packets */
	int32_t  calib_ring_sum;
	uint32_t calib_ring_count;
};

/* Apply FREQ_VALUE to hardware, clamped to valid range. */
static void hfclkaudio_set(uint32_t freq_value)
{
	uint16_t clamped = (uint16_t)CLAMP((int32_t)freq_value,
					   (int32_t)APLL_FREQ_MIN,
					   (int32_t)APLL_FREQ_MAX);

	if (clamped != g_freq_value) {
		g_freq_value = clamped;
		nrfx_clock_hfclkaudio_config_set(g_freq_value);
	}
}

/*
 * Compute per-packet error in µs from ring buffer level.
 * Positive err_us → freq should decrease (HFCLKAUDIO too fast).
 * Negative err_us → freq should increase (HFCLKAUDIO too slow).
 */
static int32_t err_us_from_ring(uint32_t ring_level)
{
	return ((int32_t)RING_TARGET_LEVEL - (int32_t)ring_level) * RING_SCALE_US;
}

static void drift_state_transition(struct drift_state_data *dc,
				   enum clk_sync_drift_state new_state)
{
	dc->state = new_state;
	atomic_set(&g_state, (atomic_val_t)new_state);

	switch (new_state) {
	case DRIFT_STATE_INIT:
		atomic_set(&g_locked, 0);
		break;

	case DRIFT_STATE_CALIB:
		dc->calib_ring_sum   = 0;
		dc->calib_ring_count = 0;
		printk("[drift] CALIB\n");
		break;

	case DRIFT_STATE_OFFSET:
		printk("[drift] OFFSET center=0x%04x\n", dc->center_freq);
		break;

	case DRIFT_STATE_LOCKED:
		atomic_set(&g_locked, 1);
		printk("[drift] LOCKED freq=0x%04x\n", g_freq_value);
		break;
	}
}

static void reset_internal(struct drift_state_data *dc)
{
	dc->center_freq = APLL_FREQ_CENTER;
	drift_state_transition(dc, DRIFT_STATE_INIT);
	/* freq_value intentionally NOT reset — the last converged value is the
	 * best estimate of crystal offset and avoids audible clicks on re-sync. */
}

static void clk_sync_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct drift_state_data dc = {
		.state       = DRIFT_STATE_INIT,
		.center_freq = APLL_FREQ_CENTER,
	};

	/* Warm-start: read the hardware register so firmware restart picks up
	 * the last converged value rather than cold-starting from nominal. */
	g_freq_value = nrfx_clock_hfclkaudio_config_get();
	if (g_freq_value == 0U) {
		g_freq_value = APLL_FREQ_CENTER;
	}

	while (1) {
		/* Block until iso_recv() delivers a notification (or reset). */
		k_sem_take(&clk_sync_sem, K_FOREVER);

		/* Handle BIG re-sync / loss. */
		if (atomic_cas(&g_reset, 1, 0)) {
			reset_internal(&dc);
			continue;
		}

		/* Consume notification; drop spurious wakes. */
		if (!atomic_cas(&g_rx.new_data, 1, 0)) {
			continue;
		}

		uint32_t ring_level = (uint32_t)atomic_get(&g_rx.ring_level);

		switch (dc.state) {

		case DRIFT_STATE_INIT:
			/* First valid packet after reset → start calibration window. */
			drift_state_transition(&dc, DRIFT_STATE_CALIB);
			break;

		case DRIFT_STATE_CALIB: {
			/*
			 * Accumulate ring level for CALIB_PACKET_COUNT packets.
			 * At window end, compute avg_ring_err to set center_freq.
			 */
			dc.calib_ring_sum += (int32_t)ring_level;
			dc.calib_ring_count++;

			if (dc.calib_ring_count < CALIB_PACKET_COUNT) {
				break;
			}

			/* Window complete: avg deviation from target. */
			int32_t avg_ring = dc.calib_ring_sum / (int32_t)CALIB_PACKET_COUNT;
			int32_t avg_err_us = ((int32_t)RING_TARGET_LEVEL - avg_ring)
					     * RING_SCALE_US
					     / DRIFT_REGULATOR_DIV_FACTOR;
			int32_t freq_adj = APLL_FREQ_ADJ(avg_err_us);
			uint32_t cf = (uint32_t)((int32_t)APLL_FREQ_CENTER + freq_adj);

			if (cf > APLL_FREQ_MAX || cf < APLL_FREQ_MIN) {
				printk("[drift] CALIB invalid center (avg_ring=%d), retry\n",
				       avg_ring);
				drift_state_transition(&dc, DRIFT_STATE_INIT);
				break;
			}

			dc.center_freq = cf;
			hfclkaudio_set(dc.center_freq);
			drift_state_transition(&dc, DRIFT_STATE_OFFSET);
			break;
		}

		case DRIFT_STATE_OFFSET: {
			/*
			 * Apply proportional residual correction per packet.
			 * Transition to LOCKED when ring is close enough to target.
			 */
			int32_t err_us = err_us_from_ring(ring_level);

			err_us /= DRIFT_REGULATOR_DIV_FACTOR;
			int32_t freq_adj = APLL_FREQ_ADJ(err_us);

			hfclkaudio_set((uint32_t)((int32_t)dc.center_freq + freq_adj));

			printk("[drift] OFFSET err=%d freq=0x%04x\n",
			       err_us, g_freq_value);

			if (err_us < DRIFT_ERR_THRESH_LOCK &&
			    err_us > -DRIFT_ERR_THRESH_LOCK) {
				drift_state_transition(&dc, DRIFT_STATE_LOCKED);
			}
			break;
		}

		case DRIFT_STATE_LOCKED: {
			/*
			 * Steady-state proportional correction.
			 * Unlock if ring deviates too far from target.
			 */
			int32_t err_us = err_us_from_ring(ring_level);

			err_us /= DRIFT_REGULATOR_DIV_FACTOR;
			int32_t freq_adj = APLL_FREQ_ADJ(err_us);

			hfclkaudio_set((uint32_t)((int32_t)dc.center_freq + freq_adj));

			if (err_us > DRIFT_ERR_THRESH_UNLOCK ||
			    err_us < -DRIFT_ERR_THRESH_UNLOCK) {
				printk("[drift] UNLOCK err=%d, recalibrating\n",
				       err_us);
				drift_state_transition(&dc, DRIFT_STATE_INIT);
			}
			break;
		}
		}
	}
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void clk_sync_init(void)
{
	k_thread_create(&clk_sync_thread_data, clk_sync_stack,
			K_THREAD_STACK_SIZEOF(clk_sync_stack),
			clk_sync_thread_fn, NULL, NULL, NULL,
			CLK_SYNC_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&clk_sync_thread_data, "clk_sync");
}

void clk_sync_rx_notify(uint32_t sdu_ref_us, uint32_t ring_level)
{
	atomic_set(&g_rx.sdu_ref_us, (atomic_val_t)sdu_ref_us);
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
