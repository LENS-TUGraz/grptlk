/*
 * clk_sync.c — BLE BIG TX anchor → HFCLKAUDIO phase-error clock synchroniser
 *
 * Ported from nrf5340_audio audio_datapath.c (Nordic Semiconductor).
 * Adapted for the grptlk_audio_broadcast context:
 *   - BLE anchor source: bt_iso_chan_get_tx_sync() in iso_sent()
 *   - I2S source:        audio DMA callback (audio_rx_mono_frame in main.c)
 *   - BIG interval / I2S frame period is configured at runtime
 *
 * State machine
 * -------------
 * INIT   → Waits for first valid BLE anchor.  Records meas_start_ts and
 *          moves to CALIB.
 *
 * CALIB  → Open-loop 100 ms measurement window.  Counts the configured number
 *          of anchor wakeups, then computes the frequency error from the total BLE
 *          anchor drift vs expected DRIFT_MEAS_PERIOD_US.  Sets center_freq and
 *          moves to OFFSET.
 *
 * OFFSET → Closed-loop 100 ms windows.  Every configured correction window
 *          computes the direct cross-domain phase error
 *          (ble_ts - i2s_ts) % BIG_INTERVAL_US and applies
 *          center_freq + APLL_FREQ_ADJ(err/2).  Transitions to LOCKED when
 *          |err| < DRIFT_ERR_THRESH_LOCK for one window.
 *
 * LOCKED → Identical to OFFSET.  Returns to INIT when |err| >
 *          DRIFT_ERR_THRESH_UNLOCK.
 *
 * FREQ_VALUE encoding (nRF5340 PS §5.4.2)
 * ----------------------------------------
 *   FREQ_VALUE = round(65536 × ((12 × f_out / 32 MHz) − 4))
 *   At f_out = 12.288 MHz → FREQ_VALUE = 0x9BA6
 *   One LSB ≈ 40.7 Hz ≈ 3.3 ppm at 12.288 MHz
 */

#include "clk_sync.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <nrfx_clock.h>

/* -------------------------------------------------------------------------
 * Tuning constants
 * ------------------------------------------------------------------------- */

/* Drift measurement window: 100 ms. */
#define DRIFT_MEAS_PERIOD_US    100000

/* Proportional gain divisor: halve the per-window error before applying. */
#define DRIFT_REGULATOR_DIV     2

/* Lock / unlock hysteresis thresholds (µs). */
#define DRIFT_ERR_THRESH_LOCK   16
#define DRIFT_ERR_THRESH_UNLOCK 32

/* Runaway guard: discard window if |err| exceeds this after divide (µs). */
#define RUNAWAY_THRESH_US       500

/* HFCLKAUDIO FREQ_VALUE register limits (nRF5340 PS §5.4.2). */
#define FREQ_NOMINAL  0x9BA6u  /* 12.288 MHz */
#define FREQ_MIN      0x8000u
#define FREQ_MAX      0xBFFFu

/*
 * APLL_FREQ_ADJ — convert µs phase error over a 100 ms window to FREQ_VALUE
 * register steps.
 *
 * Derivation (from nrf5340_audio):
 *   ppm_err  = err_us / DRIFT_MEAS_PERIOD_US × 1e6
 *            = err_us × 10
 *   Δfreq_Hz = ppm_err × 12.288 MHz / 1e6
 *            = err_us × 10 × 12.288
 *   FREQ_STEP = Δfreq_Hz / 40.7 Hz per LSB
 *             ≈ err_us × 10 × 12.288 / 40.7
 *             ≈ err_us × 3.02
 *             ≈ -(err_us × 1000 / 331)
 * Sign: positive err_us → audio clock is slow → increase FREQ_VALUE.
 * The nrf5340_audio implementation negates: return -(t*1000/331).
 * We match that sign convention.
 */
#define APLL_FREQ_ADJ(t)  (-((int32_t)(t) * 1000) / 331)

/* -------------------------------------------------------------------------
 * State machine
 * ------------------------------------------------------------------------- */

enum clk_state {
	CLK_STATE_INIT   = 0,
	CLK_STATE_CALIB,
	CLK_STATE_OFFSET,
	CLK_STATE_LOCKED,
};

/* -------------------------------------------------------------------------
 * Shared state (written from any context, read by sync thread)
 * ------------------------------------------------------------------------- */

static atomic_t g_ts_us     = ATOMIC_INIT(0); /* latest BLE TX anchor (µs) */
static atomic_t g_i2s_ts_us = ATOMIC_INIT(0); /* latest I2S DMA frame (µs) */
static atomic_t g_reset     = ATOMIC_INIT(1); /* 1 = thread should re-init  */
static atomic_t g_locked    = ATOMIC_INIT(0); /* 1 = controller converged   */
static uint32_t g_blk_period_us = 5000U;
static uint32_t g_waiting_cnt   = DRIFT_MEAS_PERIOD_US / 5000U;

/* Semaphore given by clk_sync_anchor_notify() each BIG anchor. */
static K_SEM_DEFINE(clk_sync_sem, 0, 1);

/* -------------------------------------------------------------------------
 * Sync thread
 * ------------------------------------------------------------------------- */

#define CLK_SYNC_STACK_SIZE 1024
#define CLK_SYNC_PRIORITY   1   /* above lc3_encoder (2) and lc3_decoder (3) */

K_THREAD_STACK_DEFINE(clk_sync_stack, CLK_SYNC_STACK_SIZE);
static struct k_thread clk_sync_thread_data;

/*
 * err_us_calculate — direct cross-domain phase error (µs).
 *
 * Returns (ble_ts - i2s_ts) wrapped to
 * (−configured_big_interval/2, +configured_big_interval/2].
 */
static int32_t err_us_calculate(uint32_t ble_ts, uint32_t i2s_ts)
{
	int64_t total = (int64_t)ble_ts - (int64_t)i2s_ts;
	bool neg = (total < 0);
	uint32_t blk_period_us = g_blk_period_us;

	if (neg) {
		total = -total;
	}

	int32_t err = (int32_t)(total % blk_period_us);

	if (err > (int32_t)(blk_period_us / 2U)) {
		err -= (int32_t)blk_period_us;
	}

	return neg ? -err : err;
}

static void clk_sync_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	enum clk_state state      = CLK_STATE_INIT;
	uint16_t       center_freq = FREQ_NOMINAL;
	uint32_t       meas_start_ts = 0;
	uint32_t       wait_count  = 0;
	uint32_t       prev_ble_ts = 0;

	/* Restore last converged value across firmware restarts. */
	uint16_t hw = nrfx_clock_hfclkaudio_config_get();

	if (hw != 0U) {
		center_freq = hw;
	}

	while (1) {
		k_sem_take(&clk_sync_sem, K_FOREVER);

		/* External reset (BIG torn down / re-created). */
		if (atomic_cas(&g_reset, 1, 0)) {
			state         = CLK_STATE_INIT;
			wait_count    = 0;
			meas_start_ts = 0;
			prev_ble_ts   = 0;
			atomic_set(&g_locked, 0);
			printk("[clk] RESET → INIT interval=%u wait=%u\n",
			       g_blk_period_us, g_waiting_cnt);
			continue;
		}

		uint32_t ble_ts = (uint32_t)atomic_get(&g_ts_us);

		switch (state) {

		/* ---------------------------------------------------------- */
		case CLK_STATE_INIT:
			meas_start_ts = ble_ts;
			prev_ble_ts   = ble_ts;
			wait_count    = 0;
			state         = CLK_STATE_CALIB;
			printk("[clk] CALIB start ts=%u interval=%u wait=%u\n",
			       ble_ts, g_blk_period_us, g_waiting_cnt);
			break;

		/* ---------------------------------------------------------- */
		case CLK_STATE_CALIB:
			prev_ble_ts = ble_ts;
			wait_count++;

			if (wait_count < g_waiting_cnt) {
				break;
			}

			/* End of 100 ms CALIB window. */
			{
				uint32_t elapsed = ble_ts - meas_start_ts;
				int32_t  err_us  = (int32_t)DRIFT_MEAS_PERIOD_US
						   - (int32_t)elapsed;
				int32_t  adj     = APLL_FREQ_ADJ(err_us);
				int32_t  cf      = (int32_t)FREQ_NOMINAL + adj;

				printk("[clk] CALIB: elapsed=%u err=%d adj=%d cf=0x%04x\n",
				       elapsed, err_us, adj, (unsigned)cf);

				if (cf < (int32_t)FREQ_MIN || cf > (int32_t)FREQ_MAX) {
					printk("[clk] CALIB out of range → INIT\n");
					state      = CLK_STATE_INIT;
					wait_count = 0;
					break;
				}

				center_freq = (uint16_t)cf;
				nrfx_clock_hfclkaudio_config_set(center_freq);
				printk("[clk] OFFSET start center=0x%04x\n", center_freq);
				state      = CLK_STATE_OFFSET;
				wait_count = 0;
			}
			break;

		/* ---------------------------------------------------------- */
		case CLK_STATE_OFFSET:  /* FALLTHROUGH */
		case CLK_STATE_LOCKED:
			prev_ble_ts = ble_ts;
			wait_count++;

			if (wait_count < g_waiting_cnt) {
				break;
			}

			/* End of 100 ms correction window. */
			{
				uint32_t i2s_ts = (uint32_t)atomic_get(&g_i2s_ts_us);
				int32_t  err_us = err_us_calculate(prev_ble_ts, i2s_ts)
						  / DRIFT_REGULATOR_DIV;

				printk("[clk] %s: ble=%u i2s=%u err=%d us\n",
				       state == CLK_STATE_LOCKED ? "LOCKED" : "OFFSET",
				       prev_ble_ts, i2s_ts, err_us);

				wait_count = 0;

				/* Runaway guard. */
				if (err_us > RUNAWAY_THRESH_US ||
				    err_us < -RUNAWAY_THRESH_US) {
					printk("[clk] RUNAWAY err=%d → skip\n", err_us);
					break;
				}

				uint16_t new_freq = (uint16_t)CLAMP(
					(int32_t)center_freq + APLL_FREQ_ADJ(err_us),
					(int32_t)FREQ_MIN,
					(int32_t)FREQ_MAX);

				nrfx_clock_hfclkaudio_config_set(new_freq);

				int32_t abs_err = err_us < 0 ? -err_us : err_us;

				if (state == CLK_STATE_OFFSET) {
					if (abs_err < DRIFT_ERR_THRESH_LOCK) {
						atomic_set(&g_locked, 1);
						state = CLK_STATE_LOCKED;
						printk("[clk] LOCKED freq=0x%04x\n", new_freq);
					}
				} else { /* LOCKED */
					if (abs_err > DRIFT_ERR_THRESH_UNLOCK) {
						atomic_set(&g_locked, 0);
						state      = CLK_STATE_INIT;
						wait_count = 0;
						printk("[clk] UNLOCK err=%d → INIT\n", err_us);
					}
				}
			}
			break;
		}
	}
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

int clk_sync_init(uint32_t blk_period_us)
{
	if (blk_period_us == 0U || blk_period_us > DRIFT_MEAS_PERIOD_US ||
	    (DRIFT_MEAS_PERIOD_US % blk_period_us) != 0U) {
		printk("[clk] invalid interval=%u us\n", blk_period_us);
		return -EINVAL;
	}

	g_blk_period_us = blk_period_us;
	g_waiting_cnt = DRIFT_MEAS_PERIOD_US / blk_period_us;
	printk("[clk] init interval=%u wait=%u\n", g_blk_period_us, g_waiting_cnt);

	k_thread_create(&clk_sync_thread_data, clk_sync_stack,
			K_THREAD_STACK_SIZEOF(clk_sync_stack),
			clk_sync_thread_fn, NULL, NULL, NULL,
			CLK_SYNC_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&clk_sync_thread_data, "clk_sync");
	return 0;
}

void clk_sync_anchor_notify(uint32_t ts_us)
{
	atomic_set(&g_ts_us, (atomic_val_t)ts_us);
	k_sem_give(&clk_sync_sem);
}

void clk_sync_i2s_notify(uint32_t ts_us)
{
	atomic_set(&g_i2s_ts_us, (atomic_val_t)ts_us);
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
