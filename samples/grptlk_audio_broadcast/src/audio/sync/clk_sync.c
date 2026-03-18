/*
 * clk_sync.c — BLE BIG TX anchor → HFCLKAUDIO integral clock synchroniser
 *
 * How it works
 * ------------
 * The nRF5340 HFCLKAUDIO fractional PLL is driven from the 32 MHz crystal
 * that is shared with the BLE radio.  Despite sharing the same source, the
 * HFCLKAUDIO divide ratio is set independently and drifts relative to the
 * BLE BIG anchor once the codec is running.  Over a 60-second session at a
 * typical ±20 ppm crystal error the accumulated drift is ~2.4 ms — enough
 * to cause periodic DMA underruns (glitches).
 *
 * This module eliminates that drift by:
 *   1. Receiving BLE BIG TX anchor timestamps (µs) from iso_sent() via
 *      bt_iso_chan_get_tx_sync(), fed into clk_sync_anchor_notify().
 *   2. In a dedicated high-priority thread, accumulating inter-anchor
 *      interval errors over FILTER_FRAMES (20) frames to average out
 *      HCI transport jitter (±100–500 µs per frame).
 *   3. Applying at most ±1 FREQ_VALUE step per filter window — the slowest
 *      rate that still corrects ±20 ppm drift within ~10 seconds.
 *
 * Why bt_iso_chan_get_tx_sync() instead of iso_sent() directly
 * ------------------------------------------------------------
 * The iso_sent() callback carries no hardware timestamp.  bt_iso_chan_get_tx_sync()
 * returns a struct bt_iso_tx_info with a µs-resolution BT controller anchor
 * timestamp — the TX-side equivalent of bt_iso_recv_info->ts on the receiver.
 *
 * Why pure integral (no proportional term)
 * ----------------------------------------
 * HCI transport adds ±100–500 µs per-frame jitter, far larger than the real
 * clock drift (~0.1 µs/frame at 20 ppm).  A proportional term reacts to
 * jitter every 5 ms, applying large FREQ_VALUE steps that cause audible
 * frequency discontinuities (clicks, knocking).  The integrator, fed a
 * filtered average, sees only the true long-term drift and corrects silently.
 *
 * FREQ_VALUE encoding (nRF5340 PS §5.4.2):
 *   FREQ_VALUE = round(65536 × ((12 × f_out / 32 MHz) − 4))
 *   At f_out = 12.288 MHz → FREQ_VALUE = 0x9BA6
 *   Step size ≈ 32e6 / (12 × 65536) ≈ 40.7 Hz per LSB
 *
 * Convergence: at 20 ppm drift, accumulated error over 20 frames (100 ms)
 * is 20 ppm × 100 ms = 2 µs → 1 step corrects ~40 Hz → converges in ~10 s.
 * Jitter averages to zero over the 20-frame window.
 */

#include "clk_sync.h"

#include <zephyr/kernel.h>
#include <nrfx_clock.h>

/* -------------------------------------------------------------------------
 * Tuning constants
 * ------------------------------------------------------------------------- */

/* Number of BIG frames to average before making a FREQ_VALUE decision.
 * 20 frames = 100 ms window.  HCI jitter averages to ~0 over this window;
 * real 20 ppm drift accumulates to ~2 µs — enough to trigger a correction. */
#define FILTER_FRAMES 20

/* Maximum FREQ_VALUE correction per filter window: ±1 step = ±40.7 Hz.
 * Limits the rate of frequency change to prevent audible transients. */
#define MAX_STEP_PER_WINDOW 1

/* Accumulated error threshold (µs over FILTER_FRAMES) to trigger a step.
 * The BLE ticker runs at 32.768 kHz (1 tick = 30.5 µs).  A 5 ms BIG interval
 * is 163.84 ticks, which rounds to 163 or 164 — producing ±25 µs quantisation
 * noise in every anchor timestamp.  This noise is ~250× larger than the real
 * 20 ppm crystal drift (0.1 µs/frame).  Setting the threshold above the noise
 * ceiling (40 µs) means the PLL ignores ticker artefacts entirely and only
 * corrects genuine multi-tick accumulated bias. */
#define CORRECTION_THRESH_US 40

/* Windup limit on the accumulated error (µs).
 * Caps to ±500 µs to handle startup or large one-off jitter events. */
#define MAX_ACCUM_US 500

/* HFCLKAUDIO FREQ_VALUE register limits (nRF5340 PS §5.4.2).
 * Full range is 0x0000–0xFFFF but the datasheet mandates:
 *   f_out ∈ [10.666, 13.333] MHz  → FREQ_VALUE ∈ [0x8000, 0xBFFF]
 * Using a narrower window (±~1.6 MHz around 12.288 MHz) for safety. */
#define FREQ_NOMINAL 0x9BA6u  /* 12.288 MHz */
#define FREQ_MIN     0x8000u
#define FREQ_MAX     0xBFFFu

/* Lock: single-window accumulated error must be below this (µs).
 * Must be < CORRECTION_THRESH_US (40) for hysteresis.  35 µs is just inside
 * the 32kHz ticker noise ceiling (~37 µs max), so a freq sitting at the
 * quantisation boundary with |window_err| ≤ 35 µs counts as locked. */
#define LOCK_THRESH_US 35

/* Number of consecutive filter windows within threshold before "locked". */
#define LOCK_COUNT 20

/* Runaway guard: discard inter-anchor measurement if |phase_err| exceeds
 * this (µs).  Triggered by BLE packet loss or multi-frame scheduling gaps. */
#define RUNAWAY_THRESH_US 500

/* BIG downlink interval in µs (LC3Plus 5 ms). */
#define BIG_INTERVAL_US 5000

/* -------------------------------------------------------------------------
 * Shared state (written from any context, read by sync thread)
 * ------------------------------------------------------------------------- */

static atomic_t g_ts_us  = ATOMIC_INIT(0); /* latest TX anchor timestamp */
static atomic_t g_reset  = ATOMIC_INIT(1); /* 1 = thread should re-initialise */
static atomic_t g_locked = ATOMIC_INIT(0); /* 1 = integrator has converged */

static K_SEM_DEFINE(clk_sync_sem, 0, 1); /* depth=1: one pending notification max */

/* -------------------------------------------------------------------------
 * Sync thread
 * ------------------------------------------------------------------------- */

#define CLK_SYNC_STACK_SIZE 1024
#define CLK_SYNC_PRIORITY   1 /* above lc3_encoder (2) and lc3_decoder (3) */

K_THREAD_STACK_DEFINE(clk_sync_stack, CLK_SYNC_STACK_SIZE);
static struct k_thread clk_sync_thread_data;

static void reset_internal(int32_t *accum, int *frame_count,
			    int *lock_count, uint32_t *ts_prev)
{
	*accum       = 0;
	*frame_count = 0;
	*lock_count  = 0;
	*ts_prev     = 0;
	atomic_set(&g_locked, 0);
	/* freq_value intentionally NOT reset — the converged value is the best
	 * estimate of the crystal offset and should be preserved across BIG
	 * re-syncs.  Snapping back to FREQ_NOMINAL causes an 81 Hz instantaneous
	 * step on the live I2S BCLK which is audible as a click/knock. */
}

static void clk_sync_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	/* Initialise from the hardware register so a firmware restart picks up
	 * the last converged value rather than always starting from nominal. */
	uint16_t freq_value = nrfx_clock_hfclkaudio_config_get();

	if (freq_value == 0U) {
		freq_value = FREQ_NOMINAL; /* hardware not yet started */
	}

	int32_t  accum         = 0;
	int      frame_count   = 0;
	int      lock_count    = 0;
	uint32_t ts_prev       = 0;
	bool     have_baseline = false;

	while (1) {
		k_sem_take(&clk_sync_sem, K_FOREVER);

		if (atomic_cas(&g_reset, 1, 0)) {
			reset_internal(&accum, &frame_count, &lock_count, &ts_prev);
			have_baseline = false;
			continue;
		}

		uint32_t ts_now = (uint32_t)atomic_get(&g_ts_us);

		if (!have_baseline) {
			ts_prev       = ts_now;
			have_baseline = true;
			continue;
		}

		/* Signed inter-anchor interval error.
		 * int32_t cast handles µs counter wrap correctly. */
		int32_t interval_us = (int32_t)(ts_now - ts_prev);
		int32_t phase_err   = interval_us - BIG_INTERVAL_US;
		ts_prev = ts_now;

		/* Runaway guard: discard measurements caused by BLE packet loss
		 * or multi-interval scheduling gaps.  Keep FREQ_VALUE and lock
		 * state unchanged — the oscillator didn't jump, we just missed
		 * reference edges.  Resetting lock_count here would cause
		 * permanent locked=0 whenever BLE air loss occurs (one missed
		 * frame → next inter-arrival = N×5000 µs → guard fires → lock
		 * reset → never accumulates 20 consecutive clean windows). */
		if (phase_err > RUNAWAY_THRESH_US || phase_err < -RUNAWAY_THRESH_US) {
			accum       = 0;
			frame_count = 0;
			continue;
		}

		accum = CLAMP(accum + phase_err, -MAX_ACCUM_US, MAX_ACCUM_US);
		frame_count++;

		if (frame_count < FILTER_FRAMES) {
			continue;
		}

		/* --- End of filter window: decide correction -------------------- */
		int32_t window_err = accum;

		accum       = 0;
		frame_count = 0;

		/* Positive window_err → interval was too long → audio clock slow
		 * → increase FREQ_VALUE to speed up HFCLKAUDIO. */
		int32_t step = 0;

		if (window_err > CORRECTION_THRESH_US) {
			step = MAX_STEP_PER_WINDOW;
		} else if (window_err < -CORRECTION_THRESH_US) {
			step = -MAX_STEP_PER_WINDOW;
		}

		if (step != 0) {
			uint16_t new_freq = (uint16_t)CLAMP((int32_t)freq_value + step,
							    (int32_t)FREQ_MIN,
							    (int32_t)FREQ_MAX);
			bool was_locked = atomic_get(&g_locked) != 0;

			if (new_freq != freq_value) {
				freq_value = new_freq;
				nrfx_clock_hfclkaudio_config_set(freq_value);
			}
			printk("[clk] freq=0x%04x step=%+d err=%+d us%s\n",
			       freq_value, (int)step, (int)window_err,
			       was_locked ? " [UNLOCK]" : "");
			lock_count = 0;
			atomic_set(&g_locked, 0);
		} else {
			if (lock_count < LOCK_COUNT) {
				lock_count++;
			}
			if (lock_count == LOCK_COUNT && !atomic_get(&g_locked)) {
				printk("[clk] LOCKED freq=0x%04x\n", freq_value);
				atomic_set(&g_locked, 1);
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

void clk_sync_anchor_notify(uint32_t ts_us)
{
	atomic_set(&g_ts_us, (atomic_val_t)ts_us);
	/* k_sem_give is ISR-safe and never blocks.  If the thread hasn't consumed
	 * the previous notification yet (depth=1 semaphore), this is a no-op —
	 * the thread will see the freshest timestamp via g_ts_us on its next wake. */
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
