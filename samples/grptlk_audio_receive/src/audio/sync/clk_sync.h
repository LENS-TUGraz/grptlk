#ifndef CLK_SYNC_H_
#define CLK_SYNC_H_

/*
 * clk_sync — BLE BIG anchor → HFCLKAUDIO PI controller
 *
 * Slaves the nRF5340 HFCLKAUDIO fractional PLL to the BLE BIG anchor
 * timestamps delivered by iso_recv().  A dedicated high-priority thread
 * (prio 1) wakes on each anchor, measures the inter-arrival interval error
 * against the expected 5000 µs, and trims FREQ_VALUE via
 * nrfx_clock_hfclkaudio_config_set() to eliminate clock drift between the
 * BLE radio crystal and the CS47L63 I2S BCLK.
 *
 * The PI controller drives the steady-state phase error below ±5 µs, which
 * is well within one sample period at 16 kHz (62.5 µs), preventing DMA
 * starvation or overflow glitches over arbitrarily long sessions.
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * Call once before audio_start().  Spawns the clk_sync_thread at priority 1.
 * The thread blocks waiting for anchor notifications; no CPU is consumed
 * before the first iso_recv() call.
 */
void clk_sync_init(void);

/*
 * Called from iso_recv() on every valid BIS1 packet.
 * Safe to call from any context (ISR, BT RX thread): only performs an
 * atomic store followed by k_sem_give(), neither of which blocks.
 */
void clk_sync_anchor_notify(uint32_t ts_us);

/*
 * Reset the PI integrator and lock counter.
 * Call from setup_iso_datapaths() on every BIG re-sync so the controller
 * starts fresh rather than applying a stale correction from the previous
 * session.
 */
void clk_sync_reset(void);

/*
 * Returns true when |phase_err| < 5 µs for 20 consecutive BIG intervals
 * (~100 ms).  Suitable for gating a "sync acquired" LED or log message.
 */
bool clk_sync_is_locked(void);

#endif /* CLK_SYNC_H_ */
