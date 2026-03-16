#ifndef CLK_SYNC_H_
#define CLK_SYNC_H_

/*
 * clk_sync — BLE BIG TX anchor → HFCLKAUDIO integral clock synchroniser
 *
 * Slaves the nRF5340 HFCLKAUDIO fractional PLL to the BLE BIG TX anchor
 * timestamps delivered by bt_iso_chan_get_tx_sync() inside iso_sent().
 * A dedicated high-priority thread wakes on each TX anchor, measures the
 * inter-anchor interval error against the expected 5000 µs, and trims
 * FREQ_VALUE via nrfx_clock_hfclkaudio_config_set() to eliminate drift
 * between the BLE radio crystal and the CS47L63 I2S BCLK.
 *
 * The integrator drives the steady-state phase error below ±5 µs, which is
 * well within one sample period at 16 kHz (62.5 µs), preventing DMA
 * starvation or overflow glitches over arbitrarily long sessions.
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * Call once after audio_start() and after BIG creation.
 * Spawns the clk_sync thread at Zephyr priority 1.
 * The thread blocks until the first iso_sent() call delivers an anchor.
 */
void clk_sync_init(void);

/*
 * Called from iso_sent() after bt_iso_chan_get_tx_sync() succeeds.
 * Safe to call from any context: only performs an atomic store + k_sem_give().
 */
void clk_sync_anchor_notify(uint32_t ts_us);

/*
 * Reset the integrator and lock counter.
 * Call when the BIG is torn down or re-created so the controller starts
 * fresh rather than applying a stale correction.
 */
void clk_sync_reset(void);

/*
 * Returns true when the accumulated phase error has been below the lock
 * threshold for 20 consecutive filter windows (~2 s).
 */
bool clk_sync_is_locked(void);

#endif /* CLK_SYNC_H_ */
