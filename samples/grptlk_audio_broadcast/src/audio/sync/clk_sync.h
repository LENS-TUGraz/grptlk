#ifndef CLK_SYNC_H_
#define CLK_SYNC_H_

/*
 * clk_sync — BLE BIG TX anchor → HFCLKAUDIO phase-error clock synchroniser
 *
 * Slaves the nRF5340 HFCLKAUDIO fractional PLL to the BLE BIG TX anchor
 * using a direct cross-domain phase error:
 *   err = (ble_anchor_ts - i2s_dma_ts) % BIG_INTERVAL_US
 *
 * State machine (ported from nrf5340_audio audio_datapath.c):
 *   INIT   → wait for first valid anchor
 *   CALIB  → 100 ms open-loop measurement to find PLL centre frequency
 *   OFFSET → 100 ms windows, proportional correction; transitions to LOCKED
 *   LOCKED → maintenance mode; reverts to INIT on large error
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
 * Called from the audio DMA callback (audio_rx_mono_frame) each I2S frame.
 * Safe to call from any context: only performs an atomic store.
 */
void clk_sync_i2s_notify(uint32_t ts_us);

/*
 * Reset the state machine back to INIT.
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
