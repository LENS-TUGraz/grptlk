#ifndef CLK_SYNC_H_
#define CLK_SYNC_H_

/*
 * clk_sync — BLE BIG anchor → HFCLKAUDIO drift compensation
 *
 * 4-state machine (INIT → CALIB → OFFSET → LOCKED) ported from the
 * nrf5340_audio audio_datapath.c drift compensation module.
 *
 * Error signal:
 *   CALIB:        average ring buffer level over 100 ms window (20 packets)
 *   OFFSET/LOCKED: per-packet ring level deviation from RING_TARGET_LEVEL
 *
 * The ring buffer directly measures the HFCLKAUDIO rate mismatch:
 *   ring filling  → I2S too slow → HFCLKAUDIO too slow → increase freq
 *   ring draining → I2S too fast → HFCLKAUDIO too fast → decrease freq
 *
 * HFCLKAUDIO drives both I2S DAC (downlink) and I2S ADC (uplink), so a
 * single frequency correction fixes both paths.  TX FIFO level is a health
 * indicator only — no separate TX correction is needed.
 *
 * FREQ_VALUE encoding (nRF5340 PS §5.4.2):
 *   FREQ_VALUE = round(65536 × ((12 × f_out / 32 MHz) − 4))
 *   At f_out = 12.288 MHz → FREQ_VALUE = 0x9BA6
 *   Step size ≈ 40.7 Hz per LSB
 */

#include <stdint.h>
#include <stdbool.h>

enum clk_sync_drift_state {
	DRIFT_STATE_INIT = 0,
	DRIFT_STATE_CALIB = 1,
	DRIFT_STATE_OFFSET = 2,
	DRIFT_STATE_LOCKED = 3,
};

/* Spawns clk_sync_thread at priority 1. Call once before audio_start(). */
void clk_sync_init(void);

/*
 * Called from iso_recv() on each BIS1 packet with BT_ISO_FLAGS_TS set.
 * ISR-safe: only atomic writes + k_sem_give() inside.
 */
void clk_sync_rx_notify(uint32_t sdu_ref_us, uint32_t ring_level);

/* Call from setup_iso_datapaths() on every BIG re-sync. */
void clk_sync_reset(void);

/* Returns uplink queue pointer; defined here to avoid circular dependencies. */
struct k_msgq *clk_sync_get_uplink_q(void);

bool clk_sync_is_locked(void);
uint16_t clk_sync_freq_get(void);
enum clk_sync_drift_state clk_sync_state_get(void);

#endif /* CLK_SYNC_H_ */
