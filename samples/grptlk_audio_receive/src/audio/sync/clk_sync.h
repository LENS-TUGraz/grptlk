#ifndef CLK_SYNC_H_
#define CLK_SYNC_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * clk_sync - controller-agnostic HFCLKAUDIO drift compensation
 *
 * The receiver uses playback ring fill, sampled just before each decoded frame
 * is written into the ring, as a direct indicator of local audio clock drift.
 * The target is a small pre-write headroom rather than midpoint fill.
 */

enum clk_sync_drift_state {
	DRIFT_STATE_INIT = 0,
	DRIFT_STATE_CALIB = 1,
	DRIFT_STATE_OFFSET = 2,
	DRIFT_STATE_LOCKED = 3,
};

/* Spawns clk_sync_thread at priority 1. Call once before audio_start(). */
void clk_sync_init(void);

/* Called from the decoder thread before writing a decoded LC3 frame. */
void clk_sync_rx_notify(uint16_t ring_level);

/* Call from setup_iso_datapaths() on every BIG re-sync. */
void clk_sync_reset(void);

/* Returns uplink queue pointer; defined here to avoid circular dependencies. */
struct k_msgq *clk_sync_get_uplink_q(void);

bool clk_sync_is_locked(void);
uint16_t clk_sync_freq_get(void);
enum clk_sync_drift_state clk_sync_state_get(void);

#endif /* CLK_SYNC_H_ */
