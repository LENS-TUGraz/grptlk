#ifndef GRPTLK_VAD_H_
#define GRPTLK_VAD_H_

#include <stdint.h>
#include <zephyr/kernel.h>

/**
 * Initialise the VAD module (libfvad + VOX state machine).
 *
 * @param aggressiveness  libfvad mode 0-3 (0 = quality, 3 = aggressive)
 * @param onset_frames    Consecutive speech frames before TX activation
 * @param hangover_frames Silence frames after last speech before TX off
 * @param tx_sem          Uplink TX semaphore (given on PTT activation)
 * @return 0 on success, negative errno on failure
 */
int vad_init(int aggressiveness, int onset_frames, int hangover_frames,
	     struct k_sem *tx_sem);

/**
 * Feed a mono PCM frame through the VAD.  Call from encoder thread only.
 *
 * For 10 ms mode (160 samples) the frame is processed immediately.
 * For 5 ms mode (80 samples) two consecutive frames are accumulated
 * into a 10 ms buffer before processing.
 *
 * Internally controls ptt_active via the VOX state machine.
 *
 * @param pcm         Mono 16-bit PCM samples
 * @param num_samples Number of samples (80 or 160)
 */
void vad_process_frame(const int16_t *pcm, uint16_t num_samples);

/**
 * Reset the VOX state machine to IDLE and clear ptt_active.
 * Call when VAD mode is disabled or on stream re-sync.
 */
void vad_reset(void);

#endif /* GRPTLK_VAD_H_ */
