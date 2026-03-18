#ifndef GRPTLK_AUDIO_H_
#define GRPTLK_AUDIO_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#define AUDIO_SAMPLE_RATE_HZ    16000
#define AUDIO_FRAME_DURATION_MS 5
#define AUDIO_CHANNELS          2
#define AUDIO_BITS_PER_SAMPLE   16
#define AUDIO_SAMPLES_PER_FRAME \
	((AUDIO_SAMPLE_RATE_HZ * AUDIO_FRAME_DURATION_MS) / 1000)
#define AUDIO_BLOCK_BYTES \
	(AUDIO_SAMPLES_PER_FRAME * AUDIO_CHANNELS * (AUDIO_BITS_PER_SAMPLE / 8))

/*
 * audio_rx_cb_t — called from the DMA ISR every 5 ms with a captured mono
 * PCM frame (80 × int16).  The callback must be ISR-safe: it should only
 * write into a lock-free ring slot (atomic index increment + memcpy), never
 * block or call Zephyr kernel primitives that may schedule.
 */
typedef void (*audio_rx_cb_t)(const int16_t *mono_frame);

/*
 * audio_init — initialise the CS47L63 codec.
 *
 *   playback_q  : message queue that the DMA ISR drains for speaker output.
 *                 Items are stereo PCM blocks (AUDIO_BLOCK_BYTES each).
 *                 The decoder thread is the sole producer; depth=2 is enough.
 *
 *   rx_cb       : called from the DMA ISR with each captured mono mic frame.
 *                 Must write into a lock-free ring — no blocking allowed.
 */
int audio_init(struct k_msgq *playback_q, audio_rx_cb_t rx_cb);

/* Start the I2S DMA engine and lock the CS47L63 FLL.
 * After this returns, the DMA ISR fires every 5 ms. */
int audio_start(void);

/* Adjust output volume by step_db dB (+3 = up, -3 = down).
 * Clamped to the CS47L63 register range. */
int audio_volume_adjust(int8_t step_db);

/* Returns the number of DAC underruns (freeze-frame injections) since the last
 * call and resets the counter to zero.  Call once per log window from the
 * decoder thread. */
uint32_t audio_underrun_count_reset(void);

#endif /* GRPTLK_AUDIO_H_ */
