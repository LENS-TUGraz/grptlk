#ifndef GRPTLK_AUDIO_H_
#define GRPTLK_AUDIO_H_

#include <stdint.h>
#include <zephyr/kernel.h>

#define AUDIO_SAMPLE_RATE_HZ 16000
#define AUDIO_FRAME_DURATION_MS 5
#define AUDIO_CHANNELS 2
#define AUDIO_BITS_PER_SAMPLE 16
#define AUDIO_SAMPLES_PER_FRAME \
	((AUDIO_SAMPLE_RATE_HZ * AUDIO_FRAME_DURATION_MS) / 1000)
#define AUDIO_BLOCK_BYTES \
	(AUDIO_SAMPLES_PER_FRAME * AUDIO_CHANNELS * (AUDIO_BITS_PER_SAMPLE / 8))

typedef void (*audio_rx_cb_t)(const int16_t *mono_frame);

int audio_init(struct k_msgq *playback_q, audio_rx_cb_t rx_cb);
int audio_start(void);

/* Adjust output volume by step_db dB (+3 = up, -3 = down).
 * Clamped to [−64 dB, 0 dB]. Only available on CS47L63-equipped boards. */
int audio_volume_adjust(int8_t step_db);

/* Return the number of DMA playback underruns since the last call and
 * reset the counter to zero.  Safe to call from any thread. */
uint32_t audio_underrun_count_reset(void);

#endif /* GRPTLK_AUDIO_H_ */
