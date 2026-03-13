#ifndef GRPTLK_AUDIO_H_
#define GRPTLK_AUDIO_H_

#include <stdint.h>
#include <zephyr/kernel.h>

#define AUDIO_SAMPLE_RATE_HZ	16000
#define AUDIO_FRAME_DURATION_MS 5
#define AUDIO_CHANNELS		2
#define AUDIO_BITS_PER_SAMPLE	16
#define AUDIO_SAMPLES_PER_FRAME ((AUDIO_SAMPLE_RATE_HZ * AUDIO_FRAME_DURATION_MS) / 1000)
#define AUDIO_BLOCK_BYTES (AUDIO_SAMPLES_PER_FRAME * AUDIO_CHANNELS * (AUDIO_BITS_PER_SAMPLE / 8))

#include "audio_drift.h"

int audio_init(struct audio_drift_ctx *dl_drift, struct audio_drift_ctx *ul_drift);
int audio_start(void);

/* Adjust output volume by step_db dB (+3 = up, -3 = down).
 * Clamped to [−64 dB, 0 dB]. Only available on CS47L63-equipped boards. */
int audio_volume_adjust(int8_t step_db);

#endif /* GRPTLK_AUDIO_H_ */
