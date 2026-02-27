#ifndef GRPTLK_AUDIO_IO_H_
#define GRPTLK_AUDIO_IO_H_

#include <stdint.h>
#include <zephyr/kernel.h>

#define AUDIO_IO_SAMPLE_RATE_HZ 16000
#define AUDIO_IO_FRAME_DURATION_MS 10
#define AUDIO_IO_CHANNELS 2
#define AUDIO_IO_BITS_PER_SAMPLE 16
#define AUDIO_IO_SAMPLES_PER_FRAME \
	((AUDIO_IO_SAMPLE_RATE_HZ * AUDIO_IO_FRAME_DURATION_MS) / 1000)
#define AUDIO_IO_BLOCK_BYTES \
	(AUDIO_IO_SAMPLES_PER_FRAME * AUDIO_IO_CHANNELS * (AUDIO_IO_BITS_PER_SAMPLE / 8))

typedef void (*audio_io_rx_cb_t)(const int16_t *mono_frame);

int audio_io_init(struct k_msgq *playback_q, audio_io_rx_cb_t rx_cb);
int audio_io_start(void);

#endif /* GRPTLK_AUDIO_IO_H_ */
