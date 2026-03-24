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
	((AUDIO_SAMPLE_RATE_HZ / 1000) * AUDIO_CHANNELS * (AUDIO_BITS_PER_SAMPLE / 8))

/* 1ms I2S block constants for the ring-buffer audio datapath. */
#define AUDIO_BLK_SAMPLES_MONO   (AUDIO_SAMPLE_RATE_HZ / 1000U)  /* 16 */
#define AUDIO_BLKS_PER_FRAME     (AUDIO_FRAME_DURATION_MS)        /* 5  */
#define AUDIO_RING_NUM_BLKS      10U  /* ~10ms buffer — 3 slots slack vs 5-block write */

BUILD_ASSERT(AUDIO_BLK_SAMPLES_MONO * AUDIO_BLKS_PER_FRAME == AUDIO_SAMPLES_PER_FRAME,
	     "1ms block count x block size must equal frame size");

/*
 * audio_ring — circular FIFO of 1ms stereo I2S blocks.
 * Producer: iso_recv BT RX thread (writes AUDIO_BLKS_PER_FRAME blocks per LC3 frame).
 * Consumer: I2S DMA ISR (reads 1 block per 1ms).
 */
struct audio_ring {
	uint32_t (*fifo)[AUDIO_BLK_SAMPLES_MONO];
	volatile uint16_t *prod_idx;
	volatile uint16_t *cons_idx;
	uint16_t num_blks;
};

typedef void (*audio_rx_cb_t)(const int16_t *mono_frame);

int audio_init(struct audio_ring *ring, audio_rx_cb_t rx_cb);
int audio_start(void);

/* Adjust output volume by step_db dB (+3 = up, -3 = down).
 * Clamped to [−64 dB, 0 dB]. Only available on CS47L63-equipped boards. */
int audio_volume_adjust(int8_t step_db);

/* Return the number of DMA playback underruns since the last call and
 * reset the counter to zero.  Safe to call from any thread. */
uint32_t audio_underrun_count_reset(void);

/* Return the number of 1ms blocks successfully served from the ring to the
 * DMA since the last call and reset the counter to zero.
 * Safe to call from any thread. */
uint32_t audio_served_count_reset(void);

/* Return the longest consecutive underrun run (ms) since the last call and
 * reset to zero.  A run > 1 means the ring was empty for multiple consecutive
 * DMA callbacks — audible as a silence burst or rumble.
 * Safe to call from any thread. */
uint32_t audio_max_consec_underrun_reset(void);

#endif /* GRPTLK_AUDIO_H_ */
