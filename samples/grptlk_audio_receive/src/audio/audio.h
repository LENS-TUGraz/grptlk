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

/* 1ms I2S block constants for the ring-buffer audio datapath. */
#define AUDIO_BLK_PERIOD_US      1000U
#define AUDIO_BLK_SAMPLES_MONO   (AUDIO_SAMPLE_RATE_HZ / 1000U)  /* 16 */
#define AUDIO_BLKS_PER_FRAME     (AUDIO_FRAME_DURATION_MS)        /* 5  */
#define AUDIO_RING_NUM_BLKS      7U  /* ~7ms buffer */

BUILD_ASSERT(AUDIO_BLK_SAMPLES_MONO * AUDIO_BLKS_PER_FRAME == AUDIO_SAMPLES_PER_FRAME,
	     "1ms block count × block size must equal frame size");

/*
 * audio_ring — circular FIFO of 1ms stereo I2S blocks.
 * Producer: decoder thread (writes AUDIO_BLKS_PER_FRAME blocks per LC3 frame).
 * Consumer: I2S DMA ISR (reads 1 block per 1ms).
 *
 * Single-producer / single-consumer with volatile indices — safe on
 * Cortex-M33 without locks.
 */
struct audio_ring {
	uint32_t (*fifo)[AUDIO_BLK_SAMPLES_MONO];
	volatile uint16_t *prod_idx;
	volatile uint16_t *cons_idx;
	uint16_t num_blks;
};

/*
 * audio_rx_cb_t — called from the DMA ISR every 5 ms with a captured mono
 * PCM frame (AUDIO_SAMPLES_PER_FRAME × int16).  The callback must be
 * ISR-safe: write into a lock-free ring slot only, never block.
 */
typedef void (*audio_rx_cb_t)(const int16_t *mono_frame);

/*
 * audio_init — initialise the CS47L63 codec.
 *
 *   ring   : circular ring buffer that the DMA ISR drains for speaker output.
 *            Each slot is one 1ms stereo I2S block (AUDIO_BLK_SAMPLES_MONO
 *            uint32_t words).  The decoder thread is the sole producer.
 *
 *   rx_cb  : called from the DMA ISR with each captured mono mic frame.
 *            Must write into a lock-free ring — no blocking allowed.
 */
int audio_init(struct audio_ring *ring, audio_rx_cb_t rx_cb);

/* Start the I2S DMA engine and lock the CS47L63 FLL.
 * After this returns, the DMA ISR fires every 1 ms. */
int audio_start(void);

/* Adjust output volume by step_db dB (+3 = up, -3 = down).
 * Clamped to the CS47L63 register range. */
int audio_volume_adjust(int8_t step_db);

/* Returns the number of DAC underruns (freeze-frame injections) since the last
 * call and resets the counter to zero.  Call once per log window from the
 * decoder thread. */
uint32_t audio_underrun_count_reset(void);

#endif /* GRPTLK_AUDIO_H_ */
