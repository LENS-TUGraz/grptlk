/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GRPTLK_AUDIO_I2S_H_
#define GRPTLK_AUDIO_I2S_H_

#include <stdint.h>

#define AUDIO_I2S_SAMPLE_RATE_HZ      16000U
#define AUDIO_I2S_FRAME_DURATION_MS   1U
#define AUDIO_I2S_CHANNELS            2U
#define AUDIO_I2S_BITS_PER_SAMPLE     16U
#define AUDIO_I2S_SAMPLES_PER_BLOCK \
	((AUDIO_I2S_SAMPLE_RATE_HZ * AUDIO_I2S_FRAME_DURATION_MS) / 1000U)
/* Compile-time default: 5 ms frame @ 16 kHz = 80 words. */
#define AUDIO_I2S_WORDS_PER_BLOCK_DEFAULT  AUDIO_I2S_SAMPLES_PER_BLOCK
/* Kept for back-compat; resolves to the compile-time default. */
#define AUDIO_I2S_WORDS_PER_BLOCK          AUDIO_I2S_WORDS_PER_BLOCK_DEFAULT
#define AUDIO_I2S_BLOCK_BYTES \
	(AUDIO_I2S_SAMPLES_PER_BLOCK * AUDIO_I2S_CHANNELS * (AUDIO_I2S_BITS_PER_SAMPLE / 8U))

/* Callback receives the RX and TX buffers released by DMA completion.
 * Called from I2S interrupt context every 1ms. */
typedef void (*audio_i2s_blk_cb_t)(uint32_t *rx_buf_released,
				   const uint32_t *tx_buf_released);

int audio_i2s_init(void);
int audio_i2s_start(uint32_t *rx_buf_initial, uint32_t *tx_buf_initial);
int audio_i2s_set_next_buf(uint32_t *rx_buf, uint32_t *tx_buf);
void audio_i2s_blk_cb_register(audio_i2s_blk_cb_t cb);
void audio_i2s_stop(void);

/*
 * audio_i2s_set_block_size — set the DMA transfer size in 32-bit words.
 *
 * Must be called before audio_i2s_start().  The value is:
 *   words = (sample_rate_hz * frame_duration_ms) / 1000
 * e.g. 5 ms @ 16 kHz → 80 words; 10 ms @ 16 kHz → 160 words.
 * Static DMA buffers must be large enough for the configured size.
 */
void audio_i2s_set_block_size(uint32_t words_per_block);

#endif /* GRPTLK_AUDIO_I2S_H_ */
