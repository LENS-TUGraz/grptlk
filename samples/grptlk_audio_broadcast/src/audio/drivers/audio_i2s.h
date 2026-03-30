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
#define AUDIO_I2S_WORDS_PER_BLOCK     AUDIO_I2S_SAMPLES_PER_BLOCK
#define AUDIO_I2S_BLOCK_BYTES \
	(AUDIO_I2S_SAMPLES_PER_BLOCK * AUDIO_I2S_CHANNELS * (AUDIO_I2S_BITS_PER_SAMPLE / 8U))

/* LC3 frame is always 5 ms regardless of DMA block size */
#define AUDIO_I2S_LC3_FRAME_MS      5U
#define AUDIO_I2S_SAMPLES_PER_FRAME \
	((AUDIO_I2S_SAMPLE_RATE_HZ * AUDIO_I2S_LC3_FRAME_MS) / 1000U)  /* 80 */
#define AUDIO_I2S_BLKS_PER_FRAME \
	(AUDIO_I2S_SAMPLES_PER_FRAME / AUDIO_I2S_SAMPLES_PER_BLOCK)    /* 5 */

typedef void (*audio_i2s_blk_cb_t)(uint32_t *rx_buf_released,
				   const uint32_t *tx_buf_released);

int audio_i2s_init(void);
int audio_i2s_start(uint32_t *rx_buf_initial, uint32_t *tx_buf_initial);
int audio_i2s_set_next_buf(uint32_t *rx_buf, uint32_t *tx_buf);
void audio_i2s_blk_cb_register(audio_i2s_blk_cb_t cb);
void audio_i2s_stop(void);

#endif /* GRPTLK_AUDIO_I2S_H_ */
