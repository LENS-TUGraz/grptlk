/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GRPTLK_G726_ZEPHYR_H_
#define GRPTLK_G726_ZEPHYR_H_

#include <stdint.h>

#include "third_party/spandsp_g726/spandsp_g726.h"

#ifdef __cplusplus
extern "C" {
#endif

#define G726_ZEPHYR_SAMPLE_RATE_HZ 8000U

#define G726_ZEPHYR_5MS_SAMPLES 40U
#define G726_ZEPHYR_5MS_BYTES   20U

#define G726_ZEPHYR_10MS_SAMPLES 80U
#define G726_ZEPHYR_10MS_BYTES   40U

struct g726_zephyr_encoder_state {
	struct spandsp_g726_32_state core;
};

struct g726_zephyr_decoder_state {
	struct spandsp_g726_32_state core;
};

void g726_zephyr_encoder_init(struct g726_zephyr_encoder_state *state);
void g726_zephyr_encoder_reset(struct g726_zephyr_encoder_state *state);

void g726_zephyr_decoder_init(struct g726_zephyr_decoder_state *state);
void g726_zephyr_decoder_reset(struct g726_zephyr_decoder_state *state);

int g726_zephyr_encode_5ms(struct g726_zephyr_encoder_state *state,
			   const int16_t pcm[G726_ZEPHYR_5MS_SAMPLES],
			   uint8_t adpcm[G726_ZEPHYR_5MS_BYTES]);

int g726_zephyr_decode_5ms(struct g726_zephyr_decoder_state *state,
			   const uint8_t adpcm[G726_ZEPHYR_5MS_BYTES],
			   int16_t pcm[G726_ZEPHYR_5MS_SAMPLES]);

int g726_zephyr_encode_10ms(struct g726_zephyr_encoder_state *state,
			    const int16_t pcm[G726_ZEPHYR_10MS_SAMPLES],
			    uint8_t adpcm[G726_ZEPHYR_10MS_BYTES]);

int g726_zephyr_decode_10ms(struct g726_zephyr_decoder_state *state,
			    const uint8_t adpcm[G726_ZEPHYR_10MS_BYTES],
			    int16_t pcm[G726_ZEPHYR_10MS_SAMPLES]);

#ifdef __cplusplus
}
#endif

#endif /* GRPTLK_G726_ZEPHYR_H_ */
