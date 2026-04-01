/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "codec/g726_zephyr.h"

#include <errno.h>
#include <stddef.h>

static int g726_zephyr_encode_frame(struct g726_zephyr_encoder_state *state,
				    const int16_t *pcm,
				    size_t sample_count,
				    uint8_t *adpcm,
				    size_t byte_count)
{
	size_t sample_index;
	size_t byte_index;

	if ((state == NULL) || (pcm == NULL) || (adpcm == NULL)) {
		return -EINVAL;
	}

	if ((sample_count / 2U) != byte_count) {
		return -EINVAL;
	}

	for (sample_index = 0U, byte_index = 0U;
	     sample_index < sample_count;
	     sample_index += 2U, byte_index++) {
		uint8_t low_nibble;
		uint8_t high_nibble;

		/*
		 * Use SpanDSP's linear PCM scaling before encoding. The first
		 * sample in each pair is stored in the low nibble to match the
		 * right-packed byte stream order that SpanDSP uses for G.726.
		 */
		low_nibble = spandsp_g726_32_encode_sample(&state->core,
							   (int16_t) (pcm[sample_index] >> 2));
		high_nibble = spandsp_g726_32_encode_sample(
			&state->core, (int16_t) (pcm[sample_index + 1U] >> 2));

		adpcm[byte_index] = (uint8_t) ((high_nibble << 4) | (low_nibble & 0x0F));
	}

	return 0;
}

static int g726_zephyr_decode_frame(struct g726_zephyr_decoder_state *state,
				    const uint8_t *adpcm,
				    size_t byte_count,
				    int16_t *pcm,
				    size_t sample_count)
{
	size_t sample_index;
	size_t byte_index;

	if ((state == NULL) || (adpcm == NULL) || (pcm == NULL)) {
		return -EINVAL;
	}

	if ((sample_count / 2U) != byte_count) {
		return -EINVAL;
	}

	for (sample_index = 0U, byte_index = 0U;
	     byte_index < byte_count;
	     sample_index += 2U, byte_index++) {
		uint8_t packed_code = adpcm[byte_index];

		pcm[sample_index] = spandsp_g726_32_decode_code(&state->core,
								packed_code & 0x0F);
		pcm[sample_index + 1U] = spandsp_g726_32_decode_code(
			&state->core, (packed_code >> 4) & 0x0F);
	}

	return 0;
}

void g726_zephyr_encoder_init(struct g726_zephyr_encoder_state *state)
{
	g726_zephyr_encoder_reset(state);
}

void g726_zephyr_encoder_reset(struct g726_zephyr_encoder_state *state)
{
	if (state != NULL) {
		spandsp_g726_32_reset(&state->core);
	}
}

void g726_zephyr_decoder_init(struct g726_zephyr_decoder_state *state)
{
	g726_zephyr_decoder_reset(state);
}

void g726_zephyr_decoder_reset(struct g726_zephyr_decoder_state *state)
{
	if (state != NULL) {
		spandsp_g726_32_reset(&state->core);
	}
}

int g726_zephyr_encode_5ms(struct g726_zephyr_encoder_state *state,
			   const int16_t pcm[G726_ZEPHYR_5MS_SAMPLES],
			   uint8_t adpcm[G726_ZEPHYR_5MS_BYTES])
{
	return g726_zephyr_encode_frame(state, pcm, G726_ZEPHYR_5MS_SAMPLES, adpcm,
					 G726_ZEPHYR_5MS_BYTES);
}

int g726_zephyr_decode_5ms(struct g726_zephyr_decoder_state *state,
			   const uint8_t adpcm[G726_ZEPHYR_5MS_BYTES],
			   int16_t pcm[G726_ZEPHYR_5MS_SAMPLES])
{
	return g726_zephyr_decode_frame(state, adpcm, G726_ZEPHYR_5MS_BYTES, pcm,
					 G726_ZEPHYR_5MS_SAMPLES);
}

int g726_zephyr_encode_10ms(struct g726_zephyr_encoder_state *state,
			    const int16_t pcm[G726_ZEPHYR_10MS_SAMPLES],
			    uint8_t adpcm[G726_ZEPHYR_10MS_BYTES])
{
	return g726_zephyr_encode_frame(state, pcm, G726_ZEPHYR_10MS_SAMPLES, adpcm,
					 G726_ZEPHYR_10MS_BYTES);
}

int g726_zephyr_decode_10ms(struct g726_zephyr_decoder_state *state,
			    const uint8_t adpcm[G726_ZEPHYR_10MS_BYTES],
			    int16_t pcm[G726_ZEPHYR_10MS_SAMPLES])
{
	return g726_zephyr_decode_frame(state, adpcm, G726_ZEPHYR_10MS_BYTES, pcm,
					 G726_ZEPHYR_10MS_SAMPLES);
}
