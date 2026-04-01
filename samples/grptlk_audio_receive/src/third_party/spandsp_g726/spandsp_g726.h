/*
 * Derived from SpanDSP's G.726 implementation:
 *   - src/g726.c
 *   - src/spandsp/private/g726.h
 *
 * Trimmed for sample-local G.726-32 linear PCM use in Zephyr.
 *
 * SPDX-License-Identifier: LGPL-2.1-only
 */

#ifndef GRPTLK_SPANDSP_G726_32_H_
#define GRPTLK_SPANDSP_G726_32_H_

#include <stdint.h>

struct spandsp_g726_32_state {
	int32_t yl;
	int16_t yu;
	int16_t dms;
	int16_t dml;
	int16_t ap;

	int16_t a[2];
	int16_t b[6];
	int16_t pk[2];
	int16_t dq[6];
	int16_t sr[2];
	int td;
};

void spandsp_g726_32_reset(struct spandsp_g726_32_state *state);
uint8_t spandsp_g726_32_encode_sample(struct spandsp_g726_32_state *state,
				      int16_t sample_14bit);
int16_t spandsp_g726_32_decode_code(struct spandsp_g726_32_state *state,
				    uint8_t code);

#endif /* GRPTLK_SPANDSP_G726_32_H_ */
