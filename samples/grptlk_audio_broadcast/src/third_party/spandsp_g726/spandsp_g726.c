/*
 * Derived from SpanDSP's G.726 implementation:
 *   - src/g726.c
 *   - src/spandsp/bit_operations.h
 *   - src/spandsp/private/g726.h
 *
 * Reduced to the linear PCM G.726-32 core needed by grptlk_audio_broadcast.
 * The original arch-specific top-bit helper is replaced with a compiler
 * builtin fallback so the code stays toolchain-neutral inside Zephyr.
 *
 * SPDX-License-Identifier: LGPL-2.1-only
 */

#include "third_party/spandsp_g726/spandsp_g726.h"

#include <stddef.h>

static const int g726_32_dqlntab[16] = {
	-2048, 4, 135, 213, 273, 323, 373, 425,
	425, 373, 323, 273, 213, 135, 4, -2048,
};

static const int g726_32_witab[16] = {
	-384, 576, 1312, 2048, 3584, 6336, 11360, 35904,
	35904, 11360, 6336, 3584, 2048, 1312, 576, -384,
};

static const int g726_32_fitab[16] = {
	0x000, 0x000, 0x000, 0x200, 0x200, 0x200, 0x600, 0xE00,
	0xE00, 0x600, 0x200, 0x200, 0x200, 0x000, 0x000, 0x000,
};

static const int qtab_726_32[7] = {
	-124, 80, 178, 246, 300, 349, 400,
};

static inline int spandsp_top_bit(unsigned int bits)
{
#if defined(__GNUC__) || defined(__clang__)
	return (bits == 0U) ? -1 : (31 - __builtin_clz(bits));
#else
	int result = -1;

	while (bits != 0U) {
		bits >>= 1U;
		result++;
	}

	return result;
#endif
}

static inline int spandsp_abs_int(int value)
{
	return (value < 0) ? -value : value;
}

static int16_t spandsp_fmult(int16_t an, int16_t srn)
{
	int16_t anmag;
	int16_t anexp;
	int16_t anmant;
	int16_t wanexp;
	int16_t wanmant;
	int16_t retval;

	anmag = (an > 0) ? an : ((-an) & 0x1FFF);
	anexp = (int16_t) (spandsp_top_bit((unsigned int) anmag) - 5);
	anmant = (anmag == 0) ? 32 :
			      (anexp >= 0) ? (anmag >> anexp) : (anmag << -anexp);
	wanexp = anexp + ((srn >> 6) & 0x0F) - 13;

	wanmant = (int16_t) ((anmant * (srn & 0x3F) + 0x30) >> 4);
	retval = (wanexp >= 0) ? ((wanmant << wanexp) & 0x7FFF) : (wanmant >> -wanexp);

	return ((an ^ srn) < 0) ? -retval : retval;
}

static inline int16_t spandsp_predictor_zero(struct spandsp_g726_32_state *state)
{
	int index;
	int sezi;

	sezi = spandsp_fmult(state->b[0] >> 2, state->dq[0]);
	for (index = 1; index < 6; index++) {
		sezi += spandsp_fmult(state->b[index] >> 2, state->dq[index]);
	}

	return (int16_t) sezi;
}

static inline int16_t spandsp_predictor_pole(struct spandsp_g726_32_state *state)
{
	return spandsp_fmult(state->a[1] >> 2, state->sr[1]) +
	       spandsp_fmult(state->a[0] >> 2, state->sr[0]);
}

static int spandsp_step_size(struct spandsp_g726_32_state *state)
{
	int y;
	int dif;
	int al;

	if (state->ap >= 256) {
		return state->yu;
	}

	y = state->yl >> 6;
	dif = state->yu - y;
	al = state->ap >> 2;

	if (dif > 0) {
		y += (dif * al) >> 6;
	} else if (dif < 0) {
		y += (dif * al + 0x3F) >> 6;
	}

	return y;
}

static int16_t spandsp_quantize(int d, int y, const int table[], int quantizer_states)
{
	int16_t dqm;
	int16_t exp;
	int16_t mant;
	int16_t dl;
	int16_t dln;
	int size;
	int index;

	dqm = (d < 0) ? (int16_t) -d : (int16_t) d;
	exp = (int16_t) (spandsp_top_bit((unsigned int) (dqm >> 1)) + 1);
	mant = (int16_t) (((dqm << 7) >> exp) & 0x7F);
	dl = (int16_t) ((exp << 7) + mant);

	dln = dl - (int16_t) (y >> 2);

	size = (quantizer_states - 1) >> 1;
	for (index = 0; index < size; index++) {
		if (dln < table[index]) {
			break;
		}
	}

	if (d < 0) {
		return (int16_t) (((size << 1) + 1) - index);
	}

	if ((index == 0) && ((quantizer_states & 1) != 0)) {
		return (int16_t) quantizer_states;
	}

	return (int16_t) index;
}

static int16_t spandsp_reconstruct(int sign, int dqln, int y)
{
	int16_t dql;
	int16_t dex;
	int16_t dqt;
	int16_t dq;

	dql = (int16_t) (dqln + (y >> 2));

	if (dql < 0) {
		return sign ? -0x8000 : 0;
	}

	dex = (dql >> 7) & 0x0F;
	dqt = (int16_t) (128 + (dql & 127));
	dq = (int16_t) ((dqt << 7) >> (14 - dex));

	return sign ? (dq - 0x8000) : dq;
}

static void spandsp_update(struct spandsp_g726_32_state *state,
			   int y,
			   int wi,
			   int fi,
			   int dq,
			   int sr,
			   int dqsez)
{
	int16_t mag;
	int16_t exp;
	int16_t a2p;
	int16_t a1ul;
	int16_t pks1;
	int16_t fa1;
	int16_t ylint;
	int16_t dqthr;
	int16_t ylfrac;
	int16_t thr;
	int16_t pk0;
	int index;
	int tr;

	a2p = 0;
	pk0 = (dqsez < 0) ? 1 : 0;

	mag = (int16_t) (dq & 0x7FFF);
	ylint = (int16_t) (state->yl >> 15);
	ylfrac = (int16_t) ((state->yl >> 10) & 0x1F);
	thr = (ylint > 9) ? (31 << 10) : ((32 + ylfrac) << ylint);
	dqthr = (int16_t) ((thr + (thr >> 1)) >> 1);

	if (!state->td) {
		tr = 0;
	} else if (mag <= dqthr) {
		tr = 0;
	} else {
		tr = 1;
	}

	state->yu = (int16_t) (y + ((wi - y) >> 5));
	if (state->yu < 544) {
		state->yu = 544;
	} else if (state->yu > 5120) {
		state->yu = 5120;
	}

	state->yl += state->yu + ((-state->yl) >> 6);

	if (tr) {
		state->a[0] = 0;
		state->a[1] = 0;
		state->b[0] = 0;
		state->b[1] = 0;
		state->b[2] = 0;
		state->b[3] = 0;
		state->b[4] = 0;
		state->b[5] = 0;
	} else {
		pks1 = pk0 ^ state->pk[0];

		a2p = state->a[1] - (state->a[1] >> 7);
		if (dqsez != 0) {
			fa1 = pks1 ? state->a[0] : -state->a[0];

			if (fa1 < -8191) {
				a2p -= 0x100;
			} else if (fa1 > 8191) {
				a2p += 0xFF;
			} else {
				a2p += fa1 >> 5;
			}

			if ((pk0 ^ state->pk[1]) != 0) {
				if (a2p <= -12160) {
					a2p = -12288;
				} else if (a2p >= 12416) {
					a2p = 12288;
				} else {
					a2p -= 0x80;
				}
			} else if (a2p <= -12416) {
				a2p = -12288;
			} else if (a2p >= 12160) {
				a2p = 12288;
			} else {
				a2p += 0x80;
			}
		}

		state->a[1] = a2p;

		state->a[0] -= state->a[0] >> 8;
		if (dqsez != 0) {
			if (pks1 == 0) {
				state->a[0] += 192;
			} else {
				state->a[0] -= 192;
			}
		}

		a1ul = (int16_t) (15360 - a2p);
		if (state->a[0] < -a1ul) {
			state->a[0] = -a1ul;
		} else if (state->a[0] > a1ul) {
			state->a[0] = a1ul;
		}

		for (index = 0; index < 6; index++) {
			state->b[index] -= state->b[index] >> 8;
			if ((dq & 0x7FFF) != 0) {
				if ((dq ^ state->dq[index]) >= 0) {
					state->b[index] += 128;
				} else {
					state->b[index] -= 128;
				}
			}
		}
	}

	for (index = 5; index > 0; index--) {
		state->dq[index] = state->dq[index - 1];
	}

	if (mag == 0) {
		state->dq[0] = (dq >= 0) ? 0x20 : (int16_t) 0xFC20;
	} else {
		exp = (int16_t) (spandsp_top_bit((unsigned int) mag) + 1);
		state->dq[0] = (dq >= 0) ? ((exp << 6) + ((mag << 6) >> exp)) :
					   ((exp << 6) + ((mag << 6) >> exp) - 0x400);
	}

	state->sr[1] = state->sr[0];
	if (sr == 0) {
		state->sr[0] = 0x20;
	} else if (sr > 0) {
		exp = (int16_t) (spandsp_top_bit((unsigned int) sr) + 1);
		state->sr[0] = (int16_t) ((exp << 6) + ((sr << 6) >> exp));
	} else if (sr > -32768) {
		mag = (int16_t) -sr;
		exp = (int16_t) (spandsp_top_bit((unsigned int) mag) + 1);
		state->sr[0] = (int16_t) ((exp << 6) + ((mag << 6) >> exp) - 0x400);
	} else {
		state->sr[0] = (int16_t) 0xFC20;
	}

	state->pk[1] = state->pk[0];
	state->pk[0] = pk0;

	if (tr) {
		state->td = 0;
	} else if (a2p < -11776) {
		state->td = 1;
	} else {
		state->td = 0;
	}

	state->dms += ((int16_t) fi - state->dms) >> 5;
	state->dml += (((int16_t) (fi << 2) - state->dml) >> 7);

	if (tr) {
		state->ap = 256;
	} else if (y < 1536) {
		state->ap += (0x200 - state->ap) >> 4;
	} else if (state->td) {
		state->ap += (0x200 - state->ap) >> 4;
	} else if (spandsp_abs_int((state->dms << 2) - state->dml) >= (state->dml >> 3)) {
		state->ap += (0x200 - state->ap) >> 4;
	} else {
		state->ap += (-state->ap) >> 4;
	}
}

void spandsp_g726_32_reset(struct spandsp_g726_32_state *state)
{
	int index;

	if (state == NULL) {
		return;
	}

	state->yl = 34816;
	state->yu = 544;
	state->dms = 0;
	state->dml = 0;
	state->ap = 0;

	for (index = 0; index < 2; index++) {
		state->a[index] = 0;
		state->pk[index] = 0;
		state->sr[index] = 32;
	}

	for (index = 0; index < 6; index++) {
		state->b[index] = 0;
		state->dq[index] = 32;
	}

	state->td = 0;
}

uint8_t spandsp_g726_32_encode_sample(struct spandsp_g726_32_state *state,
				      int16_t sample_14bit)
{
	int16_t sezi;
	int16_t sei;
	int16_t se;
	int16_t d;
	int16_t sr;
	int16_t dqsez;
	int16_t dq;
	int16_t code;
	int y;

	sezi = spandsp_predictor_zero(state);
	sei = (int16_t) (sezi + spandsp_predictor_pole(state));
	se = (int16_t) (sei >> 1);
	d = (int16_t) (sample_14bit - se);

	y = spandsp_step_size(state);
	code = spandsp_quantize(d, y, qtab_726_32, 15);
	dq = spandsp_reconstruct(code & 8, g726_32_dqlntab[code], y);

	sr = (dq < 0) ? (int16_t) (se - (dq & 0x3FFF)) : (int16_t) (se + dq);
	dqsez = (int16_t) (sr + (sezi >> 1) - se);

	spandsp_update(state, y, g726_32_witab[code], g726_32_fitab[code], dq, sr, dqsez);

	return (uint8_t) code;
}

int16_t spandsp_g726_32_decode_code(struct spandsp_g726_32_state *state, uint8_t code)
{
	int16_t sezi;
	int16_t sei;
	int16_t se;
	int16_t sr;
	int16_t dq;
	int16_t dqsez;
	int y;

	code &= 0x0F;

	sezi = spandsp_predictor_zero(state);
	sei = (int16_t) (sezi + spandsp_predictor_pole(state));

	y = spandsp_step_size(state);
	dq = spandsp_reconstruct(code & 8, g726_32_dqlntab[code], y);

	se = (int16_t) (sei >> 1);
	sr = (dq < 0) ? (int16_t) (se - (dq & 0x3FFF)) : (int16_t) (se + dq);
	dqsez = (int16_t) (sr + (sezi >> 1) - se);

	spandsp_update(state, y, g726_32_witab[code], g726_32_fitab[code], dq, sr, dqsez);

	return (int16_t) (sr << 2);
}
