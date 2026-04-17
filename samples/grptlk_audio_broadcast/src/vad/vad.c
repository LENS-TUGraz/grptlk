#include "vad/vad.h"

#include <string.h>
#include <zephyr/kernel.h>

#include "audio/audio.h"
#include "audio/sync/clk_sync.h"
#include "io/buttons.h"

/* libfvad internals — we need struct Fvad for static allocation. */
#include "fvad.h"
#include "vad/vad_core.h"

/* Fvad struct layout (from fvad.c) — duplicated here so we can
 * statically allocate without pulling in malloc. */
struct Fvad {
	VadInstT core;
	size_t rate_idx;
};

enum vox_state {
	VOX_IDLE,
	VOX_ACTIVE,
	VOX_HANGOVER,
};

static struct Fvad fvad_inst;
static struct k_sem *vad_tx_sem;
static enum vox_state vox;
static int cfg_onset_frames;
static int cfg_hangover_frames;
static int onset_count;
static int hangover_count;

/* 5 ms accumulation buffer (always 160 samples = 10 ms at 16 kHz). */
static int16_t vad_accum_buf[AUDIO_SAMPLE_RATE_HZ / 100];
static uint8_t vad_accum_half;

static void vox_activate(void)
{
	vox = VOX_ACTIVE;
	onset_count = 0;
	atomic_set(&ptt_active, 1);
	clk_sync_reset();
	if (vad_tx_sem != NULL) {
		k_sem_give(vad_tx_sem);
	}
	printk("[VAD] TX start\n");
}

static void vox_deactivate(void)
{
	vox = VOX_IDLE;
	onset_count = 0;
	hangover_count = 0;
	atomic_set(&ptt_active, 0);
#if defined(CONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT)
	extern void ptt_session_bis_reset(void);
	ptt_session_bis_reset();
#endif
	printk("[VAD] TX stop\n");
}

static bool frame_above_energy_gate(const int16_t *pcm, size_t num_samples)
{
#if CONFIG_GRPTLK_VAD_ENERGY_THRESHOLD > 0
	const int16_t threshold = CONFIG_GRPTLK_VAD_ENERGY_THRESHOLD;
	int16_t peak = 0;

	for (size_t i = 0; i < num_samples; i++) {
		int16_t abs_val = (pcm[i] < 0) ? -pcm[i] : pcm[i];

		if (abs_val > peak) {
			peak = abs_val;
		}
	}

	return peak >= threshold;
#else
	return true;
#endif
}

static void vox_process(const int16_t *pcm, size_t num_samples)
{
	int speech;

	if (!frame_above_energy_gate(pcm, num_samples)) {
		speech = 0;
	} else {
		speech = fvad_process(&fvad_inst, pcm, num_samples);
		if (speech < 0) {
			speech = 0;
		}
	}

	switch (vox) {
	case VOX_IDLE:
		if (speech) {
			onset_count++;
			if (onset_count >= cfg_onset_frames) {
				vox_activate();
			}
		} else {
			onset_count = 0;
		}
		break;

	case VOX_ACTIVE:
		if (!speech) {
			vox = VOX_HANGOVER;
			hangover_count = 1;
		}
		break;

	case VOX_HANGOVER:
		if (speech) {
			vox = VOX_ACTIVE;
			hangover_count = 0;
		} else {
			hangover_count++;
			if (hangover_count >= cfg_hangover_frames) {
				vox_deactivate();
			}
		}
		break;
	}
}

int vad_init(int aggressiveness, int onset_frames, int hangover_frames,
	     struct k_sem *tx_sem)
{
	/* Initialise libfvad on the static instance (no malloc). */
	int rv = WebRtcVad_InitCore(&fvad_inst.core);

	if (rv != 0) {
		return -EIO;
	}

	fvad_inst.rate_idx = 0; /* default 8 kHz, overridden below */

	if (fvad_set_mode(&fvad_inst, aggressiveness) != 0) {
		return -EINVAL;
	}

	if (fvad_set_sample_rate(&fvad_inst, AUDIO_SAMPLE_RATE_HZ) != 0) {
		return -EINVAL;
	}

	cfg_onset_frames = onset_frames;
	cfg_hangover_frames = hangover_frames;
	vad_tx_sem = tx_sem;
	vox = VOX_IDLE;
	onset_count = 0;
	hangover_count = 0;
	vad_accum_half = 0;

	printk("[VAD] init: aggressiveness=%d onset=%d hangover=%d energy_gate=%d\n",
	       aggressiveness, onset_frames, hangover_frames,
	       CONFIG_GRPTLK_VAD_ENERGY_THRESHOLD);
	return 0;
}

void vad_process_frame(const int16_t *pcm, uint16_t num_samples)
{
	const uint16_t vad_frame_samples = AUDIO_SAMPLE_RATE_HZ / 100; /* 160 */

	if (num_samples < vad_frame_samples) {
		/* 5 ms mode: accumulate two halves. */
		memcpy(&vad_accum_buf[vad_accum_half * num_samples], pcm,
		       num_samples * sizeof(int16_t));
		if (vad_accum_half == 0) {
			vad_accum_half = 1;
			return;
		}
		vad_accum_half = 0;
		vox_process(vad_accum_buf, vad_frame_samples);
	} else {
		vox_process(pcm, num_samples);
	}
}

void vad_reset(void)
{
	if (vox == VOX_ACTIVE || vox == VOX_HANGOVER) {
		vox_deactivate();
	}
	vox = VOX_IDLE;
	onset_count = 0;
	hangover_count = 0;
	vad_accum_half = 0;
}
