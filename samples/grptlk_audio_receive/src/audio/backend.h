#ifndef GRPTLK_AUDIO_BACKEND_H_
#define GRPTLK_AUDIO_BACKEND_H_

#include <stdbool.h>
#include <stdint.h>

enum audio_backend_capture_channel {
	AUDIO_BACKEND_CAPTURE_CHANNEL_AUTO = -1,
	AUDIO_BACKEND_CAPTURE_CHANNEL_LEFT = 0,
	AUDIO_BACKEND_CAPTURE_CHANNEL_RIGHT = 1,
};

struct audio_backend_config {
	bool capture_enabled;
	int capture_channel_hint;
};

/* Sample-local hardware backend contract. The generic audio engine owns the
 * I2S/ring/PCM flow; the backend owns chip-specific control and routing. */
int audio_backend_init(struct audio_backend_config *config);
int audio_backend_prepare_stream_start(void);
int audio_backend_volume_adjust(int8_t step_db);
int audio_backend_input_source_switch(bool use_line_in, int *capture_channel_hint);

#endif /* GRPTLK_AUDIO_BACKEND_H_ */
