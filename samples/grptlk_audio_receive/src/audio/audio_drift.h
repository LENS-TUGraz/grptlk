#ifndef GRPTLK_AUDIO_DRIFT_H_
#define GRPTLK_AUDIO_DRIFT_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/ring_buffer.h>

/**
 * @brief Drift compensation context for a PCM buffer.
 *
 * This context manages an elastic ring buffer that performs
 * occasional frame dropping or duplication to compensate for
 * asynchronous clock domains.
 */
struct audio_drift_ctx {
	struct ring_buf *ringbuf;
	uint32_t frame_size; /* 2 for mono, 4 for stereo */

	uint32_t prefill_frames;
	uint32_t low_watermark_frames;
	uint32_t high_watermark_frames;

	bool is_started; /* False until prefill is reached */

	/* Statistics */
	uint32_t dropped_frames;
	uint32_t inserted_frames;
	uint32_t underruns;
	uint32_t overruns;

	uint8_t last_frame[512];
};

/**
 * @brief Initialize an audio drift context.
 *
 * @param ctx The context to initialize.
 * @param rb The backing ring buffer (must be initialized by the caller).
 * @param frame_size Size of a single PCM frame in bytes (e.g. 2 for mono 16-bit, 4 for stereo
 * 16-bit).
 * @param prefill_frames Number of frames to buffer before starting reads.
 * @param low_water_frames Low watermark; below this, duplicate frames are inserted.
 * @param high_water_frames High watermark; above this, frames are dropped.
 */
void audio_drift_init(struct audio_drift_ctx *ctx, struct ring_buf *rb, uint32_t frame_size,
		      uint32_t prefill_frames, uint32_t low_water_frames,
		      uint32_t high_water_frames);

/**
 * @brief Write PCM frames into the drift buffer.
 *
 * @param ctx The drift context.
 * @param data PCM data to write.
 * @param frames Number of frames to write.
 * @return Number of frames successfully written.
 */
uint32_t audio_drift_write(struct audio_drift_ctx *ctx, const void *data, uint32_t frames);

/**
 * @brief Read PCM frames from the drift buffer.
 *
 * If the buffer length falls below the low watermark, a frame might
 * be duplicated. If it goes above the high watermark, a frame might be dropped.
 * If there is not enough data, zeroes or the last frame will be used to pad.
 *
 * @param ctx The drift context.
 * @param data Buffer to read into.
 * @param frames Number of frames requested.
 * @return Number of valid frames actually pulled from the buffer. The rest of the
 *         requested frames in 'data' are synthetically generated (padded).
 */
uint32_t audio_drift_read(struct audio_drift_ctx *ctx, void *data, uint32_t frames);

/**
 * @brief Reset the drift buffer and state.
 */
void audio_drift_reset(struct audio_drift_ctx *ctx);

/**
 * @brief Get the current fill level in frames.
 */
uint32_t audio_drift_fill_level(struct audio_drift_ctx *ctx);

#endif /* GRPTLK_AUDIO_DRIFT_H_ */
