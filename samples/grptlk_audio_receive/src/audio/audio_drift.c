#include "audio_drift.h"
#include <string.h>

void audio_drift_init(struct audio_drift_ctx *ctx, struct ring_buf *rb, uint32_t frame_size,
		      uint32_t prefill_frames, uint32_t low_water_frames,
		      uint32_t high_water_frames)
{
	ctx->ringbuf = rb;
	ctx->frame_size = frame_size;
	ctx->prefill_frames = prefill_frames;
	ctx->low_watermark_frames = low_water_frames;
	ctx->high_watermark_frames = high_water_frames;
	ctx->is_started = false;
	ctx->dropped_frames = 0;
	ctx->inserted_frames = 0;
	ctx->underruns = 0;
	ctx->overruns = 0;
	memset(ctx->last_frame, 0, sizeof(ctx->last_frame));
}

void audio_drift_reset(struct audio_drift_ctx *ctx)
{
	ring_buf_reset(ctx->ringbuf);
	ctx->is_started = false;
	memset(ctx->last_frame, 0, sizeof(ctx->last_frame));
}

uint32_t audio_drift_fill_level(struct audio_drift_ctx *ctx)
{
	if (ctx == NULL || ctx->ringbuf == NULL) {
		return 0;
	}
	return ring_buf_size_get(ctx->ringbuf) / ctx->frame_size;
}

uint32_t audio_drift_write(struct audio_drift_ctx *ctx, const void *data, uint32_t frames)
{
	uint32_t bytes = frames * ctx->frame_size;
	uint32_t written = ring_buf_put(ctx->ringbuf, (const uint8_t *)data, bytes);

	if (written < bytes) {
		ctx->overruns++;
	}
	return written / ctx->frame_size;
}

uint32_t audio_drift_read(struct audio_drift_ctx *ctx, void *data, uint32_t frames)
{
	uint32_t out_frames = 0;
	uint8_t *dst = (uint8_t *)data;
	bool modified_this_tx = false;

	uint32_t available = audio_drift_fill_level(ctx);

	if (!ctx->is_started) {
		if (available >= ctx->prefill_frames) {
			ctx->is_started = true;
		} else {
			memset(data, 0, frames * ctx->frame_size);
			return 0;
		}
	}

	for (uint32_t i = 0; i < frames; i++) {
		available = audio_drift_fill_level(ctx);

		if (available == 0) {
			ctx->is_started = false;
			ctx->underruns++;
			memset(dst, 0, ctx->frame_size);
			memset(ctx->last_frame, 0, ctx->frame_size);
			dst += ctx->frame_size;
			continue;
		}

		/* Below low watermark: duplicate last frame to stretch the buffer. Max 1 per call.
		 */
		if (!modified_this_tx && available < ctx->low_watermark_frames) {
			memcpy(dst, ctx->last_frame, ctx->frame_size);
			ctx->inserted_frames++;
			modified_this_tx = true;
			dst += ctx->frame_size;
			continue;
		}

		/* Above high watermark: drop one frame to drain the buffer. Max 1 per call. */
		if (!modified_this_tx && available > ctx->high_watermark_frames) {
			uint8_t temp[512];
			ring_buf_get(ctx->ringbuf, temp, ctx->frame_size);
			ctx->dropped_frames++;
			modified_this_tx = true;
			available--;
		}

		if (available > 0) {
			ring_buf_get(ctx->ringbuf, dst, ctx->frame_size);
			memcpy(ctx->last_frame, dst, ctx->frame_size);
			out_frames++;
		} else {
			/* drop_frame consumed the last available frame */
			memset(dst, 0, ctx->frame_size);
			memset(ctx->last_frame, 0, ctx->frame_size);
			ctx->underruns++;
			ctx->is_started = false;
		}

		dst += ctx->frame_size;
	}

	return out_frames;
}
