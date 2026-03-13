#include "audio.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(audio_max9867, CONFIG_MAX9867_LOG_LEVEL);

#include "drivers/audio_i2s.h"
#include "drivers/max9867.h"
#include "io/qdec.h"

#define CHANNELS	 AUDIO_CHANNELS
#define FRAMES_PER_BLOCK AUDIO_SAMPLES_PER_FRAME
#define BLOCK_BYTES	 AUDIO_BLOCK_BYTES

#define RX_BLOCK_COUNT	    16
#define TX_BLOCK_COUNT	    8
#define MAX_VOL_Q15	    32767
#define VOLUME_3DB_STEP_Q15 911

#if defined(CONFIG_BOARD_RBV2H_NRF5340_CPUAPP)
#define AUDIO_USE_NRFX_I2S 1
#else
#define AUDIO_USE_NRFX_I2S 0
#endif

BUILD_ASSERT(BLOCK_BYTES == 320, "max9867 backend expects 5 ms stereo 16-bit blocks");
#if AUDIO_USE_NRFX_I2S
BUILD_ASSERT(AUDIO_SAMPLES_PER_FRAME == AUDIO_I2S_SAMPLES_PER_BLOCK,
	     "audio and audio_i2s frame size must match");
BUILD_ASSERT(AUDIO_BLOCK_BYTES == AUDIO_I2S_BLOCK_BYTES,
	     "audio and audio_i2s block size must match");
#endif

#if !AUDIO_USE_NRFX_I2S
static const struct device *i2s_dev;
K_MEM_SLAB_DEFINE_STATIC(rx_slab, BLOCK_BYTES, RX_BLOCK_COUNT, 4);
K_MEM_SLAB_DEFINE_STATIC(tx_slab, BLOCK_BYTES, TX_BLOCK_COUNT, 4);
#else
static uint32_t i2s_rx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_rx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];
#endif

static struct audio_drift_ctx *playback_drift;
static struct audio_drift_ctx *capture_drift;
static bool initialized;
static bool started;

#if !AUDIO_USE_NRFX_I2S
K_THREAD_STACK_DEFINE(rx_stack, 4096);
static struct k_thread rx_thread_data;
K_THREAD_STACK_DEFINE(tx_stack, 2048);
static struct k_thread tx_thread_data;
#endif

static int32_t current_vol_q15 = MAX_VOL_Q15;

static int volume_apply_delta_q15(int32_t delta_q15)
{
	current_vol_q15 += delta_q15;
	if (current_vol_q15 > MAX_VOL_Q15) {
		current_vol_q15 = MAX_VOL_Q15;
	} else if (current_vol_q15 < 0) {
		current_vol_q15 = 0;
	}

	return max9867_set_volume((int16_t)current_vol_q15, (int16_t)MAX_VOL_Q15);
}

static inline void downmix_stereo_block_to_mono(const int16_t *stereo, int16_t *mono)
{
	for (size_t i = 0; i < FRAMES_PER_BLOCK; i++) {
		/* MAX9867 on RBV2H outputs audio on the left channel only;
		 * the right channel is always zero.  Use left directly. */
		mono[i] = stereo[2 * i + 0];
	}
}

#if AUDIO_USE_NRFX_I2S
static void tx_buffer_fill(uint32_t *tx_words)
{
	if (playback_drift != NULL) {
		audio_drift_read(playback_drift, tx_words, 1);
	} else {
		memset(tx_words, 0, BLOCK_BYTES);
	}
}

static void i2s_process_rx_block(const uint32_t *rx_words)
{
	int16_t mono_frame[FRAMES_PER_BLOCK];

	downmix_stereo_block_to_mono((const int16_t *)rx_words, mono_frame);

	if (capture_drift != NULL) {
		audio_drift_write(capture_drift, mono_frame, 1);
	}
}

static void i2s_block_complete(uint32_t *rx_buf_released, const uint32_t *tx_buf_released)
{
	static uint8_t tx_buf_sel;
	static uint32_t rx_ok_count;
	uint32_t *next_tx_buf;
	int err;

	ARG_UNUSED(tx_buf_released);

	if (rx_buf_released != NULL) {
		const int16_t *stereo = (const int16_t *)rx_buf_released;
		int16_t mono0 = (int16_t)(((int32_t)stereo[0] + (int32_t)stereo[1]) / 2);

		rx_ok_count++;
		if (rx_ok_count <= 5U || (rx_ok_count % 200U) == 0U) {
			LOG_DBG("audio rx: block=%u first_samples=[%d,%d,%d,%d] mono0=%d",
				rx_ok_count, stereo[0], stereo[1], stereo[2], stereo[3], mono0);
		}

		i2s_process_rx_block(rx_buf_released);
	}

	next_tx_buf = (tx_buf_sel == 0U) ? i2s_tx_buf_a : i2s_tx_buf_b;
	tx_buf_sel ^= 1U;
	tx_buffer_fill(next_tx_buf);

	err = audio_i2s_set_next_buf(rx_buf_released, next_tx_buf);
	if (err != 0) {
		LOG_ERR("audio_i2s_set_next_buf failed: %d", err);
	}
}
#else
static void tx_thread_fn(void *p1, void *p2, void *p3)
{
	int err;
	int ret;
	static bool first_tx_logged;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_DBG("audio tx thread running");

	while (1) {
		void *txblk;
		err = k_mem_slab_alloc(&tx_slab, &txblk, K_FOREVER);
		if (err) {
			continue;
		}

		if (playback_drift != NULL) {
			audio_drift_read(playback_drift, txblk, 1);
		} else {
			memset(txblk, 0, BLOCK_BYTES);
		}

		while ((ret = i2s_write(i2s_dev, txblk, BLOCK_BYTES)) == -EAGAIN) {
			k_msleep(1);
		}

		if (!first_tx_logged && ret == 0) {
			first_tx_logged = true;
			LOG_DBG("audio tx: first block queued (%u bytes)", BLOCK_BYTES);
		}

		if (ret < 0) {
			LOG_ERR("i2s tx write failed: %d, recovering", ret);
			k_mem_slab_free(&tx_slab, &txblk);

			i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
			k_msleep(10);

			for (int k = 0; k < 4; k++) {
				void *tmp;
				if (k_mem_slab_alloc(&tx_slab, &tmp, K_NO_WAIT) == 0) {
					memset(tmp, 0, BLOCK_BYTES);
					if (i2s_write(i2s_dev, tmp, BLOCK_BYTES) != 0) {
						k_mem_slab_free(&tx_slab, &tmp);
					}
				}
			}

			i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
		}
	}
}

static void rx_thread_fn(void *p1, void *p2, void *p3)
{
	int err;
	uint8_t stereo_buf[BLOCK_BYTES];
	int16_t mono_frame[FRAMES_PER_BLOCK];
	static uint32_t rx_ok_count;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_DBG("audio rx thread running");

	while (1) {
		size_t got = sizeof(stereo_buf);
		err = i2s_buf_read(i2s_dev, stereo_buf, &got);
		if (err == 0 && got == BLOCK_BYTES) {
			downmix_stereo_block_to_mono((const int16_t *)stereo_buf, mono_frame);
			rx_ok_count++;

			if (rx_ok_count <= 5U || (rx_ok_count % 200U) == 0U) {
				LOG_DBG("audio rx: block=%u first_samples=[%d,%d,%d,%d] mono0=%d",
					rx_ok_count, ((const int16_t *)stereo_buf)[0],
					((const int16_t *)stereo_buf)[1],
					((const int16_t *)stereo_buf)[2],
					((const int16_t *)stereo_buf)[3], mono_frame[0]);
			}

			if (capture_drift != NULL) {
				audio_drift_write(capture_drift, mono_frame, 1);
			}
		} else if (err != -EAGAIN && err != -ETIMEDOUT) {
			LOG_ERR("i2s read error: %d, recovering rx", err);
			i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
			k_sleep(K_MSEC(10));
			i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
		}
	}
}
#endif

int audio_init(struct audio_drift_ctx *dl_drift, struct audio_drift_ctx *ul_drift)
{
	int err;

	if (dl_drift == NULL || ul_drift == NULL) {
		return -EINVAL;
	}

	if (initialized) {
		return 0;
	}

	playback_drift = dl_drift;
	capture_drift = ul_drift;

	err = max9867_init();
	if (err) {
		LOG_ERR("max9867_init failed: %d", err);
		return err;
	}
	(void)max9867_status();

#if AUDIO_USE_NRFX_I2S
	audio_i2s_blk_cb_register(i2s_block_complete);
	err = audio_i2s_init();
	if (err) {
		LOG_ERR("audio_i2s_init failed: %d", err);
		return err;
	}
	LOG_INF("audio init: nrfx i2s configured for rbv2h");
#else
	i2s_dev = DEVICE_DT_GET(DT_ALIAS(i2s_node0));
	if (!device_is_ready(i2s_dev)) {
		LOG_ERR("i2s device not ready");
		return -ENODEV;
	}

#if CONFIG_GRPTLK_I2S_CODEC_MASTER
#define I2S_CLK_OPTIONS (I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE)
#else
#define I2S_CLK_OPTIONS (I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER)
#endif

	struct i2s_config rx_cfg = {
		.word_size = 16,
		.channels = CHANNELS,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_CLK_OPTIONS,
		.frame_clk_freq = AUDIO_SAMPLE_RATE_HZ,
		.mem_slab = &rx_slab,
		.block_size = BLOCK_BYTES,
		.timeout = 2000,
	};

	err = i2s_configure(i2s_dev, I2S_DIR_RX, &rx_cfg);
	if (err) {
		LOG_ERR("i2s rx configure failed: %d", err);
		return err;
	}
	LOG_INF("audio init: i2s rx configured word=%u ch=%u rate=%u options=0x%x",
		rx_cfg.word_size, rx_cfg.channels, rx_cfg.frame_clk_freq, rx_cfg.options);

	struct i2s_config tx_cfg = {
		.word_size = 16,
		.channels = CHANNELS,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_CLK_OPTIONS,
		.frame_clk_freq = AUDIO_SAMPLE_RATE_HZ,
		.mem_slab = &tx_slab,
		.block_size = BLOCK_BYTES,
		.timeout = 1000,
	};

	err = i2s_configure(i2s_dev, I2S_DIR_TX, &tx_cfg);
	if (err) {
		LOG_ERR("i2s tx configure failed: %d", err);
		return err;
	}
	LOG_INF("audio init: i2s tx configured word=%u ch=%u rate=%u options=0x%x",
		tx_cfg.word_size, tx_cfg.channels, tx_cfg.frame_clk_freq, tx_cfg.options);

	for (int i = 0; i < 4; i++) {
		void *txblk;
		if (k_mem_slab_alloc(&tx_slab, &txblk, K_FOREVER) == 0) {
			memset(txblk, 0, BLOCK_BYTES);
			err = i2s_write(i2s_dev, txblk, BLOCK_BYTES);
			if (err < 0) {
				k_mem_slab_free(&tx_slab, &txblk);
			}
		}
	}
#endif

	initialized = true;
	return 0;
}

int audio_start(void)
{
	int err;

	if (!initialized) {
		return -EPERM;
	}

	if (started) {
		return 0;
	}

#if AUDIO_USE_NRFX_I2S
	tx_buffer_fill(i2s_tx_buf_a);
	tx_buffer_fill(i2s_tx_buf_b);

	err = audio_i2s_start(i2s_rx_buf_a, i2s_tx_buf_a);
	if (err) {
		LOG_ERR("audio_i2s_start failed: %d", err);
		return err;
	}

	err = audio_i2s_set_next_buf(i2s_rx_buf_b, i2s_tx_buf_b);
	if (err) {
		LOG_ERR("audio_i2s_set_next_buf failed: %d", err);
		return err;
	}
	LOG_INF("audio start: nrfx i2s started for rbv2h");
#else
	k_thread_create(&tx_thread_data, tx_stack, K_THREAD_STACK_SIZEOF(tx_stack), tx_thread_fn,
			NULL, NULL, NULL, 4, 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "audio_tx");

	err = i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_START);
	if (err) {
		LOG_ERR("i2s start failed: %d", err);
		return err;
	}
	LOG_INF("audio start: i2s trigger start ok");

	k_thread_create(&rx_thread_data, rx_stack, K_THREAD_STACK_SIZEOF(rx_stack), rx_thread_fn,
			NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	k_thread_name_set(&rx_thread_data, "audio_rx");
#endif

	err = qdec_start(audio_volume_adjust);
	if (err) {
		LOG_ERR("qdec_start failed: %d", err);
		return err;
	}

	started = true;
	return 0;
}

int audio_volume_adjust(int8_t step_db)
{
	return volume_apply_delta_q15(step_db * VOLUME_3DB_STEP_Q15 / 3);
}
