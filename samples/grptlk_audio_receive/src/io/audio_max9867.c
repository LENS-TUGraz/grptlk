#include "audio.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "drivers/audio_i2s.h"
#include "drivers/max9867.h"

#define CHANNELS AUDIO_CHANNELS
#define FRAMES_PER_BLOCK AUDIO_SAMPLES_PER_FRAME
#define BLOCK_BYTES AUDIO_BLOCK_BYTES

#define RX_BLOCK_COUNT 16
#define TX_BLOCK_COUNT 8
#define MAX_VOL_Q15 13668
#define VOLUME_3DB_STEP_Q15 911

#if defined(CONFIG_BOARD_RBV2H_NRF5340_CPUAPP) || defined(CONFIG_BOARD_RBV2H_NRF5340_CPUAPP_NS)
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

#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && \
	DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels) && \
	DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channel_names) && \
	DT_NODE_HAS_PROP(DT_PATH(zephyr_user), enable_gpios)
#define AUDIO_HAS_ADC_VOLUME_CTRL 1
#else
#define AUDIO_HAS_ADC_VOLUME_CTRL 0
#endif

#if DT_HAS_ALIAS(qdec0) && DT_NODE_HAS_STATUS(DT_ALIAS(qdec0), okay)
#define AUDIO_HAS_QDEC_VOLUME_CTRL 1
#define AUDIO_QDEC_STEPS DT_PROP(DT_ALIAS(qdec0), steps)
#else
#define AUDIO_HAS_QDEC_VOLUME_CTRL 0
#endif

#define AUDIO_HAS_VOLUME_CTRL (AUDIO_HAS_ADC_VOLUME_CTRL || AUDIO_HAS_QDEC_VOLUME_CTRL)

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

#if AUDIO_HAS_VOLUME_CTRL
K_THREAD_STACK_DEFINE(vol_stack, 1024);
static struct k_thread vol_thread_data;
#endif

#if AUDIO_HAS_ADC_VOLUME_CTRL
static const struct adc_dt_spec pot =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), potentiometer);
static const struct gpio_dt_spec p1_09_en =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), enable_gpios);
#endif

#if AUDIO_HAS_QDEC_VOLUME_CTRL
static const struct device *const qdec = DEVICE_DT_GET(DT_ALIAS(qdec0));
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
		int32_t l = stereo[2 * i + 0];
		int32_t r = stereo[2 * i + 1];
		mono[i] = (int16_t)((l + r) / 2);
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
			printk("audio rx: block=%u first_samples=[%d,%d,%d,%d] mono0=%d\n",
			       rx_ok_count, stereo[0], stereo[1], stereo[2], stereo[3], mono0);
		}

		i2s_process_rx_block(rx_buf_released);
	}

	next_tx_buf = (tx_buf_sel == 0U) ? i2s_tx_buf_a : i2s_tx_buf_b;
	tx_buf_sel ^= 1U;
	tx_buffer_fill(next_tx_buf);

	err = audio_i2s_set_next_buf(rx_buf_released, next_tx_buf);
	if (err != 0) {
		printk("audio_i2s_set_next_buf failed: %d\n", err);
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

	printk("audio tx thread running\n");

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
			printk("audio tx: first block queued (%u bytes)\n", BLOCK_BYTES);
		}

		if (ret < 0) {
			printk("i2s tx write failed: %d, recovering\n", ret);
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

	printk("audio rx thread running\n");

	while (1) {
		size_t got = sizeof(stereo_buf);
		err = i2s_buf_read(i2s_dev, stereo_buf, &got);
		if (err == 0 && got == BLOCK_BYTES) {
			downmix_stereo_block_to_mono((const int16_t *)stereo_buf, mono_frame);
			rx_ok_count++;

			if (rx_ok_count <= 5U || (rx_ok_count % 200U) == 0U) {
				printk("audio rx: block=%u first_samples=[%d,%d,%d,%d] mono0=%d\n",
				       rx_ok_count,
				       ((const int16_t *)stereo_buf)[0],
				       ((const int16_t *)stereo_buf)[1],
				       ((const int16_t *)stereo_buf)[2],
				       ((const int16_t *)stereo_buf)[3],
				       mono_frame[0]);
			}

			if (capture_drift != NULL) {
				audio_drift_write(capture_drift, mono_frame, 1);
			}
		} else if (err != -EAGAIN && err != -ETIMEDOUT) {
			printk("i2s read error: %d, recovering rx\n", err);
			i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
			k_sleep(K_MSEC(10));
			i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
		}
	}
}
#endif

#if AUDIO_HAS_VOLUME_CTRL
static void volume_thread_fn(void *p1, void *p2, void *p3)
{
#if AUDIO_HAS_ADC_VOLUME_CTRL
#define POT_MUTE_ON_MV  100
#define POT_MUTE_OFF_MV 120
#define POT_MAX_MV      3300
	static bool muted = true;
	static uint8_t last_reg = 0xFF;
	int err;
	int16_t sample;
	struct adc_sequence seq = { 0 };
#elif AUDIO_HAS_QDEC_VOLUME_CTRL
	struct sensor_value val;
	int err;
#endif

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

#if AUDIO_HAS_ADC_VOLUME_CTRL
	adc_sequence_init_dt(&pot, &seq);
	seq.buffer = &sample;
	seq.buffer_size = sizeof(sample);
#endif

	while (1) {
#if AUDIO_HAS_ADC_VOLUME_CTRL
		err = adc_read_dt(&pot, &seq);
		if (err) {
			k_sleep(K_MSEC(200));
			continue;
		}

		int32_t mv = sample;
		err = adc_raw_to_millivolts_dt(&pot, &mv);
		if (err) {
			k_sleep(K_MSEC(200));
			continue;
		}

		gpio_pin_set_dt(&p1_09_en, (mv > 0) ? 1 : 0);

		if (mv <= POT_MUTE_ON_MV) {
			if (!muted) {
				max9867_set_mute(true);
				muted = true;
				last_reg = 0xFF;
			}
			k_sleep(K_MSEC(100));
			continue;
		} else if (muted && mv >= POT_MUTE_OFF_MV) {
			max9867_set_mute(false);
			muted = false;
		}

		if (mv > POT_MAX_MV) {
			mv = POT_MAX_MV;
		}

		int32_t span = POT_MAX_MV - POT_MUTE_OFF_MV;
		int32_t mv_rel = mv - POT_MUTE_OFF_MV;
		uint16_t vol_q15 =
			(uint16_t)((mv_rel * (int32_t)MAX_VOL_Q15 + span / 2) / span);
		uint8_t new_reg =
			(uint8_t)(40 - ((int32_t)vol_q15 * 41 / 0x7FFF));

		if (new_reg != last_reg) {
			current_vol_q15 = vol_q15;
			(void)max9867_set_volume((int16_t)vol_q15, (int16_t)MAX_VOL_Q15);
			last_reg = new_reg;
		}

		k_sleep(K_MSEC(100));
#elif AUDIO_HAS_QDEC_VOLUME_CTRL
		err = sensor_sample_fetch(qdec);
		if (err) {
			k_sleep(K_MSEC(50));
			continue;
		}

		err = sensor_channel_get(qdec, SENSOR_CHAN_ROTATION, &val);
		if (err) {
			k_sleep(K_MSEC(50));
			continue;
		}

		if (val.val1 != 0) {
			int32_t rotation = (val.val1 < 0) ? -val.val1 : val.val1;
			int32_t detents =
				DIV_ROUND_CLOSEST(rotation * AUDIO_QDEC_STEPS, 360);
			int32_t delta_q15;

			if (detents == 0) {
				detents = 1;
			}

			delta_q15 = ((val.val1 < 0) ? 1 : -1) *
				    detents * VOLUME_3DB_STEP_Q15 / 3;
			(void)volume_apply_delta_q15(delta_q15);
			printk("[VOL] qdec %s (%ld deg, %ld detents)\n",
			       val.val1 < 0 ? "up" : "down",
			       (long)val.val1, (long)detents);
		}

		k_sleep(K_MSEC(25));
#endif
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

#if AUDIO_HAS_ADC_VOLUME_CTRL
	if (!adc_is_ready_dt(&pot)) {
		printk("adc device not ready\n");
		return -EIO;
	}

	err = adc_channel_setup_dt(&pot);
	if (err) {
		printk("adc_channel_setup_dt failed: %d\n", err);
		return -EIO;
	}

	if (gpio_is_ready_dt(&p1_09_en)) {
		(void)gpio_pin_configure_dt(&p1_09_en, GPIO_OUTPUT_INACTIVE);
	}
#endif

#if AUDIO_HAS_QDEC_VOLUME_CTRL
	if (!device_is_ready(qdec)) {
		printk("qdec device not ready\n");
		return -ENODEV;
	}
#endif

	err = max9867_init();
	if (err) {
		printk("max9867_init failed: %d\n", err);
		return err;
	}
	(void)max9867_status();

#if AUDIO_USE_NRFX_I2S
	audio_i2s_blk_cb_register(i2s_block_complete);
	err = audio_i2s_init();
	if (err) {
		printk("audio_i2s_init failed: %d\n", err);
		return err;
	}
	printk("audio init: nrfx i2s configured for rbv2h\n");
#else
	i2s_dev = DEVICE_DT_GET(DT_ALIAS(i2s_node0));
	if (!device_is_ready(i2s_dev)) {
		printk("i2s device not ready\n");
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
		printk("i2s rx configure failed: %d\n", err);
		return err;
	}
	printk("audio init: i2s rx configured word=%u ch=%u rate=%u options=0x%x\n",
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
		printk("i2s tx configure failed: %d\n", err);
		return err;
	}
	printk("audio init: i2s tx configured word=%u ch=%u rate=%u options=0x%x\n",
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
		printk("audio_i2s_start failed: %d\n", err);
		return err;
	}

	err = audio_i2s_set_next_buf(i2s_rx_buf_b, i2s_tx_buf_b);
	if (err) {
		printk("audio_i2s_set_next_buf failed: %d\n", err);
		return err;
	}
	printk("audio start: nrfx i2s started for rbv2h\n");
#else
	k_thread_create(&tx_thread_data, tx_stack, K_THREAD_STACK_SIZEOF(tx_stack),
			tx_thread_fn, NULL, NULL, NULL, 4, 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "audio_tx");

	err = i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_START);
	if (err) {
		printk("i2s start failed: %d\n", err);
		return err;
	}
	printk("audio start: i2s trigger start ok\n");

	k_thread_create(&rx_thread_data, rx_stack, K_THREAD_STACK_SIZEOF(rx_stack),
			rx_thread_fn, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	k_thread_name_set(&rx_thread_data, "audio_rx");
#endif

#if AUDIO_HAS_VOLUME_CTRL
	k_thread_create(&vol_thread_data, vol_stack, K_THREAD_STACK_SIZEOF(vol_stack),
			volume_thread_fn, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
	k_thread_name_set(&vol_thread_data, "audio_vol");
#endif

	started = true;
	return 0;
}

int audio_volume_adjust(int8_t step_db)
{
#if !AUDIO_HAS_VOLUME_CTRL
	return volume_apply_delta_q15(step_db * VOLUME_3DB_STEP_Q15 / 3);
#else
	ARG_UNUSED(step_db);
	return 0;
#endif
}
