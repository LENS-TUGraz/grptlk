#include "audio.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "drivers/max9867.h"

#define CHANNELS AUDIO_CHANNELS
#define FRAMES_PER_BLOCK AUDIO_SAMPLES_PER_FRAME
#define BLOCK_BYTES AUDIO_BLOCK_BYTES

#define RX_BLOCK_COUNT 16
#define TX_BLOCK_COUNT 8

BUILD_ASSERT(BLOCK_BYTES == 640, "max9867 backend expects 10 ms stereo 16-bit blocks");

#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && \
	DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels) && \
	DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channel_names) && \
	DT_NODE_HAS_PROP(DT_PATH(zephyr_user), enable_gpios)
#define AUDIO_HAS_VOLUME_CTRL 1
#else
#define AUDIO_HAS_VOLUME_CTRL 0
#endif

static const struct device *i2s_dev;
K_MEM_SLAB_DEFINE_STATIC(rx_slab, BLOCK_BYTES, RX_BLOCK_COUNT, 4);
K_MEM_SLAB_DEFINE_STATIC(tx_slab, BLOCK_BYTES, TX_BLOCK_COUNT, 4);

static struct k_msgq *playback_q;
static audio_rx_cb_t rx_cb;
static bool initialized;
static bool started;

K_THREAD_STACK_DEFINE(rx_stack, 4096);
static struct k_thread rx_thread_data;
K_THREAD_STACK_DEFINE(tx_stack, 2048);
static struct k_thread tx_thread_data;
#if AUDIO_HAS_VOLUME_CTRL
K_THREAD_STACK_DEFINE(vol_stack, 1024);
static struct k_thread vol_thread_data;
#endif

#if AUDIO_HAS_VOLUME_CTRL
static const struct adc_dt_spec pot =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), potentiometer);
static const struct gpio_dt_spec p1_09_en =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), enable_gpios);
#endif

static inline void downmix_stereo_block_to_mono(const int16_t *stereo, int16_t *mono)
{
	for (size_t i = 0; i < FRAMES_PER_BLOCK; i++) {
		int32_t l = stereo[2 * i + 0];
		int32_t r = stereo[2 * i + 1];
		mono[i] = (int16_t)((l + r) / 2);
	}
}

static void tx_thread_fn(void *p1, void *p2, void *p3)
{
	int err;
	int ret;
	uint8_t in[BLOCK_BYTES];

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("audio tx thread running\n");

	while (1) {
		uint32_t free_slabs = k_mem_slab_num_free_get(&tx_slab);
		uint32_t in_i2s = TX_BLOCK_COUNT - free_slabs;

		if (in_i2s < 2) {
			if (playback_q == NULL ||
			    k_msgq_get(playback_q, in, K_NO_WAIT) != 0) {
				memset(in, 0, BLOCK_BYTES);
			}
		} else {
			if (playback_q == NULL ||
			    k_msgq_get(playback_q, in, K_MSEC(5)) != 0) {
				continue;
			}
		}

		void *txblk;
		err = k_mem_slab_alloc(&tx_slab, &txblk, K_FOREVER);
		if (err) {
			continue;
		}
		memcpy(txblk, in, BLOCK_BYTES);

		while ((ret = i2s_write(i2s_dev, txblk, BLOCK_BYTES)) == -EAGAIN) {
			k_msleep(1);
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

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("audio rx thread running\n");

	while (1) {
		size_t got = sizeof(stereo_buf);
		err = i2s_buf_read(i2s_dev, stereo_buf, &got);
		if (err == 0 && got == BLOCK_BYTES) {
			downmix_stereo_block_to_mono((const int16_t *)stereo_buf, mono_frame);
			if (rx_cb != NULL) {
				rx_cb(mono_frame);
			}
		} else if (err != -EAGAIN && err != -ETIMEDOUT) {
			printk("i2s read error: %d\n", err);
			k_sleep(K_MSEC(10));
		}
	}
}

#if AUDIO_HAS_VOLUME_CTRL
static void volume_thread_fn(void *p1, void *p2, void *p3)
{
#define POT_MUTE_ON_MV  100
#define POT_MUTE_OFF_MV 120
#define POT_MAX_MV      3300
#define MAX_VOL_Q15     13668
	static bool muted = true;
	static uint8_t last_reg = 0xFF;
	int err;
	int16_t sample;
	struct adc_sequence seq = { 0 };

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	adc_sequence_init_dt(&pot, &seq);
	seq.buffer = &sample;
	seq.buffer_size = sizeof(sample);

	while (1) {
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
			(void)max9867_set_volume((int16_t)vol_q15, (int16_t)MAX_VOL_Q15);
			last_reg = new_reg;
		}

		k_sleep(K_MSEC(100));
	}
}
#endif

int audio_init(struct k_msgq *tx_q, audio_rx_cb_t mono_rx_cb)
{
	int err;

	if (tx_q == NULL || mono_rx_cb == NULL) {
		return -EINVAL;
	}

	if (initialized) {
		return 0;
	}

	playback_q = tx_q;
	rx_cb = mono_rx_cb;

#if AUDIO_HAS_VOLUME_CTRL
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

	err = max9867_init();
	if (err) {
		printk("max9867_init failed: %d\n", err);
		return err;
	}

	i2s_dev = DEVICE_DT_GET(DT_ALIAS(i2s_node0));
	if (!device_is_ready(i2s_dev)) {
		printk("i2s device not ready\n");
		return -ENODEV;
	}

	struct i2s_config rx_cfg = {
		.word_size = 16,
		.channels = CHANNELS,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE,
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

	struct i2s_config tx_cfg = {
		.word_size = 16,
		.channels = CHANNELS,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE,
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

	for (int i = 0; i < 6; i++) {
		void *txblk;
		if (k_mem_slab_alloc(&tx_slab, &txblk, K_FOREVER) == 0) {
			memset(txblk, 0, BLOCK_BYTES);
			err = i2s_write(i2s_dev, txblk, BLOCK_BYTES);
			if (err < 0) {
				k_mem_slab_free(&tx_slab, &txblk);
			}
		}
	}

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

	k_thread_create(&tx_thread_data, tx_stack, K_THREAD_STACK_SIZEOF(tx_stack),
			tx_thread_fn, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "audio_tx");

	err = i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_START);
	if (err) {
		printk("i2s start failed: %d\n", err);
		return err;
	}

	k_thread_create(&rx_thread_data, rx_stack, K_THREAD_STACK_SIZEOF(rx_stack),
			rx_thread_fn, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	k_thread_name_set(&rx_thread_data, "audio_rx");

#if AUDIO_HAS_VOLUME_CTRL
	k_thread_create(&vol_thread_data, vol_stack, K_THREAD_STACK_SIZEOF(vol_stack),
			volume_thread_fn, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
	k_thread_name_set(&vol_thread_data, "audio_vol");
#endif

	started = true;
	return 0;
}
