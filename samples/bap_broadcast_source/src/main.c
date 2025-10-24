#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <math.h>
#include "lc3.h"
#include <zephyr/drivers/i2s.h>
#include "max9867.h"

#define BROADCAST_ENQUEUE_COUNT 3U
#define TOTAL_BUF_NEEDED (BROADCAST_ENQUEUE_COUNT * CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT)

static struct bt_bap_lc3_preset preset_active = BT_BAP_LC3_BROADCAST_PRESET_16_2_1(
	BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
	BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

#define BROADCAST_SAMPLE_RATE 16000
#define MAX_SAMPLE_RATE 16000
#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)

#define AUDIO_VOLUME (INT16_MAX - 3000) /* codec does clipping above INT16_MAX - 3000 */
#define AUDIO_TONE_FREQUENCY_HZ 800

#define SAMPLE_NO 16
#define NUM_BUFFERS 32
#define PREALLOC_BUFFERS 8
#define BLOCK_SIZE (sizeof(data))

static int16_t send_pcm_data[MAX_NUM_SAMPLES];
static int16_t data[SAMPLE_NO * 2] = {};

#define MEM_SLAB_CACHE_ATTR

static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(32))
	_k_mem_slab_buf_tx_0_mem_slab[(NUM_BUFFERS)*WB_UP(BLOCK_SIZE)];

static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_0_mem_slab) =
	Z_MEM_SLAB_INITIALIZER(tx_0_mem_slab, _k_mem_slab_buf_tx_0_mem_slab,
						   WB_UP(BLOCK_SIZE), NUM_BUFFERS);

/* Stereo frame = L + R, 16-bit each */
#define I2S_STEREO_FRAME_BYTES (2u * sizeof(int16_t))
#define I2S_BLOCK_FRAMES (BLOCK_SIZE / I2S_STEREO_FRAME_BYTES)

/* How many PCM samples we actually filled in send_pcm_data */
static const size_t send_pcm_samples =
	(MAX_FRAME_DURATION_US * BROADCAST_SAMPLE_RATE) / USEC_PER_SEC;

/* Playback read index into send_pcm_data (mono) */
static size_t send_pcm_rd = 0;

/* ---------- audio format (keep these near your other defines) ---------- */
#define SAMPLE_RATE_HZ      16000
#define CHANNELS            2
#define SAMPLE_BYTES        2   /* 16-bit */

/* Use 10 ms I/O blocks to keep queues moving */
#define BLOCK_MS            10
#define BLOCK_BYTES         ((SAMPLE_RATE_HZ * CHANNELS * SAMPLE_BYTES * BLOCK_MS) / 1000)

/* Use one shared slab for both RX and TX (zero-copy) */
#define SLAB_BLOCKS         12  /* 12 * 10 ms = 120 ms total buffering */
K_MEM_SLAB_DEFINE_STATIC(i2s_slab, BLOCK_BYTES, SLAB_BLOCKS, 4);

/* ---- Audio format ---- */
#define SAMPLE_RATE_HZ      16000
#define CHANNELS            2
#define SAMPLE_BYTES        2   /* 16-bit */

/* Driver RX block = 10 ms of audio (helps keep RX queue draining) */
#define BLOCK_MS            10
#define BLOCK_BYTES         ((SAMPLE_RATE_HZ * CHANNELS * SAMPLE_BYTES * BLOCK_MS) / 1000)

/* RX queue backing storage (driver allocates from this slab) */
#define RX_BLOCK_COUNT      8   /* 8 * 10 ms = 80 ms buffering in driver */
K_MEM_SLAB_DEFINE_STATIC(rx_mem_slab, BLOCK_BYTES, RX_BLOCK_COUNT, 4);

/* 1 second circular buffer in RAM to store captured audio */
#define RING_MS             1000
#define RING_BYTES          ((SAMPLE_RATE_HZ * CHANNELS * SAMPLE_BYTES * RING_MS) / 1000)
static uint8_t mic_ring[RING_BYTES];
static volatile size_t mic_wr = 0; /* write index into ring (bytes) */

static const struct device *i2s = DEVICE_DT_GET(DT_ALIAS(i2s_node0)); /* match your DTS alias */

/* Copy helper that wraps into mic_ring */
static void ring_write(const uint8_t *src, size_t n)
{
    size_t first = MIN(n, RING_BYTES - mic_wr);
    memcpy(&mic_ring[mic_wr], src, first);
    size_t remaining = n - first;
    if (remaining) {
        memcpy(&mic_ring[0], src + first, remaining);
    }
    mic_wr = (mic_wr + n) % RING_BYTES;
}

/* Fill a TX block (stereo-interleaved) from send_pcm_data (mono) */
static inline void i2s_fill_block_from_send_pcm(int16_t *dst_stereo, size_t frames)
{
	for (size_t i = 0; i < frames; i++)
	{
		int16_t s = send_pcm_data[send_pcm_rd++];
		if (send_pcm_rd >= send_pcm_samples)
		{
			send_pcm_rd = 0;
		}
		/* L, R */
		*dst_stereo++ = s;
		*dst_stereo++ = s;
	}
}

/**
 * Use the math lib to generate a sine-wave using 16 bit samples into a buffer.
 *
 * @param buf Destination buffer
 * @param length_us Length of the buffer in microseconds
 * @param frequency_hz frequency in Hz
 * @param sample_rate_hz sample-rate in Hz.
 */
static void fill_audio_buf_sin(int16_t *buf, int length_us, int frequency_hz, int sample_rate_hz)
{
	const int sine_period_samples = sample_rate_hz / frequency_hz;
	const unsigned int num_samples = (length_us * sample_rate_hz) / USEC_PER_SEC;
	const float step = 2 * 3.1415f / sine_period_samples;

	for (unsigned int i = 0; i < num_samples; i++)
	{
		const float sample = sinf(i * step);

		buf[i] = (int16_t)(AUDIO_VOLUME * sample);
	}
}

static struct broadcast_source_stream
{
	struct bt_bap_stream stream;
	uint16_t seq_num;
	size_t sent_cnt;
	lc3_encoder_t lc3_encoder;
	lc3_encoder_mem_16k_t lc3_encoder_mem;
} streams[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
static struct bt_bap_broadcast_source *broadcast_source;

NET_BUF_POOL_FIXED_DEFINE(tx_pool, TOTAL_BUF_NEEDED, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
						  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);


static bool stopping;

static K_SEM_DEFINE(sem_started, 0U, 1U);
static K_SEM_DEFINE(sem_stopped, 0U, 1U);

static int freq_hz;
static int frame_duration_us;
static int frames_per_sdu;
static int octets_per_frame;

static K_SEM_DEFINE(lc3_encoder_sem, 0U, TOTAL_BUF_NEEDED);

static void send_data(struct broadcast_source_stream *source_stream)
{
	struct bt_bap_stream *stream = &source_stream->stream;
	struct net_buf *buf;
	int ret;

	if (stopping)
	{
		return;
	}

	buf = net_buf_alloc(&tx_pool, K_FOREVER);
	if (buf == NULL)
	{
		printk("Could not allocate buffer when sending on %p\n", stream);
		return;
	}

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	uint8_t lc3_encoded_buffer[preset_active.qos.sdu];

	if (source_stream->lc3_encoder == NULL)
	{
		printk("LC3 encoder not setup, cannot encode data.\n");
		net_buf_unref(buf);
		return;
	}

	ret = lc3_encode(source_stream->lc3_encoder, LC3_PCM_FORMAT_S16, send_pcm_data, 1,
					 octets_per_frame, lc3_encoded_buffer);
	if (ret == -1)
	{
		printk("LC3 encoder failed - wrong parameters?: %d", ret);
		net_buf_unref(buf);
		return;
	}

	net_buf_add_mem(buf, lc3_encoded_buffer, preset_active.qos.sdu);

	ret = bt_bap_stream_send(stream, buf, source_stream->seq_num++);
	if (ret < 0)
	{
		/* This will end broadcasting on this stream. */
		printk("Unable to broadcast data on %p: %d\n", stream, ret);
		net_buf_unref(buf);
		return;
	}

	source_stream->sent_cnt++;
	if ((source_stream->sent_cnt % 1000U) == 0U)
	{
		printk("Stream %p: Sent %u total ISO packets\n", stream, source_stream->sent_cnt);
	}
}

static void init_lc3_thread(void *arg1, void *arg2, void *arg3)
{
	const struct bt_audio_codec_cfg *codec_cfg = &preset_active.codec_cfg;
	int ret;

	ret = bt_audio_codec_cfg_get_freq(codec_cfg);
	if (ret > 0)
	{
		freq_hz = bt_audio_codec_cfg_freq_to_freq_hz(ret);
	}
	else
	{
		return;
	}

	ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
	if (ret > 0)
	{
		frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
	}
	else
	{
		printk("Error: Frame duration not set, cannot start codec.");
		return;
	}

	octets_per_frame = bt_audio_codec_cfg_get_octets_per_frame(codec_cfg);
	frames_per_sdu = bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true);

	if (freq_hz < 0)
	{
		printk("Error: Codec frequency not set, cannot start codec.");
		return;
	}

	if (frame_duration_us < 0)
	{
		printk("Error: Frame duration not set, cannot start codec.");
		return;
	}

	if (octets_per_frame < 0)
	{
		printk("Error: Octets per frame not set, cannot start codec.");
		return;
	}

	fill_audio_buf_sin(send_pcm_data, frame_duration_us, AUDIO_TONE_FREQUENCY_HZ, freq_hz);

	/* Create the encoder instance. This shall complete before stream_started() is called. */
	for (size_t i = 0U; i < ARRAY_SIZE(streams); i++)
	{
		printk("Initializing lc3 encoder for stream %zu\n", i);
		streams[i].lc3_encoder = lc3_setup_encoder(frame_duration_us, freq_hz, 0,
												   &streams[i].lc3_encoder_mem);

		if (streams[i].lc3_encoder == NULL)
		{
			printk("ERROR: Failed to setup LC3 encoder - wrong parameters?\n");
		}
	}

	while (true)
	{
		// Only send on the first BIS in the BIG
		k_sem_take(&lc3_encoder_sem, K_FOREVER);
		send_data(&streams[0]);
	}
}

#define LC3_ENCODER_STACK_SIZE 4 * 4096
#define LC3_ENCODER_PRIORITY 5

K_THREAD_DEFINE(encoder, LC3_ENCODER_STACK_SIZE, init_lc3_thread, NULL, NULL, NULL,
				LC3_ENCODER_PRIORITY, 0, -1);

static void stream_started_cb(struct bt_bap_stream *stream)
{
	struct broadcast_source_stream *source_stream =
		CONTAINER_OF(stream, struct broadcast_source_stream, stream);

	source_stream->seq_num = 0U;
	source_stream->sent_cnt = 0U;
}

static void stream_sent_cb(struct bt_bap_stream *stream)
{
	k_sem_give(&lc3_encoder_sem);
}

static struct bt_bap_stream_ops stream_ops = {
	.started = stream_started_cb,
	.sent = stream_sent_cb,
};

static int setup_broadcast_source(struct bt_bap_broadcast_source **source)
{
	struct bt_bap_broadcast_source_stream_param
		stream_params[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
	struct bt_bap_broadcast_source_subgroup_param
		subgroup_param[CONFIG_BT_BAP_BROADCAST_SRC_SUBGROUP_COUNT];
	struct bt_bap_broadcast_source_param create_param = {0};
	const size_t streams_per_subgroup = ARRAY_SIZE(stream_params) / ARRAY_SIZE(subgroup_param);
	uint8_t front_center[] = {BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
												  BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_LEFT))};
	uint8_t back_center[] = {BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
												 BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_RIGHT))};
	int err;

	for (size_t i = 0U; i < ARRAY_SIZE(subgroup_param); i++)
	{
		subgroup_param[i].params_count = streams_per_subgroup;
		subgroup_param[i].params = stream_params + i * streams_per_subgroup;
		subgroup_param[i].codec_cfg = &preset_active.codec_cfg;
	}

	for (size_t j = 0U; j < ARRAY_SIZE(stream_params); j++)
	{
		stream_params[j].stream = &streams[j].stream;
		stream_params[j].data = j == 0 ? front_center : back_center;
		stream_params[j].data_len = j == 0 ? sizeof(front_center) : sizeof(back_center);
		bt_bap_stream_cb_register(stream_params[j].stream, &stream_ops);
	}

	create_param.params_count = ARRAY_SIZE(subgroup_param);
	create_param.params = subgroup_param;
	create_param.qos = &preset_active.qos;
	create_param.encryption = false;
	create_param.packing = BT_ISO_PACKING_SEQUENTIAL;

	printk("Creating broadcast source with %zu subgroups with %zu streams\n",
		   ARRAY_SIZE(subgroup_param), ARRAY_SIZE(subgroup_param) * streams_per_subgroup);

	err = bt_bap_broadcast_source_create(&create_param, source);
	if (err != 0)
	{
		printk("Unable to create broadcast source: %d\n", err);
		return err;
	}

	return 0;
}

static void source_started_cb(struct bt_bap_broadcast_source *source)
{
	printk("Broadcast source %p started\n", source);
	k_sem_give(&sem_started);
}

static void source_stopped_cb(struct bt_bap_broadcast_source *source, uint8_t reason)
{
	printk("Broadcast source %p stopped with reason 0x%02X\n", source, reason);
	k_sem_give(&sem_stopped);
}

int main(void)
{
	static struct bt_bap_broadcast_source_cb broadcast_source_cb = {
		.started = source_started_cb,
		.stopped = source_stopped_cb,
	};
	struct bt_le_ext_adv *adv;
	int err;

	for (size_t i = 0U; i < ARRAY_SIZE(send_pcm_data); i++)
	{
		/* Initialize mock data */
		send_pcm_data[i] = i;
	}

	k_thread_start(encoder);

	struct i2s_config i2s_cfg;
	const struct device *dev_i2s = DEVICE_DT_GET(DT_ALIAS(i2s_node0));

	err = max9867_init();
	if (err)
	{
		printk("MAX9867 init failed: %d\n", err);
		return err;
	}

	if (!device_is_ready(dev_i2s))
	{
		printk("I2S device not ready\n");
		return -ENODEV;
	}


	/* Configure RX in SLAVE mode (codec provides BCLK/LRCLK) */
    struct i2s_config rx_cfg = {
        .word_size      = 16,                       /* bits per sample */
        .channels       = CHANNELS,
        .format         = I2S_FMT_DATA_FORMAT_I2S, /* standard I2S */
        .options        = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE,
        .frame_clk_freq = SAMPLE_RATE_HZ,          /* sample rate (informational in slave) */
        .mem_slab       = &rx_mem_slab,            /* driver’s RX queue backing store */
        .block_size     = BLOCK_BYTES,             /* one 10 ms block */
        .timeout        = 1000,                    /* ms: read timeout */
    };

	err = i2s_configure(i2s, I2S_DIR_RX, &rx_cfg);
if (err) {
    printk("i2s_configure RX failed: %d\n", err);
    return err;
}

/* Do NOT call PREPARE here */
err = i2s_trigger(i2s, I2S_DIR_RX, I2S_TRIGGER_START);
if (err) {
    printk("RX START failed: %d\n", err);
    return err;
}


    printk("Capturing mic audio into 1s ring buffer...\n");

    /* Scratch buffer big enough for one RX block */
    uint8_t block_buf[BLOCK_BYTES];

    while (1) {
        size_t got = sizeof(block_buf);
        /* Copies one RX block from the driver into block_buf, and frees it internally */
        err = i2s_buf_read(i2s, block_buf, &got);
        if (err == 0) {
            /* Store into circular buffer */
            ring_write(block_buf, got);
			// printk("Captured %x %x %x %x ...\n", block_buf[0], block_buf[1], block_buf[2], block_buf[3]);
        } else if (err == -EAGAIN || err == -EBUSY) {
            /* No block ready yet / busy, try again shortly */
            k_sleep(K_USEC(200));
        } else if (err == -ETIMEDOUT) {
            /* Driver timeout (depends on cfg.timeout); keep trying */
            continue;
        } else {
            printk("i2s_buf_read error: %d\n", err);
            break;
        }
    }


	/* Configure I2S stream */
	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = 16000;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = 2000;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE;
	i2s_cfg.mem_slab = &tx_0_mem_slab;

	err = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);
	if (err < 0)
	{
		printk("Failed to configure I2S stream\n");
		return err;
	}

	/* ---- I2S: pre-queue a few TX blocks with the sine ---- */
	for (int i = 0; i < PREALLOC_BUFFERS; i++)
	{
		void *tx_block;
		int ret = k_mem_slab_alloc(&tx_0_mem_slab, &tx_block, K_FOREVER);
		if (ret < 0)
		{
			printk("I2S: slab alloc failed %d\n", ret);
			return ret;
		}

		/* Fill one block worth of stereo frames from send_pcm_data */
		i2s_fill_block_from_send_pcm((int16_t *)tx_block, I2S_BLOCK_FRAMES);

		/* Queue the block; driver takes ownership and will free it after TX */
		ret = i2s_write(dev_i2s, tx_block, BLOCK_SIZE);
		if (ret < 0)
		{
			printk("I2S: write prequeue %d failed: %d\n", i, ret);
			/* Only free on write failure; otherwise driver frees it */
			k_mem_slab_free(&tx_0_mem_slab, &tx_block);
			return ret;
		}
	}

	/* Start transmission (must be in READY and have data queued) */
	int ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0)
	{
		printk("I2S: start failed: %d\n", ret);
		return ret;
	}


	/* ---- I2S: keep the queue filled ---- */
	while (1)
	{
		void *tx_block;

		/* Try grab a fresh TX block; if none, yield briefly */
		ret = k_mem_slab_alloc(&tx_0_mem_slab, &tx_block, K_NO_WAIT);
		if (ret < 0)
		{
			k_msleep(1);
			continue;
		}

		i2s_fill_block_from_send_pcm((int16_t *)tx_block, I2S_BLOCK_FRAMES);

		/* Keep retrying on EAGAIN (bus busy); driver frees on success */
		while ((ret = i2s_write(dev_i2s, tx_block, BLOCK_SIZE)) < 0)
		{
			if (ret == -EAGAIN)
			{
				k_msleep(1);
				continue;
			}
			printk("I2S: write failed: %d\n", ret);
			k_mem_slab_free(&tx_0_mem_slab, &tx_block);
			return ret;
		}
	}

	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	printk("Bluetooth initialized\n");

	err = bt_bap_broadcast_source_register_cb(&broadcast_source_cb);
	if (err != 0)
	{
		printk("Failed to register broadcast source callbacks (err %d)\n", err);
		return 0;
	}

	while (true)
	{
		/* Broadcast Audio Streaming Endpoint advertising data */
		NET_BUF_SIMPLE_DEFINE(ad_buf, BT_UUID_SIZE_16 + BT_AUDIO_BROADCAST_ID_SIZE);
		NET_BUF_SIMPLE_DEFINE(base_buf, 128);
		struct bt_data ext_ad[2];
		struct bt_data per_ad;
		uint32_t broadcast_id;

		/* Create a connectable advertising set */
		err = bt_le_ext_adv_create(BT_BAP_ADV_PARAM_BROADCAST_FAST, NULL, &adv);
		if (err != 0)
		{
			printk("Unable to create extended advertising set: %d\n", err);
			return 0;
		}

		/* Set periodic advertising parameters */
		err = bt_le_per_adv_set_param(adv, BT_BAP_PER_ADV_PARAM_BROADCAST_FAST);
		if (err)
		{
			printk("Failed to set periodic advertising parameters (err %d)\n", err);
			return 0;
		}

		printk("Creating broadcast source\n");
		err = setup_broadcast_source(&broadcast_source);
		if (err != 0)
		{
			printk("Unable to setup broadcast source: %d\n", err);
			return 0;
		}

		broadcast_id = 0x123456;

		/* Setup extended advertising data */
		net_buf_simple_add_le16(&ad_buf, BT_UUID_BROADCAST_AUDIO_VAL);
		net_buf_simple_add_le24(&ad_buf, broadcast_id);
		ext_ad[0].type = BT_DATA_SVC_DATA16;
		ext_ad[0].data_len = ad_buf.len;
		ext_ad[0].data = ad_buf.data;
		ext_ad[1] = (struct bt_data)BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
											sizeof(CONFIG_BT_DEVICE_NAME) - 1);
		err = bt_le_ext_adv_set_data(adv, ext_ad, 2, NULL, 0);
		if (err != 0)
		{
			printk("Failed to set extended advertising data: %d\n", err);
			return 0;
		}

		/* Setup periodic advertising data */
		err = bt_bap_broadcast_source_get_base(broadcast_source, &base_buf);
		if (err != 0)
		{
			printk("Failed to get encoded BASE: %d\n", err);
			return 0;
		}

		per_ad.type = BT_DATA_SVC_DATA16;
		per_ad.data_len = base_buf.len;
		per_ad.data = base_buf.data;
		err = bt_le_per_adv_set_data(adv, &per_ad, 1);
		if (err != 0)
		{
			printk("Failed to set periodic advertising data: %d\n", err);
			return 0;
		}

		/* Start extended advertising */
		err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
		if (err)
		{
			printk("Failed to start extended advertising: %d\n", err);
			return 0;
		}

		/* Enable Periodic Advertising */
		err = bt_le_per_adv_start(adv);
		if (err)
		{
			printk("Failed to enable periodic advertising: %d\n", err);
			return 0;
		}

		printk("Starting broadcast source\n");
		stopping = false;
		err = bt_bap_broadcast_source_start(broadcast_source, adv);
		if (err != 0)
		{
			printk("Unable to start broadcast source: %d\n", err);
			return 0;
		}

		/* Wait for broadcast source to be started */
		k_sem_take(&sem_started, K_FOREVER);
		printk("Broadcast source started\n");

		/* Initialize sending */
		for (size_t i = 0U; i < ARRAY_SIZE(streams); i++)
		{
			for (unsigned int j = 0U; j < BROADCAST_ENQUEUE_COUNT; j++)
			{
				stream_sent_cb(&streams[i].stream);
			}
		}

		printk("Let's gooooooooooo\n");

		// err = bt_bap_broadcast_source_stop(broadcast_source);
		// if (err != 0) {
		// 	printk("Unable to stop broadcast source: %d\n", err);
		// 	return 0;
		// }

		/* Wait for broadcast source to be stopped */
		k_sem_take(&sem_stopped, K_FOREVER);
		printk("Broadcast source stopped\n");
	}
	return 0;
}
