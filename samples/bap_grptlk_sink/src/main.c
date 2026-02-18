#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/i2s.h>
#include <math.h>
#include "lc3.h"
#include "max9867.h"

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN 30

#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE,    \
										   BT_LE_SCAN_OPT_NONE,       \
										   BT_GAP_SCAN_FAST_INTERVAL, \
										   BT_GAP_SCAN_FAST_WINDOW)

#define PA_RETRY_COUNT 6

#define BIS_ISO_CHAN_COUNT 3

static const struct device *i2s_dev;

/* --------------------------- Audio & I2S format --------------------------- */

#define SAMPLE_RATE_HZ 16000 /* 16 kHz */
#define CHANNELS 2			 /* stereo */
#define SAMPLE_BYTES 2		 /* 16-bit */

#define BLOCK_MS 10
#define FRAMES_PER_BLOCK ((SAMPLE_RATE_HZ * BLOCK_MS) / 1000)	 /* 160 frames */
#define BLOCK_BYTES (FRAMES_PER_BLOCK * CHANNELS * SAMPLE_BYTES) /* 640 B */

/* RX: slab used by the I2S driver (we consume via i2s_buf_read) */
#define RX_BLOCK_COUNT 24 /* 240 ms RX buffering */
K_MEM_SLAB_DEFINE_STATIC(rx_slab, BLOCK_BYTES, RX_BLOCK_COUNT, 4);

/* dedicated stacks and TCBs for RX/TX/LC3 threads */
K_THREAD_STACK_DEFINE(rx_stack, 4096);
static struct k_thread rx_thread;

/* LC3 needs 10 ms @ 16 kHz = 160 mono samples */
#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES ((MAX_FRAME_DURATION_US * SAMPLE_RATE_HZ) / USEC_PER_SEC) /* 160 */
static int16_t send_pcm_data[MAX_NUM_SAMPLES];	

K_MSGQ_DEFINE(pcm_msgq, sizeof(int16_t) * MAX_NUM_SAMPLES, 16, 4);

static bool per_adv_found;
static bool per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t per_sid;
static uint32_t per_interval_us;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_sync_lost, 0, BIS_ISO_CHAN_COUNT);

static bool iso_datapaths_setup = false;

lc3_encoder_t lc3_encoder;
lc3_encoder_mem_16k_t lc3_encoder_mem;

#define BROADCAST_SAMPLE_RATE 16000
#define MAX_SAMPLE_RATE 16000
#define MAX_FRAME_DURATION_US 10000
// #define MAX_NUM_SAMPLES ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)

static int freq_hz;
static int frame_duration_us;
static int frames_per_sdu;
static int octets_per_frame;

// static K_SEM_DEFINE(lc3_encoder_sem, 0U, 1U);

static void scan_recv(const struct bt_le_scan_recv_info *info,
					  struct net_buf_simple *buf)
{
	if (!per_adv_found && info->interval)
	{
		per_adv_found = true;

		per_sid = info->sid;
		per_interval_us = BT_CONN_INTERVAL_TO_US(info->interval);
		bt_addr_le_copy(&per_addr, info->addr);

		k_sem_give(&sem_per_adv);
	}
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

static void sync_cb(struct bt_le_per_adv_sync *sync,
					struct bt_le_per_adv_sync_synced_info *info)
{
	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
					const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
		   bt_le_per_adv_sync_get_index(sync), le_addr);

	per_adv_lost = true;
	k_sem_give(&sem_per_sync_lost);
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
					   const struct bt_iso_biginfo *biginfo)
{
	k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.biginfo = biginfo_cb,
};

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT,
						  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
						  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static uint16_t seq_num;
static uint32_t iso_send_count = 0U;
static uint8_t iso_data[CONFIG_BT_ISO_TX_MTU] = {0};

static struct bt_iso_chan *bis[];

static int16_t send_pcm_data[MAX_NUM_SAMPLES];

#define AUDIO_VOLUME (INT16_MAX - 3000) /* codec does clipping above INT16_MAX - 3000 */
#define AUDIO_TONE_FREQUENCY_HZ 1600

/**
 * Use the math lib to generate a sine-wave using 16 bit samples into a buffer.
 *
 * @param buf Destination buffer
 * @param length_us Length of the buffer in microseconds
 * @param frequency_hz frequency in Hz
 * @param sample_rate_hz sample-rate in Hz.
 */
// static void fill_audio_buf_sin(int16_t *buf, int length_us, int frequency_hz, int sample_rate_hz)
// {
// 	const int sine_period_samples = sample_rate_hz / frequency_hz;
// 	const unsigned int num_samples = (length_us * sample_rate_hz) / USEC_PER_SEC;
// 	const float step = 2 * 3.1415f / sine_period_samples;

// 	for (unsigned int i = 0; i < num_samples; i++)
// 	{
// 		const float sample = sinf(i * step);

// 		buf[i] = (int16_t)(AUDIO_VOLUME * sample);
// 	}
// }

static void iso_sent(struct bt_iso_chan *chan)
{
	iso_datapaths_setup = false; // todo: shitty alignment for sending

	int err;
	int ret;
	struct net_buf *buf;
	// static uint8_t lc3_encoded_buffer[40] = {0x00};

	buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
	if (!buf)
	{
		printk("Data buffer allocate timeout\n");
		return;
	}

	if (lc3_encoder == NULL)
	{
		printk("LC3 encoder not setup, cannot encode data.\n");
		net_buf_unref(buf);
		return;
	}

	// memset(iso_data, 0, sizeof(iso_data));

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

	/* Try to get a fresh 10 ms mono frame; otherwise keep previous data */
	int16_t frame[MAX_NUM_SAMPLES];
	if (k_msgq_get(&pcm_msgq, frame, K_MSEC(2)) == 0)
	{
		memcpy(send_pcm_data, frame, sizeof(frame));
	}

	// printk("send_pcm_data: %x%x%x%x\n", send_pcm_data[0], send_pcm_data[1], send_pcm_data[2], send_pcm_data[3], send_pcm_data[4]);
	/* else: keep last send_pcm_data contents (avoids popping to silence) */

	uint8_t lc3_encoded_buffer[CONFIG_BT_ISO_TX_MTU];
	ret = lc3_encode(lc3_encoder,
						 LC3_PCM_FORMAT_S16,
						 send_pcm_data, 1,
						 octets_per_frame,
						 lc3_encoded_buffer);
	if (ret == -1)
	{
		printk("LC3 encode failed\n");
		net_buf_unref(buf);
		return;
	}

	net_buf_add_mem(buf, lc3_encoded_buffer, sizeof(lc3_encoded_buffer));



	// ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, send_pcm_data, 1,
	// 				 octets_per_frame, iso_data);
	// if (ret == -1)
	// {
	// 	printk("LC3 encoder failed - wrong parameters?: %d", ret);
	// 	net_buf_unref(buf);
	// 	return;
	// }

	
	// net_buf_add_mem(buf, iso_data, sizeof(iso_data));
	err = bt_iso_chan_send(chan, buf, seq_num);
	if (err < 0)
	{
		printk("Unable to broadcast data on channel %p : %d", chan, err);
		net_buf_unref(buf);
		return;
	}

	// printk("TX: seq_num: %d - payload: %d\n", seq_num, iso_send_count);

	iso_send_count++;
	seq_num++;
}

static uint32_t iso_recv_count = 0U;

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
					 struct net_buf *buf)
{
	uint32_t count = 0;
	uint8_t from_big_creator = 0;
	uint8_t bis_index = 0;

	if (buf->len == CONFIG_BT_ISO_TX_MTU)
	{
		count = sys_get_le32(buf->data);
		from_big_creator = buf->data[4];
		bis_index = buf->data[5];
	}
	if (iso_datapaths_setup)
	{
		// k_sem_give(&lc3_encoder_sem);
		iso_sent(bis[1]); // only call once to start!
	}

	iso_recv_count++;
}

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n",
		   chan, reason);

	if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST)
	{
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.sent = iso_sent,
	.recv = iso_recv,
	.connected = iso_connected,
	.disconnected = iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_rx_qos;
static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = CONFIG_BT_ISO_TX_MTU,
	.rtn = 2,
	.phy = BT_GAP_LE_PHY_2M,
};

static struct bt_iso_chan_qos bis_iso_qos = {
	.tx = &iso_tx_qos,
	.rx = &iso_rx_qos,
};

static struct bt_iso_chan bis_iso_chan[] = {
	{
		.ops = &iso_ops,
		.qos = &bis_iso_qos,
	},
	{
		.ops = &iso_ops,
		.qos = &bis_iso_qos,
	},
	{
		.ops = &iso_ops,
		.qos = &bis_iso_qos,
	},
};

static struct bt_iso_chan *bis[] = {
	&bis_iso_chan[0],
	&bis_iso_chan[1],
	&bis_iso_chan[2],
};

static struct bt_iso_big_sync_param big_sync_param = {
	.bis_channels = bis,
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_bitfield = (BIT_MASK(BIS_ISO_CHAN_COUNT)),
	.mse = BT_ISO_SYNC_MSE_ANY,
	.sync_timeout = 100, /* in 10 ms units */
};

static void reset_semaphores(void)
{
	k_sem_reset(&sem_per_adv);
	k_sem_reset(&sem_per_sync);
	k_sem_reset(&sem_per_sync_lost);
	k_sem_reset(&sem_per_big_info);
	k_sem_reset(&sem_big_sync);
	k_sem_reset(&sem_big_sync_lost);
}

// static void init_lc3_thread(void *arg1, void *arg2, void *arg3)
// {

// 	printk("Initializing lc3 encoder\n");
// 	frame_duration_us = 10000;
// 	freq_hz = 16000;
// 	octets_per_frame = 40;
// 	frames_per_sdu = 1;

// 	fill_audio_buf_sin(send_pcm_data, frame_duration_us, AUDIO_TONE_FREQUENCY_HZ, freq_hz);

// 	lc3_encoder = lc3_setup_encoder(frame_duration_us, freq_hz, 0,
// 									&lc3_encoder_mem);

// 	if (lc3_encoder == NULL)
// 	{
// 		printk("ERROR: Failed to setup LC3 encoder - wrong parameters?\n");
// 		return;
// 	}

// 	// k_sem_take(&lc3_encoder_sem, K_FOREVER);
// 	// iso_sent(bis[1]);
// }

// #define LC3_ENCODER_STACK_SIZE 4 * 4096
// #define LC3_ENCODER_PRIORITY 5

// K_THREAD_DEFINE(encoder, LC3_ENCODER_STACK_SIZE, init_lc3_thread, NULL, NULL, NULL,
// 				LC3_ENCODER_PRIORITY, 0, -1);

static inline void downmix_stereo_block_to_mono(const int16_t *stereo, int16_t *mono)
{
	for (size_t i = 0; i < FRAMES_PER_BLOCK; i++)
	{
		int32_t L = stereo[2 * i + 0];
		int32_t R = stereo[2 * i + 1];
		mono[i] = (int16_t)((L + R) / 2);
	}
}

static void rx_thread_fn(void *p1, void *p2, void *p3)
{
	int err;
	uint8_t stereo_buf[BLOCK_BYTES];
	int16_t mono_frame[MAX_NUM_SAMPLES];

	printk("RX thread: capturing mic audio...\n");
	while (1)
	{
		size_t got = sizeof(stereo_buf);
		err = i2s_buf_read(i2s_dev, stereo_buf, &got);
		if (err == 0 && got == BLOCK_BYTES)
		{
			// printk("Received MIC Data: %x %x %x %x ...\n",
			// 	   stereo_buf[0], stereo_buf[1], stereo_buf[2], stereo_buf[3]);
			/* Downmix to mono (10 ms) and queue for LC3 */
			downmix_stereo_block_to_mono((const int16_t *)stereo_buf, mono_frame);
			if (k_msgq_put(&pcm_msgq, mono_frame, K_NO_WAIT) != 0)
			{
				int16_t drop[MAX_NUM_SAMPLES];
				(void)k_msgq_get(&pcm_msgq, drop, K_NO_WAIT);
				(void)k_msgq_put(&pcm_msgq, mono_frame, K_NO_WAIT);
			}
		}
		else if (err == -EAGAIN || err == -EBUSY || err == -ETIMEDOUT)
		{
			k_usleep(200);
		}
		else
		{
			printk("i2s_buf_read error: %d\n", err);
			break;
		}


	}
}

int main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	struct bt_iso_big *big;
	uint32_t sem_timeout_us;
	int err;

	printk("Starting GRPTLK Receiver\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	/* Init codec (board support) */
	err = max9867_init();
	if (err)
	{
		printk("MAX9867 init failed: %d\n", err);
		return 0;
	}

	i2s_dev = DEVICE_DT_GET(DT_ALIAS(i2s_node0));
	if (!device_is_ready(i2s_dev))
	{
		printk("I2S not ready\n");
		return 0;
	}

	struct i2s_config rx_cfg = {
		.word_size = 16,
		.channels = CHANNELS,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE,
		.frame_clk_freq = SAMPLE_RATE_HZ,
		.mem_slab = &rx_slab,
		.block_size = BLOCK_BYTES,
		.timeout = 1000,
	};

	err = i2s_configure(i2s_dev, I2S_DIR_RX, &rx_cfg);
	if (err)
	{
		printk("I2S RX cfg failed: %d\n", err);
		return 0;
	}

	/* Start BOTH directions together (prevents nrfx I2S state errors). */
	err = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (err)
	{
		printk("I2S DIR_RX START failed: %d\n", err);
		return 0;
	}

	k_tid_t rx_tid = k_thread_create(&rx_thread, rx_stack, K_THREAD_STACK_SIZEOF(rx_stack),
									 rx_thread_fn, NULL, NULL, NULL,
									 K_PRIO_PREEMPT(4), 0, K_NO_WAIT);
	k_thread_name_set(rx_tid, "i2s_rx");

	printk("Initializing lc3 encoder\n");
	frame_duration_us = 10000;
	freq_hz = 16000;
	octets_per_frame = 40;
	frames_per_sdu = 1;

	// fill_audio_buf_sin(send_pcm_data, frame_duration_us, AUDIO_TONE_FREQUENCY_HZ, freq_hz);
	memset(send_pcm_data, 0, sizeof(send_pcm_data));

	lc3_encoder = lc3_setup_encoder(frame_duration_us, freq_hz, 0,
									&lc3_encoder_mem);

	if (lc3_encoder == NULL)
	{
		printk("ERROR: Failed to setup LC3 encoder - wrong parameters?\n");
		return -EIO;
	}

	printk("Scan callbacks register...");
	bt_le_scan_cb_register(&scan_callbacks);
	printk("success.\n");

	printk("Periodic Advertising callbacks register...");
	bt_le_per_adv_sync_cb_register(&sync_callbacks);
	printk("Success.\n");

	do
	{
		reset_semaphores();
		per_adv_lost = false;

		printk("Start scanning...");
		err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
		if (err)
		{
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		printk("Waiting for periodic advertising...\n");
		per_adv_found = false;
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err)
		{
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("Found periodic advertising.\n");

		printk("Stop scanning...");
		err = bt_le_scan_stop();
		if (err)
		{
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		printk("Creating Periodic Advertising Sync...");
		bt_addr_le_copy(&sync_create_param.addr, &per_addr);
		sync_create_param.options = 0;
		sync_create_param.sid = per_sid;
		sync_create_param.skip = 0;
		/* Multiple PA interval with retry count and convert to unit of 10 ms */
		sync_create_param.timeout = (per_interval_us * PA_RETRY_COUNT) /
									(10 * USEC_PER_MSEC);
		sem_timeout_us = per_interval_us * PA_RETRY_COUNT;
		err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
		if (err)
		{
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		printk("Waiting for periodic sync...\n");
		err = k_sem_take(&sem_per_sync, K_USEC(sem_timeout_us));
		if (err)
		{
			printk("failed (err %d)\n", err);

			printk("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err)
			{
				printk("failed (err %d)\n", err);
				return 0;
			}
			continue;
		}
		printk("Periodic sync established.\n");

		printk("Waiting for BIG info...\n");
		err = k_sem_take(&sem_per_big_info, K_USEC(sem_timeout_us));
		if (err)
		{
			printk("failed (err %d)\n", err);

			if (per_adv_lost)
			{
				continue;
			}

			printk("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err)
			{
				printk("failed (err %d)\n", err);
				return 0;
			}
			continue;
		}
		printk("Periodic sync established.\n");

	big_sync_create:
		printk("Create BIG Sync...\n");
		err = bt_iso_big_sync(sync, &big_sync_param, &big);
		if (err)
		{
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
		{
			printk("Waiting for BIG sync chan %u...\n", chan);
			err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
			if (err)
			{
				break;
			}
			printk("BIG sync chan %u successful.\n", chan);
		}
		if (err)
		{
			printk("failed (err %d)\n", err);

			printk("BIG Sync Terminate...");
			err = bt_iso_big_terminate(big);
			if (err)
			{
				printk("failed (err %d)\n", err);
				return 0;
			}
			printk("done.\n");

			goto per_sync_lost_check;
		}
		printk("BIG sync established.\n");

		for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
		{
			printk("Setting data path chan %u...\n", chan);

			const struct bt_iso_chan_path hci_path = {
				.pid = BT_ISO_DATA_PATH_HCI,
				.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
			};

			uint8_t dir = BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;

			if (chan == 0)
			{
				dir = BT_HCI_DATAPATH_DIR_CTLR_TO_HOST;
			}

			err = bt_iso_setup_data_path(bis[chan], dir, &hci_path);
			if (err != 0)
			{
				printk("Failed to setup ISO RX data path: %d\n", err);
			}

			printk("Setting data path complete chan %u.\n", chan);
		}

		iso_datapaths_setup = true;

		for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
		{
			printk("Waiting for BIG sync lost chan %u...\n", chan);
			err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
			if (err)
			{
				printk("failed (err %d)\n", err);
				return 0;
			}
			printk("BIG sync lost chan %u.\n", chan);
		}
		printk("BIG sync lost.\n");

	per_sync_lost_check:
		printk("Check for periodic sync lost...\n");
		err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
		if (err)
		{
			/* Periodic Sync active, go back to creating BIG Sync */
			goto big_sync_create;
		}
		printk("Periodic sync lost.\n");
	} while (true);
}