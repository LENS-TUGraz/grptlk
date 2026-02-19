/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/sys/byteorder.h>
#include "lc3.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include "drivers/max9867.h"

/* -------------------------------------------------------------------------- */
/*                               Configuration                                */
/* -------------------------------------------------------------------------- */

/* Active BAP Preset */
/* We use the 16_2_1 preset: 16 kHz, 10 ms framing, 40 octets per frame. 
 * This macro defines the QoS and Codec Configuration used by the stack.
 * All other audio parameters (buffer sizes, encoder setup) are derived from this.
 */
static struct bt_bap_lc3_preset preset_active =
    BT_BAP_LC3_BROADCAST_PRESET_16_2_1(BT_AUDIO_LOCATION_FRONT_LEFT |
                                           BT_AUDIO_LOCATION_FRONT_RIGHT,
                                       BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

/* Derived Audio Parameters */
/* 16_2_1 => 16000 Hz, 10000 us duration */
/* Samples per frame = (16000 * 10000) / 1000000 = 160 */
#define PCM_SAMPLES_PER_FRAME    160

/* Uplink Configuration */
/* THIS IS THE BIS ON WHICH THE GRPTLK RECEIVER TRANSMITS BACK TO THE BROADCASTER */
#define UPLINK_BIS               3

/* Scanning / Sync Configuration */
#define TIMEOUT_SYNC_CREATE      K_SECONDS(10)
#define NAME_LEN                 30
#define PA_RETRY_COUNT           6
#define BIS_ISO_CHAN_COUNT       5

#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE,    \
                                           BT_LE_SCAN_OPT_NONE,       \
                                           BT_GAP_SCAN_FAST_INTERVAL, \
                                           BT_GAP_SCAN_FAST_WINDOW)

/* Decoder Thread Configuration */
#define DECODER_STACK_SIZE       4096
#define DECODER_PRIORITY         5
#define ENCODER_STACK_SIZE       4096
#define ENCODER_PRIORITY         5

/* ISO / BAP Configuration */
/* Matches the broadcaster's explicit config or derived. */
/* Kconfig usually defines this but we can use a macro if needed for buffer sizes */
#ifndef CONFIG_BT_ISO_TX_MTU
#define CONFIG_BT_ISO_TX_MTU 40
#endif

/* -------------------------------------------------------------------------- */
/*                               Globals & Objects                            */
/* -------------------------------------------------------------------------- */

/* LC3 Decoder Objects */
static lc3_decoder_t lc3_decoder;
static lc3_decoder_mem_16k_t lc3_decoder_mem;

static int16_t recv_pcm_data[PCM_SAMPLES_PER_FRAME];

/* LC3 Encoder Objects */
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_16k_t lc3_encoder_mem;
static int16_t tx_pcm_data[PCM_SAMPLES_PER_FRAME];

/* Decoder Thread Objects */
K_THREAD_STACK_DEFINE(decoder_stack, DECODER_STACK_SIZE);

static struct k_thread decoder_thread_data;

K_THREAD_STACK_DEFINE(encoder_stack, ENCODER_STACK_SIZE);
static struct k_thread encoder_thread_data;

/* Sync / Scan Objects */
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

/* Encoded Audio FIFO (RX) */
/* Queue to decouple ISO RX interrupt from decoding logic */

K_MSGQ_DEFINE(lc3_rx_q, CONFIG_BT_ISO_TX_MTU, 4, 4);

/* Encoded Audio FIFO (TX) */
/* Queue to decouple Encoder thread from ISO TX interrupt */
K_MSGQ_DEFINE(lc3_tx_q, CONFIG_BT_ISO_TX_MTU, 4, 4);

/* ISO / BAP Objects */
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT,
                          BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
                          CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static struct bt_iso_chan *bis[BIS_ISO_CHAN_COUNT];
static struct bt_iso_chan bis_iso_chan[BIS_ISO_CHAN_COUNT];

static uint16_t seq_num;
static uint8_t iso_data[CONFIG_BT_ISO_TX_MTU] = {0};

/* -------------------------------------------------------------------------- */
/*                               Volume Control                               */
/* -------------------------------------------------------------------------- */

static const struct adc_dt_spec pot = ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), potentiometer);
static const struct gpio_dt_spec p1_09_en = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), enable_gpios);

void volume_control_thread(void *p1, void *p2, void *p3)
{
/* --- Tuning knobs --- */
#define POT_MUTE_ON_MV 100	/* mute when <= 0.10 V */
#define POT_MUTE_OFF_MV 120 /* unmute when >= 0.12 V (hysteresis) */
#define POT_MAX_MV 3300		/* clamp top of range */
#define MAX_VOL_Q15 13668	/* cap at ~50% */

	static bool muted = true;

	int err;
	int16_t sample;
	struct adc_sequence seq = {0}; 

	/* Initialize sequence from DT spec, then point it to our buffer */
	adc_sequence_init_dt(&pot, &seq);
	seq.buffer = &sample;
	seq.buffer_size = sizeof(sample);

	while (1)
	{
		err = adc_read_dt(&pot, &seq);
		if (err)
		{
			printk("adc_read_dt() failed: %d\n", err);
			k_sleep(K_MSEC(200));
			continue;
		}

		int32_t mv = sample; /* promote to 32-bit for conversion helpers */
		err = adc_raw_to_millivolts_dt(&pot, &mv);
		if (err)
		{
			printk("adc_raw_to_millivolts_dt() failed: %d (raw=%d)\n", err, sample);
			k_sleep(K_MSEC(200));
			continue;
		}

		/* --- NEW: enable pin high iff pot > 0 mV --- */
		if (mv > 0)
		{
			gpio_pin_set_dt(&p1_09_en, 1); /* enable/high */
		}
		else
		{
			gpio_pin_set_dt(&p1_09_en, 0); /* disable/low */
		}

		/* Hysteretic mute */
		if (mv <= POT_MUTE_ON_MV)
		{
			if (!muted)
			{
				max9867_set_mute(true);
				muted = true;
			}
			k_sleep(K_MSEC(100));
			continue;
		}
		else if (muted && mv >= POT_MUTE_OFF_MV)
		{
			max9867_set_mute(false);
			muted = false;
		}

		/* Scale [POT_MUTE_OFF_MV..POT_MAX_MV] -> [0..MAX_VOL_Q15] */
		if (mv > POT_MAX_MV)
			mv = POT_MAX_MV;
		int32_t span = POT_MAX_MV - POT_MUTE_OFF_MV;
		int32_t mv_rel = (mv - POT_MUTE_OFF_MV);
		uint16_t vol_q15 = (uint16_t)((mv_rel * (int32_t)MAX_VOL_Q15 + span / 2) / span);

		(void)max9867_set_volume((int16_t)vol_q15, (int16_t)MAX_VOL_Q15);

		// int vol_pct = (int)((vol_q15 * 100 + MAX_VOL_Q15 / 2) / MAX_VOL_Q15);
		// printk("pot=%ld mV  vol_q15=%u  vol=%d%%  muted=%d  en=%d\n",
		// 	   (long)mv, vol_q15, vol_pct, (int)muted,
		// 	   gpio_pin_get_dt(&p1_09_en));

		k_sleep(K_MSEC(100));
	}
}

K_THREAD_DEFINE(vol_thread_id, 1024, volume_control_thread, NULL, NULL, NULL,
				7, 0, 0);

/* -------------------------------------------------------------------------- */
/*                               Decoder Thread                               */
/* -------------------------------------------------------------------------- */

static void decoder_thread_func(void *arg1, void *arg2, void *arg3)
{
    int ret;
    uint8_t encoded_buf[CONFIG_BT_ISO_TX_MTU];
    
    /* 16_2_1 implies 40 bytes per frame */
    int octets_per_frame = preset_active.qos.sdu;

    printk("Decoder thread started\n");

    while (1) {
        /* Wait for encoded frame from ISO RX */
        ret = k_msgq_get(&lc3_rx_q, encoded_buf, K_FOREVER);
        if (ret != 0) {
            continue;
        }

        /* Decode */
        ret = lc3_decode(lc3_decoder,
                         encoded_buf,
                         octets_per_frame,
                         LC3_PCM_FORMAT_S16,
                         recv_pcm_data,
                         1);

        if (ret != 0) {
            printk("LC3 decode failed: %d\n", ret);
        } else {
            /* Successfully decoded */
            /* Print first sample to verify audio content */
            printk("Decoded Frame: %d samples. First sample: %d\n", 
                   PCM_SAMPLES_PER_FRAME, recv_pcm_data[0]);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                               Encoder Thread                               */
/* -------------------------------------------------------------------------- */

static void encoder_thread_func(void *arg1, void *arg2, void *arg3)
{
    int ret;
    uint8_t encoded_buf[CONFIG_BT_ISO_TX_MTU];
    int octets_per_frame = preset_active.qos.sdu;
    static int frame_count = 0;

    printk("Encoder thread started\n");

    while (1) {
        /* Generate Dummy Audio (Sine Wave) */
        /* 160 samples per 10ms frame at 16kHz */
        /* Simple sine wave generation */
        for (int i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
            /* Generate a tone around 400Hz */
            /* (2 * PI * 400 * t) / 16000 */
            /* Using a simplified approximation or just a counter for verification if math.h is an issue, 
               but we have math.h included in other samples. 
               Let's use a simple counter pattern that wraps to check continuity, simpler than sin/cos overhead. */
             tx_pcm_data[i] = (int16_t)((frame_count + i) & 0x3FFF); 
        }
        frame_count += PCM_SAMPLES_PER_FRAME;

        /* Encode */
        ret = lc3_encode(lc3_encoder,
                         LC3_PCM_FORMAT_S16,
                         tx_pcm_data,
                         1,
                         octets_per_frame,
                         encoded_buf);

        if (ret != 0) {
            printk("LC3 encode failed: %d\n", ret);
        } else {
            /* Push to TX Queue */
            /* We block if queue is full to throttle the encoder to the ISO interval */
            /* But actually, we should time this. 
               However, in the broadcaster, we relied on queue blocking or k_sleep.
               For this receiver, let's throttle to 10ms roughly or rely on queue backpressure.
               Queue size is 4. If we run free, we fill it instantly.
               Let's add a k_sleep(K_MSEC(10)) to mimic real audio source. */
            
            k_msgq_put(&lc3_tx_q, encoded_buf, K_FOREVER);
            k_sleep(K_USEC(10000)); /* 10ms interval */
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                               ISO Callbacks                                */
/* -------------------------------------------------------------------------- */

static void iso_connected(struct bt_iso_chan *chan) {
    printk("ISO Channel %p connected\n", chan);
    k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason) {
    printk("ISO Channel %p disconnected with reason 0x%02x\n", chan, reason);
    if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
        k_sem_give(&sem_big_sync_lost);
    }
}

static void iso_sent(struct bt_iso_chan *chan) {
    /* Uplink dummy transmission (Echo logic preserved from original sample) */
    int err;
    struct net_buf *buf;

    /* Only send on the designated Uplink BIS */
    if (chan != bis[UPLINK_BIS - 1]) {
        return;
    }

    buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
    if (!buf) {
        printk("Data buffer allocate timeout\n");
        return;
    }

    /* Send encoded audio from queue instead of dummy counter */
    // memset(iso_data, seq_num & 0xFF, sizeof(iso_data));
    
    /* Try to get encoded frame from Queue */
    /* Use K_NO_WAIT so we don't block the ISR */
    err = k_msgq_get(&lc3_tx_q, iso_data, K_NO_WAIT);
    if (err == -ENOMSG) {
        /* Queue empty - Send silence or previous data */
        /* Just zero it out to send silence if we haven't encoded fast enough */
        memset(iso_data, 0, sizeof(iso_data));
        // printk("TX Queue empty, sending silence\n");
    }

    net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
    net_buf_add_mem(buf, iso_data, CONFIG_BT_ISO_TX_MTU);

    err = bt_iso_chan_send(chan, buf, seq_num);
    if (err < 0) {
        printk("Unable to send uplink data on channel %p : %d\n", chan, err);
        net_buf_unref(buf);
        return;
    }
    seq_num++;
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
                     struct net_buf *buf) {
    /* 
     * Push received payload to decoder queue.
     * Use K_NO_WAIT to avoid blocking the ISR.
     */
    if (buf->len == CONFIG_BT_ISO_TX_MTU) {
        int ret = k_msgq_put(&lc3_rx_q, buf->data, K_NO_WAIT);
        if (ret != 0) {
             /* Queue full, dropping frame */
        }
    }

    /* Kickstart uplink upon first reception */
    if (iso_datapaths_setup) {
         /* Latch: Disable triggers to ensure we only kickstart once */
         iso_datapaths_setup = false;
         iso_sent(bis[UPLINK_BIS - 1]);
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
	.rtn = 1,
	.phy = BT_GAP_LE_PHY_2M,
};

static struct bt_iso_chan_qos bis_iso_qos = {
	.tx = &iso_tx_qos,
	.rx = &iso_rx_qos,
};

static struct bt_iso_chan bis_iso_chan[] = {
	{ .ops = &iso_ops, .qos = &bis_iso_qos },
	{ .ops = &iso_ops, .qos = &bis_iso_qos },
	{ .ops = &iso_ops, .qos = &bis_iso_qos },
	{ .ops = &iso_ops, .qos = &bis_iso_qos },
	{ .ops = &iso_ops, .qos = &bis_iso_qos },
};

static struct bt_iso_chan *bis[] = {
	&bis_iso_chan[0],
	&bis_iso_chan[1],
	&bis_iso_chan[2],
	&bis_iso_chan[3],
	&bis_iso_chan[4],
};

/* -------------------------------------------------------------------------- */
/*                               Scanning & Sync                              */
/* -------------------------------------------------------------------------- */

static struct bt_iso_big_sync_param big_sync_param = {
	.bis_channels = bis,
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_bitfield = (BIT_MASK(BIS_ISO_CHAN_COUNT)),
	.mse = BT_ISO_SYNC_MSE_ANY,
	.sync_timeout = 100, /* in 10 ms units */
};

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

static void reset_semaphores(void)
{
	k_sem_reset(&sem_per_adv);
	k_sem_reset(&sem_per_sync);
	k_sem_reset(&sem_per_sync_lost);
	k_sem_reset(&sem_per_big_info);
	k_sem_reset(&sem_big_sync);
	k_sem_reset(&sem_big_sync_lost);
}

/* -------------------------------------------------------------------------- */
/*                               Main Function                                */
/* -------------------------------------------------------------------------- */

int main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	struct bt_iso_big *big;
	uint32_t sem_timeout_us;
	int err;

	printk("Starting GRPTLK Receiver\n");

	/* --- Volume Control Setup --- */
	if (!adc_is_ready_dt(&pot))
	{
		printk("ADC device not ready\n");
		return -EIO;
	}
	err = adc_channel_setup_dt(&pot);
	if (err)
	{
		printk("adc_channel_setup_dt() failed: %d\n", err);
		return -EIO;
	}

	if (!gpio_is_ready_dt(&p1_09_en))
	{
		printk("P1.09 GPIO not ready\n");
	}
	else
	{
		gpio_pin_configure_dt(&p1_09_en, GPIO_OUTPUT_INACTIVE);
	}

	err = max9867_init();
	if (err)
	{
		printk("MAX9867 init failed: %d\n", err);
		/* Continue anyway, maybe just using internal audio or debug */
	}

	/* --- LC3 Setup --- */
	printk("Initializing lc3 decoder\n");
    memset(recv_pcm_data, 0, sizeof(recv_pcm_data));

	/* Using parameters derived from the active BAP preset */
	lc3_decoder = lc3_setup_decoder(preset_active.qos.interval, 16000, 0, &lc3_decoder_mem);

	if (lc3_decoder == NULL)
	{
		printk("ERROR: Failed to setup LC3 decoder - wrong parameters?\n");
		return -EIO;
	}

    /* Start decoder thread */
    k_thread_create(&decoder_thread_data, decoder_stack,
                    K_THREAD_STACK_SIZEOF(decoder_stack),
                    decoder_thread_func, NULL, NULL, NULL,
                    DECODER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&decoder_thread_data, "lc3_decoder");

    /* --- Encoder Setup --- */
    printk("Initializing lc3 encoder\n");
    lc3_encoder = lc3_setup_encoder(preset_active.qos.interval, 16000, 0, &lc3_encoder_mem);
    if (lc3_encoder == NULL)
    {
        printk("ERROR: Failed to setup LC3 encoder\n");
        return -EIO;
    }

    /* Start encoder thread */
    k_thread_create(&encoder_thread_data, encoder_stack,
                    K_THREAD_STACK_SIZEOF(encoder_stack),
                    encoder_thread_func, NULL, NULL, NULL,
                    ENCODER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&encoder_thread_data, "lc3_encoder");

	/* --- Bluetooth Setup --- */
	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
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