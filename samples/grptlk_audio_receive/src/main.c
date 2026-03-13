#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/ring_buffer.h>

#include "audio.h"
#include "led.h"
#include "lc3.h"
#include "button.h"

#define NUM_PRIME_PACKETS 2
K_SEM_DEFINE(tx_sem, 0, NUM_PRIME_PACKETS);

/* Called from button.c work queue when PTT becomes active. */
static void on_ptt_activated(void)
{
	k_sem_give(&tx_sem);
}

/* BAP preset base: kept as-is per project requirement.
 * Runtime QoS fields are overridden below for LC3Plus 5 ms operation. */
static struct bt_bap_lc3_preset preset_active = BT_BAP_LC3_BROADCAST_PRESET_16_2_1(
	BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
	BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

/* LC3Plus 5 ms @ 16 kHz: interval=5000 us, sdu=20 bytes, rtn=1 */
static void override_preset_for_lc3plus_5ms(void)
{
	preset_active.qos.interval = 5000U;
	preset_active.qos.sdu = 20U;
	preset_active.qos.rtn = BT_ISO_BROADCAST_RTN_MAX;
}

#define SAMPLE_RATE_HZ	      AUDIO_SAMPLE_RATE_HZ
#define PCM_SAMPLES_PER_FRAME AUDIO_SAMPLES_PER_FRAME
#define BLOCK_BYTES	      AUDIO_BLOCK_BYTES
#define TIMEOUT_SYNC_CREATE   K_SECONDS(10)
#define PA_RETRY_COUNT	      6
#define BIS_ISO_CHAN_COUNT    5

#if (CONFIG_GRPTLK_UPLINK_BIS < 2) || (CONFIG_GRPTLK_UPLINK_BIS > BIS_ISO_CHAN_COUNT)
#error "CONFIG_GRPTLK_UPLINK_BIS must be in [2..BIS_ISO_CHAN_COUNT]"
#endif

#ifndef CONFIG_BT_ISO_TX_MTU
#define CONFIG_BT_ISO_TX_MTU 40
#endif

#define BT_LE_SCAN_CUSTOM                                                                          \
	BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, BT_LE_SCAN_OPT_NONE, BT_GAP_SCAN_FAST_INTERVAL,   \
			 BT_GAP_SCAN_FAST_WINDOW)

#define DECODER_STACK_SIZE 6144
#define DECODER_PRIORITY   2
#define ENCODER_STACK_SIZE 6144
#define ENCODER_PRIORITY   3

/* Downlink playback drift buffer (stereo, 320 bytes per frame) */
#define DL_RINGBUF_SIZE	     4096
#define DL_PREFILL_FRAMES    1
#define DL_LOW_WATER_FRAMES  1
#define DL_HIGH_WATER_FRAMES 4
RING_BUF_DECLARE(dl_ringbuf, DL_RINGBUF_SIZE);
static struct audio_drift_ctx dl_drift;

/* Uplink capture drift buffer (mono, 160 bytes per frame) */
#define UL_RINGBUF_SIZE	     2048
#define UL_PREFILL_FRAMES    1
#define UL_LOW_WATER_FRAMES  1
#define UL_HIGH_WATER_FRAMES 4
RING_BUF_DECLARE(ul_ringbuf, UL_RINGBUF_SIZE);
static struct audio_drift_ctx ul_drift;

struct lc3_frame {
	uint16_t len;
	uint8_t data[CONFIG_BT_ISO_TX_MTU];
	uint8_t _pad[2]; /* Pad to 44 bytes for K_MSGQ 4-byte boundary */
};

/* Downlink encoded frames from BIS1 */
K_MSGQ_DEFINE(lc3_rx_q, sizeof(struct lc3_frame), 2, 4);

/* Uplink encoded frames (mic -> LC3 -> ISO TX) */
K_MSGQ_DEFINE(lc3_tx_q, CONFIG_BT_ISO_TX_MTU, 2, 4);

static lc3_decoder_t lc3_decoder;
static lc3_decoder_mem_16k_t lc3_decoder_mem __aligned(4);
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_16k_t lc3_encoder_mem __aligned(4);

K_THREAD_STACK_DEFINE(decoder_stack, DECODER_STACK_SIZE);
static struct k_thread decoder_thread_data;

K_THREAD_STACK_DEFINE(encoder_stack, ENCODER_STACK_SIZE);
static struct k_thread encoder_thread_data;

#define TX_THREAD_STACK_SIZE 1024
#define TX_THREAD_PRIORITY   2

K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);
static struct k_thread tx_thread_data;

/* Actual number of BISes in the discovered BIG, updated from biginfo_cb. */
static uint8_t big_actual_num_bis = BIS_ISO_CHAN_COUNT;
/* BIS index (1-based) currently used for uplink TX. */
static uint8_t active_uplink_bis = CONFIG_GRPTLK_UPLINK_BIS;

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

/* Number of BIS channels that reached connected state in current BIG sync attempt. */
static atomic_t big_chan_connected;

/* Set to true after ISO data paths are set up; cleared by iso_recv() after
 * the uplink kickstart so the self-sustaining iso_sent chain begins from
 * within the BT ISO callback context (matching iso_receive / grptlk_audio_receive
 * pattern required for proprietary BIG-member TX on nRF5340). */
static bool iso_datapaths_setup;

/* Set once at least one uplink SDU was accepted for transmission. */
static atomic_t uplink_tx_active;

/* Timer to detect broadcaster disconnect (BIS1 inactivity) */
static struct k_timer bis1_activity_timer;
static struct k_work bis1_disconnect_work;
static struct bt_iso_big *grptlk_big;

static void bis1_disconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	printk("BIS1 activity timeout! Terminating BIG sync.\n");
	k_timer_stop(&bis1_activity_timer);

	if (grptlk_big != NULL) {
		int err = bt_iso_big_terminate(grptlk_big);
		if (err != 0) {
			printk("Failed to terminate BIG on timeout: %d\n", err);
		}
	}
}

static void bis1_activity_timeout_handler(struct k_timer *timer)
{
	k_work_submit(&bis1_disconnect_work);
}

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT *NUM_PRIME_PACKETS,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static struct bt_iso_chan bis_iso_chan[BIS_ISO_CHAN_COUNT];
static struct bt_iso_chan *bis[BIS_ISO_CHAN_COUNT];

static uint16_t seq_num;

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

static struct bt_iso_big_sync_param big_sync_param = {
	/* Sync all BISes so any uplink BIS can be selected at runtime. */
	.bis_channels = bis,
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_bitfield = BIT_MASK(BIS_ISO_CHAN_COUNT),
	.mse = BT_ISO_SYNC_MSE_ANY,
	.sync_timeout = 100, /* 10 ms units */
};

static void decoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	struct lc3_frame frame;
	int16_t mono_pcm[PCM_SAMPLES_PER_FRAME];
	int16_t stereo_buf[PCM_SAMPLES_PER_FRAME * AUDIO_CHANNELS];

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		int ret;

		ret = k_msgq_get(&lc3_rx_q, &frame, K_MSEC(6));

		bool got_frame = (ret == 0 && frame.len > 0U);

		/* Always advance the LC3 decoder state. On a missing/invalid frame
		 * run PLC (NULL data) — liblc3 returns ret=1 and writes synthesised
		 * concealment audio into mono_pcm. Use that output as-is; only zero
		 * on a hard decoder error (ret < 0). Silencing PLC frames discards
		 * the concealment audio and causes a click on every lost packet. */
		const uint8_t *lc3_data = got_frame ? frame.data : NULL;
		int lc3_len = got_frame ? (int)frame.len : 0;
		ret = lc3_decode(lc3_decoder, lc3_data, lc3_len, LC3_PCM_FORMAT_S16, mono_pcm, 1);
		if (ret < 0) {
			memset(mono_pcm, 0, sizeof(mono_pcm));
		}

		/* Mono downlink -> stereo output (L=R). */
		for (size_t i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
			stereo_buf[2 * i] = mono_pcm[i];
			stereo_buf[2 * i + 1] = mono_pcm[i];
		}

		audio_drift_write(&dl_drift, stereo_buf, 1);
	}
}

static void encoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	int16_t mono_pcm[PCM_SAMPLES_PER_FRAME] __aligned(4);
	uint8_t encoded_buf[CONFIG_BT_ISO_TX_MTU] __aligned(4);
	const int octets_per_frame = preset_active.qos.sdu;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		int ret;
		uint32_t read = audio_drift_read(&ul_drift, mono_pcm, 1);
		static uint32_t mic_log_count;

		if (mic_log_count < 10U || (mic_log_count % 200U) == 0U) {
			int32_t peak = 0;

			for (size_t i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
				int32_t sample = mono_pcm[i];

				if (sample < 0) {
					sample = -sample;
				}

				if (sample > peak) {
					peak = sample;
				}
			}

			printk("mic pre-lc3: read=%u peak=%ld samples=[%d,%d,%d,%d,%d,%d,%d,%d]\n",
			       read, (long)peak, mono_pcm[0], mono_pcm[1], mono_pcm[2], mono_pcm[3],
			       mono_pcm[4], mono_pcm[5], mono_pcm[6], mono_pcm[7]);
		}
		mic_log_count++;

		ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, mono_pcm, 1, octets_per_frame,
				 encoded_buf);
		if (ret < 0) {
			continue;
		}

		(void)k_msgq_put(&lc3_tx_q, encoded_buf, K_FOREVER);
	}
}

static struct bt_iso_chan *iso_select_uplink_chan(void)
{
#if defined(CONFIG_GRPTLK_UPLINK_RANDOM)
	uint8_t num_uplink = (big_actual_num_bis > 1U) ? (big_actual_num_bis - 1U) : 0U;

	if (num_uplink == 0U) {
		return NULL;
	}

	uint8_t idx = (uint8_t)(sys_rand32_get() % num_uplink);

	active_uplink_bis = idx + 1U;
	return bis[active_uplink_bis];
#else
	active_uplink_bis = CONFIG_GRPTLK_UPLINK_BIS - 1U;
	return bis[active_uplink_bis];
#endif
}

static int uplink_send_next(struct bt_iso_chan *chan);

static void tx_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		struct bt_iso_chan *ul_chan;

		k_sem_take(&tx_sem, K_FOREVER);
		ul_chan = iso_select_uplink_chan();
		if (ul_chan != NULL) {
			(void)uplink_send_next(ul_chan);
		}
	}
}

static int uplink_send_next(struct bt_iso_chan *chan)
{
	struct net_buf *buf;
	uint8_t enc_data[CONFIG_BT_ISO_TX_MTU];
	int err;

	buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
	if (!buf) {
		static uint32_t pool_err_cnt;
		if (atomic_get(&uplink_tx_active) != 0 && (pool_err_cnt++ % 50U) == 0U) {
			printk("iso_sent: net_buf pool exhausted (cnt=%u)\n", pool_err_cnt);
		}
		return -ENOMEM;
	}

	if (k_msgq_get(&lc3_tx_q, enc_data, K_NO_WAIT) != 0) {
		static uint32_t tx_underrun_cnt;
		if (atomic_get(&uplink_tx_active) != 0 && (tx_underrun_cnt++ % 100U) == 0U) {
			printk("Uplink TX underrun - sending silence (cnt=%u)\n", tx_underrun_cnt);
		}
		memset(enc_data, 0, sizeof(enc_data));
	}

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	net_buf_add_mem(buf, enc_data, sizeof(enc_data));

	err = bt_iso_chan_send(chan, buf, seq_num);
	if (err < 0) {
		printk("Unable to send uplink data on channel %p: %d\n", chan, err);
		net_buf_unref(buf);
		return err;
	}

	atomic_set(&uplink_tx_active, 1);
	seq_num++;
	return 0;
}

static void iso_sent(struct bt_iso_chan *chan)
{
	/* Re-arm TX only for uplink BISes and only while PTT is held.
	 * When PTT is released the chain starves: no more packets are sent. */
	if (chan != bis[0] && button_ptt_is_active()) {
		k_sem_give(&tx_sem);
	}
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	struct lc3_frame frame;

	/* Receiver downlink is BIS1 only. */
	if (chan != bis[0]) {
		return;
	}

	if ((buf != NULL) && (buf->len <= CONFIG_BT_ISO_TX_MTU) &&
	    ((info->flags & BT_ISO_FLAGS_VALID) != 0U)) {
		frame.len = buf->len;
		memcpy(frame.data, buf->data, buf->len);

		k_timer_start(&bis1_activity_timer, K_MSEC(500), K_NO_WAIT);
	} else {
		/* Packet lost/invalid: decoder thread will run PLC. */
		frame.len = 0U;
	}

	if (k_msgq_put(&lc3_rx_q, &frame, K_NO_WAIT) != 0) {
		struct lc3_frame dropped;
		(void)k_msgq_get(&lc3_rx_q, &dropped, K_NO_WAIT);
		(void)k_msgq_put(&lc3_rx_q, &frame, K_NO_WAIT);
	}

	/* Kickstart the uplink on first confirmed downlink reception.
	 * This matches the iso_receive / grptlk_audio_receive pattern: the initial
	 * bt_iso_chan_send() for the BIG-member uplink must happen from within the
	 * BT ISO callback context so the controller schedules it at the correct
	 * BIG anchor event. Priming from the main thread before any packet arrives
	 * does not reliably trigger the iso_sent chain on the nRF5340. */
	if (iso_datapaths_setup) {
		iso_datapaths_setup = false;
		/* Only kickstart the uplink chain if PTT is already held at sync time. */
		if (button_ptt_is_active()) {
			k_sem_give(&tx_sem);
		}
	}
}

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	atomic_inc(&big_chan_connected);
	k_sem_give(&sem_big_sync);

	if (chan == bis[0]) {
		k_timer_start(&bis1_activity_timer, K_MSEC(500), K_NO_WAIT);
	}
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n", chan, reason);
	if (chan == bis[active_uplink_bis]) {
		atomic_set(&uplink_tx_active, 0);
	}

	if (chan == bis[0]) {
		k_timer_stop(&bis1_activity_timer);
	}

	if (reason == BT_HCI_ERR_CONN_FAIL_TO_ESTAB || reason == BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
		/* Channel did not fully connect in current BIG sync attempt. */
		k_sem_give(&sem_big_sync);
	} else {
		/* Connected channel dropped later. */
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.connected = iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
	.recv = iso_recv,
};

static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
{
	ARG_UNUSED(buf);

	if (!per_adv_found && info->interval) {
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

static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
	ARG_UNUSED(sync);
	ARG_UNUSED(info);
	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	ARG_UNUSED(sync);
	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr);

	per_adv_lost = true;
	k_sem_give(&sem_per_sync_lost);
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync, const struct bt_iso_biginfo *biginfo)
{
	ARG_UNUSED(sync);
	big_actual_num_bis = MIN(biginfo->num_bis, (uint8_t)BIS_ISO_CHAN_COUNT);

	static bool printed = false;
	if (!printed) {
		printk("BIG has %u BIS(es), syncing to %u\n", biginfo->num_bis, big_actual_num_bis);
		printed = true;
	}
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

static bool bis_channels_idle(void)
{
	for (uint8_t i = 0U; i < BIS_ISO_CHAN_COUNT; i++) {
		if (bis[i]->iso != NULL) {
			return false;
		}
	}

	return true;
}

static void wait_bis_channels_idle(k_timeout_t timeout)
{
	int64_t deadline_ms = k_uptime_get() + k_ticks_to_ms_floor64(timeout.ticks);

	while (!bis_channels_idle()) {
		if (k_uptime_get() >= deadline_ms) {
			break;
		}

		/* Prefer consuming disconnect notifications if available. */
		(void)k_sem_take(&sem_big_sync_lost, K_MSEC(100));
	}
}

static void bis_channels_init(void)
{
	for (uint8_t i = 0U; i < BIS_ISO_CHAN_COUNT; i++) {
		bis_iso_chan[i].ops = &iso_ops;
		bis_iso_chan[i].qos = &bis_iso_qos;
		bis[i] = &bis_iso_chan[i];
	}
}

static int setup_iso_datapaths(void)
{
	int err;

	for (uint8_t i = 0U; i < big_sync_param.num_bis; i++) {
		struct bt_iso_chan *chan = bis[i];
		const struct bt_iso_chan_path hci_path = {
			.pid = BT_ISO_DATA_PATH_HCI,
			.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
		};
		/* BIS1 (index 0): downlink CTLR->HOST; all others: uplink HOST->CTLR */
		uint8_t dir = (i == 0U) ? BT_HCI_DATAPATH_DIR_CTLR_TO_HOST
					: BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;

		err = bt_iso_setup_data_path(chan, dir, &hci_path);
		if (err == -EACCES) {
			/* Controller reports "command disallowed" when a stale path is
			 * still configured on a reused BIS handle. Remove both possible
			 * directions and retry once.
			 */
			printk("Data path chan %u busy, removing stale path and retrying...\n", i);
			(void)bt_iso_remove_data_path(chan, BT_HCI_DATAPATH_DIR_CTLR_TO_HOST);
			(void)bt_iso_remove_data_path(chan, BT_HCI_DATAPATH_DIR_HOST_TO_CTLR);
			err = bt_iso_setup_data_path(chan, dir, &hci_path);
		}

		if (err) {
			printk("Failed to setup ISO data path for chan %u: %d\n", i, err);
			return err;
		}
	}

	seq_num = 0U;
	atomic_set(&uplink_tx_active, 0);
	k_msgq_purge(&lc3_tx_q);

	/* Signal iso_recv() to kickstart the uplink on first confirmed downlink
	 * reception.  Do NOT prime here from the main thread: the nRF5340
	 * controller requires the first bt_iso_chan_send() on a BIG-member uplink
	 * channel to happen from within the BT ISO callback context so it can be
	 * scheduled at the correct BIG anchor event. */
	iso_datapaths_setup = true;

	return 0;
}

static int lc3_workers_start(void)
{
	lc3_decoder =
		lc3_setup_decoder(preset_active.qos.interval, SAMPLE_RATE_HZ, 0, &lc3_decoder_mem);
	if (lc3_decoder == NULL) {
		printk("ERROR: Failed to setup LC3 decoder\n");
		return -EIO;
	}

	lc3_encoder =
		lc3_setup_encoder(preset_active.qos.interval, SAMPLE_RATE_HZ, 0, &lc3_encoder_mem);
	if (lc3_encoder == NULL) {
		printk("ERROR: Failed to setup LC3 encoder\n");
		return -EIO;
	}

	k_thread_create(&decoder_thread_data, decoder_stack, K_THREAD_STACK_SIZEOF(decoder_stack),
			decoder_thread_func, NULL, NULL, NULL, DECODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&decoder_thread_data, "lc3_decoder");

	k_thread_create(&encoder_thread_data, encoder_stack, K_THREAD_STACK_SIZEOF(encoder_stack),
			encoder_thread_func, NULL, NULL, NULL, ENCODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&encoder_thread_data, "lc3_encoder");

	k_thread_create(&tx_thread_data, tx_thread_stack, K_THREAD_STACK_SIZEOF(tx_thread_stack),
			tx_thread, NULL, NULL, NULL, TX_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "iso_tx");

	return 0;
}

int main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	uint32_t sem_timeout_us;
	int err;
	int led_err;

	override_preset_for_lc3plus_5ms();

	printk("Starting GRPTLK Receiver\n");
#if defined(CONFIG_GRPTLK_UPLINK_RANDOM)
	printk("Config: downlink=BIS1, uplink=random from BIS2..BIS%u\n", BIS_ISO_CHAN_COUNT);
#else
	printk("Config: downlink=BIS1, uplink=static BIS%u\n", CONFIG_GRPTLK_UPLINK_BIS);
#endif

	led_err = led_init();
	if (led_err) {
		printk("led_init failed: %d\n", led_err);
	} else {
		(void)led_set_receiver_synced(false);
	}

	(void)button_init(audio_volume_adjust, on_ptt_activated);

	k_timer_init(&bis1_activity_timer, bis1_activity_timeout_handler, NULL);
	k_work_init(&bis1_disconnect_work, bis1_disconnect_work_handler);

	bis_channels_init();

	audio_drift_init(&dl_drift, &dl_ringbuf, BLOCK_BYTES, DL_PREFILL_FRAMES,
			 DL_LOW_WATER_FRAMES, DL_HIGH_WATER_FRAMES);
	audio_drift_init(&ul_drift, &ul_ringbuf, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME,
			 UL_PREFILL_FRAMES, UL_LOW_WATER_FRAMES, UL_HIGH_WATER_FRAMES);

	err = audio_init(&dl_drift, &ul_drift);
	if (err) {
		printk("audio_init failed: %d\n", err);
		return err;
	}

	err = audio_start();
	if (err) {
		printk("audio_start failed: %d\n", err);
		return err;
	}

	err = lc3_workers_start();
	if (err) {
		return err;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	bt_le_scan_cb_register(&scan_callbacks);
	bt_le_per_adv_sync_cb_register(&sync_callbacks);

	do {
		reset_semaphores();
		per_adv_lost = false;
		if (led_err == 0) {
			(void)led_set_receiver_synced(false);
		}

		printk("Start scanning...\n");
		err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
		if (err) {
			printk("bt_le_scan_start failed: %d\n", err);
			return err;
		}

		printk("Waiting for periodic advertising...\n");
		per_adv_found = false;
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err) {
			printk("sem_per_adv failed: %d\n", err);
			return err;
		}

		printk("Found periodic advertising.\n");
		err = bt_le_scan_stop();
		if (err) {
			printk("bt_le_scan_stop failed: %d\n", err);
			return err;
		}

		bt_addr_le_copy(&sync_create_param.addr, &per_addr);
		sync_create_param.options = 0;
		sync_create_param.sid = per_sid;
		sync_create_param.skip = 0;
		sync_create_param.timeout =
			(per_interval_us * PA_RETRY_COUNT) / (10 * USEC_PER_MSEC);
		sem_timeout_us = per_interval_us * PA_RETRY_COUNT;

		printk("Creating Periodic Advertising Sync...\n");
		err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
		if (err) {
			printk("bt_le_per_adv_sync_create failed: %d\n", err);
			return err;
		}

		printk("Waiting for periodic sync...\n");
		err = k_sem_take(&sem_per_sync, K_USEC(sem_timeout_us));
		if (err) {
			printk("Periodic sync timeout\n");
			(void)bt_le_per_adv_sync_delete(sync);
			continue;
		}

		printk("Waiting for BIG info...\n");
		err = k_sem_take(&sem_per_big_info, K_USEC(sem_timeout_us));
		if (err) {
			printk("BIG info timeout\n");
			if (!per_adv_lost) {
				(void)bt_le_per_adv_sync_delete(sync);
			}
			continue;
		}

big_sync_create:
		if (!bis_channels_idle()) {
			printk("Waiting for previous BIG channel cleanup...\n");
			wait_bis_channels_idle(K_SECONDS(2));
		}

		if (!bis_channels_idle()) {
			printk("Previous BIG channels still allocated, waiting for new "
			       "BIGInfo...\n");
			goto per_sync_lost_check;
		}

		big_sync_param.num_bis = big_actual_num_bis;
		big_sync_param.bis_bitfield = BIT_MASK(big_actual_num_bis);
		printk("Create BIG Sync (num_bis=%u, bitfield=0x%x)...\n", big_sync_param.num_bis,
		       big_sync_param.bis_bitfield);
		atomic_set(&big_chan_connected, 0);
		err = bt_iso_big_sync(sync, &big_sync_param, &grptlk_big);
		if (err) {
			printk("bt_iso_big_sync failed: %d\n", err);
			if (err == -EALREADY) {
				wait_bis_channels_idle(K_SECONDS(2));
				goto per_sync_lost_check;
			}
			return err;
		}

		for (uint8_t i = 0U; i < big_actual_num_bis; i++) {
			printk("Waiting for BIG sync chan %u...\n", i);
			err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
			if (err) {
				break;
			}
			printk("BIG sync chan %u successful.\n", i);
		}

		if (err || (int)atomic_get(&big_chan_connected) < (int)big_actual_num_bis) {
			printk("BIG sync failed (err %d, connected %d/%u)\n", err,
			       (int)atomic_get(&big_chan_connected), big_actual_num_bis);

			err = bt_iso_big_terminate(grptlk_big);
			if ((err != 0) && (err != -EINVAL) && (err != -EIO)) {
				printk("bt_iso_big_terminate failed: %d\n", err);
				return err;
			}
			wait_bis_channels_idle(K_SECONDS(2));
			goto per_sync_lost_check;
		}

		printk("BIG sync established.\n");
		err = setup_iso_datapaths();
		if (err) {
			int term_err = bt_iso_big_terminate(grptlk_big);

			if ((term_err != 0) && (term_err != -EINVAL) && (term_err != -EIO)) {
				printk("bt_iso_big_terminate failed: %d\n", term_err);
				return term_err;
			}
			wait_bis_channels_idle(K_SECONDS(2));
			goto per_sync_lost_check;
		}
		if (led_err == 0) {
			int led_set_err = led_set_receiver_synced(true);

			if (led_set_err) {
				printk("led_set_receiver_synced(true) failed: %d\n", led_set_err);
			}
		}

		for (uint8_t i = 0U; i < big_actual_num_bis; i++) {
			printk("Waiting for BIG sync lost chan %u...\n", i);
			err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
			if (err) {
				printk("sem_big_sync_lost failed: %d\n", err);
				return err;
			}
			printk("BIG sync lost chan %u.\n", i);
		}
		printk("BIG sync lost.\n");
		if (led_err == 0) {
			int led_set_err = led_set_receiver_synced(false);

			if (led_set_err) {
				printk("led_set_receiver_synced(false) failed: %d\n", led_set_err);
			}
		}

per_sync_lost_check:
		printk("Check for periodic sync lost...\n");
		err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
		if (err) {
			printk("Waiting for fresh BIGInfo...\n");
			k_sem_reset(&sem_per_big_info);
			err = k_sem_take(&sem_per_big_info, K_SECONDS(5));
			if (err == 0) {
				goto big_sync_create;
			}

			printk("No BIGInfo, waiting for PA sync to terminate...\n");
			(void)k_sem_take(&sem_per_sync_lost, K_FOREVER);
		}

		printk("Periodic sync lost.\n");
	} while (true);
}
