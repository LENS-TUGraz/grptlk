/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>

#include "audio_i2s.h"
#include "cs47l63.h"
#include "cs47l63_comm.h"
#include "cs47l63_reg_conf.h"
#include "lc3.h"

#if !DT_HAS_COMPAT_STATUS_OKAY(cirrus_cs47l63)
#error "No cirrus,cs47l63 node available. Use nrf5340_audio_dk/nrf5340/cpuapp."
#endif

/* -------------------------------------------------------------------------- */
/*                               Configuration                                */
/* -------------------------------------------------------------------------- */

static struct bt_bap_lc3_preset preset_active =
	BT_BAP_LC3_BROADCAST_PRESET_16_2_1(BT_AUDIO_LOCATION_FRONT_LEFT |
					       BT_AUDIO_LOCATION_FRONT_RIGHT,
					       BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

#define SAMPLE_RATE_HZ              AUDIO_I2S_SAMPLE_RATE_HZ
#define PCM_SAMPLES_PER_FRAME       AUDIO_I2S_SAMPLES_PER_BLOCK
#define TIMEOUT_SYNC_CREATE         K_SECONDS(10)
#define PA_RETRY_COUNT              6
#define BIS_ISO_CHAN_COUNT          5
#define NUM_PRIME_PACKETS           2
#define MIC_PEAK_DETECT_THRESHOLD   64

/* THIS IS THE BIS ON WHICH THE RECEIVER SENDS UPLINK AUDIO */
#define UPLINK_BIS                  3

#if (UPLINK_BIS < 2) || (UPLINK_BIS > BIS_ISO_CHAN_COUNT)
#error "UPLINK_BIS must be in [2..BIS_ISO_CHAN_COUNT] because BIS1 is downlink"
#endif

#ifndef CONFIG_BT_ISO_TX_MTU
#define CONFIG_BT_ISO_TX_MTU 40
#endif

#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, \
					   BT_LE_SCAN_OPT_NONE, \
					   BT_GAP_SCAN_FAST_INTERVAL, \
					   BT_GAP_SCAN_FAST_WINDOW)

#define DECODER_STACK_SIZE          4096
#define DECODER_PRIORITY            5
#define ENCODER_STACK_SIZE          4096
#define ENCODER_PRIORITY            5

/* -------------------------------------------------------------------------- */
/*                               Audio Objects                                */
/* -------------------------------------------------------------------------- */

static cs47l63_t codec_driver;

static uint32_t i2s_rx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_rx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];

/* Mic PCM -> LC3 encoder */
K_MSGQ_DEFINE(pcm_msgq, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 16, 4);

/* LC3 decoded downlink -> I2S TX words */
K_MSGQ_DEFINE(tx_msgq, AUDIO_I2S_BLOCK_BYTES, 8, 4);

struct lc3_frame {
	uint16_t len;
	uint8_t data[CONFIG_BT_ISO_TX_MTU];
};

/* Downlink encoded frames from BIS1 */
K_MSGQ_DEFINE(lc3_rx_q, sizeof(struct lc3_frame), 8, 4);

/* Uplink encoded frames for UPLINK_BIS */
K_MSGQ_DEFINE(lc3_tx_q, CONFIG_BT_ISO_TX_MTU, 8, 4);

static lc3_decoder_t lc3_decoder;
static lc3_decoder_mem_16k_t lc3_decoder_mem;
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_16k_t lc3_encoder_mem;

static int16_t selected_mic_channel = -1; /* -1 auto, 0 left, 1 right */

K_THREAD_STACK_DEFINE(decoder_stack, DECODER_STACK_SIZE);
static struct k_thread decoder_thread_data;

K_THREAD_STACK_DEFINE(encoder_stack, ENCODER_STACK_SIZE);
static struct k_thread encoder_thread_data;

/* -------------------------------------------------------------------------- */
/*                               BT/ISO Objects                               */
/* -------------------------------------------------------------------------- */

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

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool,
			  BIS_ISO_CHAN_COUNT * NUM_PRIME_PACKETS,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE,
			  NULL);

static struct bt_iso_chan bis_iso_chan[BIS_ISO_CHAN_COUNT];
static struct bt_iso_chan *bis[BIS_ISO_CHAN_COUNT];
static struct bt_iso_chan *sync_bis[UPLINK_BIS];

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
	/* Keep BIS indices contiguous for controller sync scheduler:
	 * sync BIS1..BIS(UPLINK_BIS), but use data paths only on BIS1 and UPLINK_BIS.
	 */
	.bis_channels = sync_bis,
	.num_bis = ARRAY_SIZE(sync_bis),
	.bis_bitfield = BIT_MASK(UPLINK_BIS),
	.mse = BT_ISO_SYNC_MSE_ANY,
	.sync_timeout = 100, /* 10 ms units */
};

/* -------------------------------------------------------------------------- */
/*                               Audio Helpers                                */
/* -------------------------------------------------------------------------- */

static inline int32_t abs_s16(int16_t s)
{
	return (s < 0) ? -(int32_t)s : (int32_t)s;
}

static int codec_reg_conf_write(const uint32_t config[][2], size_t num_of_regs)
{
	for (size_t i = 0; i < num_of_regs; i++) {
		const uint32_t reg = config[i][0];
		const uint32_t value = config[i][1];
		uint32_t ret;

		if (reg == SPI_BUSY_WAIT) {
			k_busy_wait(value);
			continue;
		}

		ret = cs47l63_write_reg(&codec_driver, reg, value);
		if (ret != CS47L63_STATUS_OK) {
			printk("CS47L63 write failed: reg=0x%08x val=0x%08x ret=%u\n",
			       reg, value, ret);
			return -EIO;
		}
	}

	return 0;
}

static int codec_audio_path_prepare(void)
{
	int err;

	err = cs47l63_comm_init(&codec_driver);
	if (err) {
		printk("cs47l63_comm_init failed: %d\n", err);
		return err;
	}

	err = codec_reg_conf_write(soft_reset, ARRAY_SIZE(soft_reset));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(clock_configuration, ARRAY_SIZE(clock_configuration));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(GPIO_configuration, ARRAY_SIZE(GPIO_configuration));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(asp1_enable, ARRAY_SIZE(asp1_enable));
	if (err) {
		return err;
	}

	/* Route on-board PDM microphone to ASP1 TX (captured on I2S RX). */
	err = codec_reg_conf_write(pdm_mic_enable_configure,
					   ARRAY_SIZE(pdm_mic_enable_configure));
	if (err) {
		return err;
	}

	/* Enable analog output path for downlink playback on headset/speaker. */
	err = codec_reg_conf_write(output_enable, ARRAY_SIZE(output_enable));
	if (err) {
		return err;
	}

	err = cs47l63_write_reg(&codec_driver, CS47L63_OUT1L_VOLUME_1,
				OUT_VOLUME_DEFAULT | VOLUME_UPDATE_BIT);
	if (err != CS47L63_STATUS_OK) {
		printk("CS47L63 output volume set failed: %d\n", err);
		return -EIO;
	}

	return 0;
}

static void pcm_msgq_push(const int16_t *mono_frame)
{
	static uint32_t drop_cnt;

	if (k_msgq_put(&pcm_msgq, mono_frame, K_NO_WAIT) != 0) {
		if ((drop_cnt++ % 200U) == 0U) {
			printk("PCM queue full - dropping mic frame (cnt=%u)\n", drop_cnt);
		}
	}
}

static inline void i2s_word_unpack(uint32_t word, int16_t *left, int16_t *right)
{
	*left = (int16_t)(word & 0xFFFF);
	*right = (int16_t)(word >> 16);
}

static void stereo_peak_analyze_words(const uint32_t *rx_words,
				      int32_t *left_peak,
				      int32_t *right_peak)
{
	int32_t l_peak = 0;
	int32_t r_peak = 0;

	for (size_t i = 0; i < AUDIO_I2S_SAMPLES_PER_BLOCK; i++) {
		int16_t left;
		int16_t right;
		int32_t la;
		int32_t ra;

		i2s_word_unpack(rx_words[i], &left, &right);
		la = abs_s16(left);
		ra = abs_s16(right);

		if (la > l_peak) {
			l_peak = la;
		}
		if (ra > r_peak) {
			r_peak = ra;
		}
	}

	*left_peak = l_peak;
	*right_peak = r_peak;
}

static void extract_selected_channel_to_mono(const uint32_t *rx_words,
					     int16_t *mono,
					     uint8_t channel)
{
	for (size_t i = 0; i < AUDIO_I2S_SAMPLES_PER_BLOCK; i++) {
		int16_t left;
		int16_t right;

		i2s_word_unpack(rx_words[i], &left, &right);
		mono[i] = (channel == 0U) ? left : right;
	}
}

static void i2s_process_rx_block(const uint32_t *rx_words)
{
	int16_t mono_frame[PCM_SAMPLES_PER_FRAME];
	int32_t left_peak;
	int32_t right_peak;
	uint8_t ch;

	stereo_peak_analyze_words(rx_words, &left_peak, &right_peak);

	if (selected_mic_channel < 0 &&
	    (left_peak > MIC_PEAK_DETECT_THRESHOLD || right_peak > MIC_PEAK_DETECT_THRESHOLD)) {
		selected_mic_channel = (right_peak > left_peak) ? 1 : 0;
		printk("MIC channel auto-selected: %s\n",
		       selected_mic_channel == 0 ? "LEFT" : "RIGHT");
	}

	if (selected_mic_channel >= 0) {
		ch = (uint8_t)selected_mic_channel;
	} else {
		ch = (right_peak > left_peak) ? 1U : 0U;
	}

	extract_selected_channel_to_mono(rx_words, mono_frame, ch);
	pcm_msgq_push(mono_frame);
}

static void tx_buffer_fill(uint32_t *tx_words)
{
	static uint32_t silence_inject_cnt;

	if (k_msgq_get(&tx_msgq, tx_words, K_NO_WAIT) != 0) {
		memset(tx_words, 0, AUDIO_I2S_BLOCK_BYTES);
		if ((silence_inject_cnt++ % 200U) == 0U) {
			printk("Downlink playback: injecting silence (cnt=%u)\n",
			       silence_inject_cnt);
		}
	}
}

static void i2s_block_complete(uint32_t *rx_buf_released, const uint32_t *tx_buf_released)
{
	static uint8_t tx_buf_sel;
	static uint32_t i2s_requeue_err_cnt;
	uint32_t *next_tx_buf;
	int err;

	ARG_UNUSED(tx_buf_released);

	if (rx_buf_released != NULL) {
		i2s_process_rx_block(rx_buf_released);
	}

	next_tx_buf = (tx_buf_sel == 0U) ? i2s_tx_buf_a : i2s_tx_buf_b;
	tx_buf_sel ^= 1U;
	tx_buffer_fill(next_tx_buf);

	err = audio_i2s_set_next_buf(rx_buf_released, next_tx_buf);
	if (err != 0) {
		if ((i2s_requeue_err_cnt++ % 50U) == 0U) {
			printk("audio_i2s_set_next_buf failed: %d (cnt=%u)\n",
			       err, i2s_requeue_err_cnt);
		}
	}
}

static int i2s_audio_start(void)
{
	int err;

	audio_i2s_blk_cb_register(i2s_block_complete);

	err = audio_i2s_init();
	if (err) {
		printk("audio_i2s_init failed: %d\n", err);
		return err;
	}

	memset(i2s_tx_buf_a, 0, sizeof(i2s_tx_buf_a));
	memset(i2s_tx_buf_b, 0, sizeof(i2s_tx_buf_b));

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

	printk("I2S RX/TX started via NRFX API (ACLK + DTS pinctrl)\n");
	return 0;
}

/* -------------------------------------------------------------------------- */
/*                           LC3 Worker Threads                               */
/* -------------------------------------------------------------------------- */

static void decoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	struct lc3_frame frame;
	int16_t mono_pcm[PCM_SAMPLES_PER_FRAME];
	uint32_t tx_words[AUDIO_I2S_WORDS_PER_BLOCK];
	const int octets_per_frame = preset_active.qos.sdu;
	static uint32_t tx_q_drop_cnt;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		int ret;

		/* Keep cadence near 10 ms; timeout drives PLC when packet is missing. */
		ret = k_msgq_get(&lc3_rx_q, &frame, K_MSEC(12));

		if (ret == 0 && frame.len > 0U) {
			ret = lc3_decode(lc3_decoder, frame.data, frame.len,
					 LC3_PCM_FORMAT_S16, mono_pcm, 1);
		} else {
			ret = lc3_decode(lc3_decoder, NULL, octets_per_frame,
					 LC3_PCM_FORMAT_S16, mono_pcm, 1);
		}

		if (ret < 0) {
			memset(mono_pcm, 0, sizeof(mono_pcm));
		}

		/* Mono downlink -> stereo output (L=R). */
		for (size_t i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
			const uint16_t s = (uint16_t)mono_pcm[i];
			tx_words[i] = ((uint32_t)s << 16) | s;
		}

		if (k_msgq_put(&tx_msgq, tx_words, K_NO_WAIT) != 0) {
			if ((tx_q_drop_cnt++ % 100U) == 0U) {
				printk("Playback queue full - dropping decoded frame (cnt=%u)\n",
				       tx_q_drop_cnt);
			}
		}
	}
}

static void encoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	int16_t mono_pcm[PCM_SAMPLES_PER_FRAME];
	uint8_t encoded_buf[CONFIG_BT_ISO_TX_MTU];
	const int octets_per_frame = preset_active.qos.sdu;
	static uint32_t tx_drop_cnt;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		int ret;

		ret = k_msgq_get(&pcm_msgq, mono_pcm, K_FOREVER);
		if (ret != 0) {
			continue;
		}

		ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, mono_pcm, 1,
				 octets_per_frame, encoded_buf);
		if (ret < 0) {
			continue;
		}

		if (k_msgq_put(&lc3_tx_q, encoded_buf, K_NO_WAIT) != 0) {
			uint8_t drop[CONFIG_BT_ISO_TX_MTU];

			/* Keep newest frame for low-latency uplink behavior. */
			(void)k_msgq_get(&lc3_tx_q, drop, K_NO_WAIT);
			(void)k_msgq_put(&lc3_tx_q, encoded_buf, K_NO_WAIT);
			if (atomic_get(&uplink_tx_active) != 0 &&
			    (tx_drop_cnt++ % 100U) == 0U) {
				printk("Uplink encoded queue full - dropping oldest (cnt=%u)\n",
				       tx_drop_cnt);
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
/*                               ISO Callbacks                                */
/* -------------------------------------------------------------------------- */

static int uplink_send_next(struct bt_iso_chan *chan)
{
	struct net_buf *buf;
	uint8_t enc_data[CONFIG_BT_ISO_TX_MTU];
	int err;

	buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
	if (!buf) {
		static uint32_t pool_err_cnt;
		if (atomic_get(&uplink_tx_active) != 0 &&
		    (pool_err_cnt++ % 50U) == 0U) {
			printk("iso_sent: net_buf pool exhausted (cnt=%u)\n", pool_err_cnt);
		}
		return -ENOMEM;
	}

	if (k_msgq_get(&lc3_tx_q, enc_data, K_NO_WAIT) != 0) {
		static uint32_t tx_underrun_cnt;
		if (atomic_get(&uplink_tx_active) != 0 &&
		    (tx_underrun_cnt++ % 100U) == 0U) {
			printk("Uplink TX underrun - sending silence (cnt=%u)\n",
			       tx_underrun_cnt);
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
	/* Only transmit uplink on selected BIS. */
	if (chan != bis[UPLINK_BIS - 1]) {
		return;
	}

	(void)uplink_send_next(chan);
}

static void iso_recv(struct bt_iso_chan *chan,
		     const struct bt_iso_recv_info *info,
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
	} else {
		/* Packet lost/invalid: decoder thread will run PLC. */
		frame.len = 0U;
	}

	(void)k_msgq_put(&lc3_rx_q, &frame, K_NO_WAIT);

	/* Kickstart the uplink on first confirmed downlink reception.
	 * This matches the iso_receive / grptlk_audio_receive pattern: the initial
	 * bt_iso_chan_send() for the BIG-member uplink must happen from within the
	 * BT ISO callback context so the controller schedules it at the correct
	 * BIG anchor event. Priming from the main thread before any packet arrives
	 * does not reliably trigger the iso_sent chain on the nRF5340. */
	if (iso_datapaths_setup) {
		iso_datapaths_setup = false;
		(void)uplink_send_next(bis[UPLINK_BIS - 1]);
	}
}

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	atomic_inc(&big_chan_connected);
	k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n", chan, reason);
	if (chan == bis[UPLINK_BIS - 1]) {
		atomic_set(&uplink_tx_active, 0);
	}

	if (reason == BT_HCI_ERR_CONN_FAIL_TO_ESTAB ||
	    reason == BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
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

/* -------------------------------------------------------------------------- */
/*                               Scan/Sync CBs                                */
/* -------------------------------------------------------------------------- */

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
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

static void sync_cb(struct bt_le_per_adv_sync *sync,
		    struct bt_le_per_adv_sync_synced_info *info)
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

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
		       const struct bt_iso_biginfo *biginfo)
{
	ARG_UNUSED(sync);
	ARG_UNUSED(biginfo);
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

/* -------------------------------------------------------------------------- */
/*                               Setup Helpers                                */
/* -------------------------------------------------------------------------- */

static void bis_channels_init(void)
{
	for (uint8_t i = 0U; i < BIS_ISO_CHAN_COUNT; i++) {
		bis_iso_chan[i].ops = &iso_ops;
		bis_iso_chan[i].qos = &bis_iso_qos;
		bis[i] = &bis_iso_chan[i];
	}

	for (uint8_t i = 0U; i < ARRAY_SIZE(sync_bis); i++) {
		sync_bis[i] = bis[i];
	}
}

static int bis_channel_index(const struct bt_iso_chan *chan)
{
	for (uint8_t i = 0U; i < BIS_ISO_CHAN_COUNT; i++) {
		if (bis[i] == chan) {
			return i;
		}
	}

	return -1;
}

static int setup_iso_datapaths(void)
{
	int err;

	for (uint8_t i = 0U; i < big_sync_param.num_bis; i++) {
		struct bt_iso_chan *chan = sync_bis[i];
		int chan_idx = bis_channel_index(chan);
		const struct bt_iso_chan_path hci_path = {
			.pid = BT_ISO_DATA_PATH_HCI,
			.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
		};
		uint8_t dir;

		if (chan_idx < 0) {
			printk("Unknown BIS channel pointer %p\n", chan);
			return -EINVAL;
		}

		/* Only configure channels actually used by this app:
		 * - BIS1 (downlink audio)
		 * - UPLINK_BIS (uplink mic)
		 */
		if ((chan != bis[0]) && (chan != bis[UPLINK_BIS - 1])) {
			printk("Skipping data path chan %d (unused)\n", chan_idx);
			continue;
		}

		printk("Setting data path chan %d...\n", chan_idx);

		/* Receiver role in group-talk:
		 * - BIS1: downlink CTLR->HOST
		 * - UPLINK_BIS: uplink HOST->CTLR
		 */
		dir = (chan == bis[0]) ? BT_HCI_DATAPATH_DIR_CTLR_TO_HOST
				       : BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;

		err = bt_iso_setup_data_path(chan, dir, &hci_path);
		if (err == -EACCES) {
			/* Controller reports "command disallowed" when a stale path is
			 * still configured on a reused BIS handle. Remove both possible
			 * directions and retry once.
			 */
			printk("Data path chan %d busy, removing stale path and retrying...\n",
			       chan_idx);
			(void)bt_iso_remove_data_path(chan, BT_HCI_DATAPATH_DIR_CTLR_TO_HOST);
			(void)bt_iso_remove_data_path(chan, BT_HCI_DATAPATH_DIR_HOST_TO_CTLR);
			err = bt_iso_setup_data_path(chan, dir, &hci_path);
		}

		if (err) {
			printk("Failed to setup ISO data path for chan %d: %d\n", chan_idx, err);
			return err;
		}

		printk("Setting data path complete chan %d.\n", chan_idx);
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

	k_thread_create(&decoder_thread_data, decoder_stack,
			K_THREAD_STACK_SIZEOF(decoder_stack), decoder_thread_func,
			NULL, NULL, NULL, DECODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&decoder_thread_data, "lc3_decoder");

	k_thread_create(&encoder_thread_data, encoder_stack,
			K_THREAD_STACK_SIZEOF(encoder_stack), encoder_thread_func,
			NULL, NULL, NULL, ENCODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&encoder_thread_data, "lc3_encoder");

	return 0;
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

	printk("Starting GRPTLK Receiver Audio DK\n");
	printk("Config: downlink=BIS1, uplink=BIS%u, synced BIS bitfield=0x%02x\n",
	       UPLINK_BIS, (unsigned int)big_sync_param.bis_bitfield);

	bis_channels_init();

	/* --- Audio setup: CS47L63 + NRFX I2S --- */
	err = codec_audio_path_prepare();
	if (err) {
		printk("Codec prepare failed: %d\n", err);
		return err;
	}

	/* Match nRF5340 Audio reference flow: toggle FLL before I2S starts. */
	err = codec_reg_conf_write(FLL_toggle, ARRAY_SIZE(FLL_toggle));
	if (err) {
		printk("Codec FLL toggle failed: %d\n", err);
		return err;
	}

	err = i2s_audio_start();
	if (err) {
		printk("I2S start failed: %d\n", err);
		return err;
	}

	err = lc3_workers_start();
	if (err) {
		return err;
	}

	/* --- Bluetooth setup --- */
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
		sync_create_param.timeout = (per_interval_us * PA_RETRY_COUNT) /
					   (10 * USEC_PER_MSEC);
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
			printk("Previous BIG channels still allocated, waiting for new BIGInfo...\n");
			goto per_sync_lost_check;
		}

		printk("Create BIG Sync...\n");
		atomic_set(&big_chan_connected, 0);
		err = bt_iso_big_sync(sync, &big_sync_param, &big);
		if (err) {
			printk("bt_iso_big_sync failed: %d\n", err);
			if (err == -EALREADY) {
				wait_bis_channels_idle(K_SECONDS(2));
				goto per_sync_lost_check;
			}
			return err;
		}

		for (uint8_t i = 0U; i < big_sync_param.num_bis; i++) {
			int chan_idx = bis_channel_index(sync_bis[i]);

			printk("Waiting for BIG sync chan %d...\n", chan_idx);
			err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
			if (err) {
				break;
			}
			printk("BIG sync chan %d successful.\n", chan_idx);
		}

		if (err || (int)atomic_get(&big_chan_connected) < (int)big_sync_param.num_bis) {
			printk("BIG sync failed (err %d, connected %d/%u)\n",
			       err, (int)atomic_get(&big_chan_connected), big_sync_param.num_bis);

			err = bt_iso_big_terminate(big);
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
			int term_err = bt_iso_big_terminate(big);

			if ((term_err != 0) && (term_err != -EINVAL) && (term_err != -EIO)) {
				printk("bt_iso_big_terminate failed: %d\n", term_err);
				return term_err;
			}
			wait_bis_channels_idle(K_SECONDS(2));
			goto per_sync_lost_check;
		}

		for (uint8_t i = 0U; i < big_sync_param.num_bis; i++) {
			int chan_idx = bis_channel_index(sync_bis[i]);

			printk("Waiting for BIG sync lost chan %d...\n", chan_idx);
			err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
			if (err) {
				printk("sem_big_sync_lost failed: %d\n", err);
				return err;
			}
			printk("BIG sync lost chan %d.\n", chan_idx);
		}
		printk("BIG sync lost.\n");

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
