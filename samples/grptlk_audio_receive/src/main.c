/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * GRPTLK Audio Receiver — main.c
 *
 * Overview
 * --------
 * This file wires together the BLE BIG receiver, the LC3 codec threads, and
 * the audio backend (CS47L63 on nRF5340 Audio DK).
 *
 * Audio data flows — Phase 1 (lock-free rings, LC3 in threads)
 * -------------------------------------------------------------
 *
 * DOWNLINK  BIS1 → speaker
 * ~~~~~~~~~~~~~~~~~~~~~~~~
 *  BLE radio (every 5 ms BIG anchor)
 *    └─ iso_recv() [BT ISO callback]
 *         writes LC3 frame into downlink_ring  [lock-free, 3 slots, no mutex]
 *    └─ lc3_decoder thread  [prio 1, 6144 B stack]
 *         reads downlink_ring → lc3_decode() → mono→stereo upmix
 *         → k_msgq_put(tx_msgq)  [depth 1, 320 bytes/item]
 *    └─ DMA ISR (i2s_block_complete, every 5 ms)
 *         k_msgq_get(tx_msgq, K_NO_WAIT) → I2S TX DMA → CS47L63 DAC → speaker
 *
 *  Packet loss: iso_recv() writes a zero-length slot into the ring.
 *  The decoder thread calls lc3_decode(NULL) which runs PLC — no click.
 *  Timeout guard: if the ring is empty for >5.5 ms the decoder also runs PLC.
 *
 * UPLINK  microphone → BIS2..5
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  CS47L63 PDM mic → I2S DMA RX (every 5 ms)
 *    └─ DMA ISR: i2s_process_rx_block() → audio_rx_mono_frame() [rx_cb]
 *         writes mono PCM into uplink_ring  [lock-free, 3 slots, no mutex]
 *    └─ lc3_encoder thread  [prio 2, 4096 B stack]
 *         reads uplink_ring → lc3_encode()
 *         → k_msgq_put(lc3_tx_q)  [depth 2, 20 bytes/item, drop-oldest]
 *    └─ iso_tx thread  [prio 2, 1024 B stack]
 *         k_sem_take(tx_sem) ← armed by iso_sent() or kickstart in iso_recv()
 *         bt_iso_chan_send(uplink_bis, encoded_frame)
 *    └─ iso_sent() [BT ISO callback]
 *         re-arms tx_sem while ptt_active — self-sustaining send chain
 *
 * Why lock-free rings instead of K_MSGQ for mic and downlink
 * -----------------------------------------------------------
 * K_MSGQ uses an internal spinlock and copies data under that lock.  The DMA
 * ISR calls into K_MSGQ at high interrupt priority — safe, but adds latency
 * and jitter from the lock acquisition.
 *
 * The lock-free dual-slot ring (2 atomic slots, write_idx / read_idx) avoids
 * all locking:
 *   - Producer writes to slot[write_idx % 2], then increments write_idx.
 *   - Consumer sees write_idx > read_idx, reads slot[read_idx % 2], increments.
 *   - With 2 slots the producer is always one frame ahead without aliasing.
 *   - Only atomic operations — no spinlock, no context-switch, ISR-safe.
 *
 * tx_msgq (decoder → DMA ISR) stays a K_MSGQ because:
 *   - The DMA ISR uses K_NO_WAIT (never blocks), so latency is minimal.
 *   - The decoder thread uses K_NO_WAIT + drop-oldest, matching the old design.
 *   - Keeping it as K_MSGQ makes the silence-inject path in cs47l63.c simple.
 *
 * lc3_tx_q (encoder → iso_tx thread) stays a K_MSGQ because:
 *   - iso_tx blocks on tx_sem, not on the queue — it only queue-gets after
 *     the semaphore fires, so there is no ISR involvement at all.
 *
 * Latency improvement vs the original K_MSGQ design
 * --------------------------------------------------
 *  Before: iso_recv() → k_msgq_put(lc3_rx_q) → wake decoder thread
 *          → k_msgq_get → lc3_decode → k_msgq_put(tx_msgq) → DMA ISR
 *          Worst case buffering: 2 × 5 ms (queue depth) + scheduling jitter
 *
 *  After:  iso_recv() → ring_put (atomic, no lock) → decoder thread wakes
 *          → ring_get → lc3_decode → k_msgq_put(tx_msgq) → DMA ISR
 *          Worst case buffering: 1 × 5 ms (ring depth 2, one slot latency)
 *          Scheduling jitter: unchanged (thread woken by semaphore)
 *
 * PTT (push-to-talk)
 * ------------------
 * ptt_active (io/buttons.c) gates the uplink chain:
 *   0 = iso_sent() does not re-arm tx_sem → chain starves → no uplink TX
 *   1 = iso_sent() re-arms tx_sem → chain self-sustains
 *
 * Thread summary
 * --------------
 * Name          Priority  Stack    Blocking on
 * clk_sync         1      1024 B   clk_sync_sem (every BIG anchor, 5 ms)
 * lc3_decoder      1      6144 B   downlink_ring sem (5.5 ms timeout → PLC)
 * lc3_encoder      2      4096 B   uplink_ring sem (forever)
 * iso_tx           2      1024 B   tx_sem (forever)
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/net_buf.h>

#include "audio/audio.h"
#include "audio/drivers/audio_i2s.h"
#include "audio/sync/clk_sync.h"
#include "io/buttons.h"
#include "io/led.h"
#include "lc3.h"

/* =========================================================================
 * Constants and configuration
 * ========================================================================= */

/* BIG topology: BIS1 = downlink (broadcaster→receiver), BIS2..5 = uplink. */
#define BIS_ISO_CHAN_COUNT 5

/* Compile-time sizing aliases — used for static buffers, msgq item size, and LC3 setup.
 * Sample rate stays fixed at 16 kHz; frame duration and SDU size are derived at runtime
 * from BIGInfo (big_sdu_interval_us / big_max_sdu) and applied to preset_active.qos. */
#define SAMPLE_RATE_HZ        AUDIO_SAMPLE_RATE_HZ
#define PCM_SAMPLES_PER_FRAME AUDIO_SAMPLES_PER_FRAME
#define BLOCK_BYTES           AUDIO_BLOCK_BYTES

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define PA_RETRY_COUNT      6

#ifndef CONFIG_BT_ISO_TX_MTU
#define CONFIG_BT_ISO_TX_MTU 40
#endif

#if (CONFIG_GRPTLK_UPLINK_BIS < 2) || (CONFIG_GRPTLK_UPLINK_BIS > BIS_ISO_CHAN_COUNT)
#error "CONFIG_GRPTLK_UPLINK_BIS must be in [2..BIS_ISO_CHAN_COUNT]"
#endif

/* LC3 worker thread parameters. */
#define DECODER_STACK_SIZE   6144
#define DECODER_PRIORITY     1
#define ENCODER_STACK_SIZE   4096
#define ENCODER_PRIORITY     2
#define TX_THREAD_STACK_SIZE 1024
#define TX_THREAD_PRIORITY   2

/* Custom BLE scan params: active scan, fast interval/window. */
#define BT_LE_SCAN_CUSTOM \
	BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, BT_LE_SCAN_OPT_NONE, \
			 BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW)

/* =========================================================================
 * Lock-free triple-slot ring buffers
 * =========================================================================
 *
 * Each ring has RING_SLOTS=2 slots.  One slot is "being written" (producer),
 * one is "being read" (consumer).
 * With 2 slots, producer and consumer never share a slot — no lock needed.
 *
 * Protocol:
 *   write_idx  — producer increments after writing each slot
 *   read_idx   — consumer reads slot[read_idx % 2] when write_idx > read_idx
 *   Both are atomic_t so the increment is visible across ISR/thread boundary.
 *
 * downlink_ring: iso_recv() [BT callback] → lc3_decoder thread
 *   Slot holds one LC3-encoded frame (≤20 bytes) + its length.
 *   len=0 means packet loss → decoder calls lc3_decode(NULL) → PLC.
 *
 * uplink_ring: DMA ISR (i2s_process_rx_block) → lc3_encoder thread
 *   Slot holds one raw mono PCM frame (80 × int16 = 160 bytes).
 *
 * A semaphore accompanies each ring so the consumer thread can sleep when
 * the ring is empty rather than busy-polling.  The producer gives the sem
 * after each write; the consumer takes it before each read.
 */

#define RING_SLOTS 2

/* --- Downlink ring (BLE → decoder thread) --------------------------------- */

struct dl_slot {
	uint16_t len;                      /* 0 = loss/PLC */
	uint8_t  data[CONFIG_BT_ISO_TX_MTU];
};

static struct dl_slot  downlink_slots[RING_SLOTS];
static atomic_t        downlink_write_idx = ATOMIC_INIT(0);
static atomic_t        downlink_read_idx  = ATOMIC_INIT(0);
static K_SEM_DEFINE(   downlink_sem, 0, RING_SLOTS);

static void downlink_ring_put(const uint8_t *data, uint16_t len)
{
	atomic_val_t   idx  = atomic_get(&downlink_write_idx);
	struct dl_slot *slot = &downlink_slots[idx % RING_SLOTS];

	slot->len = len;
	if (len > 0U && data != NULL) {
		memcpy(slot->data, data, len);
	}
	atomic_set(&downlink_write_idx, idx + 1);
	k_sem_give(&downlink_sem);
}

/* Called from the decoder thread.  Blocks up to timeout waiting for a slot. */
static bool downlink_ring_get(struct dl_slot *out, k_timeout_t timeout)
{
	if (k_sem_take(&downlink_sem, timeout) != 0) {
		return false; /* timeout → caller runs PLC */
	}
	atomic_val_t ridx = atomic_get(&downlink_read_idx);

	*out = downlink_slots[ridx % RING_SLOTS];
	atomic_set(&downlink_read_idx, ridx + 1);
	return true;
}

/* --- Uplink ring (DMA ISR → encoder thread) ------------------------------- */

static int16_t  uplink_slots[RING_SLOTS][PCM_SAMPLES_PER_FRAME];
static atomic_t uplink_write_idx = ATOMIC_INIT(0);
static atomic_t uplink_read_idx  = ATOMIC_INIT(0);
static K_SEM_DEFINE(uplink_sem, 0, RING_SLOTS);

/* Called from DMA ISR — ISR-safe (atomic + memcpy only). */
static void uplink_ring_put(const int16_t *mono_frame)
{
	atomic_val_t idx = atomic_get(&uplink_write_idx);

	memcpy(uplink_slots[idx % RING_SLOTS], mono_frame,
	       sizeof(int16_t) * PCM_SAMPLES_PER_FRAME);
	atomic_set(&uplink_write_idx, idx + 1);
	k_sem_give(&uplink_sem);
}

/* Called from the encoder thread — blocks forever waiting for mic data. */
static void uplink_ring_get(int16_t *out)
{
	k_sem_take(&uplink_sem, K_FOREVER);

	atomic_val_t ridx = atomic_get(&uplink_read_idx);

	memcpy(out, uplink_slots[ridx % RING_SLOTS],
	       sizeof(int16_t) * PCM_SAMPLES_PER_FRAME);
	atomic_set(&uplink_read_idx, ridx + 1);
}

/* =========================================================================
 * Decoded stereo PCM queue: decoder thread → DMA ISR
 *
 * Still a K_MSGQ because the DMA ISR consumes it with K_NO_WAIT (no block),
 * and the silence-inject fallback in cs47l63.c is cleaner with a queue.
 * ========================================================================= */
K_MSGQ_DEFINE(tx_msgq, BLOCK_BYTES, 1, 4);

/* =========================================================================
 * Encoded uplink queue: encoder thread → iso_tx thread
 *
 * Depth=1: safe once clk_sync has locked HFCLKAUDIO to the BLE anchor.
 * After lock the encoder (DMA-clocked) and iso_tx (BIG-anchor-clocked) run
 * from the same effective oscillator (±5 µs), so one slot is sufficient.
 * During the ~10 s convergence window the drop-oldest policy discards the
 * stale frame silently; iso_tx falls back to silence on underrun.
 * ========================================================================= */
K_MSGQ_DEFINE(lc3_tx_q, CONFIG_BT_ISO_TX_MTU, 1, 4);

/* =========================================================================
 * Uplink TX semaphore
 * ========================================================================= */
#define NUM_PRIME_PACKETS 2
K_SEM_DEFINE(tx_sem, 0, NUM_PRIME_PACKETS);

/* =========================================================================
 * LC3 codec state
 * ========================================================================= */

/* BAP preset base — QoS fields overridden at runtime for LC3Plus 5 ms. */
static struct bt_bap_lc3_preset preset_active = BT_BAP_LC3_BROADCAST_PRESET_16_2_1(
	BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
	BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

#define GRPTLK_VENDOR_CODEC_ID   BT_HCI_CODING_FORMAT_VS
#define GRPTLK_VENDOR_COMPANY_ID 0xDEAD
#define GRPTLK_VENDOR_VENDOR_ID  0xBEEF

static void override_preset_for_lc3plus_5ms(void)
{
	preset_active.qos.interval = 5000U;
	preset_active.qos.sdu      = 20U;
	preset_active.qos.rtn      = 2U;
	/* qos.interval/qos.sdu are updated from BIGInfo before BIG sync. The
	 * broadcast codec identity is logged from the BASE in periodic
	 * advertising. */
}

static lc3_decoder_t lc3_decoder;
static lc3_decoder_mem_16k_t lc3_decoder_mem;
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_16k_t lc3_encoder_mem;

/* =========================================================================
 * BLE ISO / BIG state
 * ========================================================================= */

static uint8_t  big_actual_num_bis  = BIS_ISO_CHAN_COUNT;
static uint8_t  active_uplink_bis   = CONFIG_GRPTLK_UPLINK_BIS - 1U;
/* Populated in biginfo_cb; used to configure LC3 and I2S after BIG info arrives. */
static uint32_t big_sdu_interval_us = 5000U;  /* frame duration µs — default 5 ms */
static uint16_t big_max_sdu         = 20U;    /* octets per frame  — default 20 B  */

static bool         per_adv_found;
static bool         per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t      per_sid;
static uint32_t     per_interval_us;
static atomic_t     base_logged;
static atomic_t     base_parse_warned;

static K_SEM_DEFINE(sem_per_adv,       0, 1);
static K_SEM_DEFINE(sem_per_sync,      0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_per_big_info,  0, 1);
static K_SEM_DEFINE(sem_big_sync,      0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_sync_lost, 0, BIS_ISO_CHAN_COUNT);

static atomic_t big_chan_connected;
static atomic_t uplink_tx_active;

/* Set true after ISO data paths are configured.  Cleared by iso_recv() when
 * it fires the uplink kickstart.  The first bt_iso_chan_send() on a BIG-member
 * uplink channel must happen from within the BT ISO callback context so the
 * nRF5340 controller schedules it at the correct BIG anchor event. */
static bool iso_datapaths_setup;

static struct bt_iso_big *grptlk_big;

/* -------------------------------------------------------------------------
 * BIS1 inactivity watchdog
 * ------------------------------------------------------------------------- */
static struct k_timer bis1_activity_timer;
static struct k_work  bis1_disconnect_work;

static void bis1_disconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	printk("BIS1 activity timeout — terminating BIG sync\n");
	k_timer_stop(&bis1_activity_timer);
	if (grptlk_big != NULL) {
		int err = bt_iso_big_terminate(grptlk_big);

		if (err != 0) {
			printk("bt_iso_big_terminate failed: %d\n", err);
		}
	}
}

static void bis1_activity_timeout_handler(struct k_timer *timer)
{
	k_work_submit(&bis1_disconnect_work);
}

/* =========================================================================
 * ISO channel objects
 * ========================================================================= */

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT * NUM_PRIME_PACKETS,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static struct bt_iso_chan     bis_iso_chan[BIS_ISO_CHAN_COUNT];
static struct bt_iso_chan    *bis[BIS_ISO_CHAN_COUNT];
static uint16_t               seq_num;

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
	.bis_channels = bis,
	.num_bis      = BIS_ISO_CHAN_COUNT,
	.bis_bitfield = BIT_MASK(BIS_ISO_CHAN_COUNT),
	.mse          = 1,
	.sync_timeout = 100,
};

/* =========================================================================
 * Downlink audio path: BIS1 → downlink_ring → lc3_decoder → tx_msgq → speaker
 * ========================================================================= */

/* Called from the DMA ISR every 5 ms with a captured mono frame.
 * Writes into the uplink ring (lock-free) — no blocking, ISR-safe. */
static void audio_rx_mono_frame(const int16_t *mono_frame)
{
	uplink_ring_put(mono_frame);
}

/* lc3_decoder thread: reads the downlink ring, decodes (or PLC on loss/timeout),
 * upmixes mono→stereo, and feeds tx_msgq for the I2S DMA. */
static void decoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	struct dl_slot slot;
	int16_t mono_pcm[PCM_SAMPLES_PER_FRAME];
	int16_t stereo_buf[PCM_SAMPLES_PER_FRAME * AUDIO_CHANNELS];
	static uint32_t rx_frame_cnt;
	static uint32_t rx_plc_cnt;     /* frames decoded via PLC (no real packet) */
	static uint32_t rx_dec_err_cnt; /* lc3_decode() hard failures */
	static uint32_t rx_drop_cnt;    /* tx_msgq drop-oldest (decoder > DMA rate) */

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		int ret;

		/* Block up to 5.5 ms waiting for the next BLE frame.
		 * If nothing arrives (packet loss or anchor jitter) run PLC. */
		bool got_frame = downlink_ring_get(&slot, K_USEC(5500));

		const uint8_t *lc3_data = (got_frame && slot.len > 0U) ? slot.data : NULL;
		const int      lc3_len  = (got_frame && slot.len > 0U) ? (int)slot.len : 0;

		if (lc3_data == NULL) {
			rx_plc_cnt++;
		}

		/* lc3_decode(NULL, 0, ...) runs PLC — smooth concealment, no click. */
		ret = lc3_decode(lc3_decoder, lc3_data, lc3_len,
				 LC3_PCM_FORMAT_S16, mono_pcm, 1);
		if (ret < 0) {
			memset(mono_pcm, 0, sizeof(mono_pcm));
			rx_dec_err_cnt++;
		}

		/* Mono downlink → stereo (L = R). */
		for (size_t i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
			stereo_buf[2 * i]     = mono_pcm[i];
			stereo_buf[2 * i + 1] = mono_pcm[i];
		}

		/* Drop-oldest policy: always give the DMA the freshest frame. */
		if (k_msgq_put(&tx_msgq, stereo_buf, K_NO_WAIT) != 0) {
			uint32_t dropped[PCM_SAMPLES_PER_FRAME];

			(void)k_msgq_get(&tx_msgq, dropped, K_NO_WAIT);
			(void)k_msgq_put(&tx_msgq, stereo_buf, K_NO_WAIT);
			rx_drop_cnt++;
		}

		if ((rx_frame_cnt++ % 200U) == 0U) {
			printk("[rx] frame=%u plc=%u err=%u drop=%u tx_q=%u locked=%d\n",
			       rx_frame_cnt, rx_plc_cnt, rx_dec_err_cnt, rx_drop_cnt,
			       k_msgq_num_used_get(&tx_msgq),
			       clk_sync_is_locked() ? 1 : 0);
			rx_plc_cnt     = 0;
			rx_dec_err_cnt = 0;
			rx_drop_cnt    = 0;
		}
	}
}

/* =========================================================================
 * Uplink audio path: mic → uplink_ring → lc3_encoder → lc3_tx_q → BIS2..5
 * ========================================================================= */

/* lc3_encoder thread: reads the uplink ring, encodes, feeds lc3_tx_q. */
static void encoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	int16_t mono_pcm[PCM_SAMPLES_PER_FRAME];
	uint8_t encoded_buf[CONFIG_BT_ISO_TX_MTU];
	const int octets_per_frame = preset_active.qos.sdu;
	static uint32_t enc_frame_cnt;
	static uint32_t enc_drop_cnt; /* lc3_tx_q overflow events */

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		int ret;

		/* Block until the DMA ISR puts a new mic frame into the ring. */
		uplink_ring_get(mono_pcm);

		ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, mono_pcm, 1,
				 octets_per_frame, encoded_buf);
		if (ret < 0) {
			continue;
		}

		if (k_msgq_put(&lc3_tx_q, encoded_buf, K_NO_WAIT) != 0) {
			uint8_t drop[CONFIG_BT_ISO_TX_MTU];

			(void)k_msgq_get(&lc3_tx_q, drop, K_NO_WAIT);
			(void)k_msgq_put(&lc3_tx_q, encoded_buf, K_NO_WAIT);
			enc_drop_cnt++;
		}

		if ((enc_frame_cnt++ % 200U) == 0U) {
			printk("[tx] frame=%u enc_drop=%u ul_q=%u\n",
			       enc_frame_cnt, enc_drop_cnt,
			       k_msgq_num_used_get(&lc3_tx_q));
			enc_drop_cnt = 0;
		}
	}
}

/* =========================================================================
 * Uplink BIS selection and ISO TX thread
 * ========================================================================= */

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
			printk("iso_tx: net_buf pool exhausted (cnt=%u)\n", pool_err_cnt);
		}
		return -ENOMEM;
	}

	if (k_msgq_get(&lc3_tx_q, enc_data, K_NO_WAIT) != 0) {
		static uint32_t tx_underrun_cnt;

		if (atomic_get(&uplink_tx_active) != 0 && (tx_underrun_cnt++ % 100U) == 0U) {
			printk("Uplink TX underrun — sending silence (cnt=%u)\n",
			       tx_underrun_cnt);
		}
		memset(enc_data, 0, sizeof(enc_data));
	}

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	net_buf_add_mem(buf, enc_data, sizeof(enc_data));

	err = bt_iso_chan_send(chan, buf, seq_num);
	if (err < 0) {
		printk("bt_iso_chan_send failed on %p: %d\n", chan, err);
		net_buf_unref(buf);
		return err;
	}

	/* Feed the uplink TX anchor to clk_sync from thread context (not from
	 * iso_sent BT callback — bt_iso_chan_get_tx_sync issues a synchronous
	 * HCI command and must not be called from a BT event callback). */
	struct bt_iso_tx_info tx_info;
	static uint32_t tx_sync_fail_cnt;

	if (bt_iso_chan_get_tx_sync(chan, &tx_info) == 0) {
		clk_sync_anchor_notify(tx_info.ts);
	} else {
		tx_sync_fail_cnt++;
		if ((tx_sync_fail_cnt % 50U) == 1U) {
			printk("[clk] bt_iso_chan_get_tx_sync failed (cnt=%u)\n",
			       tx_sync_fail_cnt);
		}
	}

	atomic_set(&uplink_tx_active, 1);
	seq_num++;
	return 0;
}

/* =========================================================================
 * BT ISO callbacks
 * ========================================================================= */

static void iso_sent(struct bt_iso_chan *chan)
{
	if (chan != bis[0] && atomic_get(&ptt_active) != 0) {
		k_sem_give(&tx_sem);
	}
}

/* iso_recv: receives BIG SDUs on BIS1 (downlink only).
 *
 * Valid frames → downlink_ring for the decoder thread.
 * Lost/invalid frames → zero-length slot → decoder runs PLC.
 *
 * Uplink kickstart: on the very first confirmed reception, prime tx_sem once
 * from within this callback.  The nRF5340 controller requires the first
 * bt_iso_chan_send() on a BIG-member uplink to happen inside the BT ISO
 * callback context so it can align it to the BIG anchor event. */
static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	if (chan != bis[0]) {
		return;
	}

	if ((buf != NULL) && (buf->len <= CONFIG_BT_ISO_TX_MTU) &&
	    ((info->flags & BT_ISO_FLAGS_VALID) != 0U)) {
		downlink_ring_put(buf->data, (uint16_t)buf->len);
		k_timer_start(&bis1_activity_timer, K_MSEC(500), K_NO_WAIT);
		/* clk_sync is fed from iso_sent() uplink TX anchors instead —
		 * that directly tracks the rate at which iso_tx consumes encoded
		 * frames, eliminating enc_drop with lc3_tx_q depth=1. */
	} else {
		downlink_ring_put(NULL, 0U); /* loss → PLC */
	}

	if (iso_datapaths_setup) {
		iso_datapaths_setup = false;
		if (atomic_get(&ptt_active) != 0) {
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
	if (reason == BT_HCI_ERR_CONN_FAIL_TO_ESTAB ||
	    reason == BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
		k_sem_give(&sem_big_sync);
	} else {
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.connected    = iso_connected,
	.disconnected = iso_disconnected,
	.sent         = iso_sent,
	.recv         = iso_recv,
};

/* =========================================================================
 * BLE scan and periodic advertising callbacks
 * ========================================================================= */

static bool base_skip_len_prefixed_blob(struct net_buf_simple *buf, uint8_t *len_out)
{
	uint8_t len;

	if (buf->len < sizeof(len)) {
		return false;
	}

	len = net_buf_simple_pull_u8(buf);
	if (buf->len < len) {
		return false;
	}

	if (len_out != NULL) {
		*len_out = len;
	}

	net_buf_simple_pull_mem(buf, len);

	return true;
}

static bool base_is_grptlk_vendor_codec(uint8_t codec_id, uint16_t cid, uint16_t vid)
{
	return (codec_id == GRPTLK_VENDOR_CODEC_ID) &&
	       (cid == GRPTLK_VENDOR_COMPANY_ID) &&
	       (vid == GRPTLK_VENDOR_VENDOR_ID);
}

static int base_log_service_data(const struct bt_data *data)
{
	struct net_buf_simple buf;
	uint32_t presentation_delay_us;
	uint8_t subgroup_count;

	if ((data == NULL) || (data->type != BT_DATA_SVC_DATA16) ||
	    (data->data_len < (BT_UUID_SIZE_16 + 3U + 1U))) {
		return -ENOENT;
	}

	net_buf_simple_init_with_data(&buf, (void *)data->data, data->data_len);
	if (net_buf_simple_pull_le16(&buf) != BT_UUID_BASIC_AUDIO_VAL) {
		return -ENOENT;
	}

	if (buf.len < 4U) {
		return -EINVAL;
	}

	presentation_delay_us = net_buf_simple_pull_le24(&buf);
	subgroup_count = net_buf_simple_pull_u8(&buf);
	if (subgroup_count == 0U) {
		return -EINVAL;
	}

	printk("BASE: presentation_delay=%u us, num_subgroups=%u\n",
	       presentation_delay_us, subgroup_count);

	for (uint8_t subgroup_idx = 0U; subgroup_idx < subgroup_count; subgroup_idx++) {
		uint8_t bis_count;
		uint8_t codec_id;
		uint8_t codec_cfg_len;
		uint8_t meta_len;
		uint16_t cid;
		uint16_t vid;
		uint32_t bis_bitfield = 0U;
		const char *codec_note = "";

		if (buf.len < 1U + 1U + 2U + 2U + 1U + 1U) {
			return -EINVAL;
		}

		bis_count = net_buf_simple_pull_u8(&buf);
		if (bis_count == 0U) {
			return -EINVAL;
		}

		codec_id = net_buf_simple_pull_u8(&buf);
		cid = net_buf_simple_pull_le16(&buf);
		vid = net_buf_simple_pull_le16(&buf);

		if (!base_skip_len_prefixed_blob(&buf, &codec_cfg_len) ||
		    !base_skip_len_prefixed_blob(&buf, &meta_len)) {
			return -EINVAL;
		}

		for (uint8_t bis_idx = 0U; bis_idx < bis_count; bis_idx++) {
			uint8_t bis_index;
			uint8_t bis_codec_cfg_len;

			if (buf.len < 2U) {
				return -EINVAL;
			}

			bis_index = net_buf_simple_pull_u8(&buf);
			bis_codec_cfg_len = net_buf_simple_pull_u8(&buf);
			if (buf.len < bis_codec_cfg_len) {
				return -EINVAL;
			}

			net_buf_simple_pull_mem(&buf, bis_codec_cfg_len);

			if ((bis_index > 0U) && (bis_index <= 32U)) {
				bis_bitfield |= (uint32_t)1U << (bis_index - 1U);
			}
		}

		if (base_is_grptlk_vendor_codec(codec_id, cid, vid)) {
			codec_note = " (GRPTLK vendor codec)";
		} else if (codec_id == BT_HCI_CODING_FORMAT_VS) {
			codec_note = " (vendor specific)";
		}

		printk("BASE subgroup %u: codec_id=0x%02x%s, cid=0x%04x, vid=0x%04x, "
		       "codec_cfg_len=%u, meta_len=%u, num_bis=%u, bis_bitfield=0x%08x\n",
		       subgroup_idx + 1U, codec_id, codec_note, cid, vid,
		       codec_cfg_len, meta_len, bis_count, bis_bitfield);
	}

	return (buf.len == 0U) ? 0 : -EINVAL;
}

static bool pa_decode_base(struct bt_data *data, void *user_data)
{
	int err;

	ARG_UNUSED(user_data);

	err = base_log_service_data(data);
	if (err == -ENOENT) {
		return true;
	}

	if (err != 0) {
		if (atomic_cas(&base_parse_warned, 0, 1)) {
			printk("BASE parse failed: %d\n", err);
		}
		return false;
	}

	atomic_set(&base_logged, 1);

	return false;
}

static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
{
	ARG_UNUSED(buf);
	if (!per_adv_found && info->interval) {
		per_adv_found   = true;
		per_sid         = info->sid;
		per_interval_us = BT_CONN_INTERVAL_TO_US(info->interval);
		bt_addr_le_copy(&per_addr, info->addr);
		k_sem_give(&sem_per_adv);
	}
}

static struct bt_le_scan_cb scan_callbacks = { .recv = scan_recv };

static void sync_cb(struct bt_le_per_adv_sync *sync,
		    struct bt_le_per_adv_sync_synced_info *info)
{
	ARG_UNUSED(sync);
	ARG_UNUSED(info);
	k_sem_give(&sem_per_sync);
}

static void pa_recv_cb(struct bt_le_per_adv_sync *sync,
		       const struct bt_le_per_adv_sync_recv_info *info,
		       struct net_buf_simple *buf)
{
	struct net_buf_simple ad_copy = *buf;

	ARG_UNUSED(sync);
	ARG_UNUSED(info);

	if (atomic_get(&base_logged) != 0) {
		return;
	}

	bt_data_parse(&ad_copy, pa_decode_base, NULL);
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
	big_actual_num_bis  = MIN(biginfo->num_bis, (uint8_t)BIS_ISO_CHAN_COUNT);
	big_sdu_interval_us = biginfo->sdu_interval;
	big_max_sdu         = biginfo->max_sdu;
	/* Only print on first detection — callback fires every ~200 ms while
	 * the PA scanner is active and would spam the log once synced. */
	if (grptlk_big == NULL) {
		printk("BIG: %u BIS(es), sdu_interval=%u us, max_sdu=%u B, syncing to %u\n",
		       biginfo->num_bis, big_sdu_interval_us, big_max_sdu, big_actual_num_bis);
	}
	k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced  = sync_cb,
	.recv    = pa_recv_cb,
	.term    = term_cb,
	.biginfo = biginfo_cb,
};

/* =========================================================================
 * BIG sync state machine helpers
 * ========================================================================= */

static void reset_semaphores(void)
{
	k_sem_reset(&sem_per_adv);
	k_sem_reset(&sem_per_sync);
	k_sem_reset(&sem_per_sync_lost);
	k_sem_reset(&sem_per_big_info);
	k_sem_reset(&sem_big_sync);
	k_sem_reset(&sem_big_sync_lost);
	atomic_set(&base_logged, 0);
	atomic_set(&base_parse_warned, 0);
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
		(void)k_sem_take(&sem_big_sync_lost, K_MSEC(100));
	}
}

static void bis_channels_init(void)
{
	for (uint8_t i = 0U; i < BIS_ISO_CHAN_COUNT; i++) {
		bis_iso_chan[i].ops = &iso_ops;
		bis_iso_chan[i].qos = &bis_iso_qos;
		bis[i]              = &bis_iso_chan[i];
	}
}

static int setup_iso_datapaths(void)
{
	int err;

	for (uint8_t i = 0U; i < big_sync_param.num_bis; i++) {
		struct bt_iso_chan *chan = bis[i];
		const struct bt_iso_chan_path hci_path = {
			.pid    = BT_ISO_DATA_PATH_HCI,
			.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
		};
		uint8_t dir = (i == 0U) ? BT_HCI_DATAPATH_DIR_CTLR_TO_HOST
					: BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;

		printk("Setting data path chan %u...\n", i);
		err = bt_iso_setup_data_path(chan, dir, &hci_path);
		if (err == -EACCES) {
			printk("Data path chan %u busy, removing stale path and retrying...\n",
			       i);
			(void)bt_iso_remove_data_path(chan, BT_HCI_DATAPATH_DIR_CTLR_TO_HOST);
			(void)bt_iso_remove_data_path(chan, BT_HCI_DATAPATH_DIR_HOST_TO_CTLR);
			err = bt_iso_setup_data_path(chan, dir, &hci_path);
		}
		if (err) {
			printk("Failed to setup ISO data path for chan %u: %d\n", i, err);
			return err;
		}
		printk("Data path set chan %u.\n", i);
	}

	clk_sync_reset();

	seq_num = 0U;
	atomic_set(&uplink_tx_active, 0);
	k_msgq_purge(&lc3_tx_q);

	/* Reset ring indices so stale frames from a previous BIG sync don't
	 * play out.  The semaphore counts are reset by draining them. */
	atomic_set(&downlink_write_idx, 0);
	atomic_set(&downlink_read_idx,  0);
	while (k_sem_take(&downlink_sem, K_NO_WAIT) == 0) {
	}
	atomic_set(&uplink_write_idx, 0);
	atomic_set(&uplink_read_idx,  0);
	while (k_sem_take(&uplink_sem, K_NO_WAIT) == 0) {
	}

	iso_datapaths_setup = true;
	return 0;
}

/* =========================================================================
 * LC3 worker thread startup
 * =========================================================================
 * Thread stacks must be at file scope — K_THREAD_STACK_DEFINE uses
 * __attribute__((section)) which cannot apply to automatic variables.
 */
K_THREAD_STACK_DEFINE(decoder_stack, DECODER_STACK_SIZE);
static struct k_thread decoder_thread_data;

K_THREAD_STACK_DEFINE(encoder_stack, ENCODER_STACK_SIZE);
static struct k_thread encoder_thread_data;

K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);
static struct k_thread tx_thread_data;

static int lc3_workers_start(void)
{
	lc3_decoder = lc3_setup_decoder(preset_active.qos.interval, SAMPLE_RATE_HZ,
					0, &lc3_decoder_mem);
	if (lc3_decoder == NULL) {
		printk("ERROR: Failed to setup LC3 decoder\n");
		return -EIO;
	}

	lc3_encoder = lc3_setup_encoder(preset_active.qos.interval, SAMPLE_RATE_HZ,
					0, &lc3_encoder_mem);
	if (lc3_encoder == NULL) {
		printk("ERROR: Failed to setup LC3 encoder\n");
		return -EIO;
	}

	k_thread_create(&decoder_thread_data, decoder_stack,
			K_THREAD_STACK_SIZEOF(decoder_stack),
			decoder_thread_func, NULL, NULL, NULL,
			DECODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&decoder_thread_data, "lc3_decoder");

	k_thread_create(&encoder_thread_data, encoder_stack,
			K_THREAD_STACK_SIZEOF(encoder_stack),
			encoder_thread_func, NULL, NULL, NULL,
			ENCODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&encoder_thread_data, "lc3_encoder");

	k_thread_create(&tx_thread_data, tx_thread_stack,
			K_THREAD_STACK_SIZEOF(tx_thread_stack),
			tx_thread, NULL, NULL, NULL,
			TX_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "iso_tx");

	return 0;
}

/* =========================================================================
 * main — system init and BIG sync state machine
 * =========================================================================
 *
 * Startup sequence:
 *   1. Audio backend (CS47L63 + I2S DMA)
 *   2. Buttons (PTT, PTT-lock, volume)
 *   3. LC3 worker threads (decoder, encoder, iso_tx)
 *   4. Bluetooth enable + scan
 *
 * BIG sync loop (restarts on every loss or timeout):
 *   scan → PA found → PA sync → BIGInfo → BIG sync → data paths
 *      └──────────── re-scan on any failure ──────────────────────┘
 */
int main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync      *sync;
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

	/* --- Status LED ---------------------------------------------------- */
	led_err = led_init();
	if (led_err) {
		printk("led_init failed: %d\n", led_err);
	} else {
		(void)led_set_receiver_synced(false);
	}

	/* --- Buttons: PTT, PTT-lock, volume --------------------------------- */
	err = buttons_init(&tx_sem);
	if (err) {
		printk("buttons_init failed: %d\n", err);
		return err;
	}

	/* --- BIS1 inactivity watchdog --------------------------------------- */
	k_timer_init(&bis1_activity_timer, bis1_activity_timeout_handler, NULL);
	k_work_init(&bis1_disconnect_work, bis1_disconnect_work_handler);

	/* --- ISO channel objects -------------------------------------------- */
	bis_channels_init();

	/* --- Audio backend (CS47L63 codec + I2S DMA) ------------------------ */
	err = audio_init(&tx_msgq, audio_rx_mono_frame);
	if (err) {
		printk("audio_init failed: %d\n", err);
		return err;
	}

	err = audio_start();
	if (err) {
		printk("audio_start failed: %d\n", err);
		return err;
	}

	/* --- Clock sync thread (BLE anchor → HFCLKAUDIO PI controller) ----- */
	clk_sync_init();

	/* --- LC3 codec threads (decoder, encoder, iso_tx) ------------------ */
	err = lc3_workers_start();
	if (err) {
		return err;
	}

	/* --- Bluetooth ----------------------------------------------------- */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	bt_le_scan_cb_register(&scan_callbacks);
	bt_le_per_adv_sync_cb_register(&sync_callbacks);

	/* --- BIG sync state machine (loops on loss / timeout) -------------- */
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

		per_adv_found = false;
		printk("Waiting for periodic advertising...\n");
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
		sync_create_param.sid     = per_sid;
		sync_create_param.skip    = 0;
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

		/* Synchronise preset_active QoS to what the broadcaster is actually sending.
		 * big_sdu_interval_us = frame duration; big_max_sdu = octets per SDU. */
		preset_active.qos.interval = big_sdu_interval_us;
		preset_active.qos.sdu      = big_max_sdu;

		/* Update I2S DMA block size to match the frame duration from BIGInfo.
		 * words_per_block = sample_rate_hz * frame_duration_ms / 1000
		 * (sample rate is always 16 kHz; only frame duration may change). */
		audio_i2s_set_block_size((AUDIO_SAMPLE_RATE_HZ * (big_sdu_interval_us / 1000U))
					 / 1000U);

		/* Re-initialise LC3 codec instances with the discovered frame duration.
		 * lc3_setup_decoder/encoder are fast pointer-setup calls; the worker
		 * threads are already running but will not consume stale data because
		 * the BIG is not yet synced — downlink_ring and uplink_ring are empty. */
		lc3_decoder = lc3_setup_decoder(preset_active.qos.interval, SAMPLE_RATE_HZ,
						0, &lc3_decoder_mem);
		if (lc3_decoder == NULL) {
			printk("ERROR: lc3_setup_decoder failed\n");
			goto per_sync_lost_check;
		}
		lc3_encoder = lc3_setup_encoder(preset_active.qos.interval, SAMPLE_RATE_HZ,
						0, &lc3_encoder_mem);
		if (lc3_encoder == NULL) {
			printk("ERROR: lc3_setup_encoder failed\n");
			goto per_sync_lost_check;
		}

big_sync_create:
		if (!bis_channels_idle()) {
			printk("Waiting for previous BIG channel cleanup...\n");
			wait_bis_channels_idle(K_SECONDS(2));
		}
		if (!bis_channels_idle()) {
			printk("Previous BIG channels still allocated, waiting for fresh BIGInfo\n");
			goto per_sync_lost_check;
		}

		big_sync_param.num_bis      = big_actual_num_bis;
		big_sync_param.bis_bitfield = BIT_MASK(big_actual_num_bis);
		printk("Create BIG Sync (num_bis=%u, bitfield=0x%x)...\n",
		       big_sync_param.num_bis, big_sync_param.bis_bitfield);

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
			printk("BIG sync failed (err %d, connected %d/%u)\n",
			       err, (int)atomic_get(&big_chan_connected), big_actual_num_bis);
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
			(void)led_set_receiver_synced(true);
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
			(void)led_set_receiver_synced(false);
		}

per_sync_lost_check:
		printk("Checking for periodic sync lost...\n");
		err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
		if (err) {
			printk("Waiting for fresh BIGInfo...\n");
			k_sem_reset(&sem_per_big_info);
			err = k_sem_take(&sem_per_big_info, K_SECONDS(5));
			if (err == 0) {
				goto big_sync_create;
			}
			printk("No BIGInfo — waiting for PA sync to terminate...\n");
			(void)k_sem_take(&sem_per_sync_lost, K_FOREVER);
		}

		printk("Periodic sync lost.\n");
	} while (true);
}
