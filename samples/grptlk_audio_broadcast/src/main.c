#include <errno.h>
#include <zephyr/kernel.h>
#include "lc3.h"

#if defined(CONFIG_SOC_NRF5340_CPUAPP)
#include <nrfx_clock.h>
#endif
#include "audio/audio.h"
#include "audio/drivers/audio_i2s.h"
#include "audio/sync/clk_sync.h"
#include "io/led.h"
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>


#define VOLUME_STEP_DB 3

#if DT_NODE_HAS_STATUS(DT_ALIAS(sw0), okay) && DT_NODE_HAS_STATUS(DT_ALIAS(sw1), okay)
#define VOL_BTN_AVAILABLE 1
static const struct gpio_dt_spec vol_dn_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec vol_up_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static struct gpio_callback vol_dn_cb_data;
static struct gpio_callback vol_up_cb_data;

/* Deferred volume work: ISRs cannot call SPI directly. */
static struct k_work vol_work;
static atomic_t vol_pending_step = ATOMIC_INIT(0);

static void vol_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int step = (int)atomic_set(&vol_pending_step, 0);

	if (step != 0) {
		audio_volume_adjust((int8_t)step);
	}
}

static void vol_dn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	printk("[VOL] down pressed\n");
	atomic_add(&vol_pending_step, -VOLUME_STEP_DB);
	k_work_submit(&vol_work);
}

static void vol_up_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	printk("[VOL] up pressed\n");
	atomic_add(&vol_pending_step, VOLUME_STEP_DB);
	k_work_submit(&vol_work);
}
#else
#define VOL_BTN_AVAILABLE 0
#endif

static int vol_buttons_init(void)
{
#if VOL_BTN_AVAILABLE
	int err;

	k_work_init(&vol_work, vol_work_handler);

	if (!gpio_is_ready_dt(&vol_dn_btn) || !gpio_is_ready_dt(&vol_up_btn)) {
		printk("Volume button GPIO not ready\n");
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&vol_dn_btn, GPIO_INPUT);
	if (err) {
		return err;
	}
	err = gpio_pin_configure_dt(&vol_up_btn, GPIO_INPUT);
	if (err) {
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&vol_dn_btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&vol_up_btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		return err;
	}

	gpio_init_callback(&vol_dn_cb_data, vol_dn_isr, BIT(vol_dn_btn.pin));
	gpio_init_callback(&vol_up_cb_data, vol_up_isr, BIT(vol_up_btn.pin));
	gpio_add_callback(vol_dn_btn.port, &vol_dn_cb_data);
	gpio_add_callback(vol_up_btn.port, &vol_up_cb_data);

	printk("Volume buttons init: sw0=vol_down, sw1=vol_up, step=%d dB\n", VOLUME_STEP_DB);
	return 0;
#else
	printk("Volume buttons: sw0/sw1 not available on this board\n");
	return 0;
#endif
}

/* PTT button (BTN3 = sw2): PTT starts inactive at boot. */
#if DT_NODE_HAS_STATUS(DT_ALIAS(sw2), okay)
#define PTT_AVAILABLE 1
static const struct gpio_dt_spec ptt_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static struct gpio_callback ptt_cb_data;
static atomic_t ptt_active = ATOMIC_INIT(0);

static void ptt_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	int val = gpio_pin_get_dt(&ptt_btn);

	atomic_set(&ptt_active, val > 0 ? 1 : 0);
	printk("[PTT] %s\n", val > 0 ? "pressed" : "released");
}
#else
#define PTT_AVAILABLE 0
static atomic_t ptt_active = ATOMIC_INIT(1);
#endif

static int ptt_init(void)
{
#if PTT_AVAILABLE
	int err;

	if (!gpio_is_ready_dt(&ptt_btn)) {
		printk("PTT button GPIO not ready\n");
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&ptt_btn, GPIO_INPUT);
	if (err) {
		printk("PTT button configure failed: %d\n", err);
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&ptt_btn, GPIO_INT_EDGE_BOTH);
	if (err) {
		printk("PTT interrupt configure failed: %d\n", err);
		return err;
	}
	gpio_init_callback(&ptt_cb_data, ptt_isr, BIT(ptt_btn.pin));
	gpio_add_callback(ptt_btn.port, &ptt_cb_data);

	atomic_set(&ptt_active, 0);
	printk("PTT init: button ready, mic TX disabled at boot\n");
	return 0;
#else
	printk("PTT: no sw2 alias, always transmitting mic\n");
	return 0;
#endif
}

/* PTT lock toggle: BTN4 (sw3) cycles between PTT mode and always-TX mode.
 * When lock is active, LED1 (led0/rgb1_red) is lit and mic always transmits. */
#if DT_NODE_HAS_STATUS(DT_ALIAS(sw3), okay) && DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
#define PTT_LOCK_AVAILABLE 1
static const struct gpio_dt_spec ptt_lock_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);
static const struct gpio_dt_spec ptt_lock_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static struct gpio_callback ptt_lock_cb_data;
static atomic_t ptt_lock_active = ATOMIC_INIT(0);
static struct k_work ptt_lock_toggle_work;

static void ptt_lock_toggle_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (atomic_get(&ptt_lock_active)) {
		atomic_set(&ptt_lock_active, 0);
		gpio_pin_set_dt(&ptt_lock_led, 0);
		atomic_set(&ptt_active, 0);
		printk("[PTT-LOCK] disabled — PTT mode active\n");
	} else {
		atomic_set(&ptt_lock_active, 1);
		gpio_pin_set_dt(&ptt_lock_led, 1);
		atomic_set(&ptt_active, 1);
		printk("[PTT-LOCK] enabled — always transmitting\n");
	}
}

static void ptt_lock_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	if (gpio_pin_get_dt(&ptt_lock_btn) > 0) {
		k_work_submit(&ptt_lock_toggle_work);
	}
}
#else
#define PTT_LOCK_AVAILABLE 0
#endif

static int ptt_lock_init(void)
{
#if PTT_LOCK_AVAILABLE
	int err;

	k_work_init(&ptt_lock_toggle_work, ptt_lock_toggle_work_handler);

	if (!gpio_is_ready_dt(&ptt_lock_btn) || !gpio_is_ready_dt(&ptt_lock_led)) {
		printk("PTT-lock button/LED GPIO not ready\n");
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&ptt_lock_btn, GPIO_INPUT);
	if (err) {
		printk("PTT-lock button configure failed: %d\n", err);
		return err;
	}
	err = gpio_pin_configure_dt(&ptt_lock_led, GPIO_OUTPUT_INACTIVE);
	if (err) {
		printk("PTT-lock LED configure failed: %d\n", err);
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&ptt_lock_btn, GPIO_INT_EDGE_BOTH);
	if (err) {
		printk("PTT-lock interrupt configure failed: %d\n", err);
		return err;
	}
	gpio_init_callback(&ptt_lock_cb_data, ptt_lock_isr, BIT(ptt_lock_btn.pin));
	gpio_add_callback(ptt_lock_btn.port, &ptt_lock_cb_data);

	printk("PTT-lock init: BTN4=toggle, LED1=status indicator\n");
	return 0;
#else
	printk("PTT-lock: sw3/led0 not available on this board\n");
	return 0;
#endif
}


/* BAP preset base: kept as-is per project requirement.
 * Runtime QoS fields are overridden below for LC3Plus 5 ms operation. */
#define GRPTLK_VENDOR_CODEC_ID  BT_HCI_CODING_FORMAT_VS
#define GRPTLK_VENDOR_COMPANY_ID 0xDEAD
#define GRPTLK_VENDOR_VENDOR_ID  0xBEEF
#define GRPTLK_CODEC_INTERVAL_US            5000U
#define GRPTLK_CODEC_SDU_BYTES              20U

static struct bt_bap_lc3_preset preset_active __maybe_unused = BT_BAP_LC3_BROADCAST_PRESET_16_2_1(
	BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
	BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);
/* LC3Plus 5 ms @ 16 kHz: interval=5000 us, sdu=20 bytes, latency=10 ms, rtn=2.
 *
 * RTN=2 (two retransmits) reduces streak_max from 3-4 to 0-1 in burst-loss
 * conditions, keeping PLC below the audible threshold.  Cost: +5ms latency
 * vs RTN=1.
 *
 * The receiver reads the vendor codec identity from the BASE and the transport
 * sizing from BIGInfo, so there is no need for extra private codec-config
 * fields here. */
static void override_preset_for_lc3plus_5ms(void)
{
	preset_active.qos.interval = GRPTLK_CODEC_INTERVAL_US;
	preset_active.qos.sdu      = GRPTLK_CODEC_SDU_BYTES;
	/* rtn=2: two retransmit opportunities per BIG event. Reduces streak_max from
	 * 3-4 to 0-1 in burst-loss conditions, keeping PLC below the audible threshold.
	 * latency=10ms = (rtn+1)*interval = 3*5ms, the required minimum for rtn=2. */
	preset_active.qos.latency  = 10U;
	preset_active.qos.rtn      = 2U;
	preset_active.codec_cfg.id = GRPTLK_VENDOR_CODEC_ID;
	preset_active.codec_cfg.cid = GRPTLK_VENDOR_COMPANY_ID;
	preset_active.codec_cfg.vid = GRPTLK_VENDOR_VENDOR_ID;
	preset_active.codec_cfg.data_len = 0U;
}

#define PCM_SAMPLES_PER_FRAME AUDIO_SAMPLES_PER_FRAME
#define SAMPLE_RATE_HZ        AUDIO_SAMPLE_RATE_HZ
#define BLOCK_BYTES           AUDIO_BLOCK_BYTES
/* BIS layout: bis[0] = TX (BIS1), bis[1..N] = RX uplink */
#define NUM_RX_BIS        (CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT - 1)
#define NUM_PRIME_PACKETS 2

/* Speaker playback ring buffer — kept for audio_init() but never written by
 * iso_recv, so the DMA output path produces silence. */
static uint32_t ring_fifo[AUDIO_RING_NUM_BLKS][AUDIO_BLK_SAMPLES_MONO];
static volatile uint16_t ring_prod_idx;
static volatile uint16_t ring_cons_idx;

static struct audio_ring playback_ring = {
	.fifo     = ring_fifo,
	.prod_idx = &ring_prod_idx,
	.cons_idx = &ring_cons_idx,
	.num_blks = AUDIO_RING_NUM_BLKS,
};

/* Mic → encoder: shared buffer + semaphore (no queue latency). */
static int16_t mic_pcm_shared[PCM_SAMPLES_PER_FRAME];
static K_SEM_DEFINE(mic_frame_sem, 0, 1);

/* Encoder → TX thread: shared buffer + atomic flag (no queue latency).
 * iso_sent() drives tx_sem unconditionally to keep the BIG stream alive.
 * The TX thread reads the freshest encoded frame or sends silence. */
static uint8_t encoded_shared[CONFIG_BT_ISO_TX_MTU];
static atomic_t encoded_data_ready = ATOMIC_INIT(0);

/* LC3 */
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_16k_t lc3_encoder_mem;

/* BAP */
static struct bt_bap_broadcast_source *broadcast_source __maybe_unused;
static struct bt_bap_stream streams[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT] __maybe_unused;

/* ISO */
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT *NUM_PRIME_PACKETS,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT);
static K_SEM_DEFINE(tx_sem, 0, NUM_PRIME_PACKETS);

static struct bt_iso_chan *bis[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
static struct bt_iso_chan bis_iso_chan[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT] __maybe_unused;
static uint16_t seq_num __maybe_unused;

static struct bt_iso_big_create_param big_create_param __maybe_unused = {
	.num_bis = CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT,
	.bis_channels = bis,
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_UNFRAMED,
};

static struct bt_iso_chan_io_qos iso_rx_qos __maybe_unused;
static struct bt_iso_chan_io_qos iso_tx_qos __maybe_unused = {
	.sdu = CONFIG_BT_ISO_TX_MTU,
	.phy = BT_GAP_LE_PHY_2M,
};
static struct bt_iso_chan_qos bis_iso_qos __maybe_unused = {
	.tx = &iso_tx_qos,
	.rx = &iso_rx_qos,
};

/* Threads */
#define ENCODER_STACK_SIZE    4096
#define ENCODER_PRIORITY      2
#define TX_THREAD_STACK_SIZE  1024
#define TX_THREAD_PRIORITY    2
K_THREAD_STACK_DEFINE(encoder_stack, ENCODER_STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);

static struct k_thread encoder_thread_data __maybe_unused;
static struct k_thread tx_thread_data __maybe_unused;

/* -------------------------------------------------------------------------- */

static void audio_rx_mono_frame(const int16_t *mono_frame)
{
	memcpy(mic_pcm_shared, mono_frame, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME);
	k_sem_give(&mic_frame_sem);
}

static void encoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	int ret;
	int16_t local_pcm[PCM_SAMPLES_PER_FRAME];
	static uint32_t enc_frame_cnt;
	int octets_per_frame = preset_active.qos.sdu;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		k_sem_take(&mic_frame_sem, K_FOREVER);
		memcpy(local_pcm, mic_pcm_shared, sizeof(local_pcm));

		/* PTT gate: broadcast silence when button is not held. */
		if (atomic_get(&ptt_active) == 0) {
			memset(local_pcm, 0, sizeof(local_pcm));
		}

		uint32_t t0 = k_cycle_get_32();

		ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, local_pcm, 1,
				 octets_per_frame, encoded_shared);
		uint32_t enc_us = k_cyc_to_us_floor32(k_cycle_get_32() - t0);

		if (ret == -1) {
			printk("LC3 encode failed\n");
			continue;
		}

		atomic_set(&encoded_data_ready, 1);

		if ((enc_frame_cnt++ % 100U) == 0U) {
			printk("[enc] frame=%u encode=%u us\n", enc_frame_cnt, enc_us);
		}
	}
}

/* -------------------------------------------------------------------------- */

static void tx_thread(void *arg1, void *arg2, void *arg3)
{
	int err;
	struct net_buf *buf;
	uint8_t enc_data[CONFIG_BT_ISO_TX_MTU];
	static uint32_t tx_frame_cnt;
	static uint32_t tx_underrun_cnt;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		k_sem_take(&tx_sem, K_FOREVER);

		buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
		if (!buf) {
			printk("tx_thread: net_buf pool exhausted\n");
			continue;
		}

		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

		/* Atomically consume encoded_data_ready: if it was 1, send the
		 * freshest encoded frame; otherwise send silence to keep the BIG
		 * stream alive while the encoder catches up. */
		if (atomic_set(&encoded_data_ready, 0) != 0) {
			memcpy(enc_data, encoded_shared, sizeof(enc_data));
		} else {
			memset(enc_data, 0, sizeof(enc_data));
			tx_underrun_cnt++;
			if ((tx_underrun_cnt % 10U) == 1U) {
				printk("[tx] underrun — sending silence (cnt=%u)\n",
				       tx_underrun_cnt);
			}
		}
		net_buf_add_mem(buf, enc_data, sizeof(enc_data));

		err = bt_iso_chan_send(bis[0], buf, seq_num);
		if (err < 0) {
			printk("Unable to broadcast data: %d\n", err);
			net_buf_unref(buf);
			continue;
		}

		seq_num++;
		if ((++tx_frame_cnt % 200U) == 0U) {
			tx_underrun_cnt = 0;
		}
	}
}

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	seq_num = 0U;
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n", chan, reason);
}

static void iso_sent(struct bt_iso_chan *chan)
{
	if (chan == bis[0]) {
		k_sem_give(&tx_sem);
		/* Feed the TX anchor timestamp to the HFCLKAUDIO integrator.
		 * bt_iso_chan_get_tx_sync() is the only source of a hardware
		 * BT controller timestamp on the broadcaster — iso_sent() itself
		 * carries no timing information. */
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
	}
}

/* Helper to get 0-based index for uplink BIS (bis[1] -> 0, bis[2] -> 1, etc.) */
static int get_uplink_bis_index(struct bt_iso_chan *chan)
{
	for (int i = 1; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
		if (chan == bis[i]) {
			return i - 1;  /* Convert to 0-based array index */
		}
	}
	return -1;  /* Not an uplink BIS (bis[0] is TX only) */
}

static void iso_recv(struct bt_iso_chan *chan,
		     const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	int bis_idx = get_uplink_bis_index(chan);

	if (bis_idx < 0) {
		return; /* Not an uplink BIS (bis[0] is TX only) */
	}

	/* Per-interval instrumentation: validate exactly NUM_RX_BIS callbacks per 5ms event.
	 * BIS2 (bis_idx=0) is always the first uplink BIS — use it as the interval boundary. */
	static uint32_t interval_call_count;
	static uint32_t interval_num;
	static uint8_t  status_mask; /* bit N set = BIS(N+2) had a valid payload this interval */
	static uint32_t last_seq;

	if (bis_idx == 0) {
		/* Dump previous interval summary when a new seq_num arrives on BIS2 */
		if (interval_call_count > 0 && info->seq_num != last_seq) {
			printk("@%u %u/%u %u%u%u%u%s\n",
			       interval_num, interval_call_count, NUM_RX_BIS,
			       (status_mask >> 0) & 1, (status_mask >> 1) & 1,
			       (status_mask >> 2) & 1, (status_mask >> 3) & 1,
			       interval_call_count != NUM_RX_BIS ? " MISMATCH" : "");
			interval_call_count = 0;
			status_mask = 0;
			interval_num++;
		}
		last_seq = info->seq_num;
	}

	interval_call_count++;
	if (!(info->flags & BT_ISO_FLAGS_ERROR) && buf->len > 0) {
		status_mask |= BIT(bis_idx);
	}

	// printk("[RX] BIS%d seq=%u flags=0x%02x len=%u\n",
	//        bis_idx + 2, info->seq_num, info->flags, buf->len);
}

static struct bt_iso_chan_ops iso_ops = {
	.connected = iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
	.recv = iso_recv,
};


static int setup_broadcast_source(struct bt_bap_broadcast_source **source)
{
	struct bt_bap_broadcast_source_stream_param
		stream_params[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
	struct bt_bap_broadcast_source_subgroup_param subgroup_param[1];
	struct bt_bap_broadcast_source_param create_param = {0};

	uint8_t left_loc[] = {BT_AUDIO_CODEC_DATA(
		BT_AUDIO_CODEC_CFG_CHAN_ALLOC, BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_LEFT))};
	uint8_t right_loc[] = {BT_AUDIO_CODEC_DATA(
		BT_AUDIO_CODEC_CFG_CHAN_ALLOC, BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_RIGHT))};

	for (size_t i = 0U; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
		stream_params[i].stream = &streams[i];
		stream_params[i].data = (i == 0) ? left_loc : right_loc;
		stream_params[i].data_len = (i == 0) ? sizeof(left_loc) : sizeof(right_loc);
	}

	subgroup_param[0].params_count = CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT;
	subgroup_param[0].params = stream_params;
	subgroup_param[0].codec_cfg = &preset_active.codec_cfg;

	create_param.params_count = ARRAY_SIZE(subgroup_param);
	create_param.params = subgroup_param;
	create_param.qos = &preset_active.qos;
	create_param.encryption = false;
	create_param.packing = BT_ISO_PACKING_SEQUENTIAL;

	return bt_bap_broadcast_source_create(&create_param, source);
}

static int setup_extended_adv(struct bt_le_ext_adv **adv)
{
	int err;

	err = bt_le_ext_adv_create(BT_BAP_ADV_PARAM_BROADCAST_FAST, NULL, adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return err;
	}

	/* Extended advertising data (Broadcast ID) */
	uint32_t broadcast_id = 0x123456;
	NET_BUF_SIMPLE_DEFINE(ad_buf, BT_UUID_SIZE_16 + BT_AUDIO_BROADCAST_ID_SIZE);
	net_buf_simple_add_le16(&ad_buf, BT_UUID_BROADCAST_AUDIO_VAL);
	net_buf_simple_add_le24(&ad_buf, broadcast_id);

	struct bt_data ext_ad[2] = {
		{
			.type = BT_DATA_SVC_DATA16,
			.data_len = ad_buf.len,
			.data = ad_buf.data,
		},
		BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
			sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	};

	err = bt_le_ext_adv_set_data(*adv, ext_ad, 2, NULL, 0);
	if (err) {
		printk("Failed to set advertising data (err %d)\n", err);
		return err;
	}

	return 0;
}

static int setup_periodic_adv(struct bt_le_ext_adv *adv)
{
	int err;

	err = bt_le_per_adv_set_param(adv, BT_BAP_PER_ADV_PARAM_BROADCAST_FAST);
	if (err) {
		printk("Failed to set periodic advertising parameters (err %d)\n", err);
		return err;
	}

	err = setup_broadcast_source(&broadcast_source);
	if (err) {
		printk("setup_broadcast_source failed: %d\n", err);
		return err;
	}

	NET_BUF_SIMPLE_DEFINE(base_buf, 128);
	err = bt_bap_broadcast_source_get_base(broadcast_source, &base_buf);
	if (err) {
		printk("get BASE failed: %d\n", err);
		return err;
	}

	struct bt_data per_ad = {
		.type = BT_DATA_SVC_DATA16,
		.data_len = base_buf.len,
		.data = base_buf.data,
	};

	err = bt_le_per_adv_set_data(adv, &per_ad, 1);
	if (err) {
		printk("set per adv data failed: %d\n", err);
		return err;
	}

	return 0;
}

static int create_big(struct bt_le_ext_adv *adv, struct bt_iso_big **big)
{
	int err;

	for (size_t i = 0; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
		bis_iso_chan[i].ops = &iso_ops;
		bis_iso_chan[i].qos = &bis_iso_qos;
		bis[i] = &bis_iso_chan[i];
	}

	for (uint8_t i = 0U; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
		bis[i]->qos->tx->rtn = preset_active.qos.rtn;
	}

	big_create_param.interval = preset_active.qos.interval;
	big_create_param.latency = preset_active.qos.latency;
	big_create_param.bis_channels = bis;

	err = bt_iso_big_create(adv, &big_create_param, big);
	if (err) {
		printk("Failed to create BIG (err %d)\n", err);
		return err;
	}

	for (uint8_t i = 0U; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
		printk("Waiting for BIG complete chan %u...\n", i);
		err = k_sem_take(&sem_big_cmplt, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return err;
		}
		printk("BIG create complete chan %u.\n", i);
	}

	return 0;
}

static int setup_iso_datapaths(void)
{
	int err;

	/* Reset the HFCLKAUDIO integrator on every BIG (re-)creation so it
	 * starts fresh rather than applying a stale correction. */
	clk_sync_reset();

	for (uint8_t i = 0U; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
		printk("Setting data path chan %u...\n", i);

		const struct bt_iso_chan_path hci_path = {
			.pid = BT_ISO_DATA_PATH_HCI,
			.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
		};

		uint8_t dir = (i == 0) ? BT_HCI_DATAPATH_DIR_HOST_TO_CTLR
				       : BT_HCI_DATAPATH_DIR_CTLR_TO_HOST;

		err = bt_iso_setup_data_path(&bis_iso_chan[i], dir, &hci_path);
		if (err != 0) {
			printk("Failed to setup ISO data path: %d\n", err);
			return err;
		}

		printk("Setting data path complete chan %u.\n", i);
	}

	return 0;
}

static void prime_and_start_iso_transmission(void)
{
	for (int i = 0; i < NUM_PRIME_PACKETS; i++) {
		k_sem_give(&tx_sem);
	}
}

/* -------------------------------------------------------------------------- */

int main(void)
{
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;
	int err;
	int led_err;

	override_preset_for_lc3plus_5ms();

#if defined(CONFIG_SOC_NRF5340_CPUAPP)
	int clk_ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);
	clk_ret -= NRFX_ERROR_BASE_NUM;
	if (clk_ret) {
		printk("Set 128 MHz clock divider failed: %d\n", clk_ret);
		return clk_ret;
	}
	printk("CPU clock set to 128 MHz\n");
#endif

	printk("Starting GRPTLK Broadcaster\n");
	led_err = led_init();
	if (led_err) {
		printk("led_init failed: %d\n", led_err);
	} else {
		(void)led_set_broadcast_running(false);
	}

	(void)ptt_init();
	(void)ptt_lock_init();

	err = audio_init(&playback_ring, audio_rx_mono_frame);
	if (err) {
		printk("audio_init failed: %d\n", err);
		return err;
	}

	err = audio_start();
	if (err) {
		printk("audio_start failed: %d\n", err);
		return err;
	}

	clk_sync_init();

	(void)vol_buttons_init();

	lc3_encoder =
		lc3_setup_encoder(preset_active.qos.interval, SAMPLE_RATE_HZ, 0, &lc3_encoder_mem);
	if (lc3_encoder == NULL) {
		printk("Failed to setup LC3 encoder\n");
		return -EIO;
	}

	k_thread_create(&encoder_thread_data, encoder_stack, K_THREAD_STACK_SIZEOF(encoder_stack),
			encoder_thread_func, NULL, NULL, NULL, ENCODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&encoder_thread_data, "lc3_encoder");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	err = setup_extended_adv(&adv);
	if (err) {
		return err;
	}

	err = setup_periodic_adv(adv);
	if (err) {
		return err;
	}

	err = bt_le_per_adv_start(adv);
	if (err) {
		printk("Failed to enable periodic advertising (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising (err %d)\n", err);
		return err;
	}

	err = create_big(adv, &big);
	if (err) {
		return err;
	}

	err = setup_iso_datapaths();
	if (err) {
		return err;
	}

	k_thread_create(&tx_thread_data, tx_thread_stack, K_THREAD_STACK_SIZEOF(tx_thread_stack),
			tx_thread, NULL, NULL, NULL, TX_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "iso_tx");

	prime_and_start_iso_transmission();

	led_err = led_set_broadcast_running(true);
	if (led_err) {
		printk("led_set_broadcast_running failed: %d\n", led_err);
	}

	return 0;
}
