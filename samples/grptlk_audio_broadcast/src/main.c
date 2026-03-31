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
#include "io/debug_gpio.h"
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
 * For LC3Plus 5ms, QoS fields are overridden at runtime.
 * For BAP 16_2_1, the preset is used as-is (no override). */
#define GRPTLK_VENDOR_CODEC_ID	 BT_HCI_CODING_FORMAT_VS
#define GRPTLK_VENDOR_COMPANY_ID 0xDEAD
#define GRPTLK_VENDOR_VENDOR_ID	 0xBEEF
#if defined(CONFIG_GRPTLK_AUDIO_FRAME_10_MS)
#define GRPTLK_CODEC_INTERVAL_US 10000U
#define GRPTLK_CODEC_SDU_BYTES	 40U
#else
#define GRPTLK_CODEC_INTERVAL_US 5000U
#define GRPTLK_CODEC_SDU_BYTES	 20U
#endif

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
#if defined(CONFIG_GRPTLK_AUDIO_FRAME_5_MS)
static void override_preset_for_lc3plus_5ms(void)
{
	preset_active.qos.interval = GRPTLK_CODEC_INTERVAL_US;
	preset_active.qos.sdu = GRPTLK_CODEC_SDU_BYTES;
	/* rtn=2: two retransmit opportunities per BIG event. Reduces streak_max from
	 * 3-4 to 0-1 in burst-loss conditions, keeping PLC below the audible threshold.
	 * latency=10ms = (rtn+1)*interval = 3*5ms, the required minimum for rtn=2. */
	preset_active.qos.latency = 10U;
	preset_active.qos.rtn = 2U;
	preset_active.codec_cfg.id = GRPTLK_VENDOR_CODEC_ID;
	preset_active.codec_cfg.cid = GRPTLK_VENDOR_COMPANY_ID;
	preset_active.codec_cfg.vid = GRPTLK_VENDOR_VENDOR_ID;
	preset_active.codec_cfg.data_len = 0U;
}
#endif

#define PCM_SAMPLES_PER_FRAME AUDIO_SAMPLES_PER_FRAME
#define SAMPLE_RATE_HZ	      AUDIO_SAMPLE_RATE_HZ
#define BLOCK_BYTES	      AUDIO_BLOCK_BYTES
/* BIS layout: bis[0] = TX (BIS1), bis[1..N] = RX uplink */
#define NUM_RX_BIS	      (CONFIG_GRPTLK_NUM_CHAN - 1)
#define NUM_PRIME_PACKETS     1

/* Speaker playback ring buffer — kept for audio_init() but never written by
 * iso_recv, so the DMA output path produces silence. */
static uint32_t ring_fifo[AUDIO_RING_NUM_BLKS][AUDIO_BLK_SAMPLES_MONO];
static volatile uint16_t ring_prod_idx;
static volatile uint16_t ring_cons_idx;

static struct audio_ring playback_ring = {
	.fifo = ring_fifo,
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

/* Mixed uplink audio from decoder → encoder */
static int16_t mixed_uplink_pcm[PCM_SAMPLES_PER_FRAME];
static atomic_t uplink_audio_ready = ATOMIC_INIT(0);

/* LC3 */
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_16k_t lc3_encoder_mem;
static lc3_decoder_t lc3_decoders[NUM_RX_BIS];
static lc3_decoder_mem_16k_t lc3_decoder_mem[NUM_RX_BIS];

/* TX ready flag: ignore iso_recv until TX path has sent NUM_PRIME_PACKETS+1 packets */
static atomic_t iso_tx_ready = ATOMIC_INIT(0);
#define ISO_TX_READY_THRESHOLD (NUM_PRIME_PACKETS + 1) /* 3 packets */

/* Uplink RX decoder */
#define NUM_RX_BIS (CONFIG_GRPTLK_NUM_CHAN - 1)

struct rx_bis_frame {
	uint8_t data[CONFIG_BT_ISO_TX_MTU];
	bool received;
	bool valid;
	uint32_t seq_num;
};

static struct rx_bis_frame rx_frames[NUM_RX_BIS];
static atomic_t rx_bis_received_count = ATOMIC_INIT(0);
static K_SEM_DEFINE(decoder_sem, 0, 1);		/* Signals decoder: all BIS received */
static K_SEM_DEFINE(decoder_proceed_sem, 0, 1); /* Decoder signals encoder */

/* PRR (Packet Reception Rate) tracking for uplink BISes */
#define PRR_WINDOW_SIZE (500000U / GRPTLK_CODEC_INTERVAL_US)
/* 5ms profile → 100 frames; 10ms profile → 50 frames; both give ~500ms window */

struct bis_prr_tracker {
	uint8_t window[PRR_WINDOW_SIZE]; /* 1 = valid, 0 = invalid/missing */
	uint8_t window_idx;		 /* Current position in circular buffer */
	uint8_t window_filled;		 /* Whether window is fully populated */
};

static struct bis_prr_tracker prr_trackers[NUM_RX_BIS];

/* BAP */
static struct bt_bap_broadcast_source *broadcast_source __maybe_unused;
static struct bt_bap_stream streams[CONFIG_GRPTLK_NUM_CHAN] __maybe_unused;

/* ISO */
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, CONFIG_GRPTLK_NUM_CHAN *NUM_PRIME_PACKETS,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, CONFIG_GRPTLK_NUM_CHAN);
static K_SEM_DEFINE(tx_sem, 0, NUM_PRIME_PACKETS);

static struct bt_iso_chan *bis[CONFIG_GRPTLK_NUM_CHAN];
static struct bt_iso_chan bis_iso_chan[CONFIG_GRPTLK_NUM_CHAN] __maybe_unused;
static uint16_t seq_num __maybe_unused;

static struct bt_iso_big_create_param big_create_param __maybe_unused = {
	.num_bis = CONFIG_GRPTLK_NUM_CHAN,
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
#define ENCODER_STACK_SIZE   4096
#define ENCODER_PRIORITY     2
#define DECODER_STACK_SIZE   4096
#define DECODER_PRIORITY     3 /* Higher than encoder (runs first) */
#define TX_THREAD_STACK_SIZE 1024
#define TX_THREAD_PRIORITY   2
K_THREAD_STACK_DEFINE(encoder_stack, ENCODER_STACK_SIZE);
K_THREAD_STACK_DEFINE(decoder_stack, DECODER_STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);

static struct k_thread encoder_thread_data __maybe_unused;
static struct k_thread decoder_thread_data __maybe_unused;
static struct k_thread tx_thread_data __maybe_unused;

/* -------------------------------------------------------------------------- */

static void audio_rx_mono_frame(const int16_t *mono_frame)
{
	clk_sync_i2s_notify(k_cyc_to_us_floor32(k_cycle_get_32()));
	memcpy(mic_pcm_shared, mono_frame, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME);
	k_sem_give(&mic_frame_sem);
}

static void encoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	int ret;
	int16_t local_pcm[PCM_SAMPLES_PER_FRAME];
	static uint32_t enc_frame_cnt;
	int octets_per_frame = preset_active.qos.sdu;
	static bool first_frame = true;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		/* Wait for decoder to complete (drives encoder timing) */
		k_sem_take(&decoder_proceed_sem, K_FOREVER);

		debug_lc3_enc_set(1);

		memcpy(local_pcm, mic_pcm_shared, sizeof(local_pcm));

		/* PTT gate: broadcast silence when button is not held. */
		if (atomic_get(&ptt_active) == 0) {
			memset(local_pcm, 0, sizeof(local_pcm));
		}

		/* Mix microphone with uplink audio if available */
		if (atomic_set(&uplink_audio_ready, 0) != 0) {
			/* 50% mic + 50% uplink with overflow protection */
			for (int i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
				int32_t mixed = (local_pcm[i] / 2) + (mixed_uplink_pcm[i] / 2);

				/* Clamp to int16 range */
				if (mixed > INT16_MAX) {
					mixed = INT16_MAX;
				} else if (mixed < INT16_MIN) {
					mixed = INT16_MIN;
				}

				local_pcm[i] = (int16_t)mixed;
			}
		}

		uint32_t t0 = k_cycle_get_32();

		ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, local_pcm, 1, octets_per_frame,
				 encoded_shared);
		uint32_t enc_us = k_cyc_to_us_floor32(k_cycle_get_32() - t0);

		if (ret == -1) {
			printk("LC3 encode failed\n");
			debug_lc3_enc_set(0);
			continue;
		}

		atomic_set(&encoded_data_ready, 1);

		debug_lc3_enc_set(0);

		/* Only first frame: trigger TX to prime the pump.
		 * After that, iso_sent() drives TX. */
		if (first_frame) {
			k_sem_give(&tx_sem);
			first_frame = false;
		}

		if ((enc_frame_cnt++ % 100U) == 0U) {
			printk("[enc] frame=%u encode=%u us\n", enc_frame_cnt, enc_us);
		}
	}
}

static void decoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	int ret;
	int16_t decoded_pcm[NUM_RX_BIS][PCM_SAMPLES_PER_FRAME];
	static uint32_t dec_frame_cnt;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		/* Wait for all BIS packets to arrive */
		k_sem_take(&decoder_sem, K_FOREVER);

		debug_lc3_dec_set(1);

		/* Clear uplink ready flag at start of each decoder cycle */
		atomic_set(&uplink_audio_ready, 0);

		/* Decode each BIS - NULL buffer triggers LC3 PLC for lost packets */
		bool bis_valid[NUM_RX_BIS] = {false}; /* Track which BISes had valid data */
		for (int i = 0; i < NUM_RX_BIS; i++) {

			struct rx_bis_frame *frame = &rx_frames[i];

			if (!frame->valid) {
				/* Packet lost - trigger LC3 PLC with NULL buffer */
				ret = lc3_decode(lc3_decoders[i], NULL, 0, LC3_PCM_FORMAT_S16,
						 decoded_pcm[i], 1);
			} else {
				/* Valid packet - normal decode */
				ret = lc3_decode(lc3_decoders[i], frame->data, sizeof(frame->data),
						 LC3_PCM_FORMAT_S16, decoded_pcm[i], 1);
			}

			bis_valid[i] = (ret == 0); /* PLC-concealed frames return success */
			frame->received = false;   /* Reset for next interval */
			frame->valid = false;	   /* Clear valid flag to prevent stale data */
		}

		/* Calculate and print PRR for each BIS (every 100 frames) */
		if ((dec_frame_cnt % 100U) == 0U) {
			printk("[prr] frame=%u", dec_frame_cnt);
			for (int i = 0; i < NUM_RX_BIS; i++) {
				struct bis_prr_tracker *tracker = &prr_trackers[i];
				uint8_t valid_count = 0;
				uint8_t window_size = tracker->window_filled ? PRR_WINDOW_SIZE
									     : tracker->window_idx;

				for (int j = 0; j < window_size; j++) {
					valid_count += tracker->window[j];
				}

				uint8_t prr = (valid_count * 100) / window_size;
				printk(" bis%u: %u%%", i, prr);
			}
			printk("\n");
		}

		/* Log which BISes were mixed this frame */
		int valid_count = 0;
		//		printk("[mix] frame=%u bis=", dec_frame_cnt);
		//		for (int i = 0; i < NUM_RX_BIS; i++) {
		//			if (bis_valid[i]) {
		//				if (valid_count > 0) {
		//					printk(",");
		//				}
		//				printk("%d", i);
		//				valid_count++;
		//			}
		//		}
		//		printk(" valid=%d\n", valid_count);

		/* Count valid BISes for mixing (count from bis_valid array) */
		for (int i = 0; i < NUM_RX_BIS; i++) {
			if (bis_valid[i]) {
				valid_count++;
			}
		}

		/* Mix all valid decoded BISes and play to speaker */
		/* One LC3 frame (80 samples) = 5 ring blocks (16 samples each) */
		for (int blk = 0; blk < AUDIO_I2S_BLKS_PER_FRAME; blk++) {
			uint16_t pi = ring_prod_idx;
			uint32_t *ring_blk = playback_ring.fifo[pi];

			/* Mix all valid decoded BISes for this block */
			for (int j = 0; j < AUDIO_I2S_SAMPLES_PER_BLOCK; j++) {
				int32_t mixed_sample = 0;

				/* Sum only valid BISes for this sample */
				for (int i = 0; i < NUM_RX_BIS; i++) {
					if (bis_valid[i]) {
						mixed_sample += decoded_pcm
							[i][blk * AUDIO_I2S_SAMPLES_PER_BLOCK + j];
					}
				}

				/* Clip to int16 range to prevent overflow */
				if (mixed_sample > INT16_MAX) {
					mixed_sample = INT16_MAX;
				} else if (mixed_sample < INT16_MIN) {
					mixed_sample = INT16_MIN;
				}

				/* Convert to int16 and duplicate to both L/R channels */
				int16_t sample = (int16_t)mixed_sample;
				ring_blk[j] = ((uint32_t)((uint16_t)sample) |
					       ((uint32_t)((uint16_t)sample) << 16));

				/* Store mixed uplink sample for encoder */
				mixed_uplink_pcm[blk * AUDIO_I2S_SAMPLES_PER_BLOCK + j] = sample;
			}

			/* Advance producer index */
			ring_prod_idx = (pi + 1U) % playback_ring.num_blks;
		}

		/* Signal encoder that uplink audio is ready */
		atomic_set(&uplink_audio_ready, 1);

		debug_lc3_dec_set(0);

		/* Signal encoder that it can now proceed */
		k_sem_give(&decoder_proceed_sem);

		dec_frame_cnt++;
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

		/* Since encoder now triggers TX, encoded_data_ready should always be set.
		 * The atomic check is kept for safety in case of timing anomalies. */
		if (atomic_set(&encoded_data_ready, 0) != 0) {
			memcpy(enc_data, encoded_shared, sizeof(enc_data));
		} else {
			/* This should never happen with encoder-driven timing */
			memset(enc_data, 0, sizeof(enc_data));
			tx_underrun_cnt++;
			printk("[tx] WARNING: Unexpected underrun (cnt=%u)\n", tx_underrun_cnt);
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
		debug_iso_sent_set(1);

		k_sem_give(&tx_sem); /* RESTORED: Drive TX timing */

		debug_iso_sent_set(0);

		/* Mark TX as ready after NUM_PRIME_PACKETS+1 packets sent.
		 * This signals iso_recv that the TX path is stable. */
		static uint32_t tx_sent_count;
		if (atomic_get(&iso_tx_ready) == 0 && ++tx_sent_count >= ISO_TX_READY_THRESHOLD) {
			atomic_set(&iso_tx_ready, 1);
		}

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
#if 0
			if ((tx_sync_fail_cnt % 50U) == 1U) {
				printk("[clk] bt_iso_chan_get_tx_sync failed (cnt=%u)\n",
				       tx_sync_fail_cnt);
			}
#endif
		}
	}
}

/* Helper to get 0-based index for uplink BIS (bis[1] -> 0, bis[2] -> 1, etc.) */
static int get_uplink_bis_index(struct bt_iso_chan *chan)
{
	for (int i = 1; i < CONFIG_GRPTLK_NUM_CHAN; i++) {
		if (chan == bis[i]) {
			return i - 1; /* Convert to 0-based array index */
		}
	}
	return -1; /* Not an uplink BIS (bis[0] is TX only) */
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	int bis_idx = get_uplink_bis_index(chan);

	if (bis_idx < 0) {
		return; /* Not an uplink BIS (bis[0] is TX only) */
	}

	/* Ignore until TX path stable */
	if (atomic_get(&iso_tx_ready) == 0) {
		return;
	}

	debug_iso_recv_set(1);

	/* Only process VALID packets */
	bool valid = (buf != NULL && buf->len > 0 && !(info->flags & BT_ISO_FLAGS_ERROR));

	struct rx_bis_frame *frame = &rx_frames[bis_idx];
	frame->received = true;
	frame->valid = valid;
	frame->seq_num = info->seq_num;

	/* Update PRR tracking for this BIS */
	struct bis_prr_tracker *tracker = &prr_trackers[bis_idx];
	tracker->window[tracker->window_idx] = valid ? 1 : 0;
	tracker->window_idx = (tracker->window_idx + 1) % PRR_WINDOW_SIZE;
	if (tracker->window_idx == 0) {
		tracker->window_filled = 1;
	}

	if (valid) {
		memcpy(frame->data, buf->data, MIN(buf->len, sizeof(frame->data)));
	}

	/* Check if all BIS received this interval */
	uint32_t count = atomic_inc(&rx_bis_received_count);
	if (count + 1 >= NUM_RX_BIS) {
		atomic_set(&rx_bis_received_count, 0);
		k_sem_give(&decoder_sem); /* Trigger decoder */
	}

	debug_iso_recv_set(0);
}

static struct bt_iso_chan_ops iso_ops = {
	.connected = iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
	.recv = iso_recv,
};

static int setup_broadcast_source(struct bt_bap_broadcast_source **source)
{
	struct bt_bap_broadcast_source_stream_param stream_params[CONFIG_GRPTLK_NUM_CHAN];
	struct bt_bap_broadcast_source_subgroup_param subgroup_param[1];
	struct bt_bap_broadcast_source_param create_param = {0};

	uint8_t left_loc[] = {BT_AUDIO_CODEC_DATA(
		BT_AUDIO_CODEC_CFG_CHAN_ALLOC, BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_LEFT))};
	uint8_t right_loc[] = {BT_AUDIO_CODEC_DATA(
		BT_AUDIO_CODEC_CFG_CHAN_ALLOC, BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_RIGHT))};

	for (size_t i = 0U; i < CONFIG_GRPTLK_NUM_CHAN; i++) {
		stream_params[i].stream = &streams[i];
		stream_params[i].data = (i == 0) ? left_loc : right_loc;
		stream_params[i].data_len = (i == 0) ? sizeof(left_loc) : sizeof(right_loc);
	}

	subgroup_param[0].params_count = CONFIG_GRPTLK_NUM_CHAN;
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

	for (size_t i = 0; i < CONFIG_GRPTLK_NUM_CHAN; i++) {
		bis_iso_chan[i].ops = &iso_ops;
		bis_iso_chan[i].qos = &bis_iso_qos;
		bis[i] = &bis_iso_chan[i];
	}

	for (uint8_t i = 0U; i < CONFIG_GRPTLK_NUM_CHAN; i++) {
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

	for (uint8_t i = 0U; i < CONFIG_GRPTLK_NUM_CHAN; i++) {
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

	for (uint8_t i = 0U; i < CONFIG_GRPTLK_NUM_CHAN; i++) {
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

#if defined(CONFIG_GRPTLK_AUDIO_FRAME_5_MS)
	override_preset_for_lc3plus_5ms();
#endif
	/* BAP 16_2_1: preset_active already holds correct spec values, no override needed */

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

	/* Debug GPIO (optional, for logic analyzer timing analysis) */
	(void)debug_gpio_init();

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

	/* Setup LC3 decoders for each uplink BIS */
	for (int i = 0; i < NUM_RX_BIS; i++) {
		lc3_decoders[i] = lc3_setup_decoder(preset_active.qos.interval, SAMPLE_RATE_HZ, 0,
						    &lc3_decoder_mem[i]);
		if (lc3_decoders[i] == NULL) {
			printk("Failed to setup LC3 decoder %d\n", i);
			return -EIO;
		}
	}

	k_thread_create(&decoder_thread_data, decoder_stack, K_THREAD_STACK_SIZEOF(decoder_stack),
			decoder_thread_func, NULL, NULL, NULL, DECODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&decoder_thread_data, "lc3_decoder");

	k_thread_create(&encoder_thread_data, encoder_stack, K_THREAD_STACK_SIZEOF(encoder_stack),
			encoder_thread_func, NULL, NULL, NULL, ENCODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&encoder_thread_data, "lc3_encoder");

	/* Initialize decoder_proceed_sem as unavailable (encoder runs after decoder) */
	k_sem_init(&decoder_proceed_sem, 0, 1);

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
