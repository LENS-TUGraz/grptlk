#include <errno.h>
#include <zephyr/kernel.h>
#include "lc3.h"

#if defined(CONFIG_SOC_NRF5340_CPUAPP)
#include <nrfx_clock.h>
#endif
#include "io/audio.h"
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

/* -------------------------------------------------------------------------- */
/*                    Volume Buttons (sw0 = Vol Down, sw1 = Vol Up)           */
/* -------------------------------------------------------------------------- */

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

/* -------------------------------------------------------------------------- */
/*                           PTT Button (BTN3 = sw2)                          */
/* -------------------------------------------------------------------------- */

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
static atomic_t ptt_active = ATOMIC_INIT(1); /* always transmit if no button */
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

	int val = gpio_pin_get_dt(&ptt_btn);
	atomic_set(&ptt_active, val > 0 ? 1 : 0);
	printk("PTT init: button is %s\n", val > 0 ? "pressed" : "released");
	return 0;
#else
	printk("PTT: no sw2 alias, always transmitting mic\n");
	return 0;
#endif
}

/* BAP preset: 16 kHz, 10 ms frames, 40 octets/frame, reliability class 1. */
static struct bt_bap_lc3_preset preset_active = BT_BAP_LC3_BROADCAST_PRESET_16_2_1(
	BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
	BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

#define PCM_SAMPLES_PER_FRAME AUDIO_SAMPLES_PER_FRAME
#define SAMPLE_RATE_HZ        AUDIO_SAMPLE_RATE_HZ
#define BLOCK_BYTES           AUDIO_BLOCK_BYTES

/* BIS layout: bis[0] = TX (BIS1), bis[1..N] = RX uplink */
#define NUM_RX_BIS        (CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT - 1)
#define NUM_PRIME_PACKETS 2

/* Queues */
K_MSGQ_DEFINE(pcm_msgq, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 3, 4);
K_MSGQ_DEFINE(tx_msgq, BLOCK_BYTES, 3, 4);
K_MSGQ_DEFINE(uplink_mix_q, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 3, 4);
K_MSGQ_DEFINE(lc3_encoded_q, CONFIG_BT_ISO_TX_MTU, 3, 4);

struct uplink_frame {
	uint16_t len;
	uint8_t data[CONFIG_BT_ISO_TX_MTU];
	uint8_t _pad[2]; /* Pad to 44 bytes to safely align with K_MSGQ 4-byte boundary */
};
struct k_msgq uplink_rx_q[NUM_RX_BIS];
char __aligned(4) uplink_rx_q_buffer[NUM_RX_BIS][3 * sizeof(struct uplink_frame)];

/* Decoder wakeup: first arriving BIS packet each interval gives the semaphore
 * immediately (zero drift, locked to BT anchor). The watchdog timer fires only
 * if the entire interval is lost — ensuring the decoder still runs PLC. */
K_SEM_DEFINE(uplink_rx_sem, 0, 1);
static struct k_timer uplink_watchdog_timer;
/* 0 = decoder is idle / ready for next interval, 1 = already woken this interval */
static atomic_t uplink_interval_fired = ATOMIC_INIT(0);

static void uplink_watchdog_expiry(struct k_timer *timer)
{
	/* Only fires when no iso_recv arrived this interval — force a PLC pass. */
	atomic_set(&uplink_interval_fired, 0);
	k_sem_give(&uplink_rx_sem);
}

/* LC3 */
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_16k_t lc3_encoder_mem;
static lc3_decoder_t lc3_decoders[NUM_RX_BIS];
static lc3_decoder_mem_16k_t lc3_decoder_mems[NUM_RX_BIS];
static int16_t send_pcm_data[PCM_SAMPLES_PER_FRAME];

/* BAP */
static struct bt_bap_broadcast_source *broadcast_source;
static struct bt_bap_stream streams[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];

/* ISO */
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT *NUM_PRIME_PACKETS,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT);
static K_SEM_DEFINE(tx_sem, 0, NUM_PRIME_PACKETS);

static struct bt_iso_chan *bis[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
static struct bt_iso_chan bis_iso_chan[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
static uint16_t seq_num;
/* Set to true when a real uplink frame is received on bis[i+1].
 * Cleared on disconnect so PLC from a gone sender is not mixed in. */
static bool bis_ever_rx[NUM_RX_BIS];

static struct bt_iso_big_create_param big_create_param = {
	.num_bis = CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT,
	.bis_channels = bis,
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_UNFRAMED,
};

static struct bt_iso_chan_io_qos iso_rx_qos;
static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = CONFIG_BT_ISO_TX_MTU,
	.phy = BT_GAP_LE_PHY_2M,
};
static struct bt_iso_chan_qos bis_iso_qos = {
	.tx = &iso_tx_qos,
	.rx = &iso_rx_qos,
};

/* Threads */
#define ENCODER_STACK_SIZE   4096
#define ENCODER_PRIORITY     5
#define DECODER_PRIORITY     5
#define TX_THREAD_STACK_SIZE 1024
#define TX_THREAD_PRIORITY   5

K_THREAD_STACK_DEFINE(encoder_stack, ENCODER_STACK_SIZE);
K_THREAD_STACK_DEFINE(uplink_dec_stack, 6144);
K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);
static struct k_thread encoder_thread_data;
static struct k_thread uplink_dec_thread_data;
static struct k_thread tx_thread_data;

/* -------------------------------------------------------------------------- */

static void audio_rx_mono_frame(const int16_t *mono_frame)
{
	if (k_msgq_put(&pcm_msgq, mono_frame, K_NO_WAIT) != 0) {
		int16_t dropped[PCM_SAMPLES_PER_FRAME];
		(void)k_msgq_get(&pcm_msgq, dropped, K_NO_WAIT);
		(void)k_msgq_put(&pcm_msgq, mono_frame, K_NO_WAIT);
	}

	/* Clock the uplink decoder from the I2S DMA interrupt — same hardware
	 * clock that drives the DAC — so tx_msgq never accumulates.
	 * The watchdog timer remains as fallback if I2S is not running. */
	if (atomic_cas(&uplink_interval_fired, 0, 1)) {
		k_sem_give(&uplink_rx_sem);
	}
}

static void encoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	int ret;
	uint8_t encoded_buf[CONFIG_BT_ISO_TX_MTU];
	static uint32_t enc_frame_cnt;
	int octets_per_frame = preset_active.qos.sdu;
	/* Last uplink frame — replayed when uplink_mix_q is empty so the encoder
	 * always mixes a smooth transition instead of a sudden silence step. */
	static int16_t last_uplink_mono[PCM_SAMPLES_PER_FRAME];
	static bool has_uplink;

	while (1) {
		ret = k_msgq_get(&pcm_msgq, send_pcm_data, K_FOREVER);
		if (ret != 0) {
			continue;
		}

		/* PTT gate: broadcast silence when button is not held. */
		if (atomic_get(&ptt_active) == 0) {
			memset(send_pcm_data, 0, sizeof(send_pcm_data));
		}

		int16_t uplink_mono[PCM_SAMPLES_PER_FRAME];
		if (k_msgq_get(&uplink_mix_q, uplink_mono, K_NO_WAIT) == 0) {
			memcpy(last_uplink_mono, uplink_mono, sizeof(last_uplink_mono));
			has_uplink = true;
		}
		/* Mix the last known uplink frame (or silence on first startup). */
		if (has_uplink) {
			for (int i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
				int32_t s =
					(int32_t)send_pcm_data[i] + (int32_t)last_uplink_mono[i];
				if (s > 32767) {
					s = 32767;
				} else if (s < -32768) {
					s = -32768;
				}
				send_pcm_data[i] = (int16_t)s;
			}
		}

		uint32_t t0 = k_cycle_get_32();
		ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, send_pcm_data, 1,
				 octets_per_frame, encoded_buf);
		uint32_t enc_us = k_cyc_to_us_floor32(k_cycle_get_32() - t0);

		if (ret == -1) {
			printk("LC3 encode failed\n");
			continue;
		}

		if ((enc_frame_cnt++ % 100U) == 0U) {
			printk("[enc] frame=%u encode=%u us pcm_q=%u lc3_q=%u\n", enc_frame_cnt,
			       enc_us, k_msgq_num_used_get(&pcm_msgq),
			       k_msgq_num_used_get(&lc3_encoded_q));
		}

		if (k_msgq_put(&lc3_encoded_q, encoded_buf, K_NO_WAIT) != 0) {
			uint8_t dropped[CONFIG_BT_ISO_TX_MTU];
			(void)k_msgq_get(&lc3_encoded_q, dropped, K_NO_WAIT);
			(void)k_msgq_put(&lc3_encoded_q, encoded_buf, K_NO_WAIT);
		}
	}
}

static void uplink_decoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	static uint32_t dec_frame_cnt;
	/* Per-BIS debug counters, reset on printk interval. */
	static uint32_t dbg_rx_cnt[NUM_RX_BIS];
	static uint32_t dbg_plc_cnt[NUM_RX_BIS];
	static uint32_t dbg_dec_err[NUM_RX_BIS];
	static int16_t dbg_peak[NUM_RX_BIS];
	/* Consecutive PLC run length per BIS (not reset on printk — tracks streaks). */
	static uint32_t dbg_plc_streak[NUM_RX_BIS];
	static uint32_t dbg_plc_streak_max[NUM_RX_BIS];
	/* Peak of the last real decoded frame per BIS — used to detect PLC spikes. */
	static int16_t dbg_last_real_peak[NUM_RX_BIS];
	/* Clip events: how many frames had mixed_pcm saturate at ±32767. */
	static uint32_t dbg_clip_cnt;
	/* active_streams value last interval — detect sudden changes. */
	static int dbg_last_active;
	static uint32_t dbg_active_change_cnt;

	while (1) {
		k_sem_take(&uplink_rx_sem, K_FOREVER);
		/* Allow the next interval's first packet to re-trigger the wakeup. */
		atomic_set(&uplink_interval_fired, 0);

		uint32_t t0_total = k_cycle_get_32();
		uint32_t dec_us[NUM_RX_BIS];
		int32_t mixed_pcm[PCM_SAMPLES_PER_FRAME] = {0};
		int active_streams = 0;

		for (int i = 0; i < NUM_RX_BIS; i++) {
			struct uplink_frame frame;
			int16_t stream_pcm[PCM_SAMPLES_PER_FRAME];

			uint32_t t0 = k_cycle_get_32();
			bool got_frame = (k_msgq_get(&uplink_rx_q[i], &frame, K_NO_WAIT) == 0 &&
					  frame.len > 0);

			if (got_frame) {
				dbg_rx_cnt[i]++;
				bis_ever_rx[i] = true;
				dbg_plc_streak[i] = 0;
			} else if (bis_ever_rx[i]) {
				dbg_plc_cnt[i]++;
				dbg_plc_streak[i]++;
				if (dbg_plc_streak[i] > dbg_plc_streak_max[i]) {
					dbg_plc_streak_max[i] = dbg_plc_streak[i];
				}
			} else {
				dbg_plc_cnt[i]++;
				dbg_plc_streak[i]++;
			}

			/* Always run the LC3 decoder to keep its internal state advancing.
			 * On a missing frame pass NULL (PLC mode) but discard the output —
			 * this prevents phase discontinuities that cause bursts of crackling
			 * every time a BT packet is dropped. */
			const uint8_t *lc3_data = got_frame ? frame.data : NULL;
			int lc3_len = got_frame ? (int)frame.len : 0;
			int ret = lc3_decode(lc3_decoders[i], lc3_data, lc3_len, LC3_PCM_FORMAT_S16,
					     stream_pcm, 1);

			if (ret < 0) {
				/* ret==1 means PLC operated (normal), ret<0 means bad params. */
				dbg_dec_err[i]++;
			}

			/* Mix real frames (ret==0) and PLC frames (ret==1) for active
			 * senders — PLC smooths over occasional dropped BT packets.
			 * Never mix PLC output from idle BISes (bis_ever_rx[i]==false):
			 * their decoder produces noise from an uninitialised state.
			 *
			 * LC3 PLC can over-synthesise from a high-energy preceding frame,
			 * producing a spike 5-7x the normal amplitude — audible as a crack.
			 * Attenuate PLC output by 50% (shift right by 1) to suppress this. */
			if (ret >= 0 && bis_ever_rx[i]) {
				int16_t peak = 0;
				for (int s = 0; s < PCM_SAMPLES_PER_FRAME; s++) {
					int16_t a =
						stream_pcm[s] < 0 ? -stream_pcm[s] : stream_pcm[s];
					if (a > peak) {
						peak = a;
					}
				}
				/* Limit any frame (real or PLC) that spikes more than 4x the
				 * recent level — catches both LC3 PLC over-synthesis and the
				 * codec re-sync burst that occurs when the radio link recovers
				 * after extended packet loss (streak_max > ~10 frames). */
				int16_t ref = dbg_last_real_peak[i];
				if (ref < 64) {
					ref = 64;
				}
				int16_t limit =
					(ret == 1) ? ref
						   : (int16_t)(ref * 4 > 32767 ? 32767 : ref * 4);
				if (peak > limit) {
					for (int s = 0; s < PCM_SAMPLES_PER_FRAME; s++) {
						stream_pcm[s] = (int16_t)((int32_t)stream_pcm[s] *
									  limit / peak);
					}
					printk("[lim] frame=%u bis%d %s peak=%d->%d\n",
					       dec_frame_cnt, i + 2, ret == 1 ? "plc" : "real",
					       peak, limit);
				}
				if (ret != 1) {
					dbg_last_real_peak[i] = (peak > limit) ? limit : peak;
				}
				if (peak > dbg_peak[i]) {
					dbg_peak[i] = peak;
				}
				active_streams++;
				for (int s = 0; s < PCM_SAMPLES_PER_FRAME; s++) {
					mixed_pcm[s] += stream_pcm[s];
				}
			}
			dec_us[i] = k_cyc_to_us_floor32(k_cycle_get_32() - t0);
		}

		/* Detect sudden active_streams change (causes amplitude step = click). */
		if (active_streams != dbg_last_active) {
			dbg_active_change_cnt++;
			dbg_last_active = active_streams;
		}

		/* Normalise by number of active streams to prevent clipping. */
		if (active_streams > 1) {
			for (int s = 0; s < PCM_SAMPLES_PER_FRAME; s++) {
				mixed_pcm[s] /= active_streams;
			}
		}

		/* Detect pre-output clipping (after normalisation) and record mixed peak. */
		bool clipped = false;
		int32_t mixed_peak = 0;
		for (int s = 0; s < PCM_SAMPLES_PER_FRAME; s++) {
			int32_t a = mixed_pcm[s] < 0 ? -mixed_pcm[s] : mixed_pcm[s];
			if (a > mixed_peak) {
				mixed_peak = a;
			}
			if (mixed_pcm[s] > 32767 || mixed_pcm[s] < -32768) {
				clipped = true;
			}
		}
		if (clipped) {
			dbg_clip_cnt++;
		}

		uint32_t total_us = k_cyc_to_us_floor32(k_cycle_get_32() - t0_total);
		if (total_us > 8000U) {
			printk("[dec] LATE frame=%u total=%u us — overran 10ms interval!\n",
			       dec_frame_cnt, total_us);
		}
		if ((dec_frame_cnt++ % 100U) == 0U) {
			printk("[dec] frame=%u total=%u us active=%d mixed_peak=%d tx_q=%u "
			       "mix_q=%u clip=%u active_chg=%u\n",
			       dec_frame_cnt, total_us, active_streams, (int)mixed_peak,
			       k_msgq_num_used_get(&tx_msgq), k_msgq_num_used_get(&uplink_mix_q),
			       dbg_clip_cnt, dbg_active_change_cnt);
			dbg_clip_cnt = 0;
			dbg_active_change_cnt = 0;
			for (int i = 0; i < NUM_RX_BIS; i++) {
				printk("  bis%d: rx=%u plc=%u err=%u peak=%d streak_max=%u (%u "
				       "us)\n",
				       i + 2, dbg_rx_cnt[i], dbg_plc_cnt[i], dbg_dec_err[i],
				       dbg_peak[i], dbg_plc_streak_max[i], dec_us[i]);
				dbg_rx_cnt[i] = 0;
				dbg_plc_cnt[i] = 0;
				dbg_dec_err[i] = 0;
				dbg_peak[i] = 0;
				dbg_plc_streak_max[i] = 0;
			}
		}

		/* Pack as I2S words: left sample in bits [31:16], right in bits [15:0].
		 * NRF_I2S_ALIGN_LEFT expects this layout (see i2s_word_unpack).
		 * Packing here (rather than storing int16_t[] and casting to uint32_t[])
		 * avoids a channel-swap caused by little-endian byte ordering. */
		uint32_t stereo_buf[PCM_SAMPLES_PER_FRAME];
		int16_t mono_mix[PCM_SAMPLES_PER_FRAME];
		for (int i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
			int32_t s = mixed_pcm[i];
			if (s > 32767) {
				s = 32767;
			} else if (s < -32768) {
				s = -32768;
			}
			int16_t sample = (int16_t)s;
			/* Mono: same sample on both channels. */
			stereo_buf[i] = ((uint32_t)(uint16_t)sample << 16) | (uint16_t)sample;
			mono_mix[i] = sample;
		}
		if (k_msgq_put(&uplink_mix_q, mono_mix, K_NO_WAIT) != 0) {
			int16_t dropped[PCM_SAMPLES_PER_FRAME];
			(void)k_msgq_get(&uplink_mix_q, dropped, K_NO_WAIT);
			(void)k_msgq_put(&uplink_mix_q, mono_mix, K_NO_WAIT);
		}

		if (k_msgq_put(&tx_msgq, stereo_buf, K_NO_WAIT) != 0) {
			uint32_t dropped[PCM_SAMPLES_PER_FRAME];
			(void)k_msgq_get(&tx_msgq, dropped, K_NO_WAIT);
			(void)k_msgq_put(&tx_msgq, stereo_buf, K_NO_WAIT);
		}
	}
}

static void tx_thread(void *arg1, void *arg2, void *arg3)
{
	int err;
	struct net_buf *buf;
	uint8_t enc_data[CONFIG_BT_ISO_TX_MTU];

	while (true) {
		k_sem_take(&tx_sem, K_FOREVER);

		buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
		if (!buf) {
			printk("tx_thread: net_buf pool exhausted\n");
			continue;
		}

		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

		if (k_msgq_get(&lc3_encoded_q, enc_data, K_NO_WAIT) != 0) {
			static uint32_t tx_silence_cnt;
			memset(enc_data, 0, sizeof(enc_data));
			if ((tx_silence_cnt++ % 10U) == 0U) {
				printk("[tx] lc3_encoded_q empty — sending silence (cnt=%u)\n",
				       tx_silence_cnt);
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
	}
}

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
	for (int i = 0; i < NUM_RX_BIS; i++) {
		if (chan == bis[i + 1]) {
			bis_ever_rx[i] = false;
			k_msgq_purge(&uplink_rx_q[i]);
			break;
		}
	}
}

static void iso_sent(struct bt_iso_chan *chan)
{
	if (chan == bis[0]) {
		k_sem_give(&tx_sem);
	}
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	struct uplink_frame frame;

	if (buf && buf->len > 0 && buf->len <= CONFIG_BT_ISO_TX_MTU &&
	    (info->flags & BT_ISO_FLAGS_VALID)) {
		frame.len = buf->len;
		memcpy(frame.data, buf->data, buf->len);
	} else {
		frame.len = 0;
	}

	/* bis[0] is TX-only; uplink channels are bis[1..N] → uplink_rx_q[0..N-1] */
	for (int i = 0; i < NUM_RX_BIS; i++) {
		if (chan == bis[i + 1]) {
			if (k_msgq_put(&uplink_rx_q[i], &frame, K_NO_WAIT) != 0) {
				struct uplink_frame dropped;
				(void)k_msgq_get(&uplink_rx_q[i], &dropped, K_NO_WAIT);
				(void)k_msgq_put(&uplink_rx_q[i], &frame, K_NO_WAIT);
			}
			break;
		}
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.connected = iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
	.recv = iso_recv,
};

/* -------------------------------------------------------------------------- */

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
	// err = bt_le_ext_adv_set_data(*adv, ad, ARRAY_SIZE(ad), NULL, 0);
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

	for (int i = 0; i < NUM_RX_BIS; i++) {
		k_msgq_init(&uplink_rx_q[i], uplink_rx_q_buffer[i], sizeof(struct uplink_frame), 3);
	}

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

	(void)vol_buttons_init();

	lc3_encoder =
		lc3_setup_encoder(preset_active.qos.interval, SAMPLE_RATE_HZ, 0, &lc3_encoder_mem);
	if (lc3_encoder == NULL) {
		printk("Failed to setup LC3 encoder\n");
		return -EIO;
	}

	for (int i = 0; i < NUM_RX_BIS; i++) {
		lc3_decoders[i] = lc3_setup_decoder(preset_active.qos.interval, SAMPLE_RATE_HZ, 0,
						    &lc3_decoder_mems[i]);
		if (lc3_decoders[i] == NULL) {
			printk("Failed to setup LC3 decoder %d\n", i);
			return -EIO;
		}
	}

	k_thread_create(&encoder_thread_data, encoder_stack, K_THREAD_STACK_SIZEOF(encoder_stack),
			encoder_thread_func, NULL, NULL, NULL, ENCODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&encoder_thread_data, "lc3_encoder");

	k_thread_create(&uplink_dec_thread_data, uplink_dec_stack,
			K_THREAD_STACK_SIZEOF(uplink_dec_stack), uplink_decoder_thread_func, NULL,
			NULL, NULL, DECODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&uplink_dec_thread_data, "uplink_dec");

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

	k_timer_init(&uplink_watchdog_timer, uplink_watchdog_expiry, NULL);
	k_timer_start(&uplink_watchdog_timer, K_USEC(preset_active.qos.interval), K_NO_WAIT);

	prime_and_start_iso_transmission();
	led_err = led_set_broadcast_running(true);
	if (led_err) {
		printk("led_set_broadcast_running failed: %d\n", led_err);
	}

	return 0;
}
