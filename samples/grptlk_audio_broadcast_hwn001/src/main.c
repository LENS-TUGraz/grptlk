#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/i2c.h>
#include "drivers/max9867.h"

#include "lc3.h"
#include <stdint.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

/* -------------------------------------------------------------------------- */
/*                               Configuration                                */
/* -------------------------------------------------------------------------- */

/* Active BAP Preset */
/* We use the 16_2_1 preset: 16 kHz, 10 ms framing, 40 octets per frame.
 * This macro defines the QoS and Codec Configuration used by the stack.
 * All other audio parameters (buffer sizes, encoder setup) are derived from
 * this.
 */
static struct bt_bap_lc3_preset preset_active =
    BT_BAP_LC3_BROADCAST_PRESET_16_2_1(BT_AUDIO_LOCATION_FRONT_LEFT |
                                           BT_AUDIO_LOCATION_FRONT_RIGHT,
                                       BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

/* Derived Audio Parameters */
/*
 * 16_2_1 => 16000 Hz, 10000 us duration.
 * Samples per frame = (16000 * 10000) / 1000000 = 160.
 * We must use a constant for the array size.
 */
#define PCM_SAMPLES_PER_FRAME 160

/* Audio Format (I2S) */
#define SAMPLE_RATE_HZ 16000
#define CHANNELS 2
#define SAMPLE_BYTES 2
#define BLOCK_MS 10
#define FRAMES_PER_BLOCK ((SAMPLE_RATE_HZ * BLOCK_MS) / 1000)   /* 160 frames */
#define BLOCK_BYTES (FRAMES_PER_BLOCK * CHANNELS * SAMPLE_BYTES) /* 640 B */

/* I2S Objects */
static const struct device *i2s_dev;

/* RX Slab: I2S driver fills these */
#define RX_BLOCK_COUNT 16
K_MEM_SLAB_DEFINE_STATIC(rx_slab, BLOCK_BYTES, RX_BLOCK_COUNT, 4);

/* TX Slab: I2S driver reads from these (speaker output) */
#define TX_BLOCK_COUNT 8
K_MEM_SLAB_DEFINE_STATIC(tx_slab, BLOCK_BYTES, TX_BLOCK_COUNT, 4);

/* PCM Queue (Mono frames for Encoder) */
K_MSGQ_DEFINE(pcm_msgq, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 16, 4);

/* Queue for passing decoded uplink audio to I2S TX (speaker) */
K_MSGQ_DEFINE(tx_msgq, BLOCK_BYTES, 8, 4);

/* Mono uplink mix fed back to the encoder for conference loopback.
 * Depth 2 = double-buffered at 10 ms each; one frame being consumed
 * by the encoder while the decoder produces the next. */
K_MSGQ_DEFINE(uplink_mix_q, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 2, 4);

/* Uplink Decoder: encoded frames queued from iso_recv, decoded in a thread */
struct uplink_frame {
    uint16_t len;
    uint8_t  data[CONFIG_BT_ISO_TX_MTU];
};
/* One message queue per RX BIS */
#define NUM_RX_BIS (CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT - 1)
struct k_msgq uplink_rx_q[NUM_RX_BIS];
char __aligned(4) uplink_rx_q_buffer[NUM_RX_BIS][8 * sizeof(struct uplink_frame)];
K_SEM_DEFINE(uplink_rx_sem, 0, 100);

K_THREAD_STACK_DEFINE(uplink_dec_stack, 6144);
static struct k_thread uplink_dec_thread_data;

/* RX Thread */
K_THREAD_STACK_DEFINE(rx_stack, 4096);
static struct k_thread rx_thread_data;

/* TX Thread */
K_THREAD_STACK_DEFINE(tx_stack, 2048);
static struct k_thread tx_thread_data;

/* Encoder / Decoder Thread Configuration */
#define ENCODER_STACK_SIZE 4096
#define ENCODER_PRIORITY   5
#define DECODER_PRIORITY   5

/* ISO / BAP Configuration */
#define NUM_PRIME_PACKETS 2

/* -------------------------------------------------------------------------- */
/*                               Globals & Objects                            */
/* -------------------------------------------------------------------------- */

/* BAP Objects */
static struct bt_bap_broadcast_source *broadcast_source;
static struct bt_bap_stream streams[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];

static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* LC3 Codec Objects */
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_16k_t lc3_encoder_mem;
static lc3_decoder_t lc3_decoders[NUM_RX_BIS];
static lc3_decoder_mem_16k_t lc3_decoder_mems[NUM_RX_BIS];
static int16_t send_pcm_data[PCM_SAMPLES_PER_FRAME];

/* Encoder Thread Objects */
K_THREAD_STACK_DEFINE(encoder_stack, ENCODER_STACK_SIZE);
static struct k_thread encoder_thread_data;

/* Encoded Audio FIFO */
/* Size of 2 ensures double buffering: one being sent, one being prepared.
 * This minimizes latency (keeping buffer shallow) while preventing underruns.
 * For true low latency, this queue should be kept as empty as possible.
 */
K_MSGQ_DEFINE(lc3_encoded_q, CONFIG_BT_ISO_TX_MTU, 2, 4);

/* ISO / BAP Objects */
NET_BUF_POOL_FIXED_DEFINE(
    bis_tx_pool, CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT *NUM_PRIME_PACKETS,
    BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), CONFIG_BT_CONN_TX_USER_DATA_SIZE,
    NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT);

static struct bt_iso_chan *bis[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
static struct bt_iso_chan
    bis_iso_chan[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];

static uint16_t seq_num;

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

/* -------------------------------------------------------------------------- */
/*                               I2S / RX Thread                              */
/* -------------------------------------------------------------------------- */

static inline void downmix_stereo_block_to_mono(const int16_t *stereo, int16_t *mono)
{
    for (size_t i = 0; i < FRAMES_PER_BLOCK; i++)
    {
        int32_t L = stereo[2 * i + 0];
        int32_t R = stereo[2 * i + 1];
        mono[i] = (int16_t)((L + R) / 2);
    }
}

/* I2S TX Thread: feeds decoded uplink audio to the speaker */
static void tx_thread_fn(void *p1, void *p2, void *p3)
{
    int err;
    int ret;
    printk("TX thread: running\n");

    while (1) {
        uint8_t in[BLOCK_BYTES];

        /* Monitor I2S buffer depth via utilized slabs */
        uint32_t free_slabs = k_mem_slab_num_free_get(&tx_slab);
        uint32_t in_i2s = TX_BLOCK_COUNT - free_slabs;

        /* If I2S has < 2 blocks (20 ms), feed immediately to avoid underrun */
        if (in_i2s < 2) {
            if (k_msgq_get(&tx_msgq, in, K_NO_WAIT) != 0) {
                static int sil_cnt = 0;
                if ((sil_cnt++ % 50) == 0) {
                    printk("Uplink TX: injecting silence (depth=%d)\n", in_i2s);
                }
                memset(in, 0, BLOCK_BYTES);
            }
        } else {
            if (k_msgq_get(&tx_msgq, in, K_MSEC(5)) != 0) {
                continue;
            }
        }

        void *txblk;
        err = k_mem_slab_alloc(&tx_slab, &txblk, K_FOREVER);
        if (err) {
            printk("TX slab alloc failed\n");
            continue;
        }
        memcpy(txblk, in, BLOCK_BYTES);

        while ((ret = i2s_write(i2s_dev, txblk, BLOCK_BYTES)) == -EAGAIN) {
            k_msleep(1);
        }

        if (ret < 0) {
            printk("I2S TX write failed: %d. Recovering...\n", ret);
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

static void rx_thread_func(void *p1, void *p2, void *p3)
{
    int err;
    uint8_t stereo_buf[BLOCK_BYTES];
    int16_t mono_frame[PCM_SAMPLES_PER_FRAME];

    printk("RX thread started: capturing audio...\n");
    while (1)
    {
        size_t got = sizeof(stereo_buf);
        err = i2s_buf_read(i2s_dev, stereo_buf, &got);
        if (err == 0 && got == BLOCK_BYTES)
        {
            downmix_stereo_block_to_mono((const int16_t *)stereo_buf, mono_frame);
            /* Push to encoder queue (drop oldest if full) */
            if (k_msgq_put(&pcm_msgq, mono_frame, K_NO_WAIT) != 0)
            {
                int16_t drop[PCM_SAMPLES_PER_FRAME];
                (void)k_msgq_get(&pcm_msgq, drop, K_NO_WAIT);
                (void)k_msgq_put(&pcm_msgq, mono_frame, K_NO_WAIT);
            }
        }
        else if (err != -EAGAIN && err != -ETIMEDOUT)
        {
            printk("I2S read error: %d\n", err);
            k_sleep(K_MSEC(10)); /* backoff */
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                               Encoder Thread                               */
/* -------------------------------------------------------------------------- */

static void encoder_thread_func(void *arg1, void *arg2, void *arg3) {
  int ret;
  uint8_t encoded_buf[CONFIG_BT_ISO_TX_MTU];

  /* Extract encoding parameters from the active BAP preset */
  /* Note: These must match the lc3_setup_encoder parameters used in main */
  /* 16_2_1 implies 40 bytes per frame */
  int octets_per_frame = preset_active.qos.sdu;

  while (1) {
    /* Wait for PCM frame (from I2S RX thread) */
    /* This naturally paces the encoder at the audio sample rate */
    ret = k_msgq_get(&pcm_msgq, send_pcm_data, K_FOREVER);
    if (ret != 0) {
        continue;
    }

    /* Mix in uplink loopback (conference bridge) if a frame is ready.
     * K_NO_WAIT: if the decoder hasn't produced yet (no remote senders,
     * or timing slip), encode mic-only audio without blocking. */
    int16_t uplink_mono[PCM_SAMPLES_PER_FRAME];
    if (k_msgq_get(&uplink_mix_q, uplink_mono, K_NO_WAIT) == 0) {
        for (int i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
            int32_t s = (int32_t)send_pcm_data[i] + (int32_t)uplink_mono[i];
            if (s > 32767)       { s = 32767; }
            else if (s < -32768) { s = -32768; }
            send_pcm_data[i] = (int16_t)s;
        }
    }

    ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, send_pcm_data, 1,
                     octets_per_frame, encoded_buf);

    if (ret == -1) {
      printk("LC3 encode failed in thread\n");
      continue;
    }

    /* Push to ISO TX queue */
    k_msgq_put(&lc3_encoded_q, encoded_buf, K_FOREVER);
  }
}

/* -------------------------------------------------------------------------- */
/*                           Uplink Decoder Thread                            */
/* -------------------------------------------------------------------------- */

static void uplink_decoder_thread_func(void *arg1, void *arg2, void *arg3)
{
    printk("Uplink decoder thread started (mixing %d BISes)\\n", NUM_RX_BIS);

    while (1) {
        /* Pace by waiting for at least one packet from iso_recv. 
         * 12 ms timeout (> 10 ms ISO interval): if no packet arrives, run PLC
         * to keep the output pipeline ticking even during sync loss. */
        k_sem_take(&uplink_rx_sem, K_MSEC(12));
        k_sem_reset(&uplink_rx_sem);

        int32_t mixed_pcm[PCM_SAMPLES_PER_FRAME] = {0};

        /* Process all RX BISes */
        for (int i = 0; i < NUM_RX_BIS; i++) {
            struct uplink_frame frame;
            int16_t stream_pcm[PCM_SAMPLES_PER_FRAME];
            int ret;

            ret = k_msgq_get(&uplink_rx_q[i], &frame, K_NO_WAIT);

            if (ret == 0 && frame.len > 0) {
                ret = lc3_decode(lc3_decoders[i], frame.data, frame.len,
                                 LC3_PCM_FORMAT_S16, stream_pcm, 1);
                if (ret != 0) {
                    memset(stream_pcm, 0, sizeof(stream_pcm));
                }
            } else {
                /* Timeout (no packet) or ghost packet (len == 0) → PLC */
                ret = lc3_decode(lc3_decoders[i], NULL, preset_active.qos.sdu,
                                 LC3_PCM_FORMAT_S16, stream_pcm, 1);
                if (ret != 0) {
                    memset(stream_pcm, 0, sizeof(stream_pcm));
                }
            }

            /* Mix into 32-bit accumulator to avoid overflow during summation */
            for (int s = 0; s < PCM_SAMPLES_PER_FRAME; s++) {
                mixed_pcm[s] += stream_pcm[s];
            }
        }

        /* Upmix mixed results (mono) → stereo (L=R) with anti-clipping protection, 
         * then push to I2S TX queue */
        int16_t stereo_buf[PCM_SAMPLES_PER_FRAME * 2];
        for (int i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
            int32_t sample = mixed_pcm[i];
            if (sample > 32767) {
                sample = 32767;
            } else if (sample < -32768) {
                sample = -32768;
            }
            stereo_buf[2 * i]     = (int16_t)sample; /* Left  */
            stereo_buf[2 * i + 1] = (int16_t)sample; /* Right */
        }

        /* Publish clipped mono mix for conference loopback into the encoder.
         * stereo_buf[2*i] == stereo_buf[2*i+1] (L=R), take left channel.
         * K_NO_WAIT: drop if encoder hasn't consumed the previous frame yet. */
        int16_t mono_mix[PCM_SAMPLES_PER_FRAME];
        for (int i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
            mono_mix[i] = stereo_buf[2 * i];
        }
        if (k_msgq_put(&uplink_mix_q, mono_mix, K_NO_WAIT) != 0) {
            static int mix_overrun_cnt = 0;
            if ((mix_overrun_cnt++ % 100) == 0) {
                printk("uplink_mix_q full - encoder lagging (cnt=%d)\n",
                       mix_overrun_cnt);
            }
        }

        if (k_msgq_put(&tx_msgq, stereo_buf, K_NO_WAIT) != 0) {
            static int overrun_cnt = 0;
            if ((overrun_cnt++ % 100) == 0) {
                printk("Uplink TX queue full - dropping frame (cnt=%d)\\n", overrun_cnt);
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                               Volume Control                               */
/* -------------------------------------------------------------------------- */

static const struct adc_dt_spec pot = ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), potentiometer);
static const struct gpio_dt_spec p1_09_en = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), enable_gpios);

void volume_control_thread(void *p1, void *p2, void *p3)
{
#define POT_MUTE_ON_MV  100
#define POT_MUTE_OFF_MV 120
#define POT_MAX_MV      3300
#define MAX_VOL_Q15     13668

    static bool    muted    = true;
    static uint8_t last_reg = 0xFF; /* sentinel: any value > 40 forces write on first iteration */

    int err;
    int16_t sample;
    struct adc_sequence seq = {0};

    adc_sequence_init_dt(&pot, &seq);
    seq.buffer = &sample;
    seq.buffer_size = sizeof(sample);

    while (1) {
        err = adc_read_dt(&pot, &seq);
        if (err) {
            printk("adc_read_dt() failed: %d\n", err);
            k_sleep(K_MSEC(200));
            continue;
        }

        int32_t mv = sample;
        err = adc_raw_to_millivolts_dt(&pot, &mv);
        if (err) {
            printk("adc_raw_to_millivolts_dt() failed: %d (raw=%d)\n", err, sample);
            k_sleep(K_MSEC(200));
            continue;
        }

        gpio_pin_set_dt(&p1_09_en, (mv > 0) ? 1 : 0);

        if (mv <= POT_MUTE_ON_MV) {
            if (!muted) {
                max9867_set_mute(true);
                muted = true;
                last_reg = 0xFF; /* force re-write after unmute */
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
        int32_t span    = POT_MAX_MV - POT_MUTE_OFF_MV;
        int32_t mv_rel  = (mv - POT_MUTE_OFF_MV);
        uint16_t vol_q15 = (uint16_t)((mv_rel * (int32_t)MAX_VOL_Q15 + span / 2) / span);

        /* Only write to MAX9867 when the hardware register byte would change.
         * The driver maps vol_q15 to a 5-bit register (41 steps). ADC noise
         * causes vol_q15 to jitter by a few units each reading, but the
         * register value only changes when the pot moves by ~177 mV. */
        uint8_t new_reg = (uint8_t)(40 - ((int32_t)vol_q15 * 41 / 0x7FFF));
        if (new_reg != last_reg) {
            (void)max9867_set_volume((int16_t)vol_q15, (int16_t)MAX_VOL_Q15);
            last_reg = new_reg;
        }

        k_sleep(K_MSEC(100));
    }
}

K_THREAD_DEFINE(vol_thread_id, 1024, volume_control_thread, NULL, NULL, NULL,
                7, 0, 0);

/* -------------------------------------------------------------------------- */
/*                               ISO Callbacks                                */
/* -------------------------------------------------------------------------- */

static void iso_connected(struct bt_iso_chan *chan) {
  printk("ISO Channel %p connected\n", chan);
  seq_num = 0U;
  k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason) {
  printk("ISO Channel %p disconnected with reason 0x%02x\n", chan, reason);
}

static void iso_sent(struct bt_iso_chan *chan) {
  if (chan == bis[0]) {
    int err;
    struct net_buf *buf;

    buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
    if (!buf) {
      printk("iso_sent: net_buf pool exhausted\n");
      return;
    }

    net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

    uint8_t enc_data[CONFIG_BT_ISO_TX_MTU];

    /* Fetch already encoded frame from queue (non-blocking) */
    if (k_msgq_get(&lc3_encoded_q, enc_data, K_NO_WAIT) != 0) {
      /* Underrun: encoder hasn't produced a frame yet — send silence */
      static int tx_underrun_cnt = 0;
      if ((tx_underrun_cnt++ % 100) == 0) {
          printk("TX underrun - sending silence (cnt=%d)\n", tx_underrun_cnt);
      }
      memset(enc_data, 0, sizeof(enc_data));
    }
    net_buf_add_mem(buf, enc_data, sizeof(enc_data));

    err = bt_iso_chan_send(chan, buf, seq_num);
    if (err < 0) {
      printk("Unable to broadcast data on channel %p : %d\n", chan, err);
      net_buf_unref(buf);
      return;
    }

    seq_num++;
  }
}

static void iso_recv(struct bt_iso_chan *chan,
                     const struct bt_iso_recv_info *info, struct net_buf *buf) {
  struct uplink_frame frame;

  if (buf && buf->len == CONFIG_BT_ISO_TX_MTU && (info->flags & BT_ISO_FLAGS_VALID)) {
      frame.len = buf->len;
      memcpy(frame.data, buf->data, buf->len);
  } else {
      /* Lost or invalid packet: push a ghost frame to trigger PLC in decoder */
      frame.len = 0;
  }

  /* Find which BIS this is */
  int bis_idx = -1;
  for (int i = 0; i < NUM_RX_BIS; i++) {
      if (chan == bis[i]) {
          bis_idx = i;
          break;
      }
  }

  if (bis_idx >= 0 && bis_idx < NUM_RX_BIS) {
      /* K_NO_WAIT: must not block in BT RX callback context */
      if (k_msgq_put(&uplink_rx_q[bis_idx], &frame, K_NO_WAIT) == 0) {
          k_sem_give(&uplink_rx_sem);
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
/*                               Setup Helpers                                */
/* -------------------------------------------------------------------------- */

static int setup_broadcast_source(struct bt_bap_broadcast_source **source) {
  struct bt_bap_broadcast_source_stream_param
      stream_params[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
  struct bt_bap_broadcast_source_subgroup_param subgroup_param[1];
  struct bt_bap_broadcast_source_param create_param = {0};

  uint8_t left_loc[] = {
      BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
                          BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_LEFT))};
  uint8_t right_loc[] = {
      BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
                          BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_RIGHT))};

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

static int setup_extended_adv(struct bt_le_ext_adv **adv) {
  int err;

  err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, NULL, adv);
  if (err) {
    printk("Failed to create advertising set (err %d)\n", err);
    return err;
  }

  err = bt_le_ext_adv_set_data(*adv, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    printk("Failed to set advertising data (err %d)\n", err);
    return err;
  }

  return 0;
}

static int setup_periodic_adv(struct bt_le_ext_adv *adv) {
  int err;

  err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
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

static int create_big(struct bt_le_ext_adv *adv, struct bt_iso_big **big) {
  int err;

  for (size_t i = 0; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
    bis_iso_chan[i].ops = &iso_ops;
    bis_iso_chan[i].qos = &bis_iso_qos;
    bis[i] = &bis_iso_chan[i];
  }

  for (uint8_t chan = 0U; chan < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT;
       chan++) {
    bis[chan]->qos->tx->rtn = preset_active.qos.rtn;
  }

  big_create_param.interval = preset_active.qos.interval;
  big_create_param.latency = preset_active.qos.latency;
  big_create_param.bis_channels = bis;

  err = bt_iso_big_create(adv, &big_create_param, big);
  if (err) {
    printk("Failed to create BIG (err %d)\n", err);
    return err;
  }

  for (uint8_t chan = 0U; chan < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT;
       chan++) {
    printk("Waiting for BIG complete chan %u...\n", chan);
    err = k_sem_take(&sem_big_cmplt, K_FOREVER);
    if (err) {
      printk("failed (err %d)\n", err);
      return err;
    }
    printk("BIG create complete chan %u.\n", chan);
  }

  return 0;
}

static int setup_iso_datapaths(void) {
  int err;

  for (uint8_t chan = 0U; chan < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT;
       chan++) {
    printk("Setting data path chan %u...\n", chan);

    const struct bt_iso_chan_path hci_path = {
        .pid = BT_ISO_DATA_PATH_HCI,
        .format = BT_HCI_CODING_FORMAT_TRANSPARENT,
    };

    uint8_t dir = (chan == 0) ? BT_HCI_DATAPATH_DIR_HOST_TO_CTLR
                              : BT_HCI_DATAPATH_DIR_CTLR_TO_HOST;

    err = bt_iso_setup_data_path(&bis_iso_chan[chan], dir, &hci_path);
    if (err != 0) {
      printk("Failed to setup ISO data path: %d\n", err);
      return err;
    }

    printk("Setting data path complete chan %u.\n", chan);
  }

  return 0;
}

static void prime_and_start_iso_transmission(void) {
  for (int i = 0; i < NUM_PRIME_PACKETS; i++) {
    iso_sent(bis[0]);
  }
}

/* -------------------------------------------------------------------------- */
/*                               Main Function                                */
/* -------------------------------------------------------------------------- */

int main(void) {
  struct bt_le_ext_adv *adv;
  struct bt_iso_big *big;
  int err;

  printk("Starting GRPTLK Broadcaster\n");

  /* --- Volume Control Setup --- */
  if (!adc_is_ready_dt(&pot)) {
      printk("ADC device not ready\n");
      return -EIO;
  }
  err = adc_channel_setup_dt(&pot);
  if (err) {
      printk("adc_channel_setup_dt() failed: %d\n", err);
      return -EIO;
  }

  if (!gpio_is_ready_dt(&p1_09_en)) {
      printk("P1.09 GPIO not ready\n");
  } else {
      gpio_pin_configure_dt(&p1_09_en, GPIO_OUTPUT_INACTIVE);
  }

  /* --- Audio Setup --- */
  printk("Initializing MAX9867 codec...\n");
  err = max9867_init();
  if (err) {
      printk("MAX9867 init failed: %d\n", err);
      return err;
  }

  i2s_dev = DEVICE_DT_GET(DT_ALIAS(i2s_node0));
  if (!device_is_ready(i2s_dev)) {
      printk("I2S device not ready\n");
      return -ENODEV;
  }

  struct i2s_config rx_cfg = {
      .word_size = 16,
      .channels = CHANNELS,
      .format = I2S_FMT_DATA_FORMAT_I2S,
      .options = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE,
      .frame_clk_freq = SAMPLE_RATE_HZ,
      .mem_slab = &rx_slab,
      .block_size = BLOCK_BYTES,
      .timeout = 2000,
  };

  err = i2s_configure(i2s_dev, I2S_DIR_RX, &rx_cfg);
  if (err) {
      printk("I2S configure failed: %d\n", err);
      return err;
  }

  struct i2s_config tx_cfg = {
      .word_size = 16,
      .channels = CHANNELS,
      .format = I2S_FMT_DATA_FORMAT_I2S,
      .options = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE,
      .frame_clk_freq = SAMPLE_RATE_HZ,
      .mem_slab = &tx_slab,
      .block_size = BLOCK_BYTES,
      .timeout = 1000,
  };

  err = i2s_configure(i2s_dev, I2S_DIR_TX, &tx_cfg);
  if (err) {
      printk("I2S TX configure failed: %d\n", err);
      return err;
  }

  /* Prefill silent TX blocks so I2S has data to start with (60 ms cushion) */
  for (int i = 0; i < 6; i++) {
      void *txblk;
      if (k_mem_slab_alloc(&tx_slab, &txblk, K_FOREVER) == 0) {
          memset(txblk, 0, BLOCK_BYTES);
          err = i2s_write(i2s_dev, txblk, BLOCK_BYTES);
          if (err < 0) {
              printk("I2S TX prefill failed: %d\n", err);
              k_mem_slab_free(&tx_slab, &txblk);
          }
      }
  }

  /* Start TX thread before triggering I2S */
  k_thread_create(&tx_thread_data, tx_stack, K_THREAD_STACK_SIZEOF(tx_stack),
                  tx_thread_fn, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
  k_thread_name_set(&tx_thread_data, "i2s_tx");

  err = i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_START);
  if (err) {
      printk("I2S start failed: %d\n", err);
      return err;
  }

  /* Start RX thread */
  k_thread_create(&rx_thread_data, rx_stack, K_THREAD_STACK_SIZEOF(rx_stack),
                  rx_thread_func, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
  k_thread_name_set(&rx_thread_data, "i2s_rx");

  /* Dynamically initialize message queues for uplinks */
  for (int i = 0; i < NUM_RX_BIS; i++) {
      k_msgq_init(&uplink_rx_q[i], uplink_rx_q_buffer[i], 
                  sizeof(struct uplink_frame), 8);
  }

  /* --- LC3 Setup --- */
  printk("Initializing lc3 encoder\\n");
  memset(send_pcm_data, 0, sizeof(send_pcm_data));

  /* Using parameters derived from the active BAP preset
     - Dur: preset_active.qos.interval (10ms for 16_2_1)
     - Hz:  16000 (implied by 16_2_1, cannot be easily extracted at runtime
     without parsing)
  */
  lc3_encoder =
      lc3_setup_encoder(preset_active.qos.interval, 16000, 0, &lc3_encoder_mem);

  if (lc3_encoder == NULL) {
    printk("ERROR: Failed to setup LC3 encoder - wrong parameters?\\n");
    return -EIO;
  }
  
  for (int i = 0; i < NUM_RX_BIS; i++) {
      lc3_decoders[i] = 
          lc3_setup_decoder(preset_active.qos.interval, 16000, 0, &lc3_decoder_mems[i]);

      if (lc3_decoders[i] == NULL) {
        printk("ERROR: Failed to setup LC3 decoder %d\\n", i);
        return -EIO;
      }
  }

  /* Start encoding thread */
  k_thread_create(&encoder_thread_data, encoder_stack,
                  K_THREAD_STACK_SIZEOF(encoder_stack), encoder_thread_func,
                  NULL, NULL, NULL, ENCODER_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&encoder_thread_data, "lc3_encoder");

  /* Start uplink decoder thread (decodes incoming mic audio from receiver) */
  k_thread_create(&uplink_dec_thread_data, uplink_dec_stack,
                  K_THREAD_STACK_SIZEOF(uplink_dec_stack),
                  uplink_decoder_thread_func, NULL, NULL, NULL,
                  DECODER_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&uplink_dec_thread_data, "uplink_dec");

  /* --- Bluetooth Setup --- */
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

  prime_and_start_iso_transmission();
  return 0;
}