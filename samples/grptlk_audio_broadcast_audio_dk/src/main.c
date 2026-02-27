#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "audio_i2s.h"
#include "lc3.h"
#include "cs47l63.h"
#include "cs47l63_comm.h"
#include "cs47l63_reg_conf.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

#if !DT_HAS_COMPAT_STATUS_OKAY(cirrus_cs47l63)
#error "No cirrus,cs47l63 node available. Use nrf5340_audio_dk/nrf5340/cpuapp."
#endif

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
#define SAMPLE_RATE_HZ AUDIO_I2S_SAMPLE_RATE_HZ
#define FRAMES_PER_BLOCK AUDIO_I2S_SAMPLES_PER_BLOCK
#define MIC_PEAK_DETECT_THRESHOLD 64
#define UPLINK_DEC_VU_BAR_WIDTH 24

/* I2S RX Buffers (double-buffered as in nrf5340_audio) */
static uint32_t i2s_rx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_rx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_a[AUDIO_I2S_WORDS_PER_BLOCK];
static uint32_t i2s_tx_buf_b[AUDIO_I2S_WORDS_PER_BLOCK];

static cs47l63_t codec_driver;

/* PCM Queue (Mono frames for Encoder) */
K_MSGQ_DEFINE(pcm_msgq, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 16, 4);

/* Queue for decoded uplink PCM (stereo words) to be played on speaker/headset */
K_MSGQ_DEFINE(tx_msgq, AUDIO_I2S_BLOCK_BYTES, 8, 4);

/* Uplink Decoder: encoded frames queued from iso_recv */
struct uplink_frame {
    uint16_t len;
    uint8_t data[CONFIG_BT_ISO_TX_MTU];
};

/* BIS0 is downlink TX (host->ctrl), BIS1..N are uplinks (ctrl->host) */
#define NUM_RX_BIS (CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT - 1)
static struct k_msgq uplink_rx_q[NUM_RX_BIS];
static char __aligned(4) uplink_rx_q_buffer[NUM_RX_BIS][8 * sizeof(struct uplink_frame)];
static K_SEM_DEFINE(uplink_rx_sem, 0, 100);

/* Uplink RX diagnostics */
static uint32_t iso_recv_total_cnt;
static uint32_t iso_recv_valid_cnt;
static uint32_t iso_recv_invalid_cnt;
static uint32_t iso_recv_flag_invalid_cnt;
static uint32_t iso_recv_bad_len_cnt;
static uint32_t iso_recv_unknown_chan_cnt;
static uint32_t iso_recv_qdrop_cnt;
static uint32_t iso_recv_chan_cnt[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];

/* Encoder Thread Configuration */
#define ENCODER_STACK_SIZE 4096
#define ENCODER_PRIORITY   5
#define DECODER_STACK_SIZE 6144
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

/* Uplink decoder thread */
K_THREAD_STACK_DEFINE(uplink_dec_stack, DECODER_STACK_SIZE);
static struct k_thread uplink_dec_thread_data;

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

static int codec_mic_path_prepare(void)
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

    /* Enable analog output path for uplink playback from ASP1 RX. */
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

static int selected_mic_channel = -1; /* -1=auto, 0=left, 1=right */

static inline int32_t abs_s16(int16_t s)
{
    return (s < 0) ? -(int32_t)s : (int32_t)s;
}

static void uplink_decoded_vu_meter_update(const int32_t *mixed_pcm)
{
    static uint32_t frame_cnt;
    static uint32_t held_peak;
    uint32_t peak = 0U;
    uint32_t level;
    char bar[UPLINK_DEC_VU_BAR_WIDTH + 1];

    for (size_t i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
        int32_t s = mixed_pcm[i];
        uint32_t a;

        if (s > 32767) {
            s = 32767;
        } else if (s < -32768) {
            s = -32768;
        }

        a = (uint32_t)abs_s16((int16_t)s);
        if (a > peak) {
            peak = a;
        }
    }

    /* Fast attack, gentle decay. */
    if (peak > held_peak) {
        held_peak = peak;
    } else {
        held_peak = (held_peak * 7U) / 8U;
    }

    /* Decoder cadence is 10 ms; print every 20 ms. */
    if ((frame_cnt++ & 0x1U) != 0U) {
        return;
    }

    level = (held_peak * UPLINK_DEC_VU_BAR_WIDTH) / 32767U;
    if (level > UPLINK_DEC_VU_BAR_WIDTH) {
        level = UPLINK_DEC_VU_BAR_WIDTH;
    }

    for (size_t i = 0; i < UPLINK_DEC_VU_BAR_WIDTH; i++) {
        bar[i] = (i < level) ? '#' : '.';
    }
    bar[UPLINK_DEC_VU_BAR_WIDTH] = '\0';

    printk("UL RX VU [%s] peak=%5u\n", bar, held_peak);
}

static inline uint8_t mic_channel_pick(int32_t left_peak, int32_t right_peak)
{
    if (selected_mic_channel >= 0) {
        return (uint8_t)selected_mic_channel;
    }

    return (right_peak > left_peak) ? 1U : 0U;
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
                                      int32_t *left_peak, int32_t *right_peak)
{
    int32_t l_peak = 0;
    int32_t r_peak = 0;

    for (size_t i = 0; i < FRAMES_PER_BLOCK; i++) {
        int16_t left;
        int16_t right;
        i2s_word_unpack(rx_words[i], &left, &right);

        const int32_t la = abs_s16(left);
        const int32_t ra = abs_s16(right);

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

static void extract_selected_channel_to_mono(const uint32_t *rx_words, int16_t *mono,
                                             uint8_t channel)
{
    for (size_t i = 0; i < FRAMES_PER_BLOCK; i++) {
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
        printk("MIC channel auto-selected: %s\n", selected_mic_channel == 0 ? "LEFT" : "RIGHT");
    }

    ch = mic_channel_pick(left_peak, right_peak);
    extract_selected_channel_to_mono(rx_words, mono_frame, ch);
    pcm_msgq_push(mono_frame);
}

static void tx_buffer_fill(uint32_t *tx_words)
{
    static uint32_t silence_inject_cnt;

    if (k_msgq_get(&tx_msgq, tx_words, K_NO_WAIT) != 0) {
        memset(tx_words, 0, AUDIO_I2S_BLOCK_BYTES);
        if ((silence_inject_cnt++ % 200U) == 0U) {
            printk("Uplink RX playback: injecting silence (cnt=%u)\n", silence_inject_cnt);
        }
    }
}

static void i2s_block_complete(uint32_t *rx_buf_released, const uint32_t *tx_buf_released)
{
    static uint32_t i2s_requeue_err_cnt;
    static uint8_t tx_buf_sel;
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
            printk("audio_i2s_set_next_buf failed: %d (cnt=%u)\n", err,
                   i2s_requeue_err_cnt);
        }
    }
}

static int i2s_capture_start(void)
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
/*                               Encoder Thread                               */
/* -------------------------------------------------------------------------- */

static void encoder_thread_func(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

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
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  printk("Uplink decoder thread started (rx BIS count=%d)\n", NUM_RX_BIS);

  while (1) {
    int32_t mixed_pcm[PCM_SAMPLES_PER_FRAME] = { 0 };
    uint32_t tx_words[AUDIO_I2S_WORDS_PER_BLOCK];
    static uint32_t tx_q_drop_cnt;
    static uint32_t dec_err_cnt;

    /* Pace at ~10 ms; if packets are missing, decoder runs PLC. */
    (void)k_sem_take(&uplink_rx_sem, K_MSEC(12));
    k_sem_reset(&uplink_rx_sem);

    for (int i = 0; i < NUM_RX_BIS; i++) {
      struct uplink_frame frame;
      int16_t stream_pcm[PCM_SAMPLES_PER_FRAME];
      int ret;

      ret = k_msgq_get(&uplink_rx_q[i], &frame, K_NO_WAIT);
      if (ret == 0 && frame.len > 0U) {
        ret = lc3_decode(lc3_decoders[i], frame.data, frame.len, LC3_PCM_FORMAT_S16,
                         stream_pcm, 1);
      } else {
        ret = lc3_decode(lc3_decoders[i], NULL, preset_active.qos.sdu, LC3_PCM_FORMAT_S16,
                         stream_pcm, 1);
      }

      if (ret < 0) {
        dec_err_cnt++;
        if ((dec_err_cnt % 100U) == 0U) {
          printk("UL RX decode errors (cnt=%u)\n", dec_err_cnt);
        }
        memset(stream_pcm, 0, sizeof(stream_pcm));
      }

      for (int s = 0; s < PCM_SAMPLES_PER_FRAME; s++) {
        mixed_pcm[s] += stream_pcm[s];
      }
    }

    uplink_decoded_vu_meter_update(mixed_pcm);

    /* Mono mix to stereo words for I2S TX (L=R). */
    for (int i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
      int32_t sample = mixed_pcm[i];
      uint16_t out16;

      if (sample > 32767) {
        sample = 32767;
      } else if (sample < -32768) {
        sample = -32768;
      }

      out16 = (uint16_t)((int16_t)sample);
      tx_words[i] = ((uint32_t)out16 << 16) | out16;
    }

    if (k_msgq_put(&tx_msgq, tx_words, K_NO_WAIT) != 0) {
      if ((tx_q_drop_cnt++ % 100U) == 0U) {
        printk("Uplink RX playback queue full - dropping frame (cnt=%u)\n",
               tx_q_drop_cnt);
      }
    }
  }
}

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
  int chan_idx = -1;
  int bis_idx = -1;
  bool has_payload;
  bool flag_valid;

  has_payload = ((buf != NULL) && (buf->len > 0U) && (buf->len <= CONFIG_BT_ISO_TX_MTU));
  flag_valid = ((info->flags & BT_ISO_FLAGS_VALID) != 0U);

  iso_recv_total_cnt++;

  /* Match robust behavior from working ISO samples: accept payload if present and
   * full-size, even if BT_ISO_FLAGS_VALID is not set by controller metadata.
   */
  if (has_payload && (buf->len == CONFIG_BT_ISO_TX_MTU)) {
    frame.len = CONFIG_BT_ISO_TX_MTU;
    memcpy(frame.data, buf->data, CONFIG_BT_ISO_TX_MTU);
    iso_recv_valid_cnt++;

    if (!flag_valid) {
      iso_recv_flag_invalid_cnt++;
    }
  } else {
    /* Invalid or lost packet: decoder will run PLC for this stream. */
    frame.len = 0U;
    iso_recv_invalid_cnt++;

    if (has_payload && (buf->len != CONFIG_BT_ISO_TX_MTU)) {
      iso_recv_bad_len_cnt++;
    }
  }

  for (int i = 0; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
    if (chan == bis[i]) {
      chan_idx = i;
      iso_recv_chan_cnt[i]++;
      break;
    }
  }

  /* Expected mapping in group-talk: BIS1 downlink TX, BIS2..BIS5 uplink RX.
   * Fallback maps chan 0 -> queue 0 for diagnostics in case controller/host
   * handle mapping differs.
   */
  if (chan_idx > 0) {
    bis_idx = chan_idx - 1;
  } else if (chan_idx == 0) {
    bis_idx = 0;
  }

  if ((iso_recv_total_cnt % 200U) == 0U) {
    printk("UL RX stats: total=%u valid=%u invalid=%u flag_inv=%u bad_len=%u qdrop=%u unk=%u ch[0..4]=[%u,%u,%u,%u,%u]\n",
           iso_recv_total_cnt, iso_recv_valid_cnt, iso_recv_invalid_cnt,
           iso_recv_flag_invalid_cnt, iso_recv_bad_len_cnt, iso_recv_qdrop_cnt,
           iso_recv_unknown_chan_cnt, iso_recv_chan_cnt[0],
           iso_recv_chan_cnt[1], iso_recv_chan_cnt[2], iso_recv_chan_cnt[3],
           iso_recv_chan_cnt[4]);
  }

  if (chan_idx < 0) {
    iso_recv_unknown_chan_cnt++;
    if ((iso_recv_unknown_chan_cnt % 20U) == 0U) {
      printk("UL RX: unknown chan %p (flags=0x%02x len=%u)\n", chan, info->flags,
             buf ? buf->len : 0U);
    }
    return;
  }

  if (bis_idx >= 0) {
    if (k_msgq_put(&uplink_rx_q[bis_idx], &frame, K_NO_WAIT) == 0) {
      k_sem_give(&uplink_rx_sem);
    } else {
      iso_recv_qdrop_cnt++;
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

  /* --- Audio Setup --- */
  printk("Initializing CS47L63 codec and mic path...\n");
  err = codec_mic_path_prepare();
  if (err) {
      printk("Codec prepare failed: %d\n", err);
      return err;
  }

  for (int i = 0; i < NUM_RX_BIS; i++) {
      k_msgq_init(&uplink_rx_q[i], uplink_rx_q_buffer[i],
                  sizeof(struct uplink_frame), 8);
  }

  /* Match nRF5340 Audio reference flow: toggle FLL before starting I2S stream. */
  err = codec_reg_conf_write(FLL_toggle, ARRAY_SIZE(FLL_toggle));
  if (err) {
      printk("Codec FLL toggle failed: %d\n", err);
      return err;
  }

  err = i2s_capture_start();
  if (err) {
      printk("I2S capture start failed: %d\n", err);
      return err;
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
      lc3_setup_encoder(preset_active.qos.interval, SAMPLE_RATE_HZ, 0, &lc3_encoder_mem);

  if (lc3_encoder == NULL) {
    printk("ERROR: Failed to setup LC3 encoder - wrong parameters?\\n");
    return -EIO;
  }

  /* Start encoding thread */
  k_thread_create(&encoder_thread_data, encoder_stack,
                  K_THREAD_STACK_SIZEOF(encoder_stack), encoder_thread_func,
                  NULL, NULL, NULL, ENCODER_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&encoder_thread_data, "lc3_encoder");

  for (int i = 0; i < NUM_RX_BIS; i++) {
    lc3_decoders[i] =
        lc3_setup_decoder(preset_active.qos.interval, SAMPLE_RATE_HZ, 0, &lc3_decoder_mems[i]);
    if (lc3_decoders[i] == NULL) {
      printk("ERROR: Failed to setup LC3 decoder %d\n", i);
      return -EIO;
    }
  }

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
