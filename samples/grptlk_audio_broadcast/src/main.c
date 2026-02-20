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

/* PCM Queue (Mono frames for Encoder) */
K_MSGQ_DEFINE(pcm_msgq, sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 16, 4);

/* RX Thread */
K_THREAD_STACK_DEFINE(rx_stack, 4096);
static struct k_thread rx_thread_data;

/* Encoder Thread Configuration */
#define ENCODER_STACK_SIZE 4096
#define ENCODER_PRIORITY 5

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
static lc3_decoder_t lc3_decoder;
static lc3_decoder_mem_16k_t lc3_decoder_mem;
static int16_t send_pcm_data[PCM_SAMPLES_PER_FRAME];

/* Encoder Thread Objects */
K_THREAD_STACK_DEFINE(encoder_stack, ENCODER_STACK_SIZE);
static struct k_thread encoder_thread_data;
static uint8_t audio_ctr = 0;

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
static uint8_t iso_data[CONFIG_BT_ISO_TX_MTU] = {0};

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

    buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
    if (!buf) {
      printk("Data buffer allocate timeout\n");
      return;
    }

    net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

    int ret;
    uint8_t enc_data[CONFIG_BT_ISO_TX_MTU];

    /* Fetch already encoded frame from queue (non-blocking) */
    ret = k_msgq_get(&lc3_encoded_q, enc_data, K_NO_WAIT);
    if (ret == 0) {
      net_buf_add_mem(buf, enc_data, sizeof(enc_data));
    } else {
      /* Underrun: Send silence or dummy data */
      static int tx_underrun_cnt = 0;
      if ((tx_underrun_cnt++ % 100) == 0) {
          printk("Broadcaster TX Underrun (Queue Empty) - sending silence (cnt=%d)\n", tx_underrun_cnt);
      }
      memset(iso_data, 0, sizeof(iso_data));
      net_buf_add_mem(buf, iso_data, sizeof(iso_data));
    }

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
  int16_t recv_pcm_data[PCM_SAMPLES_PER_FRAME];
  int err;

  if (buf->len == 0 || !(info->flags & BT_ISO_FLAGS_VALID)) {
    /* PLC */
    err = lc3_decode(lc3_decoder, NULL, preset_active.qos.sdu, LC3_PCM_FORMAT_S16, recv_pcm_data, 1);
  } else {
    err = lc3_decode(lc3_decoder, buf->data, buf->len, LC3_PCM_FORMAT_S16, recv_pcm_data, 1);
  }

  if (err == 0) {
      printk("Uplink RX Decoded: %i %i %i %i\n", recv_pcm_data[0], recv_pcm_data[1], recv_pcm_data[2], recv_pcm_data[3]);
  } else {
      printk("Uplink RX Decode Error: %d\n", err);
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
  printk("Initializing MAX9867 codec...\n");
  err = max9867_init();
  if (err) {
      printk("MAX9867 init failed: %d\n", err);
      /* Proceed anyway? Or return error? For robustness let's return error but maybe continue if just verifying logic without hardware? */
      /* Real HW: Return error. BSIM: Might fail if I2C simulated? Assuming BSIM handles it via mocks or real I2C model. */
      /* If using custom driver in BSIM, it might need I2C model. */
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

  err = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
  if (err) {
      printk("I2S start failed: %d\n", err);
      return err;
  }

  /* Start RX thread */
  k_thread_create(&rx_thread_data, rx_stack, K_THREAD_STACK_SIZEOF(rx_stack),
                  rx_thread_func, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
  k_thread_name_set(&rx_thread_data, "i2s_rx");

  /* --- LC3 Setup --- */
  printk("Initializing lc3 encoder\n");
  memset(send_pcm_data, 0, sizeof(send_pcm_data));

  /* Using parameters derived from the active BAP preset
     - Dur: preset_active.qos.interval (10ms for 16_2_1)
     - Hz:  16000 (implied by 16_2_1, cannot be easily extracted at runtime
     without parsing)
  */
  lc3_encoder =
      lc3_setup_encoder(preset_active.qos.interval, 16000, 0, &lc3_encoder_mem);

  if (lc3_encoder == NULL) {
    printk("ERROR: Failed to setup LC3 encoder - wrong parameters?\n");
    return -EIO;
  }
  
  lc3_decoder = 
      lc3_setup_decoder(preset_active.qos.interval, 16000, 0, &lc3_decoder_mem);

  if (lc3_decoder == NULL) {
    printk("ERROR: Failed to setup LC3 decoder\n");
    return -EIO;
  }

  /* Start encoding thread */
  k_thread_create(&encoder_thread_data, encoder_stack,
                  K_THREAD_STACK_SIZEOF(encoder_stack), encoder_thread_func,
                  NULL, NULL, NULL, ENCODER_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&encoder_thread_data, "lc3_encoder");

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