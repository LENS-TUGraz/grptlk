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

/* LC3 Encoder Objects */
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_16k_t lc3_encoder_mem;
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
    /* In a real app with I2S microphone input:
       - This thread would block waiting for a full PCM buffer from the I2S
       driver (via k_msgq_get or k_sem_take).
       - This ensures the encoding rate matches the hardware sampling rate
       exactly (16kHz).
       - Without I2S blocking, this loop runs as fast as the consumer drains the
       queue.
     */

    /*
     * Throttle generation to avoid buffer overrun if the consumption is slow
     * (though k_msgq_put blocking handles this too)
     */

    memset(send_pcm_data, audio_ctr++, sizeof(send_pcm_data));

    ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, send_pcm_data, 1,
                     octets_per_frame, encoded_buf);

    if (ret == -1) {
      printk("LC3 encode failed in thread\n");
      k_sleep(K_MSEC(10));
      continue;
    }

    /* Push to queue, blocking if full (flow control) */
    ret = k_msgq_put(&lc3_encoded_q, encoded_buf, K_FOREVER);
    if (ret != 0) {
      printk("Failed to put encoded frame: %d\n", ret);
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
      // printk("TX underrun!\n");
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
  printk("ISO_BROADCAST RX %p: len %u\n", chan, buf->len);
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