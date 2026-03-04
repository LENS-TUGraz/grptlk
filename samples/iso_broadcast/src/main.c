#include "uplink_stats.h"
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define BIG_SDU_INTERVAL_US CONFIG_GRPTLK_ISO_SDU_INTERVAL_US
#define BUF_ALLOC_TIMEOUT_US (BIG_SDU_INTERVAL_US * 2U)

/* BIS count follows the sysbuild knob (default 2). */
#define BIS_ISO_CHAN_COUNT CONFIG_GRPTLK_ISO_CHAN_COUNT
#define NUM_PRIME_PACKETS 2

LOG_MODULE_REGISTER(grptlk_iso_broadcast);

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT *NUM_PRIME_PACKETS,
                          BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
                          CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_term, 0, BIS_ISO_CHAN_COUNT);

static struct bt_iso_chan *bis[BIS_ISO_CHAN_COUNT];

static uint16_t seq_num;
static uint8_t iso_data[CONFIG_BT_ISO_TX_MTU] = {0};

static int iso_chan_index(struct bt_iso_chan *chan) {
  for (int i = 0; i < BIS_ISO_CHAN_COUNT; i++) {
    if (chan == bis[i]) {
      return i;
    }
  }

  return -1;
}

#define TX_THREAD_STACK_SIZE 1024
#define TX_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);
static struct k_thread tx_thread_data;
static K_SEM_DEFINE(tx_sem, NUM_PRIME_PACKETS, NUM_PRIME_PACKETS);

static void iso_connected(struct bt_iso_chan *chan) {
  LOG_INF("ISO Channel %p connected", chan);
  seq_num = 0U;
  iso_stats_reset_seq();
  k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason) {
  LOG_INF("ISO Channel %p disconnected with reason 0x%02x", chan, reason);
  k_sem_give(&sem_big_term);
}

static void tx_thread(void *arg1, void *arg2, void *arg3) {
  int err;
  struct net_buf *buf;

  while (true) {
    k_sem_take(&tx_sem, K_FOREVER);

    buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
    if (!buf) {
      iso_stats_tx_alloc_fail();
      continue;
    }

    net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

    memset(iso_data, 1, sizeof(iso_data));
    net_buf_add_mem(buf, iso_data, sizeof(iso_data));

    err = bt_iso_chan_send(bis[0], buf, seq_num);
    if (err < 0) {
      iso_stats_tx_send_fail();
      net_buf_unref(buf);
      continue;
    }

    iso_stats_tx_ok();
    seq_num++;
  }
}

static void iso_sent(struct bt_iso_chan *chan) {
  if (chan == bis[0]) {
    k_sem_give(&tx_sem);
  }
}

static void iso_recv(struct bt_iso_chan *chan,
                     const struct bt_iso_recv_info *info, struct net_buf *buf) {
  int chan_idx;

  chan_idx = iso_chan_index(chan);
  if (chan_idx < 0) {
    iso_stats_rx_unknown_chan();
    return;
  }

  /* Broadcaster should only RX uplink on BIS2..BIS5. */
  if (chan_idx == 0) {
    iso_stats_rx_unexpected_bis1();
    return;
  }

  iso_stats_rx(chan_idx, info, buf);
}

static struct bt_iso_chan_ops iso_ops = {
    .connected = iso_connected,
    .disconnected = iso_disconnected,
    .sent = iso_sent,
    .recv = iso_recv,
};

static struct bt_iso_chan_io_qos iso_rx_qos;
static struct bt_iso_chan_io_qos iso_tx_qos = {
    .sdu = CONFIG_BT_ISO_TX_MTU,
    .rtn = 2,
    .phy = BT_GAP_LE_PHY_2M,
};

static struct bt_iso_chan_qos bis_iso_qos = {
    .tx = &iso_tx_qos,
    .rx = &iso_rx_qos,
};

static struct bt_iso_chan bis_iso_chan[BIS_ISO_CHAN_COUNT];

static void iso_init_channels(void) {
  for (int i = 0; i < BIS_ISO_CHAN_COUNT; i++) {
    bis_iso_chan[i].ops = &iso_ops;
    bis_iso_chan[i].qos = &bis_iso_qos;
    bis[i] = &bis_iso_chan[i];
  }
}

static struct bt_iso_big_create_param big_create_param = {
    .num_bis = BIS_ISO_CHAN_COUNT,
    .bis_channels = bis,
    .interval = BIG_SDU_INTERVAL_US,
    .latency = 10,
    .packing = BT_ISO_PACKING_SEQUENTIAL,
    .framing = BT_ISO_FRAMING_UNFRAMED,
};

static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

int main(void) {
  struct bt_le_ext_adv *adv;
  struct bt_iso_big *big;
  int err;

  iso_init_channels();
  LOG_INF("Starting GRPTLK Broadcaster");
  iso_log_startup_hint();
  iso_start_stats_thread(bis, BIS_ISO_CHAN_COUNT);

  err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return 0;
  }

  err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, NULL, &adv);
  if (err) {
    LOG_ERR("Failed to create advertising set (err %d)", err);
    return 0;
  }

  err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    LOG_ERR("Failed to set advertising data (err %d)", err);
    return 0;
  }

  err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
  if (err) {
    LOG_ERR("Failed to set periodic advertising parameters (err %d)", err);
    return 0;
  }

  err = bt_le_per_adv_start(adv);
  if (err) {
    LOG_ERR("Failed to enable periodic advertising (err %d)", err);
    return 0;
  }

  err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
  if (err) {
    LOG_ERR("Failed to start extended advertising (err %d)", err);
    return 0;
  }

  err = bt_iso_big_create(adv, &big_create_param, &big);
  if (err) {
    LOG_ERR("Failed to create BIG (err %d)", err);
    return 0;
  }

  for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++) {
    LOG_INF("Waiting for BIG complete chan %u...", chan);

    err = k_sem_take(&sem_big_cmplt, K_FOREVER);
    if (err) {
      LOG_ERR("failed (err %d)", err);
      return 0;
    }

    LOG_INF("BIG create complete chan %u.", chan);
  }

  for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++) {
    LOG_INF("Setting data path chan %u...", chan);

    const struct bt_iso_chan_path hci_path = {
        .pid = BT_ISO_DATA_PATH_HCI,
        .format = BT_HCI_CODING_FORMAT_TRANSPARENT,
    };

    uint8_t dir = BT_HCI_DATAPATH_DIR_CTLR_TO_HOST;

    if (chan == 0) {
      dir = BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;
    }

    err = bt_iso_setup_data_path(&bis_iso_chan[chan], dir, &hci_path);
    if (err != 0) {
      LOG_ERR("Failed to setup ISO data path: %d", err);
    }

    LOG_INF("Setting data path complete chan %u.", chan);
  }

  k_thread_create(&tx_thread_data, tx_thread_stack,
                  K_THREAD_STACK_SIZEOF(tx_thread_stack), tx_thread, NULL, NULL,
                  NULL, TX_THREAD_PRIORITY, 0, K_NO_WAIT);

  for (int i = 0; i < NUM_PRIME_PACKETS; i++) {
    k_sem_give(&tx_sem);
  }
}
