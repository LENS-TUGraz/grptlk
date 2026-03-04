#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>

#include "rx_stats.h"
#include <grptlk_uplink_stats.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
/* -------------------------------------------------------------------------------
 */
/* THIS IS THE BIS ON WHICH THE GRPTLK RECEIVER TRANSMITS BACK TO THE
 * BROADCASTER  */
/* E.g., in a BIG with 5 BISes, this parameter can be [2..5] (BIS1 is
 * downlink/rx) */
#define UPLINK_BIS CONFIG_GRPTLK_UPLINK_BIS
/* -------------------------------------------------------------------------------
 */

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)

#define BT_LE_SCAN_CUSTOM                                                      \
  BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, BT_LE_SCAN_OPT_NONE,                \
                   BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW)

#define PA_RETRY_COUNT 6

#define BIS_ISO_CHAN_COUNT CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT
#define ISO_STATS_ENABLED CONFIG_GRPTLK_ISO_STATS_ENABLED

#define NUM_PRIME_PACKETS 2

LOG_MODULE_REGISTER(grptlk_iso_receive);

#if defined(CONFIG_GRPTLK_UPLINK_MODE_STATIC)
#if (UPLINK_BIS <= 1)
#error "UPLINK_BIS must be in BIS2..BISn (BIS1 is downlink)"
#endif

#if (UPLINK_BIS > BIS_ISO_CHAN_COUNT)
#error "UPLINK_BIS must be <= CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT"
#endif
#endif /* CONFIG_GRPTLK_UPLINK_MODE_STATIC */

#if (CONFIG_BT_ISO_TX_MTU < CONFIG_BT_ISO_RX_MTU)
#error "CONFIG_BT_ISO_TX_MTU must be >= CONFIG_BT_ISO_RX_MTU"
#endif

static bool per_adv_found;
static bool per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t per_sid;
static uint32_t per_interval_us;
static uint16_t iso_uplink_sdu_len = CONFIG_BT_ISO_TX_MTU;
/* Actual number of BISes in the discovered BIG, clamped to BIS_ISO_CHAN_COUNT
 */
static uint8_t big_actual_num_bis = BIS_ISO_CHAN_COUNT;
/* BIS index (1-based) actually used for uplink TX; updated by
 * iso_select_uplink_chan() */
static uint8_t active_uplink_bis = UPLINK_BIS;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_sync_lost, 0, BIS_ISO_CHAN_COUNT);

static bool iso_datapaths_setup = false;

static void scan_recv(const struct bt_le_scan_recv_info *info,
                      struct net_buf_simple *buf) {
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
                    struct bt_le_per_adv_sync_synced_info *info) {
  ARG_UNUSED(sync);
  ARG_UNUSED(info);

  k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
                    const struct bt_le_per_adv_sync_term_info *info) {
  char le_addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

  LOG_INF("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated",
          bt_le_per_adv_sync_get_index(sync), le_addr);

  per_adv_lost = true;
  k_sem_give(&sem_per_sync_lost);
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
                       const struct bt_iso_biginfo *biginfo) {
  ARG_UNUSED(sync);

  uint16_t preferred_len = biginfo->max_sdu;

  if (preferred_len == 0U) {
    preferred_len = biginfo->max_pdu;
  }
  if (preferred_len == 0U) {
    preferred_len = CONFIG_BT_ISO_TX_MTU;
  }
  if (preferred_len > CONFIG_BT_ISO_TX_MTU) {
    LOG_WRN("BIG info SDU %u exceeds TX MTU %u, clamping uplink size",
            preferred_len, CONFIG_BT_ISO_TX_MTU);
    preferred_len = CONFIG_BT_ISO_TX_MTU;
  }

  iso_uplink_sdu_len = preferred_len;

  /* Clamp to our configured capacity */
  big_actual_num_bis = MIN(biginfo->num_bis, (uint8_t)BIS_ISO_CHAN_COUNT);
  LOG_INF("BIG has %u BIS(es), syncing to %u (capacity %u)", biginfo->num_bis,
          big_actual_num_bis, (uint8_t)BIS_ISO_CHAN_COUNT);

  /* Record PHY for inclusion in uplink stats reports */
  rx_stats_set_phy(biginfo->phy);

  k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
    .synced = sync_cb,
    .term = term_cb,
    .biginfo = biginfo_cb,
};

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT,
                          BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
                          CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static uint16_t seq_num;
static uint8_t iso_data[CONFIG_BT_ISO_TX_MTU] = {0};

static struct bt_iso_chan *bis[BIS_ISO_CHAN_COUNT];

static int iso_chan_index(struct bt_iso_chan *chan) {
  for (int i = 0; i < BIS_ISO_CHAN_COUNT; i++) {
    if (chan == bis[i]) {
      return i;
    }
  }

  return -1;
}

static struct bt_iso_chan *iso_select_uplink_chan(void);

#define TX_THREAD_STACK_SIZE 1024
#define TX_THREAD_PRIORITY 5

static struct k_thread tx_thread_data;
K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);
static K_SEM_DEFINE(tx_sem, 0, 1);

static void tx_thread(void *arg1, void *arg2, void *arg3) {
  int err;
  struct net_buf *buf;
  struct bt_iso_chan *ul_chan;
  size_t uplink_len;

  while (true) {
    k_sem_take(&tx_sem, K_FOREVER);

    ul_chan = iso_select_uplink_chan();
    if (ul_chan == NULL) {
      continue;
    }

    uplink_len = iso_uplink_sdu_len;
    if ((uplink_len == 0U) || (uplink_len > sizeof(iso_data))) {
      rx_stats_tx_send_fail();
      continue;
    }

    buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
    if (!buf) {
      rx_stats_tx_alloc_fail();
      continue;
    }

    rx_stats_prepare_uplink_payload(iso_data, uplink_len, seq_num,
                                    active_uplink_bis);

    net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
    net_buf_add_mem(buf, iso_data, uplink_len);

    err = bt_iso_chan_send(ul_chan, buf, seq_num);
    if (err < 0) {
      rx_stats_tx_send_fail();
      net_buf_unref(buf);
      continue;
    }

    rx_stats_tx_ok();
    seq_num++;
  }
}

static void iso_sent(struct bt_iso_chan *chan) {
  if (chan == bis[active_uplink_bis]) {
    k_sem_give(&tx_sem);
  }
}

static struct bt_iso_chan *iso_select_uplink_chan(void);

static void iso_recv(struct bt_iso_chan *chan,
                     const struct bt_iso_recv_info *info, struct net_buf *buf) {
  int chan_idx;

  chan_idx = iso_chan_index(chan);
  if (chan_idx < 0) {
    return;
  }
  if (chan_idx != 0) {
    return;
  }

  rx_stats_rx_downlink(info, buf);

  /* Accept any SDU length up to configured RX MTU */
  if (buf && buf->len > 0U && buf->len <= CONFIG_BT_ISO_RX_MTU) {
    /* Keep uplink payload size aligned with received BIS payload size */
    iso_uplink_sdu_len = (uint16_t)buf->len;
  }
}

static struct bt_iso_chan *iso_select_uplink_chan(void) {
#if defined(CONFIG_GRPTLK_UPLINK_MODE_RANDOM)
  /* Randomly pick from the available uplink BISes (BIS2..BISn). */
  uint8_t num_uplink =
      (big_actual_num_bis > 1U) ? (big_actual_num_bis - 1U) : 0U;

  if (num_uplink == 0U) {
    return NULL;
  }
  uint8_t idx = (uint8_t)(sys_rand32_get() % num_uplink);

  active_uplink_bis = idx + 1U;
  return bis[active_uplink_bis];
#else
  return bis[UPLINK_BIS - 1];
#endif
}

static void iso_connected(struct bt_iso_chan *chan) {
  int chan_idx;

  LOG_INF("ISO Channel %p connected", chan);
  seq_num = 0U;
  rx_stats_reset_seq();

  /* Snapshot channel parameters for BIS1 (downlink) only */
  chan_idx = iso_chan_index(chan);
  if (chan_idx == 0) {
    rx_stats_update_chan_info(chan);
  }

  k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason) {
  LOG_INF("ISO Channel %p disconnected with reason 0x%02x", chan, reason);

  if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
    iso_datapaths_setup = false;
    k_sem_give(&sem_big_sync_lost);
  }
}

static struct bt_iso_chan_ops iso_ops = {
    .recv = iso_recv,
    .sent = iso_sent,
    .connected = iso_connected,
    .disconnected = iso_disconnected,
};

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

static struct bt_iso_chan bis_iso_chan[BIS_ISO_CHAN_COUNT];

static struct bt_iso_big_sync_param big_sync_param = {
    .bis_channels = bis,
    /* num_bis and bis_bitfield are set at runtime from biginfo->num_bis */
    .num_bis = BIS_ISO_CHAN_COUNT,
    .bis_bitfield = (BIT_MASK(BIS_ISO_CHAN_COUNT)),
    .mse = BT_ISO_SYNC_MSE_ANY,
    .sync_timeout = 100, /* in 10 ms units */
};

static void iso_init_channels(void) {
  for (int i = 0; i < BIS_ISO_CHAN_COUNT; i++) {
    bis_iso_chan[i].ops = &iso_ops;
    bis_iso_chan[i].qos = &bis_iso_qos;
    bis[i] = &bis_iso_chan[i];
  }
}

static void reset_semaphores(void) {
  k_sem_reset(&sem_per_adv);
  k_sem_reset(&sem_per_sync);
  k_sem_reset(&sem_per_sync_lost);
  k_sem_reset(&sem_per_big_info);
  k_sem_reset(&sem_big_sync);
  k_sem_reset(&sem_big_sync_lost);
}

int main(void) {
  struct bt_le_per_adv_sync_param sync_create_param;
  struct bt_le_per_adv_sync *sync;
  struct bt_iso_big *big;
  uint32_t sem_timeout_us;
  int err;

  LOG_INF("Starting GRPTLK Receiver");
  iso_init_channels();

  /* Initialize the Bluetooth Subsystem */
  err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return 0;
  }

  rx_stats_init_device_id();
  rx_stats_init_report_cache(active_uplink_bis);

  k_thread_create(&tx_thread_data, tx_thread_stack,
                  K_THREAD_STACK_SIZEOF(tx_thread_stack), tx_thread, NULL, NULL,
                  NULL, TX_THREAD_PRIORITY, 0, K_NO_WAIT);

  LOG_INF("Scan callbacks register...");
  bt_le_scan_cb_register(&scan_callbacks);
  LOG_INF("success.");

  LOG_INF("Periodic Advertising callbacks register...");
  bt_le_per_adv_sync_cb_register(&sync_callbacks);
  LOG_INF("Success.");

  do {
    reset_semaphores();
    per_adv_lost = false;

    LOG_INF("Start scanning...");
    err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
    if (err) {
      LOG_ERR("failed (err %d)", err);
      return 0;
    }
    LOG_INF("success.");

    LOG_INF("Waiting for periodic advertising...");
    per_adv_found = false;
    err = k_sem_take(&sem_per_adv, K_FOREVER);
    if (err) {
      LOG_ERR("failed (err %d)", err);
      return 0;
    }
    LOG_INF("Found periodic advertising.");

    LOG_INF("Stop scanning...");
    err = bt_le_scan_stop();
    if (err) {
      LOG_ERR("failed (err %d)", err);
      return 0;
    }
    LOG_INF("success.");

    LOG_INF("Creating Periodic Advertising Sync...");
    bt_addr_le_copy(&sync_create_param.addr, &per_addr);
    sync_create_param.options = 0;
    sync_create_param.sid = per_sid;
    sync_create_param.skip = 0;
    /* Multiple PA interval with retry count and convert to unit of 10 ms */
    sync_create_param.timeout =
        (per_interval_us * PA_RETRY_COUNT) / (10 * USEC_PER_MSEC);
    sem_timeout_us = per_interval_us * PA_RETRY_COUNT;
    err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
    if (err) {
      LOG_ERR("failed (err %d)", err);
      return 0;
    }
    LOG_INF("success.");

    LOG_INF("Waiting for periodic sync...");
    err = k_sem_take(&sem_per_sync, K_USEC(sem_timeout_us));
    if (err) {
      LOG_ERR("failed (err %d)", err);

      LOG_INF("Deleting Periodic Advertising Sync...");
      err = bt_le_per_adv_sync_delete(sync);
      if (err) {
        LOG_ERR("failed (err %d)", err);
        return 0;
      }
      continue;
    }
    LOG_INF("Periodic sync established.");

    LOG_INF("Waiting for BIG info...");
    err = k_sem_take(&sem_per_big_info, K_USEC(sem_timeout_us));
    if (err) {
      LOG_ERR("failed (err %d)", err);

      if (per_adv_lost) {
        continue;
      }

      LOG_INF("Deleting Periodic Advertising Sync...");
      err = bt_le_per_adv_sync_delete(sync);
      if (err) {
        LOG_ERR("failed (err %d)", err);
        return 0;
      }
      continue;
    }
    LOG_INF("Periodic sync established.");

    iso_tx_qos.sdu = iso_uplink_sdu_len;
    LOG_INF("Uplink SDU length configured to %u bytes.", iso_tx_qos.sdu);

  big_sync_create:
    /* Use the actual BIG size discovered via biginfo, clamped to our capacity
     */
    big_sync_param.num_bis = big_actual_num_bis;
    big_sync_param.bis_bitfield = BIT_MASK(big_actual_num_bis);
    LOG_INF("Create BIG Sync (num_bis=%u, bitfield=0x%x)...",
            big_sync_param.num_bis, big_sync_param.bis_bitfield);
    err = bt_iso_big_sync(sync, &big_sync_param, &big);
    if (err) {
      LOG_ERR("failed (err %d)", err);
      return 0;
    }
    LOG_INF("success.");

    for (uint8_t chan = 0U; chan < big_actual_num_bis; chan++) {
      LOG_INF("Waiting for BIG sync chan %u...", chan);
      err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
      if (err) {
        break;
      }
      LOG_INF("BIG sync chan %u successful.", chan);
    }
    if (err) {
      LOG_ERR("failed (err %d)", err);

      LOG_INF("BIG Sync Terminate...");
      err = bt_iso_big_terminate(big);
      if (err) {
        LOG_ERR("failed (err %d)", err);
        return 0;
      }
      LOG_INF("done.");

      goto per_sync_lost_check;
    }
    LOG_INF("BIG sync established.");

    for (uint8_t chan = 0U; chan < big_actual_num_bis; chan++) {
      LOG_INF("Setting data path chan %u...", chan);

      const struct bt_iso_chan_path hci_path = {
          .pid = BT_ISO_DATA_PATH_HCI,
          .format = BT_HCI_CODING_FORMAT_TRANSPARENT,
      };

      uint8_t dir = BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;

      if (chan == 0) {
        dir = BT_HCI_DATAPATH_DIR_CTLR_TO_HOST;
      }

      err = bt_iso_setup_data_path(bis[chan], dir, &hci_path);
      if (err != 0) {
        LOG_ERR("Failed to setup ISO RX data path: %d", err);
      }

      LOG_INF("Setting data path complete chan %u.", chan);
    }

    iso_datapaths_setup = true;

    /* Give exactly 1 token to start the engine.
     * We only want ONE packet to be sent per interval. */
    k_sem_give(&tx_sem);

    for (uint8_t chan = 0U; chan < big_actual_num_bis; chan++) {
      LOG_INF("Waiting for BIG sync lost chan %u...", chan);
      err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
      if (err) {
        LOG_ERR("failed (err %d)", err);
        return 0;
      }
      LOG_INF("BIG sync lost chan %u.", chan);
    }
    LOG_INF("BIG sync lost.");

  per_sync_lost_check:
    LOG_INF("Check for periodic sync lost...");
    err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
    if (err) {
      /* Periodic Sync active, go back to creating BIG Sync */
      goto big_sync_create;
    }
    LOG_INF("Periodic sync lost.");
  } while (true);
}
