#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <grptlk_uplink_stats.h>

/* ------------------------------------------------------------------------------- */
/* THIS IS THE BIS ON WHICH THE GRPTLK RECEIVER TRANSMITS BACK TO THE BROADCASTER  */
/* E.g., in a BIG with 5 BISes, this parameter can be [2..5] (BIS1 is downlink/rx) */
#define UPLINK_BIS CONFIG_GRPTLK_UPLINK_BIS
/* ------------------------------------------------------------------------------- */

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)

#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE,    \
										   BT_LE_SCAN_OPT_NONE,       \
										   BT_GAP_SCAN_FAST_INTERVAL, \
										   BT_GAP_SCAN_FAST_WINDOW)

#define PA_RETRY_COUNT 6

#define BIS_ISO_CHAN_COUNT CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT
#define ISO_STATS_ENABLED CONFIG_GRPTLK_ISO_STATS_ENABLED

LOG_MODULE_REGISTER(grptlk_iso_receive);

#if (UPLINK_BIS <= 1)
#error "UPLINK_BIS must be in BIS2..BISn (BIS1 is downlink)"
#endif

#if (UPLINK_BIS > BIS_ISO_CHAN_COUNT)
#error "UPLINK_BIS must be <= CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT"
#endif

#if (CONFIG_BT_ISO_TX_MTU < CONFIG_BT_ISO_RX_MTU)
#error "CONFIG_BT_ISO_TX_MTU must be >= CONFIG_BT_ISO_RX_MTU"
#endif

static bool per_adv_found;
static bool per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t per_sid;
static uint32_t per_interval_us;
static uint16_t iso_uplink_sdu_len = CONFIG_BT_ISO_TX_MTU;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_sync_lost, 0, BIS_ISO_CHAN_COUNT);

static bool iso_datapaths_setup = false;

static void scan_recv(const struct bt_le_scan_recv_info *info,
					  struct net_buf_simple *buf)
{
	ARG_UNUSED(buf);

	if (!per_adv_found && info->interval)
	{
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
					struct bt_le_per_adv_sync_synced_info *info)
{
	ARG_UNUSED(sync);
	ARG_UNUSED(info);

	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
					const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	LOG_INF("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated",
		   bt_le_per_adv_sync_get_index(sync), le_addr);

	per_adv_lost = true;
	k_sem_give(&sem_per_sync_lost);
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
					   const struct bt_iso_biginfo *biginfo)
{
	ARG_UNUSED(sync);

	uint16_t preferred_len = biginfo->max_sdu;

	if (preferred_len == 0U)
	{
		preferred_len = biginfo->max_pdu;
	}
	if (preferred_len == 0U)
	{
		preferred_len = CONFIG_BT_ISO_TX_MTU;
	}
	if (preferred_len > CONFIG_BT_ISO_TX_MTU)
	{
		LOG_WRN("BIG info SDU %u exceeds TX MTU %u, clamping uplink size",
		       preferred_len, CONFIG_BT_ISO_TX_MTU);
		preferred_len = CONFIG_BT_ISO_TX_MTU;
	}

	iso_uplink_sdu_len = preferred_len;

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

struct iso_tx_stats
{
	uint32_t ok;
	uint32_t alloc_fail;
	uint32_t send_fail;
};

struct iso_rx_stats
{
	uint32_t total;
	uint32_t valid_flag;
	uint32_t lost_flag;
	uint32_t error_flag;
	uint32_t bad_len;
	uint32_t seq_gap_events;
	uint32_t seq_gap_packets;
	uint32_t old_or_dup;
};

static uint16_t seq_num;
static struct iso_tx_stats iso_uplink_tx_stats;
static struct iso_rx_stats iso_downlink_rx_stats;
static uint16_t iso_recv_expected_seq[BIS_ISO_CHAN_COUNT];
static bool iso_recv_seq_initialized[BIS_ISO_CHAN_COUNT];
static uint8_t iso_data[CONFIG_BT_ISO_TX_MTU] = {0};

#if ISO_STATS_ENABLED
BUILD_ASSERT(sizeof(struct grptlk_uplink_stats_v1) == GRPTLK_UPLINK_STATS_V1_SIZE,
	     "Stats payload format must stay 40 bytes");

static uint8_t grptlk_device_id[GRPTLK_DEVICE_ID_LEN];
static uint8_t grptlk_device_id_len;
static uint32_t report_last_ms;
static struct grptlk_uplink_stats_v1 report_cache;
#endif

static struct bt_iso_chan *bis[BIS_ISO_CHAN_COUNT];

static int iso_chan_index(struct bt_iso_chan *chan)
{
	for (int i = 0; i < BIS_ISO_CHAN_COUNT; i++)
	{
		if (chan == bis[i])
		{
			return i;
		}
	}

	return -1;
}

static struct iso_rx_stats *iso_rx_stats_for_chan(int chan_idx)
{
	if (chan_idx == 0)
	{
		return &iso_downlink_rx_stats;
	}

	return NULL;
}

#if ISO_STATS_ENABLED
static void init_device_id(void)
{
	ssize_t len = hwinfo_get_device_id(grptlk_device_id, sizeof(grptlk_device_id));

	if (len <= 0)
	{
		grptlk_device_id_len = 0U;
		memset(grptlk_device_id, 0, sizeof(grptlk_device_id));
		return;
	}

	grptlk_device_id_len = (uint8_t)MIN((size_t)len, sizeof(grptlk_device_id));
}

static void init_report_cache(void)
{
	memset(&report_cache, 0, sizeof(report_cache));
	report_cache.magic[0] = GRPTLK_UPLINK_MAGIC_0;
	report_cache.magic[1] = GRPTLK_UPLINK_MAGIC_1;
	report_cache.version = GRPTLK_UPLINK_VERSION;
	report_cache.type = GRPTLK_UPLINK_TYPE_STATS_V1;
	report_cache.uplink_bis = UPLINK_BIS;
	report_cache.dev_id_len = grptlk_device_id_len;
	memcpy(report_cache.dev_id, grptlk_device_id, sizeof(report_cache.dev_id));
	report_last_ms = k_uptime_get_32();
}

static void refresh_report_cache(void)
{
	uint32_t now = k_uptime_get_32();
	uint32_t elapsed_ms = now - report_last_ms;

	/* Live snapshot: publish current cumulative counters in every uplink packet. */
	report_cache.interval_ms = sys_cpu_to_le16((uint16_t)MIN(elapsed_ms, UINT16_MAX));
	report_cache.dl_rx_total = sys_cpu_to_le16((uint16_t)iso_downlink_rx_stats.total);
	report_cache.dl_rx_valid = sys_cpu_to_le16((uint16_t)iso_downlink_rx_stats.valid_flag);
	report_cache.dl_rx_lost = sys_cpu_to_le16((uint16_t)iso_downlink_rx_stats.lost_flag);
	report_cache.dl_rx_error = sys_cpu_to_le16((uint16_t)iso_downlink_rx_stats.error_flag);
	report_cache.dl_rx_bad_len = sys_cpu_to_le16((uint16_t)iso_downlink_rx_stats.bad_len);
	report_cache.dl_rx_gap_packets = sys_cpu_to_le16((uint16_t)iso_downlink_rx_stats.seq_gap_packets);
	report_cache.dl_rx_old_or_dup = sys_cpu_to_le16((uint16_t)iso_downlink_rx_stats.old_or_dup);
	report_cache.ul_tx_ok = sys_cpu_to_le16((uint16_t)iso_uplink_tx_stats.ok);
	report_cache.ul_tx_alloc_fail = sys_cpu_to_le16((uint16_t)iso_uplink_tx_stats.alloc_fail);
	report_cache.ul_tx_send_fail = sys_cpu_to_le16((uint16_t)iso_uplink_tx_stats.send_fail);
	report_last_ms = now;
}
#endif


static void iso_prepare_uplink_payload(size_t sdu_len)
{
	memset(iso_data, 0, sdu_len);
#if ISO_STATS_ENABLED
	refresh_report_cache();
	report_cache.seq_num = sys_cpu_to_le16(seq_num);
	report_cache.uplink_bis = UPLINK_BIS;
	report_cache.dev_id_len = grptlk_device_id_len;
	memcpy(report_cache.dev_id, grptlk_device_id, sizeof(report_cache.dev_id));
	memcpy(iso_data, &report_cache, MIN(sdu_len, sizeof(report_cache)));
#else
	memset(iso_data, (uint8_t)seq_num, sdu_len);
#endif
}

static int iso_send_uplink(struct bt_iso_chan *chan, size_t sdu_len)
{
	int err;
	struct net_buf *buf;

	if ((sdu_len == 0U) || (sdu_len > sizeof(iso_data)))
	{
		iso_uplink_tx_stats.send_fail++;
		return -EINVAL;
	}

	buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
	if (!buf)
	{
		iso_uplink_tx_stats.alloc_fail++;
		return -ENOMEM;
	}

	iso_prepare_uplink_payload(sdu_len);

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	net_buf_add_mem(buf, iso_data, sdu_len);
	err = bt_iso_chan_send(chan, buf, seq_num);
	if (err < 0)
	{
		iso_uplink_tx_stats.send_fail++;
		net_buf_unref(buf);
		return err;
	}

	iso_uplink_tx_stats.ok++;
	seq_num++;
	return 0;
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
					 struct net_buf *buf)
{
	int chan_idx;
	struct iso_rx_stats *rx_stats;
	size_t uplink_len = iso_uplink_sdu_len;

	chan_idx = iso_chan_index(chan);
	if (chan_idx < 0)
	{
		return;
	}
	if (chan_idx != 0)
	{
		return;
	}

	rx_stats = iso_rx_stats_for_chan(chan_idx);
	if (iso_recv_seq_initialized[chan_idx])
	{
		uint16_t delta =
			(uint16_t)(info->seq_num - iso_recv_expected_seq[chan_idx]);

		if ((delta > 0U) && (delta < 0x8000U))
		{
			rx_stats->seq_gap_events++;
			rx_stats->seq_gap_packets += delta;
		}
		else if (delta >= 0x8000U)
		{
			rx_stats->old_or_dup++;
		}
	}

	iso_recv_expected_seq[chan_idx] = (uint16_t)(info->seq_num + 1U);
	iso_recv_seq_initialized[chan_idx] = true;
	rx_stats->total++;

	if ((info->flags & BT_ISO_FLAGS_VALID) != 0U)
	{
		rx_stats->valid_flag++;
	}
	if ((info->flags & BT_ISO_FLAGS_LOST) != 0U)
	{
		rx_stats->lost_flag++;
	}
	if ((info->flags & BT_ISO_FLAGS_ERROR) != 0U)
	{
		rx_stats->error_flag++;
	}

	/* Accept any SDU length up to configured RX MTU */
	if ((buf == NULL) || (buf->len == 0U) || (buf->len > CONFIG_BT_ISO_RX_MTU))
	{
		rx_stats->bad_len++;
	}
	else if (buf->len > 0U)
	{
		/* Keep uplink payload size aligned with received BIS payload size */
		iso_uplink_sdu_len = (uint16_t)buf->len;
		uplink_len = buf->len;
	}

	if (iso_datapaths_setup)
	{
		(void)iso_send_uplink(bis[UPLINK_BIS - 1], uplink_len);
	}
}

static void iso_connected(struct bt_iso_chan *chan)
{
	LOG_INF("ISO Channel %p connected", chan);
	memset(iso_recv_seq_initialized, 0, sizeof(iso_recv_seq_initialized));
	k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	LOG_INF("ISO Channel %p disconnected with reason 0x%02x",
		   chan, reason);

	if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST)
	{
		iso_datapaths_setup = false;
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.recv = iso_recv,
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
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_bitfield = (BIT_MASK(BIS_ISO_CHAN_COUNT)),
	.mse = BT_ISO_SYNC_MSE_ANY,
	.sync_timeout = 100, /* in 10 ms units */
};

static void iso_init_channels(void)
{
	for (int i = 0; i < BIS_ISO_CHAN_COUNT; i++)
	{
		bis_iso_chan[i].ops = &iso_ops;
		bis_iso_chan[i].qos = &bis_iso_qos;
		bis[i] = &bis_iso_chan[i];
	}
}

static void reset_semaphores(void)
{
	k_sem_reset(&sem_per_adv);
	k_sem_reset(&sem_per_sync);
	k_sem_reset(&sem_per_sync_lost);
	k_sem_reset(&sem_per_big_info);
	k_sem_reset(&sem_big_sync);
	k_sem_reset(&sem_big_sync_lost);
}

int main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	struct bt_iso_big *big;
	uint32_t sem_timeout_us;
	int err;

	LOG_INF("Starting GRPTLK Receiver");
	iso_init_channels();
#if ISO_STATS_ENABLED
	init_device_id();
	init_report_cache();
#endif

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err)
	{
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	LOG_INF("Scan callbacks register...");
	bt_le_scan_cb_register(&scan_callbacks);
	LOG_INF("success.");

	LOG_INF("Periodic Advertising callbacks register...");
	bt_le_per_adv_sync_cb_register(&sync_callbacks);
	LOG_INF("Success.");

	do
	{
		reset_semaphores();
		per_adv_lost = false;

		LOG_INF("Start scanning...");
		err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
		if (err)
		{
			LOG_ERR("failed (err %d)", err);
			return 0;
		}
		LOG_INF("success.");

		LOG_INF("Waiting for periodic advertising...");
		per_adv_found = false;
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err)
		{
			LOG_ERR("failed (err %d)", err);
			return 0;
		}
		LOG_INF("Found periodic advertising.");

		LOG_INF("Stop scanning...");
		err = bt_le_scan_stop();
		if (err)
		{
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
		sync_create_param.timeout = (per_interval_us * PA_RETRY_COUNT) /
									(10 * USEC_PER_MSEC);
		sem_timeout_us = per_interval_us * PA_RETRY_COUNT;
		err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
		if (err)
		{
			LOG_ERR("failed (err %d)", err);
			return 0;
		}
		LOG_INF("success.");

		LOG_INF("Waiting for periodic sync...");
		err = k_sem_take(&sem_per_sync, K_USEC(sem_timeout_us));
		if (err)
		{
			LOG_ERR("failed (err %d)", err);

			LOG_INF("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err)
			{
				LOG_ERR("failed (err %d)", err);
				return 0;
			}
			continue;
		}
		LOG_INF("Periodic sync established.");

		LOG_INF("Waiting for BIG info...");
		err = k_sem_take(&sem_per_big_info, K_USEC(sem_timeout_us));
		if (err)
		{
			LOG_ERR("failed (err %d)", err);

			if (per_adv_lost)
			{
				continue;
			}

			LOG_INF("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err)
			{
				LOG_ERR("failed (err %d)", err);
				return 0;
			}
			continue;
		}
		LOG_INF("Periodic sync established.");

		iso_tx_qos.sdu = iso_uplink_sdu_len;
		LOG_INF("Uplink SDU length configured to %u bytes.",
		       iso_tx_qos.sdu);

	big_sync_create:
		LOG_INF("Create BIG Sync...");
		err = bt_iso_big_sync(sync, &big_sync_param, &big);
		if (err)
		{
			LOG_ERR("failed (err %d)", err);
			return 0;
		}
		LOG_INF("success.");

		for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
		{
			LOG_INF("Waiting for BIG sync chan %u...", chan);
			err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
			if (err)
			{
				break;
			}
			LOG_INF("BIG sync chan %u successful.", chan);
		}
		if (err)
		{
			LOG_ERR("failed (err %d)", err);

			LOG_INF("BIG Sync Terminate...");
			err = bt_iso_big_terminate(big);
			if (err)
			{
				LOG_ERR("failed (err %d)", err);
				return 0;
			}
			LOG_INF("done.");

			goto per_sync_lost_check;
		}
		LOG_INF("BIG sync established.");

		for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
		{
			LOG_INF("Setting data path chan %u...", chan);

			const struct bt_iso_chan_path hci_path = {
				.pid = BT_ISO_DATA_PATH_HCI,
				.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
			};

			uint8_t dir = BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;

			if (chan == 0)
			{
				dir = BT_HCI_DATAPATH_DIR_CTLR_TO_HOST;
			}

			err = bt_iso_setup_data_path(bis[chan], dir, &hci_path);
			if (err != 0)
			{
				LOG_ERR("Failed to setup ISO RX data path: %d", err);
			}

			LOG_INF("Setting data path complete chan %u.", chan);
		}

		iso_datapaths_setup = true;

		for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
		{
			LOG_INF("Waiting for BIG sync lost chan %u...", chan);
			err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
			if (err)
			{
				LOG_ERR("failed (err %d)", err);
				return 0;
			}
			LOG_INF("BIG sync lost chan %u.", chan);
		}
		LOG_INF("BIG sync lost.");

	per_sync_lost_check:
		LOG_INF("Check for periodic sync lost...");
		err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
		if (err)
		{
			/* Periodic Sync active, go back to creating BIG Sync */
			goto big_sync_create;
		}
		LOG_INF("Periodic sync lost.");
	} while (true);
}
