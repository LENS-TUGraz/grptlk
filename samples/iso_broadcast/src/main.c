#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <grptlk_uplink_stats.h>

#define BIG_SDU_INTERVAL_US (10000)
#define BUF_ALLOC_TIMEOUT_US (BIG_SDU_INTERVAL_US * 2U)

#define BIS_ISO_CHAN_COUNT 5
#define NUM_PRIME_PACKETS 2
#define ISO_STATS_ENABLED 1
#define ISO_STATS_PRINT_INTERVAL_MS 1000U
#define MAX_UPLINK_REPORTERS (BIS_ISO_CHAN_COUNT * 4U)

LOG_MODULE_REGISTER(grptlk_iso_broadcast);

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT * NUM_PRIME_PACKETS,
						  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
						  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_term, 0, BIS_ISO_CHAN_COUNT);

static struct bt_iso_chan *bis[];

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

BUILD_ASSERT(sizeof(struct grptlk_uplink_stats_v1) == GRPTLK_UPLINK_STATS_V1_SIZE,
	     "Stats payload format must stay 40 bytes");

struct uplink_reporter_state
{
	bool used;
	uint8_t transport_bis;
	uint8_t uplink_bis;
	uint8_t dev_id_len;
	uint8_t dev_id[GRPTLK_DEVICE_ID_LEN];
	uint16_t seq_num;
	uint16_t interval_ms;
	uint16_t dl_rx_total;
	uint16_t dl_rx_valid;
	uint16_t dl_rx_lost;
	uint16_t dl_rx_error;
	uint16_t dl_rx_bad_len;
	uint16_t dl_rx_gap_packets;
	uint16_t dl_rx_old_or_dup;
	uint16_t ul_tx_ok;
	uint16_t ul_tx_alloc_fail;
	uint16_t ul_tx_send_fail;
	uint32_t last_seen_ms;
};

static uint16_t seq_num;
static struct iso_tx_stats iso_downlink_tx_stats;
static struct iso_rx_stats iso_uplink_rx_stats[BIS_ISO_CHAN_COUNT];
static uint32_t iso_recv_unexpected_bis1_count = 0U;
static uint32_t iso_recv_unknown_chan_count = 0U;
static uint32_t iso_recv_chan_count[BIS_ISO_CHAN_COUNT];
static uint16_t iso_recv_expected_seq[BIS_ISO_CHAN_COUNT];
static bool iso_recv_seq_initialized[BIS_ISO_CHAN_COUNT];
static uint8_t iso_data[CONFIG_BT_ISO_TX_MTU] = {0};
static struct uplink_reporter_state uplink_reporters[MAX_UPLINK_REPORTERS];
static uint32_t uplink_report_frames;
static uint32_t uplink_report_short;
static uint32_t uplink_report_parse_fail;
static uint32_t uplink_report_table_full;

#if ISO_STATS_ENABLED
static struct iso_tx_stats prev_iso_downlink_tx_stats;
static struct iso_rx_stats prev_iso_uplink_rx_stats[BIS_ISO_CHAN_COUNT];
static uint32_t prev_uplink_report_frames;
static uint32_t prev_uplink_report_short;
static uint32_t prev_uplink_report_parse_fail;
static uint32_t prev_uplink_report_table_full;
#endif

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
	if ((chan_idx > 0) && (chan_idx < BIS_ISO_CHAN_COUNT))
	{
		return &iso_uplink_rx_stats[chan_idx];
	}

	return NULL;
}

#if ISO_STATS_ENABLED
static uint32_t stats_delta_u32(uint32_t now, uint32_t *prev)
{
	uint32_t delta = now - *prev;

	*prev = now;
	return delta;
}

static uint8_t clamp_dev_id_len(uint8_t len)
{
	return (len > GRPTLK_DEVICE_ID_LEN) ? GRPTLK_DEVICE_ID_LEN : len;
}

static int find_reporter_slot(const uint8_t *dev_id, uint8_t dev_id_len)
{
	int free_slot = -1;

	for (int i = 0; i < MAX_UPLINK_REPORTERS; i++)
	{
		if (!uplink_reporters[i].used)
		{
			if (free_slot < 0)
			{
				free_slot = i;
			}

			continue;
		}

		if ((uplink_reporters[i].dev_id_len == dev_id_len) &&
		    (memcmp(uplink_reporters[i].dev_id, dev_id, GRPTLK_DEVICE_ID_LEN) == 0))
		{
			return i;
		}
	}

	return free_slot;
}

static bool parse_uplink_report(int chan_idx, struct net_buf *buf)
{
	struct grptlk_uplink_stats_v1 frame;
	struct uplink_reporter_state *reporter;
	int slot;
	uint8_t dev_id_len;

	if ((buf == NULL) || (buf->len < sizeof(frame)))
	{
		uplink_report_short++;
		return false;
	}

	memcpy(&frame, buf->data, sizeof(frame));

	if ((frame.magic[0] != GRPTLK_UPLINK_MAGIC_0) ||
	    (frame.magic[1] != GRPTLK_UPLINK_MAGIC_1) ||
	    (frame.version != GRPTLK_UPLINK_VERSION) ||
	    (frame.type != GRPTLK_UPLINK_TYPE_STATS_V1))
	{
		uplink_report_parse_fail++;
		return false;
	}

	dev_id_len = clamp_dev_id_len(frame.dev_id_len);
	slot = find_reporter_slot(frame.dev_id, dev_id_len);
	if (slot < 0)
	{
		uplink_report_table_full++;
		return false;
	}

	reporter = &uplink_reporters[slot];
	reporter->used = true;
	reporter->transport_bis = (uint8_t)(chan_idx + 1);
	reporter->uplink_bis = frame.uplink_bis;
	reporter->dev_id_len = dev_id_len;
	memcpy(reporter->dev_id, frame.dev_id, sizeof(reporter->dev_id));
	reporter->seq_num = sys_le16_to_cpu(frame.seq_num);
	reporter->interval_ms = sys_le16_to_cpu(frame.interval_ms);
	reporter->dl_rx_total = sys_le16_to_cpu(frame.dl_rx_total);
	reporter->dl_rx_valid = sys_le16_to_cpu(frame.dl_rx_valid);
	reporter->dl_rx_lost = sys_le16_to_cpu(frame.dl_rx_lost);
	reporter->dl_rx_error = sys_le16_to_cpu(frame.dl_rx_error);
	reporter->dl_rx_bad_len = sys_le16_to_cpu(frame.dl_rx_bad_len);
	reporter->dl_rx_gap_packets = sys_le16_to_cpu(frame.dl_rx_gap_packets);
	reporter->dl_rx_old_or_dup = sys_le16_to_cpu(frame.dl_rx_old_or_dup);
	reporter->ul_tx_ok = sys_le16_to_cpu(frame.ul_tx_ok);
	reporter->ul_tx_alloc_fail = sys_le16_to_cpu(frame.ul_tx_alloc_fail);
	reporter->ul_tx_send_fail = sys_le16_to_cpu(frame.ul_tx_send_fail);
	reporter->last_seen_ms = k_uptime_get_32();
	uplink_report_frames++;

	return true;
}

static void format_dev_id_hex(const struct uplink_reporter_state *reporter,
			      char *out, size_t out_len)
{
	size_t written;

	if ((out_len == 0U) || (reporter->dev_id_len == 0U))
	{
		if (out_len > 0U)
		{
			out[0] = '\0';
		}

		return;
	}

	written = bin2hex(reporter->dev_id, reporter->dev_id_len, out, out_len - 1U);
	out[written] = '\0';
}

static void log_uplink_chan_stats(int chan_idx)
{
	struct iso_rx_stats *stats = &iso_uplink_rx_stats[chan_idx];
	struct iso_rx_stats *prev = &prev_iso_uplink_rx_stats[chan_idx];
	uint32_t ul_rx_total_d = stats_delta_u32(stats->total, &prev->total);
	uint32_t ul_rx_valid_d = stats_delta_u32(stats->valid_flag, &prev->valid_flag);
	uint32_t ul_rx_lost_d = stats_delta_u32(stats->lost_flag, &prev->lost_flag);
	uint32_t ul_rx_err_d = stats_delta_u32(stats->error_flag, &prev->error_flag);
	uint32_t ul_rx_bad_len_d = stats_delta_u32(stats->bad_len, &prev->bad_len);
	uint32_t ul_rx_gap_evt_d = stats_delta_u32(stats->seq_gap_events, &prev->seq_gap_events);
	uint32_t ul_rx_gap_pkt_d = stats_delta_u32(stats->seq_gap_packets, &prev->seq_gap_packets);
	uint32_t ul_rx_dup_d = stats_delta_u32(stats->old_or_dup, &prev->old_or_dup);

	LOG_INF("UL RX (BIS%d): tot=%u(+%u) valid=%u(+%u) lost=%u(+%u) err=%u(+%u) bad_len=%u(+%u) gap_pkt=%u(+%u) gap_evt=%u(+%u) dup=%u(+%u)",
	       chan_idx + 1, stats->total, ul_rx_total_d, stats->valid_flag, ul_rx_valid_d,
	       stats->lost_flag, ul_rx_lost_d, stats->error_flag, ul_rx_err_d,
	       stats->bad_len, ul_rx_bad_len_d, stats->seq_gap_packets, ul_rx_gap_pkt_d,
		   stats->seq_gap_events, ul_rx_gap_evt_d, stats->old_or_dup, ul_rx_dup_d);
}

static void log_uplink_reporter_stats(void)
{
	uint32_t now = k_uptime_get_32();

	for (int i = 0; i < MAX_UPLINK_REPORTERS; i++)
	{
		char dev_id_hex[(GRPTLK_DEVICE_ID_LEN * 2U) + 1U];
		uint32_t age_ms;

		if (!uplink_reporters[i].used)
		{
			continue;
		}

		format_dev_id_hex(&uplink_reporters[i], dev_id_hex, sizeof(dev_id_hex));
		age_ms = now - uplink_reporters[i].last_seen_ms;

		LOG_INF("UL DEV[%d]: id=%s bis_rx=%u bis_id=%u age=%ums seq=%u int=%ums dl_rx=%u valid=%u lost=%u err=%u bad=%u gap=%u dup=%u ul_tx_ok=%u alloc=%u send=%u",
		       i, dev_id_hex[0] ? dev_id_hex : "unknown",
		       uplink_reporters[i].transport_bis, uplink_reporters[i].uplink_bis,
		       age_ms, uplink_reporters[i].seq_num, uplink_reporters[i].interval_ms,
		       uplink_reporters[i].dl_rx_total, uplink_reporters[i].dl_rx_valid,
		       uplink_reporters[i].dl_rx_lost, uplink_reporters[i].dl_rx_error,
		       uplink_reporters[i].dl_rx_bad_len, uplink_reporters[i].dl_rx_gap_packets,
		       uplink_reporters[i].dl_rx_old_or_dup, uplink_reporters[i].ul_tx_ok,
		       uplink_reporters[i].ul_tx_alloc_fail, uplink_reporters[i].ul_tx_send_fail);
	}
}

static void log_iso_stats_snapshot(void)
{
	uint32_t dl_tx_ok_d = stats_delta_u32(iso_downlink_tx_stats.ok, &prev_iso_downlink_tx_stats.ok);
	uint32_t dl_tx_alloc_d = stats_delta_u32(iso_downlink_tx_stats.alloc_fail,
							 &prev_iso_downlink_tx_stats.alloc_fail);
	uint32_t dl_tx_send_d = stats_delta_u32(iso_downlink_tx_stats.send_fail,
								&prev_iso_downlink_tx_stats.send_fail);
	uint32_t uplink_frames_d = stats_delta_u32(uplink_report_frames, &prev_uplink_report_frames);
	uint32_t uplink_short_d = stats_delta_u32(uplink_report_short, &prev_uplink_report_short);
	uint32_t uplink_parse_fail_d = stats_delta_u32(uplink_report_parse_fail, &prev_uplink_report_parse_fail);
	uint32_t uplink_table_full_d = stats_delta_u32(uplink_report_table_full, &prev_uplink_report_table_full);

	LOG_INF("----- ISO_BROADCAST STATS [1s] -----");
	LOG_INF("DL TX (BIS1): ok=%u(+%u) alloc_to=%u(+%u) send_err=%u(+%u)",
		   iso_downlink_tx_stats.ok, dl_tx_ok_d, iso_downlink_tx_stats.alloc_fail,
		   dl_tx_alloc_d, iso_downlink_tx_stats.send_fail, dl_tx_send_d);

	for (int chan_idx = 1; chan_idx < BIS_ISO_CHAN_COUNT; chan_idx++)
	{
		log_uplink_chan_stats(chan_idx);
	}

	LOG_INF("UL reports: frames=%u(+%u) short=%u(+%u) parse_fail=%u(+%u) table_full=%u(+%u)",
	       uplink_report_frames, uplink_frames_d, uplink_report_short, uplink_short_d,
	       uplink_report_parse_fail, uplink_parse_fail_d,
	       uplink_report_table_full, uplink_table_full_d);
	log_uplink_reporter_stats();
}

#define ISO_STATS_THREAD_STACK_SIZE 1536
#define ISO_STATS_THREAD_PRIORITY 7
K_THREAD_STACK_DEFINE(iso_stats_thread_stack, ISO_STATS_THREAD_STACK_SIZE);
static struct k_thread iso_stats_thread_data;

static void iso_stats_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true)
	{
		k_sleep(K_MSEC(ISO_STATS_PRINT_INTERVAL_MS));
		log_iso_stats_snapshot();
	}
}
#endif

static void iso_connected(struct bt_iso_chan *chan)
{
	LOG_INF("ISO Channel %p connected", chan);
	seq_num = 0U;
	memset(iso_recv_seq_initialized, 0, sizeof(iso_recv_seq_initialized));
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	LOG_INF("ISO Channel %p disconnected with reason 0x%02x", chan, reason);
	k_sem_give(&sem_big_term);
}

static void iso_sent(struct bt_iso_chan *chan)
{
	if (chan == bis[0])
	{
		int err;
		struct net_buf *buf;

		buf = net_buf_alloc(&bis_tx_pool, K_USEC(BUF_ALLOC_TIMEOUT_US));
		if (!buf)
		{
			iso_downlink_tx_stats.alloc_fail++;
			return;
		}

		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
		// sys_put_le32(iso_send_count, &iso_data[0]);
		// iso_data[4] = 0x01; /* from BIG creator */
		// iso_data[5] = 0x00; /* BIS index */
		memset(iso_data, 1, sizeof(iso_data));
		net_buf_add_mem(buf, iso_data, sizeof(iso_data));
		err = bt_iso_chan_send(chan, buf, seq_num);
		if (err < 0)
		{
			iso_downlink_tx_stats.send_fail++;
			net_buf_unref(buf);
			return;
		}

		iso_downlink_tx_stats.ok++;
		seq_num++;
	}
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
					 struct net_buf *buf)
{
	int chan_idx;
	struct iso_rx_stats *rx_stats;

	chan_idx = iso_chan_index(chan);
	if (chan_idx < 0)
	{
		iso_recv_unknown_chan_count++;
		return;
	}

	iso_recv_chan_count[chan_idx]++;

	/* Broadcaster should only RX uplink on BIS2..BIS5. */
	if (chan_idx == 0)
	{
		iso_recv_unexpected_bis1_count++;
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

	if ((buf == NULL) || (buf->len == 0U) || (buf->len > CONFIG_BT_ISO_TX_MTU))
	{
		rx_stats->bad_len++;
		return;
	}

#if ISO_STATS_ENABLED
	(void)parse_uplink_report(chan_idx, buf);
#endif
}

static void iso_log_startup_hint(void)
{
#if ISO_STATS_ENABLED
	LOG_INF("ISO stats enabled (%u ms period)", ISO_STATS_PRINT_INTERVAL_MS);
#else
	LOG_INF("ISO stats disabled (set ISO_STATS_ENABLED=1 to enable)");
#endif
}

static void iso_start_stats_thread(void)
{
#if ISO_STATS_ENABLED
	k_tid_t tid = k_thread_create(&iso_stats_thread_data, iso_stats_thread_stack,
				      K_THREAD_STACK_SIZEOF(iso_stats_thread_stack),
				      iso_stats_thread, NULL, NULL, NULL,
				      ISO_STATS_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(tid, "iso_stats");
#endif
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

static struct bt_iso_chan bis_iso_chan[] = {
	{
		.ops = &iso_ops,
		.qos = &bis_iso_qos,
	},
	{
		.ops = &iso_ops,
		.qos = &bis_iso_qos,
	},
	{
		.ops = &iso_ops,
		.qos = &bis_iso_qos,
	},
	{
		.ops = &iso_ops,
		.qos = &bis_iso_qos,
	},
	{
		.ops = &iso_ops,
		.qos = &bis_iso_qos,
	},
};

static struct bt_iso_chan *bis[] = {
	&bis_iso_chan[0],
	&bis_iso_chan[1],
	&bis_iso_chan[2],
	&bis_iso_chan[3],
	&bis_iso_chan[4],
};

static struct bt_iso_big_create_param big_create_param = {
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_channels = bis,
	.interval = BIG_SDU_INTERVAL_US,
	.latency = 10,
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_UNFRAMED,
};

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

int main(void)
{
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;
	int err;

	LOG_INF("Starting GRPTLK Broadcaster");
	iso_log_startup_hint();
	iso_start_stats_thread();

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err)
	{
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	/* Create a non-connectable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, NULL, &adv);
	if (err)
	{
		LOG_ERR("Failed to create advertising set (err %d)", err);
		return 0;
	}

	/* Set advertising data to have complete local name set */
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err)
	{
		LOG_ERR("Failed to set advertising data (err %d)", err);
		return 0;
	}

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
	if (err)
	{
		LOG_ERR("Failed to set periodic advertising parameters (err %d)",
			err);
		return 0;
	}

	/* Enable Periodic Advertising */
	err = bt_le_per_adv_start(adv);
	if (err)
	{
		LOG_ERR("Failed to enable periodic advertising (err %d)", err);
		return 0;
	}

	/* Start extended advertising */
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err)
	{
		LOG_ERR("Failed to start extended advertising (err %d)", err);
		return 0;
	}

	/* Create BIG */
	err = bt_iso_big_create(adv, &big_create_param, &big);
	if (err)
	{
		LOG_ERR("Failed to create BIG (err %d)", err);
		return 0;
	}

	for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
	{
		LOG_INF("Waiting for BIG complete chan %u...", chan);

		err = k_sem_take(&sem_big_cmplt, K_FOREVER);
		if (err)
		{
			LOG_ERR("failed (err %d)", err);
			return 0;
		}

		LOG_INF("BIG create complete chan %u.", chan);
	}

	for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
	{
		LOG_INF("Setting data path chan %u...", chan);

		const struct bt_iso_chan_path hci_path = {
			.pid = BT_ISO_DATA_PATH_HCI,
			.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
		};

		uint8_t dir = BT_HCI_DATAPATH_DIR_CTLR_TO_HOST;

		if (chan == 0)
		{
			dir = BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;
		}

		err = bt_iso_setup_data_path(&bis_iso_chan[chan], dir, &hci_path);
		if (err != 0)
		{
			LOG_ERR("Failed to setup ISO data path: %d", err);
		}

		LOG_INF("Setting data path complete chan %u.", chan);
	}

	for (int i = 0; i < NUM_PRIME_PACKETS; i++) {
		iso_sent(bis[0]);
	}
}
