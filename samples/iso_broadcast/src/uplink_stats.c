#include "uplink_stats.h"

#if IS_ENABLED(CONFIG_GRPTLK_ISO_STATS)

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <grptlk_uplink_stats.h>

/* BIS count follows the sysbuild knob (default 2). */
#define BIS_ISO_CHAN_COUNT           CONFIG_GRPTLK_ISO_CHAN_COUNT
#define ISO_STATS_PRINT_INTERVAL_MS  100U
/* Reporters not heard from within this period are evicted from the table. */
#define UPLINK_REPORTER_STALE_MS    5000U
#define MAX_UPLINK_REPORTERS        (BIS_ISO_CHAN_COUNT * 4U)

LOG_MODULE_DECLARE(grptlk_iso_broadcast);

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
	/* Uplink loss tracking from payload seq_num. */
	uint16_t prev_seq_num;
	bool seq_initialized;
	uint32_t ul_lost_payload;
};

static struct iso_tx_stats iso_downlink_tx_stats;
static struct iso_rx_stats iso_uplink_rx_stats[BIS_ISO_CHAN_COUNT];
static uint32_t iso_recv_unexpected_bis1_count = 0U;
static uint32_t iso_recv_unknown_chan_count = 0U;
static uint32_t iso_recv_chan_count[BIS_ISO_CHAN_COUNT];
static uint16_t iso_recv_expected_seq[BIS_ISO_CHAN_COUNT];
static bool iso_recv_seq_initialized[BIS_ISO_CHAN_COUNT];
static struct uplink_reporter_state uplink_reporters[MAX_UPLINK_REPORTERS];
static uint32_t uplink_report_frames;
static uint32_t uplink_report_short;
static uint32_t uplink_report_parse_fail;
static uint32_t uplink_report_table_full;

static struct iso_tx_stats prev_iso_downlink_tx_stats;
static struct iso_rx_stats prev_iso_uplink_rx_stats[BIS_ISO_CHAN_COUNT];
static uint32_t prev_uplink_report_frames;
static uint32_t prev_uplink_report_short;
static uint32_t prev_uplink_report_parse_fail;
static uint32_t prev_uplink_report_table_full;

static struct iso_rx_stats *iso_rx_stats_for_chan(int chan_idx)
{
	if ((chan_idx > 0) && (chan_idx < BIS_ISO_CHAN_COUNT))
	{
		return &iso_uplink_rx_stats[chan_idx];
	}

	return NULL;
}

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

	uint16_t rx_seq = sys_le16_to_cpu(frame.seq_num);

	if (reporter->used && reporter->seq_initialized) {
		uint16_t expected = reporter->prev_seq_num + 1U;
		uint16_t delta = (uint16_t)(rx_seq - expected);

		if ((delta > 0U) && (delta < 0x8000U)) {
			reporter->ul_lost_payload += delta;
		}
	} else {
		reporter->ul_lost_payload = 0U;
		reporter->seq_initialized = false;
	}

	reporter->used = true;
	reporter->prev_seq_num = rx_seq;
	reporter->seq_initialized = true;
	reporter->transport_bis = (uint8_t)(chan_idx + 1);
	reporter->uplink_bis = frame.uplink_bis;
	reporter->dev_id_len = dev_id_len;
	memcpy(reporter->dev_id, frame.dev_id, sizeof(reporter->dev_id));
	reporter->seq_num = rx_seq;
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

	if ((out_len == 0U) || (reporter->dev_id_len == 0U)) {
		if (out_len > 0U) {
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
	int active_count = 0;

	/* First pass: evict stale reporters. */
	for (int i = 0; i < MAX_UPLINK_REPORTERS; i++)
	{
		char dev_id_hex[(GRPTLK_DEVICE_ID_LEN * 2U) + 1U];
		uint32_t age_ms;

		if (!uplink_reporters[i].used)
		{
			continue;
		}

		age_ms = now - uplink_reporters[i].last_seen_ms;
		if (age_ms > UPLINK_REPORTER_STALE_MS)
		{
			format_dev_id_hex(&uplink_reporters[i], dev_id_hex, sizeof(dev_id_hex));
			printk(GRPTLK_LOG_DATA_PREFIX "dev=%s status=stale age_ms=%u\n",
			       dev_id_hex[0] ? dev_id_hex : "unknown", age_ms);
			uplink_reporters[i].used = false;
			continue;
		}

		active_count++;
	}

	printk(GRPTLK_LOG_DATA_PREFIX "ts=%u active=%d bis_count=%d\n",
	       now, active_count, BIS_ISO_CHAN_COUNT);

	/* Second pass: emit one structured line per active slot. */
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

		printk(GRPTLK_LOG_DATA_PREFIX
		       "dev=%s bis_rx=%u uplink_bis=%u age_ms=%u seq=%u "
		       "dl_rx=%u dl_valid=%u dl_lost=%u dl_err=%u dl_bad=%u "
		       "dl_gap=%u dl_dup=%u ul_ok=%u ul_alloc=%u ul_send=%u ul_lost=%u\n",
		       dev_id_hex[0] ? dev_id_hex : "unknown",
		       uplink_reporters[i].transport_bis,
		       uplink_reporters[i].uplink_bis,
		       age_ms,
		       uplink_reporters[i].seq_num,
		       uplink_reporters[i].dl_rx_total,
		       uplink_reporters[i].dl_rx_valid,
		       uplink_reporters[i].dl_rx_lost,
		       uplink_reporters[i].dl_rx_error,
		       uplink_reporters[i].dl_rx_bad_len,
		       uplink_reporters[i].dl_rx_gap_packets,
		       uplink_reporters[i].dl_rx_old_or_dup,
		       uplink_reporters[i].ul_tx_ok,
		       uplink_reporters[i].ul_tx_alloc_fail,
		       uplink_reporters[i].ul_tx_send_fail,
		       uplink_reporters[i].ul_lost_payload);
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

	LOG_INF("----- ISO_BROADCAST STATS [100ms] -----");
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

void iso_log_startup_hint(void)
{
	LOG_INF("ISO stats enabled (%u ms period)", ISO_STATS_PRINT_INTERVAL_MS);
}

void iso_start_stats_thread(void)
{
	k_tid_t tid = k_thread_create(&iso_stats_thread_data, iso_stats_thread_stack,
				      K_THREAD_STACK_SIZEOF(iso_stats_thread_stack),
				      iso_stats_thread, NULL, NULL, NULL,
				      ISO_STATS_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(tid, "iso_stats");
}

void iso_stats_reset_seq(void)
{
	memset(iso_recv_seq_initialized, 0, sizeof(iso_recv_seq_initialized));
}

void iso_stats_tx_ok(void)
{
	iso_downlink_tx_stats.ok++;
}

void iso_stats_tx_alloc_fail(void)
{
	iso_downlink_tx_stats.alloc_fail++;
}

void iso_stats_tx_send_fail(void)
{
	iso_downlink_tx_stats.send_fail++;
}

void iso_stats_rx_unknown_chan(void)
{
	iso_recv_unknown_chan_count++;
}

void iso_stats_rx_unexpected_bis1(void)
{
	iso_recv_unexpected_bis1_count++;
}

void iso_stats_rx(int chan_idx, const struct bt_iso_recv_info *info, struct net_buf *buf)
{
	struct iso_rx_stats *rx_stats;

	iso_recv_chan_count[chan_idx]++;
	rx_stats = iso_rx_stats_for_chan(chan_idx);
	if (!rx_stats) {
		return;
	}

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

	(void)parse_uplink_report(chan_idx, buf);
}

#endif /* IS_ENABLED(CONFIG_GRPTLK_ISO_STATS) */
