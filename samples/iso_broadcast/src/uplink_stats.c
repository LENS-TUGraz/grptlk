#include "uplink_stats.h"

#if IS_ENABLED(CONFIG_GRPTLK_ISO_STATS)

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <grptlk_uplink_stats.h>

#define BIS_ISO_CHAN_COUNT           CONFIG_GRPTLK_ISO_CHAN_COUNT
#define ISO_STATS_PRINT_INTERVAL_MS  200U
#define CHAN_INFO_TICK_PERIOD        10U
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
	uint16_t dl_rx_gap_packets;
	uint16_t dl_rx_gap_events;
	uint16_t dl_rx_old_or_dup;
	uint16_t ul_tx_ok;
	uint16_t ul_tx_alloc_fail;
	uint16_t ul_tx_send_fail;
	uint16_t chan_max_pdu;
	uint8_t  chan_bn;
	uint8_t  chan_irc;
	uint8_t  phy;
	uint32_t last_seen_ms;
	uint16_t prev_seq_num;
	bool seq_initialized;
	uint32_t ul_lost_payload;
	uint16_t prev_dl_rx_total;
	uint16_t prev_dl_rx_valid;
	uint16_t prev_dl_rx_lost;
	uint16_t prev_dl_rx_gap_packets;
	uint16_t prev_dl_rx_old_or_dup;
	bool dl_delta_initialized;
};

static struct iso_tx_stats iso_downlink_tx_stats;
static struct iso_rx_stats iso_uplink_rx_stats[BIS_ISO_CHAN_COUNT];
static uint32_t iso_recv_unexpected_bis1_count;
static uint32_t iso_recv_unknown_chan_count;
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

static struct bt_iso_chan **stats_bis;
static int stats_bis_count;

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

static void reporter_update(struct uplink_reporter_state *r,
			    int chan_idx, uint16_t rx_seq,
			    uint8_t dev_id_len, const uint8_t *dev_id,
			    uint8_t uplink_bis, uint16_t interval_ms,
			    uint16_t dl_rx_total, uint16_t dl_rx_valid,
			    uint16_t dl_rx_lost, uint16_t dl_rx_gap_packets,
			    uint16_t dl_rx_old_or_dup,
			    uint16_t ul_tx_ok, uint16_t ul_tx_alloc_fail,
			    uint16_t ul_tx_send_fail)
{
	if (r->used && r->seq_initialized) {
		uint16_t delta = (uint16_t)(rx_seq - (r->prev_seq_num + 1U));

		if ((delta > 0U) && (delta < 0x8000U)) {
			r->ul_lost_payload += delta;
		}
	} else {
		r->ul_lost_payload = 0U;
		r->seq_initialized = false;
	}

	if (!r->used || !r->dl_delta_initialized) {
		r->prev_dl_rx_total       = dl_rx_total;
		r->prev_dl_rx_valid       = dl_rx_valid;
		r->prev_dl_rx_lost        = dl_rx_lost;
		r->prev_dl_rx_gap_packets = dl_rx_gap_packets;
		r->prev_dl_rx_old_or_dup  = dl_rx_old_or_dup;
		r->dl_delta_initialized   = true;
	}

	r->used           = true;
	r->prev_seq_num   = rx_seq;
	r->seq_initialized = true;
	r->transport_bis  = (uint8_t)(chan_idx + 1);
	r->uplink_bis     = uplink_bis;
	r->dev_id_len     = dev_id_len;
	memcpy(r->dev_id, dev_id, GRPTLK_DEVICE_ID_LEN);
	r->seq_num        = rx_seq;
	r->interval_ms    = interval_ms;
	r->dl_rx_total    = dl_rx_total;
	r->dl_rx_valid    = dl_rx_valid;
	r->dl_rx_lost     = dl_rx_lost;
	r->dl_rx_gap_packets = dl_rx_gap_packets;
	r->dl_rx_old_or_dup  = dl_rx_old_or_dup;
	r->ul_tx_ok       = ul_tx_ok;
	r->ul_tx_alloc_fail = ul_tx_alloc_fail;
	r->ul_tx_send_fail  = ul_tx_send_fail;
	r->last_seen_ms   = k_uptime_get_32();
	uplink_report_frames++;
}

static bool parse_uplink_report(int chan_idx, struct net_buf *buf)
{
	struct uplink_reporter_state *reporter;
	int slot;
	uint8_t dev_id_len;

	if ((buf == NULL) || (buf->len < sizeof(struct grptlk_uplink_stats_v1)))
	{
		uplink_report_short++;
		return false;
	}

	const uint8_t *raw = buf->data;

	if ((raw[0] != GRPTLK_UPLINK_MAGIC_0) || (raw[1] != GRPTLK_UPLINK_MAGIC_1))
	{
		uplink_report_parse_fail++;
		return false;
	}

	uint8_t version = raw[2];
	uint8_t type    = raw[3];

	if (version == GRPTLK_UPLINK_VERSION &&
	    type == GRPTLK_UPLINK_TYPE_STATS_V1 &&
	    buf->len >= sizeof(struct grptlk_uplink_stats_v1))
	{
		struct grptlk_uplink_stats_v1 frame;

		memcpy(&frame, buf->data, sizeof(frame));
		dev_id_len = clamp_dev_id_len(frame.dev_id_len);
		slot = find_reporter_slot(frame.dev_id, dev_id_len);
		if (slot < 0) {
			uplink_report_table_full++;
			return false;
		}

		reporter = &uplink_reporters[slot];
		reporter_update(reporter, chan_idx,
				sys_le16_to_cpu(frame.seq_num),
				dev_id_len, frame.dev_id,
				frame.uplink_bis,
				sys_le16_to_cpu(frame.interval_ms),
				sys_le16_to_cpu(frame.dl_rx_total),
				sys_le16_to_cpu(frame.dl_rx_valid),
				sys_le16_to_cpu(frame.dl_rx_lost),
				sys_le16_to_cpu(frame.dl_rx_gap_packets),
				sys_le16_to_cpu(frame.dl_rx_old_or_dup),
				sys_le16_to_cpu(frame.ul_tx_ok),
				sys_le16_to_cpu(frame.ul_tx_alloc_fail),
				sys_le16_to_cpu(frame.ul_tx_send_fail));
		reporter->dl_rx_gap_events = 0U;
		reporter->chan_max_pdu = 0U;
		reporter->chan_bn      = 0U;
		reporter->chan_irc     = 0U;
		reporter->phy          = 0U;
		return true;
	}

	if (version == GRPTLK_UPLINK_VERSION_2 &&
	    type == GRPTLK_UPLINK_TYPE_STATS_V2 &&
	    buf->len >= sizeof(struct grptlk_uplink_stats_v2))
	{
		struct grptlk_uplink_stats_v2 frame;

		memcpy(&frame, buf->data, sizeof(frame));
		dev_id_len = clamp_dev_id_len(frame.dev_id_len);
		slot = find_reporter_slot(frame.dev_id, dev_id_len);
		if (slot < 0) {
			uplink_report_table_full++;
			return false;
		}

		reporter = &uplink_reporters[slot];
		reporter_update(reporter, chan_idx,
				sys_le16_to_cpu(frame.seq_num),
				dev_id_len, frame.dev_id,
				frame.uplink_bis,
				sys_le16_to_cpu(frame.interval_ms),
				sys_le16_to_cpu(frame.dl_rx_total),
				sys_le16_to_cpu(frame.dl_rx_valid),
				sys_le16_to_cpu(frame.dl_rx_lost),
				sys_le16_to_cpu(frame.dl_rx_gap_packets),
				sys_le16_to_cpu(frame.dl_rx_old_or_dup),
				sys_le16_to_cpu(frame.ul_tx_ok),
				sys_le16_to_cpu(frame.ul_tx_alloc_fail),
				sys_le16_to_cpu(frame.ul_tx_send_fail));
		reporter->dl_rx_gap_events = sys_le16_to_cpu(frame.dl_rx_gap_events);
		reporter->chan_max_pdu     = sys_le16_to_cpu(frame.chan_max_pdu);
		reporter->chan_bn          = GRPTLK_CHAN_INFO_BN(frame.chan_info);
		reporter->chan_irc         = GRPTLK_CHAN_INFO_IRC(frame.chan_info);
		reporter->phy              = frame.phy;
		return true;
	}

	uplink_report_parse_fail++;
	return false;
}

static void format_dev_id_hex(const struct uplink_reporter_state *reporter,
			      char *out, size_t out_len)
{
	if ((out_len == 0U) || (reporter->dev_id_len == 0U)) {
		if (out_len > 0U) {
			out[0] = '\0';
		}
		return;
	}

	size_t written = bin2hex(reporter->dev_id, reporter->dev_id_len, out, out_len - 1U);

	out[written] = '\0';
}

static void log_uplink_chan_stats(int chan_idx)
{
	struct iso_rx_stats *stats = &iso_uplink_rx_stats[chan_idx];
	struct iso_rx_stats *prev = &prev_iso_uplink_rx_stats[chan_idx];
	uint32_t tot_d     = stats_delta_u32(stats->total,          &prev->total);
	uint32_t valid_d   = stats_delta_u32(stats->valid_flag,     &prev->valid_flag);
	uint32_t lost_d    = stats_delta_u32(stats->lost_flag,      &prev->lost_flag);
	uint32_t err_d     = stats_delta_u32(stats->error_flag,     &prev->error_flag);
	uint32_t bad_len_d = stats_delta_u32(stats->bad_len,        &prev->bad_len);
	uint32_t gap_evt_d = stats_delta_u32(stats->seq_gap_events, &prev->seq_gap_events);
	uint32_t gap_pkt_d = stats_delta_u32(stats->seq_gap_packets,&prev->seq_gap_packets);
	uint32_t dup_d     = stats_delta_u32(stats->old_or_dup,     &prev->old_or_dup);

	LOG_INF("UL RX (BIS%d): tot=%u(+%u) valid=%u(+%u) lost=%u(+%u) err=%u(+%u) "
		"bad_len=%u(+%u) gap_pkt=%u(+%u) gap_evt=%u(+%u) dup=%u(+%u) raw_rx=%u",
		chan_idx + 1,
		stats->total,          tot_d,
		stats->valid_flag,     valid_d,
		stats->lost_flag,      lost_d,
		stats->error_flag,     err_d,
		stats->bad_len,        bad_len_d,
		stats->seq_gap_packets,gap_pkt_d,
		stats->seq_gap_events, gap_evt_d,
		stats->old_or_dup,     dup_d,
		iso_recv_chan_count[chan_idx]);
}

static void log_uplink_reporter_stats(void)
{
	uint32_t now = k_uptime_get_32();
	int active_count = 0;

	for (int i = 0; i < MAX_UPLINK_REPORTERS; i++)
	{
		char dev_id_hex[(GRPTLK_DEVICE_ID_LEN * 2U) + 1U];

		if (!uplink_reporters[i].used)
		{
			continue;
		}

		uint32_t age_ms = now - uplink_reporters[i].last_seen_ms;

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

	for (int i = 0; i < MAX_UPLINK_REPORTERS; i++)
	{
		char dev_id_hex[(GRPTLK_DEVICE_ID_LEN * 2U) + 1U];

		if (!uplink_reporters[i].used)
		{
			continue;
		}

		struct uplink_reporter_state *r = &uplink_reporters[i];

		format_dev_id_hex(r, dev_id_hex, sizeof(dev_id_hex));

		uint32_t age_ms    = now - r->last_seen_ms;
		uint32_t missed    = age_ms / ISO_STATS_PRINT_INTERVAL_MS;
		uint16_t dl_tot_d  = (uint16_t)(r->dl_rx_total        - r->prev_dl_rx_total);
		uint16_t dl_val_d  = (uint16_t)(r->dl_rx_valid         - r->prev_dl_rx_valid);
		uint16_t dl_lost_d = (uint16_t)(r->dl_rx_lost          - r->prev_dl_rx_lost);
		uint16_t dl_gap_d  = (uint16_t)(r->dl_rx_gap_packets   - r->prev_dl_rx_gap_packets);
		uint16_t dl_dup_d  = (uint16_t)(r->dl_rx_old_or_dup    - r->prev_dl_rx_old_or_dup);

		r->prev_dl_rx_total       = r->dl_rx_total;
		r->prev_dl_rx_valid       = r->dl_rx_valid;
		r->prev_dl_rx_lost        = r->dl_rx_lost;
		r->prev_dl_rx_gap_packets = r->dl_rx_gap_packets;
		r->prev_dl_rx_old_or_dup  = r->dl_rx_old_or_dup;

		const char *bis_match = (r->transport_bis == r->uplink_bis) ? "match" : "mismatch";

		printk(GRPTLK_LOG_DATA_PREFIX
		       "dev=%s bis_rx=%u uplink_bis=%u bis_claim=%s age_ms=%u missed=%u seq=%u "
		       "dl_rx=%u(+%u) dl_valid=%u(+%u) dl_lost=%u(+%u) "
		       "dl_gap=%u(+%u) dl_gap_evt=%u dl_dup=%u(+%u) "
		       "ul_ok=%u ul_alloc=%u ul_send=%u ul_lost=%u "
		       "phy=%u max_pdu=%u bn=%u irc=%u\n",
		       dev_id_hex[0] ? dev_id_hex : "unknown",
		       r->transport_bis, r->uplink_bis, bis_match, age_ms, missed, r->seq_num,
		       r->dl_rx_total,        dl_tot_d,
		       r->dl_rx_valid,        dl_val_d,
		       r->dl_rx_lost,         dl_lost_d,
		       r->dl_rx_gap_packets,  dl_gap_d,
		       r->dl_rx_gap_events,
		       r->dl_rx_old_or_dup,   dl_dup_d,
		       r->ul_tx_ok, r->ul_tx_alloc_fail, r->ul_tx_send_fail, r->ul_lost_payload,
		       r->phy, r->chan_max_pdu, r->chan_bn, r->chan_irc);
	}
}

static void log_chan_info_snapshot(void)
{
	if ((stats_bis == NULL) || (stats_bis_count <= 0)) {
		return;
	}

	for (int i = 0; i < stats_bis_count; i++) {
		struct bt_iso_info info;

		if (stats_bis[i] == NULL) {
			continue;
		}

		int err = bt_iso_chan_get_info(stats_bis[i], &info);

		if (err != 0) {
			LOG_WRN("bt_iso_chan_get_info BIS%d failed: %d", i + 1, err);
			continue;
		}

		if (i == 0) {
			LOG_INF("BIS%d (DL): iso_interval=%u max_subevent=%u can_send=%d can_recv=%d "
				"sync_delay=%u latency=%u pto=%u max_pdu=%u phy=%u bn=%u irc=%u",
				i + 1,
				info.iso_interval, info.max_subevent,
				(int)info.can_send, (int)info.can_recv,
				info.broadcaster.sync_delay, info.broadcaster.latency,
				info.broadcaster.pto, info.broadcaster.max_pdu,
				info.broadcaster.phy, info.broadcaster.bn, info.broadcaster.irc);
		} else {
			LOG_INF("BIS%d (UL): iso_interval=%u max_subevent=%u can_send=%d can_recv=%d "
				"latency=%u pto=%u max_pdu=%u bn=%u irc=%u",
				i + 1,
				info.iso_interval, info.max_subevent,
				(int)info.can_send, (int)info.can_recv,
				info.sync_receiver.latency, info.sync_receiver.pto,
				info.sync_receiver.max_pdu, info.sync_receiver.bn,
				info.sync_receiver.irc);
		}
	}
}

static void log_iso_stats_snapshot(void)
{
	uint32_t dl_tx_ok_d       = stats_delta_u32(iso_downlink_tx_stats.ok,
						     &prev_iso_downlink_tx_stats.ok);
	uint32_t dl_tx_alloc_d    = stats_delta_u32(iso_downlink_tx_stats.alloc_fail,
						     &prev_iso_downlink_tx_stats.alloc_fail);
	uint32_t dl_tx_send_d     = stats_delta_u32(iso_downlink_tx_stats.send_fail,
						     &prev_iso_downlink_tx_stats.send_fail);
	uint32_t frames_d         = stats_delta_u32(uplink_report_frames,     &prev_uplink_report_frames);
	uint32_t short_d          = stats_delta_u32(uplink_report_short,      &prev_uplink_report_short);
	uint32_t parse_fail_d     = stats_delta_u32(uplink_report_parse_fail, &prev_uplink_report_parse_fail);
	uint32_t table_full_d     = stats_delta_u32(uplink_report_table_full, &prev_uplink_report_table_full);

	LOG_INF("----- ISO_BROADCAST STATS [%ums] -----", ISO_STATS_PRINT_INTERVAL_MS);
	LOG_INF("DL TX (BIS1): ok=%u(+%u) alloc_fail=%u(+%u) send_fail=%u(+%u)",
		iso_downlink_tx_stats.ok, dl_tx_ok_d,
		iso_downlink_tx_stats.alloc_fail, dl_tx_alloc_d,
		iso_downlink_tx_stats.send_fail, dl_tx_send_d);

	for (int chan_idx = 1; chan_idx < BIS_ISO_CHAN_COUNT; chan_idx++)
	{
		log_uplink_chan_stats(chan_idx);
	}

	LOG_INF("UL RX anomalies: unknown_chan=%u unexpected_bis1=%u",
		iso_recv_unknown_chan_count, iso_recv_unexpected_bis1_count);
	LOG_INF("UL reports: frames=%u(+%u) short=%u(+%u) parse_fail=%u(+%u) table_full=%u(+%u)",
		uplink_report_frames, frames_d, uplink_report_short, short_d,
		uplink_report_parse_fail, parse_fail_d, uplink_report_table_full, table_full_d);
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

	uint32_t tick = 0U;

	while (true)
	{
		k_sleep(K_MSEC(ISO_STATS_PRINT_INTERVAL_MS));
		log_iso_stats_snapshot();

		tick++;
		if (tick >= CHAN_INFO_TICK_PERIOD) {
			tick = 0U;
			log_chan_info_snapshot();
		}
	}
}

void iso_log_startup_hint(void)
{
	LOG_INF("ISO stats enabled (period=%u ms, chan-info every %u ms)",
		ISO_STATS_PRINT_INTERVAL_MS,
		ISO_STATS_PRINT_INTERVAL_MS * CHAN_INFO_TICK_PERIOD);
}

void iso_start_stats_thread(struct bt_iso_chan **bis, int count)
{
	stats_bis = bis;
	stats_bis_count = count;

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
