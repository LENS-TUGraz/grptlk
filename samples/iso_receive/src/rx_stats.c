#include "rx_stats.h"

#if IS_ENABLED(CONFIG_GRPTLK_ISO_STATS_ENABLED)

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <grptlk_uplink_stats.h>

#define BIS_ISO_CHAN_COUNT CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT

LOG_MODULE_DECLARE(grptlk_iso_receive);

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

static struct iso_tx_stats iso_uplink_tx_stats;
static struct iso_rx_stats iso_downlink_rx_stats;
static uint16_t iso_recv_expected_seq[BIS_ISO_CHAN_COUNT];
static bool iso_recv_seq_initialized[BIS_ISO_CHAN_COUNT];

BUILD_ASSERT(sizeof(struct grptlk_uplink_stats_v1) == GRPTLK_UPLINK_STATS_V1_SIZE,
	     "Stats payload format must stay 40 bytes");

static uint8_t grptlk_device_id[GRPTLK_DEVICE_ID_LEN];
static uint8_t grptlk_device_id_len;
static uint32_t report_last_ms;
static struct grptlk_uplink_stats_v1 report_cache;

/* Populate device ID from BT identity address.
 * Called after bt_enable() so the address is available.
 * The BT random address is unique per device in both BSim and real hardware. */
void rx_stats_init_device_id(void)
{
	bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
	size_t count = ARRAY_SIZE(addrs);

	bt_id_get(addrs, &count);
	if (count > 0U) {
		/* BD_ADDR is 6 bytes, fits within GRPTLK_DEVICE_ID_LEN=8. */
		grptlk_device_id_len = sizeof(addrs[0].a.val);
		memcpy(grptlk_device_id, addrs[0].a.val, grptlk_device_id_len);
	} else {
		grptlk_device_id_len = 0U;
		memset(grptlk_device_id, 0, sizeof(grptlk_device_id));
	}
}

void rx_stats_init_report_cache(uint8_t active_uplink_bis)
{
	memset(&report_cache, 0, sizeof(report_cache));
	report_cache.magic[0] = GRPTLK_UPLINK_MAGIC_0;
	report_cache.magic[1] = GRPTLK_UPLINK_MAGIC_1;
	report_cache.version = GRPTLK_UPLINK_VERSION;
	report_cache.type = GRPTLK_UPLINK_TYPE_STATS_V1;
	report_cache.uplink_bis = active_uplink_bis;
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

void rx_stats_reset_seq(void)
{
	memset(iso_recv_seq_initialized, 0, sizeof(iso_recv_seq_initialized));
}

void rx_stats_tx_ok(void)
{
	iso_uplink_tx_stats.ok++;
}

void rx_stats_tx_alloc_fail(void)
{
	iso_uplink_tx_stats.alloc_fail++;
}

void rx_stats_tx_send_fail(void)
{
	iso_uplink_tx_stats.send_fail++;
}

void rx_stats_rx_downlink(const struct bt_iso_recv_info *info, struct net_buf *buf)
{
	struct iso_rx_stats *rx_stats = &iso_downlink_rx_stats;
	int chan_idx = 0; // Downlink is always index 0

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
}

void rx_stats_prepare_uplink_payload(uint8_t *data, size_t sdu_len, uint16_t seq_num, uint8_t active_uplink_bis)
{
	refresh_report_cache();
	report_cache.seq_num = sys_cpu_to_le16(seq_num);
	report_cache.uplink_bis = active_uplink_bis;
	report_cache.dev_id_len = grptlk_device_id_len;
	memcpy(report_cache.dev_id, grptlk_device_id, sizeof(report_cache.dev_id));
	memcpy(data, &report_cache, MIN(sdu_len, sizeof(report_cache)));
}

#endif /* IS_ENABLED(CONFIG_GRPTLK_ISO_STATS_ENABLED) */
