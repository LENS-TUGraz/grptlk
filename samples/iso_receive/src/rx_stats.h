#ifndef RX_STATS_H
#define RX_STATS_H

#include <zephyr/bluetooth/iso.h>
#include <stddef.h>

struct net_buf;
struct bt_iso_recv_info;

#if IS_ENABLED(CONFIG_GRPTLK_ISO_STATS_ENABLED)

void rx_stats_init_device_id(void);
void rx_stats_init_report_cache(uint8_t active_uplink_bis);
void rx_stats_reset_seq(void);

void rx_stats_tx_ok(void);
void rx_stats_tx_alloc_fail(void);
void rx_stats_tx_send_fail(void);

void rx_stats_rx_downlink(const struct bt_iso_recv_info *info, struct net_buf *buf);

void rx_stats_prepare_uplink_payload(uint8_t *data, size_t sdu_len, uint16_t seq_num, uint8_t active_uplink_bis);

#else

static inline void rx_stats_init_device_id(void) {}
static inline void rx_stats_init_report_cache(uint8_t active_uplink_bis) {}
static inline void rx_stats_reset_seq(void) {}

static inline void rx_stats_tx_ok(void) {}
static inline void rx_stats_tx_alloc_fail(void) {}
static inline void rx_stats_tx_send_fail(void) {}

static inline void rx_stats_rx_downlink(const struct bt_iso_recv_info *info, struct net_buf *buf) {}

static inline void rx_stats_prepare_uplink_payload(uint8_t *data, size_t sdu_len, uint16_t seq_num, uint8_t active_uplink_bis)
{
	if (data && sdu_len > 0) {
		memset(data, (uint8_t)seq_num, sdu_len);
	}
}

#endif /* IS_ENABLED(CONFIG_GRPTLK_ISO_STATS_ENABLED) */

#endif /* RX_STATS_H */
