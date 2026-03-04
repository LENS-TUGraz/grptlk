#ifndef UPLINK_STATS_H
#define UPLINK_STATS_H

#include <zephyr/bluetooth/iso.h>

struct net_buf;
struct bt_iso_recv_info;
struct bt_iso_chan;

#if IS_ENABLED(CONFIG_GRPTLK_ISO_STATS)

void iso_log_startup_hint(void);
/*
 * Start the stats thread.  bis[] and count are stored so the thread can call
 * bt_iso_chan_get_info() on each channel once per second.
 */
void iso_start_stats_thread(struct bt_iso_chan **bis, int count);
void iso_stats_reset_seq(void);

void iso_stats_tx_ok(void);
void iso_stats_tx_alloc_fail(void);
void iso_stats_tx_send_fail(void);

void iso_stats_rx_unknown_chan(void);
void iso_stats_rx_unexpected_bis1(void);
void iso_stats_rx(int chan_idx, const struct bt_iso_recv_info *info, struct net_buf *buf);

#else

static inline void iso_log_startup_hint(void) {}
static inline void iso_start_stats_thread(struct bt_iso_chan **bis, int count)
{
	(void)bis;
	(void)count;
}
static inline void iso_stats_reset_seq(void) {}

static inline void iso_stats_tx_ok(void) {}
static inline void iso_stats_tx_alloc_fail(void) {}
static inline void iso_stats_tx_send_fail(void) {}

static inline void iso_stats_rx_unknown_chan(void) {}
static inline void iso_stats_rx_unexpected_bis1(void) {}
static inline void iso_stats_rx(int chan_idx, const struct bt_iso_recv_info *info, struct net_buf *buf) {}

#endif /* IS_ENABLED(CONFIG_GRPTLK_ISO_STATS) */

#endif /* UPLINK_STATS_H */
