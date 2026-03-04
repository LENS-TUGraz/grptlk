#ifndef GRPTLK_UPLINK_STATS_H_
#define GRPTLK_UPLINK_STATS_H_

#include <stdint.h>

#define GRPTLK_UPLINK_MAGIC_0 0x47
#define GRPTLK_UPLINK_MAGIC_1 0x54

#define GRPTLK_UPLINK_VERSION       1U
#define GRPTLK_UPLINK_TYPE_STATS_V1 1U
#define GRPTLK_UPLINK_STATS_V1_SIZE 40U

#define GRPTLK_UPLINK_VERSION_2      2U
#define GRPTLK_UPLINK_TYPE_STATS_V2  2U
#define GRPTLK_UPLINK_STATS_V2_SIZE  40U

/* chan_info byte: bn in bits[7:4], irc in bits[3:0] */
#define GRPTLK_CHAN_INFO_PACK(bn, irc)  ((uint8_t)(((bn) << 4) | ((irc) & 0x0FU)))
#define GRPTLK_CHAN_INFO_BN(ci)         ((uint8_t)(((ci) >> 4) & 0x0FU))
#define GRPTLK_CHAN_INFO_IRC(ci)        ((uint8_t)((ci) & 0x0FU))

#define GRPTLK_DEVICE_ID_LEN 8U

/* Prefix for structured log lines. Parsers match lines starting with this string.
 * Format: GRPTLK_DATA key=val key=val ...\n
 */
#define GRPTLK_LOG_DATA_PREFIX "GRPTLK_DATA "

struct __packed grptlk_uplink_stats_v1
{
	uint8_t  magic[2];
	uint8_t  version;
	uint8_t  type;
	uint8_t  uplink_bis;
	uint8_t  dev_id_len;
	uint8_t  reserved[2];
	uint8_t  dev_id[GRPTLK_DEVICE_ID_LEN];
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
};

struct __packed grptlk_uplink_stats_v2
{
	uint8_t  magic[2];
	uint8_t  version;
	uint8_t  type;
	uint8_t  uplink_bis;
	uint8_t  dev_id_len;
	uint8_t  phy;        /* BT_GAP_LE_PHY_* from biginfo */
	uint8_t  chan_info;  /* bn<<4 | irc from bt_iso_chan_get_info() */
	uint8_t  dev_id[GRPTLK_DEVICE_ID_LEN];
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
};

#endif /* GRPTLK_UPLINK_STATS_H_ */
