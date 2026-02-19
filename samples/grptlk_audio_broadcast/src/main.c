#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>

#define NUM_PRIME_PACKETS 2

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT * NUM_PRIME_PACKETS,
						  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
						  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT);
static K_SEM_DEFINE(sem_big_term, 0, CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT);

static struct bt_iso_chan *bis[];

static uint16_t seq_num;
static uint32_t iso_send_count = 0U;
static uint8_t iso_data[CONFIG_BT_ISO_TX_MTU] = {0};

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	seq_num = 0U;
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n", chan, reason);
	k_sem_give(&sem_big_term);
}

static void iso_sent(struct bt_iso_chan *chan)
{
	if (chan == bis[0])
	{
		int err;
		struct net_buf *buf;

		buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
		if (!buf)
		{
			printk("Data buffer allocate timeout\n");
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
			printk("Unable to broadcast data on channel %p : %d", chan, err);
			net_buf_unref(buf);
			return;
		}

		// printk("TX: seq_num: %d - payload: %d\n", seq_num, iso_send_count);

		iso_send_count++;
		seq_num++;
	}
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
					 struct net_buf *buf)
{
	printk("ISO_BROADCAST RX %p: ", chan);

	if (buf->len > 0) {
		uint16_t print_len = (buf->len > 16) ? 16 : buf->len;
		printk("payload: ");
		for (uint16_t i = 0; i < print_len; i++) {
			printk("%02X ", buf->data[i]);
		}
		if (buf->len > 16) {
			printk("... [%u more]", buf->len - 16);
		}
	}
	printk("\n");
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
	.rtn = 0,
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
	.num_bis = CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT,
	.bis_channels = bis,
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_UNFRAMED,
};

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static struct bt_bap_broadcast_source *broadcast_source;
static struct bt_bap_lc3_preset preset_active =
	BT_BAP_LC3_BROADCAST_PRESET_16_2_1(
		BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
		BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);
struct broadcast_source_stream
{
	struct bt_bap_stream stream;
	uint16_t seq_num;
	size_t sent_cnt;
	// lc3_encoder_t lc3_encoder;
	// lc3_encoder_mem_16k_t lc3_encoder_mem;
} streams[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];

static int setup_broadcast_source(struct bt_bap_broadcast_source **source)
{
	struct bt_bap_broadcast_source_stream_param
		stream_params[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
	struct bt_bap_broadcast_source_subgroup_param
		subgroup_param[1];
	struct bt_bap_broadcast_source_param create_param = {0};

	const size_t streams_per_subgroup =
		ARRAY_SIZE(stream_params) / ARRAY_SIZE(subgroup_param);

	uint8_t left_loc[] = {
		BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
							BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_LEFT))};
	uint8_t right_loc[] = {
		BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
							BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_RIGHT))};

	for (size_t i = 0U; i < ARRAY_SIZE(subgroup_param); i++)
	{
		subgroup_param[i].params_count = streams_per_subgroup;
		subgroup_param[i].params = stream_params + i * streams_per_subgroup;
		subgroup_param[i].codec_cfg = &preset_active.codec_cfg;
	}
	for (size_t j = 0U; j < ARRAY_SIZE(stream_params); j++)
	{
		stream_params[j].stream = &streams[j].stream;
		stream_params[j].data = (j == 0) ? left_loc : right_loc;
		stream_params[j].data_len = (j == 0) ? sizeof(left_loc) : sizeof(right_loc);
		// bt_bap_stream_cb_register(stream_params[j].stream, &stream_ops);
	}

	create_param.params_count = ARRAY_SIZE(subgroup_param);
	create_param.params = subgroup_param;
	create_param.qos = &preset_active.qos;
	create_param.encryption = false;
	create_param.packing = BT_ISO_PACKING_SEQUENTIAL;

	return bt_bap_broadcast_source_create(&create_param, source);
}

int main(void)
{
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;
	int err;

	printk("Starting GRPTLK Broadcaster\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	err = setup_broadcast_source(&broadcast_source);
	if (err)
	{
		printk("setup_broadcast_source failed: %d\n", err);
		return 0;
	}

	/* Create a non-connectable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, NULL, &adv);
	if (err)
	{
		printk("Failed to create advertising set (err %d)\n", err);
		return 0;
	}

	/* Set advertising data to have complete local name set */
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err)
	{
		printk("Failed to set advertising data (err %d)\n", err);
		return 0;
	}

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
	if (err)
	{
		printk("Failed to set periodic advertising parameters"
			   " (err %d)\n",
			   err);
		return 0;
	}

	/* Periodic advertising data (BASE) */
	NET_BUF_SIMPLE_DEFINE(base_buf, 128);
	err = bt_bap_broadcast_source_get_base(broadcast_source, &base_buf);
	if (err)
	{
		printk("get BASE failed: %d\n", err);
		return 0;
	}

	struct bt_data per_ad = {
		.type = BT_DATA_SVC_DATA16,
		.data_len = base_buf.len,
		.data = base_buf.data,
	};

	err = bt_le_per_adv_set_data(adv, &per_ad, 1);
	if (err)
	{
		printk("set per adv data failed: %d\n", err);
		return 0;
	}

	/* Enable Periodic Advertising */
	err = bt_le_per_adv_start(adv);
	if (err)
	{
		printk("Failed to enable periodic advertising (err %d)\n", err);
		return 0;
	}

	/* Start extended advertising */
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err)
	{
		printk("Failed to start extended advertising (err %d)\n", err);
		return 0;
	}

	big_create_param.bis_channels = bis;
	big_create_param.interval = preset_active.qos.interval;
	big_create_param.latency = preset_active.qos.latency;

	for (uint8_t chan = 0U; chan < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; chan++) {
		bis[chan]->qos->tx->rtn = preset_active.qos.rtn;
	}

	/* Create BIG */
	err = bt_iso_big_create(adv, &big_create_param, &big);
	if (err)
	{
		printk("Failed to create BIG (err %d)\n", err);
		return 0;
	}

	for (uint8_t chan = 0U; chan < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; chan++)
	{
		printk("Waiting for BIG complete chan %u...\n", chan);

		err = k_sem_take(&sem_big_cmplt, K_FOREVER);
		if (err)
		{
			printk("failed (err %d)\n", err);
			return 0;
		}

		printk("BIG create complete chan %u.\n", chan);
	}

	for (uint8_t chan = 0U; chan < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; chan++)
	{
		printk("Setting data path chan %u...\n", chan);

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
			printk("Failed to setup ISO data path: %d\n", err);
		}

		printk("Setting data path complete chan %u.\n", chan);
	}

	for (int i = 0; i < NUM_PRIME_PACKETS; i++) {
		iso_sent(bis[0]);
	}
}