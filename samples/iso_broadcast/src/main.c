#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

#define BIG_SDU_INTERVAL_US (10000)
#define BUF_ALLOC_TIMEOUT_US (BIG_SDU_INTERVAL_US * 2U)

#define BIS_ISO_CHAN_COUNT 5
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT,
						  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
						  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_term, 0, BIS_ISO_CHAN_COUNT);

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

		buf = net_buf_alloc(&bis_tx_pool, K_USEC(BUF_ALLOC_TIMEOUT_US));
		if (!buf)
		{
			printk("Data buffer allocate timeout\n");
			return;
		}

		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
		sys_put_le32(iso_send_count, &iso_data[0]);
		iso_data[4] = 0x01; /* from BIG creator */
		iso_data[5] = 0x00; /* BIS index */
		net_buf_add_mem(buf, iso_data, sizeof(iso_data));
		err = bt_iso_chan_send(chan, buf, seq_num);
		if (err < 0)
		{
			printk("Unable to broadcast data on channel %p : %d", chan, err);
			net_buf_unref(buf);
			return;
		}

		printk("TX: seq_num: %d - payload: %d\n", seq_num, iso_send_count);

		iso_send_count++;
		seq_num++;
	}
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
					 struct net_buf *buf)
{
	printk("ISO Channel %p received data: len %u, seq_num %u, ts %u, flags 0x%02x\n", chan,
		   buf->len, info->seq_num, info->ts, info->flags);
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

	printk("Starting GRPTLK Broadcaster\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
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

	/* Create BIG */
	err = bt_iso_big_create(adv, &big_create_param, &big);
	if (err)
	{
		printk("Failed to create BIG (err %d)\n", err);
		return 0;
	}

	for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
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

	for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
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

	/* Start Streaming */
	iso_sent(bis[0]);
}