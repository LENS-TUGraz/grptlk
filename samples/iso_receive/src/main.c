#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN 30

#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE,    \
										   BT_LE_SCAN_OPT_NONE,       \
										   BT_GAP_SCAN_FAST_INTERVAL, \
										   BT_GAP_SCAN_FAST_WINDOW)

#define PA_RETRY_COUNT 6

#define BIS_ISO_CHAN_COUNT 3

static bool per_adv_found;
static bool per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t per_sid;
static uint32_t per_interval_us;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_sync_lost, 0, BIS_ISO_CHAN_COUNT);

static void scan_recv(const struct bt_le_scan_recv_info *info,
					  struct net_buf_simple *buf)
{
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
	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
					const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
		   bt_le_per_adv_sync_get_index(sync), le_addr);

	per_adv_lost = true;
	k_sem_give(&sem_per_sync_lost);
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
					   const struct bt_iso_biginfo *biginfo)
{
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

static uint16_t seq_num;
static uint32_t iso_send_count = 0U;
static uint8_t iso_data[CONFIG_BT_ISO_TX_MTU] = {0};

static void iso_sent(struct bt_iso_chan *chan)
{
	int err;
	struct net_buf *buf;

	if (iso_send_count > 4) {
		return;
	}

	buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
	if (!buf)
	{
		printk("Data buffer allocate timeout\n");
		return;
	}

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	// put 0xdeadbeef into iso_data
	sys_put_le32(0xdeadbeef, &iso_data[0]);
	// sys_put_le32(iso_send_count, &iso_data[0]);
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

static struct bt_iso_chan *bis[];

static uint32_t iso_recv_count = 0U;

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
					 struct net_buf *buf)
{
	uint32_t count = 0;
	uint8_t from_big_creator = 0;
	uint8_t bis_index = 0;

	if (buf->len == CONFIG_BT_ISO_TX_MTU)
	{
		count = sys_get_le32(buf->data);
		from_big_creator = buf->data[4];
		bis_index = buf->data[5];

		if (iso_recv_count < 12) {
		printk("Incoming data on BIS %u %s with payload: %u\n",
			   bis_index, from_big_creator ? "from big creator" : "from other receiver", count);
		}
	}
	if (iso_recv_count > 10)
	{
		iso_sent(bis[1]);
	}

	iso_recv_count++;
}

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n",
		   chan, reason);

	if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST)
	{
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.sent = iso_sent,
	.recv = iso_recv,
	.connected = iso_connected,
	.disconnected = iso_disconnected,
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
};

static struct bt_iso_chan *bis[] = {
	&bis_iso_chan[0],
	&bis_iso_chan[1],
	&bis_iso_chan[2],
};

static struct bt_iso_big_sync_param big_sync_param = {
	.bis_channels = bis,
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_bitfield = (BIT_MASK(BIS_ISO_CHAN_COUNT)),
	.mse = BT_ISO_SYNC_MSE_ANY,
	.sync_timeout = 100, /* in 10 ms units */
};

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

	printk("Starting GRPTLK Receiver\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Scan callbacks register...");
	bt_le_scan_cb_register(&scan_callbacks);
	printk("success.\n");

	printk("Periodic Advertising callbacks register...");
	bt_le_per_adv_sync_cb_register(&sync_callbacks);
	printk("Success.\n");

	do
	{
		reset_semaphores();
		per_adv_lost = false;

		printk("Start scanning...");
		err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
		if (err)
		{
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		printk("Waiting for periodic advertising...\n");
		per_adv_found = false;
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err)
		{
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("Found periodic advertising.\n");

		printk("Stop scanning...");
		err = bt_le_scan_stop();
		if (err)
		{
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		printk("Creating Periodic Advertising Sync...");
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
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		printk("Waiting for periodic sync...\n");
		err = k_sem_take(&sem_per_sync, K_USEC(sem_timeout_us));
		if (err)
		{
			printk("failed (err %d)\n", err);

			printk("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err)
			{
				printk("failed (err %d)\n", err);
				return 0;
			}
			continue;
		}
		printk("Periodic sync established.\n");

		printk("Waiting for BIG info...\n");
		err = k_sem_take(&sem_per_big_info, K_USEC(sem_timeout_us));
		if (err)
		{
			printk("failed (err %d)\n", err);

			if (per_adv_lost)
			{
				continue;
			}

			printk("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err)
			{
				printk("failed (err %d)\n", err);
				return 0;
			}
			continue;
		}
		printk("Periodic sync established.\n");

	big_sync_create:
		printk("Create BIG Sync...\n");
		err = bt_iso_big_sync(sync, &big_sync_param, &big);
		if (err)
		{
			printk("failed (err %d)\n", err);
			return 0;
		}
		printk("success.\n");

		for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
		{
			printk("Waiting for BIG sync chan %u...\n", chan);
			err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
			if (err)
			{
				break;
			}
			printk("BIG sync chan %u successful.\n", chan);
		}
		if (err)
		{
			printk("failed (err %d)\n", err);

			printk("BIG Sync Terminate...");
			err = bt_iso_big_terminate(big);
			if (err)
			{
				printk("failed (err %d)\n", err);
				return 0;
			}
			printk("done.\n");

			goto per_sync_lost_check;
		}
		printk("BIG sync established.\n");

		for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
		{
			printk("Setting data path chan %u...\n", chan);

			const struct bt_iso_chan_path hci_path = {
				.pid = BT_ISO_DATA_PATH_HCI,
				.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
			};

			uint8_t dir = BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;

			if (chan == 0)
			{
				dir = BT_HCI_DATAPATH_DIR_CTLR_TO_HOST;
			}

			err = bt_iso_setup_data_path(&bis_iso_chan[chan], dir, &hci_path);
			if (err != 0)
			{
				printk("Failed to setup ISO RX data path: %d\n", err);
			}

			printk("Setting data path complete chan %u.\n", chan);
		}

		for (uint8_t chan = 0U; chan < BIS_ISO_CHAN_COUNT; chan++)
		{
			printk("Waiting for BIG sync lost chan %u...\n", chan);
			err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
			if (err)
			{
				printk("failed (err %d)\n", err);
				return 0;
			}
			printk("BIG sync lost chan %u.\n", chan);
		}
		printk("BIG sync lost.\n");

	per_sync_lost_check:
		printk("Check for periodic sync lost...\n");
		err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
		if (err)
		{
			/* Periodic Sync active, go back to creating BIG Sync */
			goto big_sync_create;
		}
		printk("Periodic sync lost.\n");
	} while (true);
}