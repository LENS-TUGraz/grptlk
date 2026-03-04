#include <errno.h>
#include <zephyr/kernel.h>
#include "lc3.h"

#if defined(CONFIG_SOC_NRF5340_CPUAPP)
#include <nrfx_clock.h>
#endif
#include "io/audio.h"
#include "io/led.h"
#include <stdint.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

/* BAP preset: 16 kHz, 10 ms frames, 40 octets/frame */
static struct bt_bap_lc3_preset preset_active =
    BT_BAP_LC3_BROADCAST_PRESET_16_2_1(BT_AUDIO_LOCATION_FRONT_LEFT |
                                           BT_AUDIO_LOCATION_FRONT_RIGHT,
                                       BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

#define PCM_SAMPLES_PER_FRAME AUDIO_SAMPLES_PER_FRAME
#define SAMPLE_RATE_HZ        AUDIO_SAMPLE_RATE_HZ
#define BLOCK_BYTES           AUDIO_BLOCK_BYTES

/* BIS layout: bis[0] = TX (BIS1), bis[1..N] = RX uplink */
#define NUM_RX_BIS (CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT - 1)
#define NUM_PRIME_PACKETS 2

/* Queues */
K_MSGQ_DEFINE(pcm_msgq,      sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 16, 4);
K_MSGQ_DEFINE(tx_msgq,       BLOCK_BYTES, 8, 4);
K_MSGQ_DEFINE(uplink_mix_q,  sizeof(int16_t) * PCM_SAMPLES_PER_FRAME, 2, 4);
K_MSGQ_DEFINE(lc3_encoded_q, CONFIG_BT_ISO_TX_MTU, 2, 4);

struct uplink_frame {
    uint16_t len;
    uint8_t  data[CONFIG_BT_ISO_TX_MTU];
};
struct k_msgq uplink_rx_q[NUM_RX_BIS];
char __aligned(4) uplink_rx_q_buffer[NUM_RX_BIS][8 * sizeof(struct uplink_frame)];

/* Semaphore capped at 1: the first arriving BIS packet wakes the decoder;
 * additional gives from other BISes in the same interval are dropped. */
K_SEM_DEFINE(uplink_rx_sem, 0, 1);

/* LC3 */
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_16k_t lc3_encoder_mem;
static lc3_decoder_t lc3_decoders[NUM_RX_BIS];
static lc3_decoder_mem_16k_t lc3_decoder_mems[NUM_RX_BIS];
static int16_t send_pcm_data[PCM_SAMPLES_PER_FRAME];

/* BAP */
static struct bt_bap_broadcast_source *broadcast_source;
static struct bt_bap_stream streams[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];

static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ISO */
NET_BUF_POOL_FIXED_DEFINE(
    bis_tx_pool, CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT * NUM_PRIME_PACKETS,
    BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), CONFIG_BT_CONN_TX_USER_DATA_SIZE,
    NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT);
static K_SEM_DEFINE(tx_sem, 0, NUM_PRIME_PACKETS);

static struct bt_iso_chan *bis[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
static struct bt_iso_chan  bis_iso_chan[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
static uint16_t seq_num;

static struct bt_iso_big_create_param big_create_param = {
    .num_bis      = CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT,
    .bis_channels = bis,
    .packing      = BT_ISO_PACKING_SEQUENTIAL,
    .framing      = BT_ISO_FRAMING_UNFRAMED,
};

static struct bt_iso_chan_io_qos iso_rx_qos;
static struct bt_iso_chan_io_qos iso_tx_qos = {
    .sdu = CONFIG_BT_ISO_TX_MTU,
    .phy = BT_GAP_LE_PHY_2M,
};
static struct bt_iso_chan_qos bis_iso_qos = {
    .tx = &iso_tx_qos,
    .rx = &iso_rx_qos,
};

/* Threads */
#define ENCODER_STACK_SIZE   4096
#define ENCODER_PRIORITY     5
#define DECODER_PRIORITY     5
#define TX_THREAD_STACK_SIZE 1024
#define TX_THREAD_PRIORITY   5

K_THREAD_STACK_DEFINE(encoder_stack,    ENCODER_STACK_SIZE);
K_THREAD_STACK_DEFINE(uplink_dec_stack, 6144);
K_THREAD_STACK_DEFINE(tx_thread_stack,  TX_THREAD_STACK_SIZE);
static struct k_thread encoder_thread_data;
static struct k_thread uplink_dec_thread_data;
static struct k_thread tx_thread_data;

/* -------------------------------------------------------------------------- */

static void audio_rx_mono_frame(const int16_t *mono_frame)
{
    if (k_msgq_put(&pcm_msgq, mono_frame, K_NO_WAIT) != 0) {
        int16_t dropped[PCM_SAMPLES_PER_FRAME];
        (void)k_msgq_get(&pcm_msgq, dropped, K_NO_WAIT);
        (void)k_msgq_put(&pcm_msgq, mono_frame, K_NO_WAIT);
    }
}

static void encoder_thread_func(void *arg1, void *arg2, void *arg3)
{
    int ret;
    uint8_t encoded_buf[CONFIG_BT_ISO_TX_MTU];
    static uint32_t enc_frame_cnt;
    int octets_per_frame = preset_active.qos.sdu;

    while (1) {
        ret = k_msgq_get(&pcm_msgq, send_pcm_data, K_FOREVER);
        if (ret != 0) {
            continue;
        }

        int16_t uplink_mono[PCM_SAMPLES_PER_FRAME];
        if (k_msgq_get(&uplink_mix_q, uplink_mono, K_NO_WAIT) == 0) {
            for (int i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
                int32_t s = (int32_t)send_pcm_data[i] + (int32_t)uplink_mono[i];
                if (s > 32767)       { s = 32767; }
                else if (s < -32768) { s = -32768; }
                send_pcm_data[i] = (int16_t)s;
            }
        }

        uint32_t t0 = k_cycle_get_32();
        ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, send_pcm_data, 1,
                         octets_per_frame, encoded_buf);
        uint32_t enc_us = k_cyc_to_us_floor32(k_cycle_get_32() - t0);

        if (ret == -1) {
            printk("LC3 encode failed\n");
            continue;
        }

        if ((enc_frame_cnt++ % 100U) == 0U) {
            printk("[enc] frame=%u encode=%u us pcm_q=%u lc3_q=%u\n",
                   enc_frame_cnt, enc_us,
                   k_msgq_num_used_get(&pcm_msgq),
                   k_msgq_num_used_get(&lc3_encoded_q));
        }

        if (k_msgq_put(&lc3_encoded_q, encoded_buf, K_NO_WAIT) != 0) {
            uint8_t dropped[CONFIG_BT_ISO_TX_MTU];
            (void)k_msgq_get(&lc3_encoded_q, dropped, K_NO_WAIT);
            (void)k_msgq_put(&lc3_encoded_q, encoded_buf, K_NO_WAIT);
        }
    }
}

static void uplink_decoder_thread_func(void *arg1, void *arg2, void *arg3)
{
    static uint32_t dec_frame_cnt;

    while (1) {
        k_sem_take(&uplink_rx_sem, K_MSEC(12));

        uint32_t t0_total = k_cycle_get_32();
        uint32_t dec_us[NUM_RX_BIS];
        int32_t mixed_pcm[PCM_SAMPLES_PER_FRAME] = {0};

        for (int i = 0; i < NUM_RX_BIS; i++) {
            struct uplink_frame frame;
            int16_t stream_pcm[PCM_SAMPLES_PER_FRAME];
            int ret = k_msgq_get(&uplink_rx_q[i], &frame, K_NO_WAIT);

            uint32_t t0 = k_cycle_get_32();
            if (ret == 0 && frame.len > 0) {
                ret = lc3_decode(lc3_decoders[i], frame.data, frame.len,
                                 LC3_PCM_FORMAT_S16, stream_pcm, 1);
                if (ret != 0) {
                    memset(stream_pcm, 0, sizeof(stream_pcm));
                }
            } else {
                lc3_decode(lc3_decoders[i], NULL, preset_active.qos.sdu,
                           LC3_PCM_FORMAT_S16, stream_pcm, 1);
            }
            dec_us[i] = k_cyc_to_us_floor32(k_cycle_get_32() - t0);

            for (int s = 0; s < PCM_SAMPLES_PER_FRAME; s++) {
                mixed_pcm[s] += stream_pcm[s];
            }
        }

        uint32_t total_us = k_cyc_to_us_floor32(k_cycle_get_32() - t0_total);
        if ((dec_frame_cnt++ % 100U) == 0U) {
            printk("[dec] frame=%u total=%u us  tx_q=%u\n",
                   dec_frame_cnt, total_us, k_msgq_num_used_get(&tx_msgq));
            for (int i = 0; i < NUM_RX_BIS; i++) {
                printk("      bis%d=%u us\n", i + 1, dec_us[i]);
            }
        }

        int16_t stereo_buf[PCM_SAMPLES_PER_FRAME * 2];
        for (int i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
            int32_t s = mixed_pcm[i];
            if (s > 32767)       { s = 32767; }
            else if (s < -32768) { s = -32768; }
            stereo_buf[2 * i]     = (int16_t)s;
            stereo_buf[2 * i + 1] = (int16_t)s;
        }

        int16_t mono_mix[PCM_SAMPLES_PER_FRAME];
        for (int i = 0; i < PCM_SAMPLES_PER_FRAME; i++) {
            mono_mix[i] = stereo_buf[2 * i];
        }
        if (k_msgq_put(&uplink_mix_q, mono_mix, K_NO_WAIT) != 0) {
            static int mix_overrun_cnt;
            if ((mix_overrun_cnt++ % 100) == 0) {
                printk("uplink_mix_q full (cnt=%d)\n", mix_overrun_cnt);
            }
        }

        if (k_msgq_put(&tx_msgq, stereo_buf, K_NO_WAIT) != 0) {
            static int overrun_cnt;
            if ((overrun_cnt++ % 100) == 0) {
                printk("tx_msgq full - dropping frame (cnt=%d)\n", overrun_cnt);
            }
        }
    }
}

static void tx_thread(void *arg1, void *arg2, void *arg3)
{
    int err;
    struct net_buf *buf;
    uint8_t enc_data[CONFIG_BT_ISO_TX_MTU];

    while (true) {
        k_sem_take(&tx_sem, K_FOREVER);

        buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
        if (!buf) {
            printk("tx_thread: net_buf pool exhausted\n");
            continue;
        }

        net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

        if (k_msgq_get(&lc3_encoded_q, enc_data, K_NO_WAIT) != 0) {
            memset(enc_data, 0, sizeof(enc_data));
        }
        net_buf_add_mem(buf, enc_data, sizeof(enc_data));

        err = bt_iso_chan_send(bis[0], buf, seq_num);
        if (err < 0) {
            printk("Unable to broadcast data: %d\n", err);
            net_buf_unref(buf);
            continue;
        }

        seq_num++;
    }
}

/* -------------------------------------------------------------------------- */

static void iso_connected(struct bt_iso_chan *chan)
{
    printk("ISO Channel %p connected\n", chan);
    seq_num = 0U;
    k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
    printk("ISO Channel %p disconnected with reason 0x%02x\n", chan, reason);
}

static void iso_sent(struct bt_iso_chan *chan)
{
    if (chan == bis[0]) {
        k_sem_give(&tx_sem);
    }
}

static void iso_recv(struct bt_iso_chan *chan,
                     const struct bt_iso_recv_info *info, struct net_buf *buf)
{
    struct uplink_frame frame;

    if (buf && buf->len == CONFIG_BT_ISO_TX_MTU && (info->flags & BT_ISO_FLAGS_VALID)) {
        frame.len = buf->len;
        memcpy(frame.data, buf->data, buf->len);
    } else {
        frame.len = 0;
    }

    /* bis[0] is TX-only; uplink channels are bis[1..N] → uplink_rx_q[0..N-1] */
    for (int i = 0; i < NUM_RX_BIS; i++) {
        if (chan == bis[i + 1]) {
            if (k_msgq_put(&uplink_rx_q[i], &frame, K_NO_WAIT) == 0) {
                k_sem_give(&uplink_rx_sem);
            }
            break;
        }
    }
}

static struct bt_iso_chan_ops iso_ops = {
    .connected    = iso_connected,
    .disconnected = iso_disconnected,
    .sent         = iso_sent,
    .recv         = iso_recv,
};

/* -------------------------------------------------------------------------- */

static int setup_broadcast_source(struct bt_bap_broadcast_source **source)
{
    struct bt_bap_broadcast_source_stream_param
        stream_params[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
    struct bt_bap_broadcast_source_subgroup_param subgroup_param[1];
    struct bt_bap_broadcast_source_param create_param = {0};

    uint8_t left_loc[] = {
        BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
                            BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_LEFT))};
    uint8_t right_loc[] = {
        BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
                            BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_RIGHT))};

    for (size_t i = 0U; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
        stream_params[i].stream   = &streams[i];
        stream_params[i].data     = (i == 0) ? left_loc : right_loc;
        stream_params[i].data_len = (i == 0) ? sizeof(left_loc) : sizeof(right_loc);
    }

    subgroup_param[0].params_count = CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT;
    subgroup_param[0].params       = stream_params;
    subgroup_param[0].codec_cfg    = &preset_active.codec_cfg;

    create_param.params_count = ARRAY_SIZE(subgroup_param);
    create_param.params       = subgroup_param;
    create_param.qos          = &preset_active.qos;
    create_param.encryption   = false;
    create_param.packing      = BT_ISO_PACKING_SEQUENTIAL;

    return bt_bap_broadcast_source_create(&create_param, source);
}

static int setup_extended_adv(struct bt_le_ext_adv **adv)
{
    int err;

    err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, NULL, adv);
    if (err) {
        printk("Failed to create advertising set (err %d)\n", err);
        return err;
    }

    err = bt_le_ext_adv_set_data(*adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Failed to set advertising data (err %d)\n", err);
        return err;
    }

    return 0;
}

static int setup_periodic_adv(struct bt_le_ext_adv *adv)
{
    int err;

    err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
    if (err) {
        printk("Failed to set periodic advertising parameters (err %d)\n", err);
        return err;
    }

    err = setup_broadcast_source(&broadcast_source);
    if (err) {
        printk("setup_broadcast_source failed: %d\n", err);
        return err;
    }

    NET_BUF_SIMPLE_DEFINE(base_buf, 128);
    err = bt_bap_broadcast_source_get_base(broadcast_source, &base_buf);
    if (err) {
        printk("get BASE failed: %d\n", err);
        return err;
    }

    struct bt_data per_ad = {
        .type     = BT_DATA_SVC_DATA16,
        .data_len = base_buf.len,
        .data     = base_buf.data,
    };

    err = bt_le_per_adv_set_data(adv, &per_ad, 1);
    if (err) {
        printk("set per adv data failed: %d\n", err);
        return err;
    }

    return 0;
}

static int create_big(struct bt_le_ext_adv *adv, struct bt_iso_big **big)
{
    int err;

    for (size_t i = 0; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
        bis_iso_chan[i].ops = &iso_ops;
        bis_iso_chan[i].qos = &bis_iso_qos;
        bis[i] = &bis_iso_chan[i];
    }

    for (uint8_t i = 0U; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
        bis[i]->qos->tx->rtn = preset_active.qos.rtn;
    }

    big_create_param.interval     = preset_active.qos.interval;
    big_create_param.latency      = preset_active.qos.latency;
    big_create_param.bis_channels = bis;

    err = bt_iso_big_create(adv, &big_create_param, big);
    if (err) {
        printk("Failed to create BIG (err %d)\n", err);
        return err;
    }

    for (uint8_t i = 0U; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
        printk("Waiting for BIG complete chan %u...\n", i);
        err = k_sem_take(&sem_big_cmplt, K_FOREVER);
        if (err) {
            printk("failed (err %d)\n", err);
            return err;
        }
        printk("BIG create complete chan %u.\n", i);
    }

    return 0;
}

static int setup_iso_datapaths(void)
{
    int err;

    for (uint8_t i = 0U; i < CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT; i++) {
        printk("Setting data path chan %u...\n", i);

        const struct bt_iso_chan_path hci_path = {
            .pid    = BT_ISO_DATA_PATH_HCI,
            .format = BT_HCI_CODING_FORMAT_TRANSPARENT,
        };

        uint8_t dir = (i == 0) ? BT_HCI_DATAPATH_DIR_HOST_TO_CTLR
                                : BT_HCI_DATAPATH_DIR_CTLR_TO_HOST;

        err = bt_iso_setup_data_path(&bis_iso_chan[i], dir, &hci_path);
        if (err != 0) {
            printk("Failed to setup ISO data path: %d\n", err);
            return err;
        }

        printk("Setting data path complete chan %u.\n", i);
    }

    return 0;
}

static void prime_and_start_iso_transmission(void)
{
    for (int i = 0; i < NUM_PRIME_PACKETS; i++) {
        k_sem_give(&tx_sem);
    }
}

/* -------------------------------------------------------------------------- */

int main(void)
{
    struct bt_le_ext_adv *adv;
    struct bt_iso_big *big;
    int err;
    int led_err;

#if defined(CONFIG_SOC_NRF5340_CPUAPP)
    int clk_ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK,
                                          NRF_CLOCK_HFCLK_DIV_1);
    clk_ret -= NRFX_ERROR_BASE_NUM;
    if (clk_ret) {
        printk("Set 128 MHz clock divider failed: %d\n", clk_ret);
        return clk_ret;
    }
    printk("CPU clock set to 128 MHz\n");
#endif

    printk("Starting GRPTLK Broadcaster\n");
    led_err = led_init();
    if (led_err) {
        printk("led_init failed: %d\n", led_err);
    } else {
        (void)led_set_broadcast_running(false);
    }

    for (int i = 0; i < NUM_RX_BIS; i++) {
        k_msgq_init(&uplink_rx_q[i], uplink_rx_q_buffer[i],
                    sizeof(struct uplink_frame), 8);
    }

    err = audio_init(&tx_msgq, audio_rx_mono_frame);
    if (err) {
        printk("audio_init failed: %d\n", err);
        return err;
    }

    err = audio_start();
    if (err) {
        printk("audio_start failed: %d\n", err);
        return err;
    }

    lc3_encoder = lc3_setup_encoder(preset_active.qos.interval, SAMPLE_RATE_HZ,
                                     0, &lc3_encoder_mem);
    if (lc3_encoder == NULL) {
        printk("Failed to setup LC3 encoder\n");
        return -EIO;
    }

    for (int i = 0; i < NUM_RX_BIS; i++) {
        lc3_decoders[i] = lc3_setup_decoder(preset_active.qos.interval,
                                              SAMPLE_RATE_HZ, 0,
                                              &lc3_decoder_mems[i]);
        if (lc3_decoders[i] == NULL) {
            printk("Failed to setup LC3 decoder %d\n", i);
            return -EIO;
        }
    }

    k_thread_create(&encoder_thread_data, encoder_stack,
                    K_THREAD_STACK_SIZEOF(encoder_stack), encoder_thread_func,
                    NULL, NULL, NULL, ENCODER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&encoder_thread_data, "lc3_encoder");

    k_thread_create(&uplink_dec_thread_data, uplink_dec_stack,
                    K_THREAD_STACK_SIZEOF(uplink_dec_stack),
                    uplink_decoder_thread_func, NULL, NULL, NULL,
                    DECODER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&uplink_dec_thread_data, "uplink_dec");

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return err;
    }

    err = setup_extended_adv(&adv);
    if (err) {
        return err;
    }

    err = setup_periodic_adv(adv);
    if (err) {
        return err;
    }

    err = bt_le_per_adv_start(adv);
    if (err) {
        printk("Failed to enable periodic advertising (err %d)\n", err);
        return err;
    }

    err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        printk("Failed to start extended advertising (err %d)\n", err);
        return err;
    }

    err = create_big(adv, &big);
    if (err) {
        return err;
    }

    err = setup_iso_datapaths();
    if (err) {
        return err;
    }

    k_thread_create(&tx_thread_data, tx_thread_stack,
                    K_THREAD_STACK_SIZEOF(tx_thread_stack),
                    tx_thread, NULL, NULL, NULL,
                    TX_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&tx_thread_data, "iso_tx");

    prime_and_start_iso_transmission();
    led_err = led_set_broadcast_running(true);
    if (led_err) {
        printk("led_set_broadcast_running failed: %d\n", led_err);
    }

    return 0;
}
