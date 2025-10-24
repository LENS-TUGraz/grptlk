/* main.c — I2S RX & TX in dedicated threads + LC3 broadcast from mic
 *
 * SoC: Nordic (nrfx I2S). Codec: MAX9867 (codec drives BCLK/LRCLK).
 * Audio: 16 kHz, 16-bit, stereo I2S; LC3 uses mono 10 ms frames (downmix).
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net_buf.h>
#include <zephyr/drivers/i2s.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>

#include "max9867.h"
#include "lc3.h"

/* --------------------------- Audio & I2S format --------------------------- */

#define SAMPLE_RATE_HZ              16000   /* 16 kHz */
#define CHANNELS                    2       /* stereo */
#define SAMPLE_BYTES                2       /* 16-bit */

#define BLOCK_MS                    10
#define FRAMES_PER_BLOCK            ((SAMPLE_RATE_HZ * BLOCK_MS) / 1000)             /* 160 frames */
#define BLOCK_BYTES                 (FRAMES_PER_BLOCK * CHANNELS * SAMPLE_BYTES)     /* 640 B */

/* RX: slab used by the I2S driver (we consume via i2s_buf_read) */
#define RX_BLOCK_COUNT              24      /* 240 ms RX buffering */
K_MEM_SLAB_DEFINE_STATIC(rx_slab, BLOCK_BYTES, RX_BLOCK_COUNT, 4);

/* TX: slab fed to i2s_write (driver frees after TX) */
#define TX_BLOCK_COUNT              24
K_MEM_SLAB_DEFINE_STATIC(tx_slab, BLOCK_BYTES, TX_BLOCK_COUNT, 4);

/* ----------------------------- LC3/BAP preset ---------------------------- */

#define BROADCAST_ENQUEUE_COUNT     3U
#define TOTAL_BUF_NEEDED (BROADCAST_ENQUEUE_COUNT * CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT)

static struct bt_bap_lc3_preset preset_active =
        BT_BAP_LC3_BROADCAST_PRESET_16_2_1(
            BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
            BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

/* LC3 needs 10 ms @ 16 kHz = 160 mono samples */
#define MAX_FRAME_DURATION_US       10000
#define MAX_NUM_SAMPLES             ((MAX_FRAME_DURATION_US * SAMPLE_RATE_HZ) / USEC_PER_SEC) /* 160 */
static int16_t send_pcm_data[MAX_NUM_SAMPLES]; /* updated from mic each ISO tick */

/* 10-ms mono frames for LC3 (160 * int16_t) */
K_MSGQ_DEFINE(pcm_msgq, sizeof(int16_t) * MAX_NUM_SAMPLES, 16, 4);
/* Stereo 10-ms blocks for loopback playback */
K_MSGQ_DEFINE(tx_msgq, BLOCK_BYTES, 16, 4);

/* ------------------------ Bluetooth broadcast plumbing ------------------- */

struct broadcast_source_stream {
    struct bt_bap_stream      stream;
    uint16_t                  seq_num;
    size_t                    sent_cnt;
    lc3_encoder_t             lc3_encoder;
    lc3_encoder_mem_16k_t     lc3_encoder_mem;
} streams[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];

static struct bt_bap_broadcast_source *broadcast_source;

NET_BUF_POOL_FIXED_DEFINE(tx_pool, TOTAL_BUF_NEEDED,
                          BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
                          CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static bool stopping;
static K_SEM_DEFINE(sem_started, 0U, 1U);
static K_SEM_DEFINE(sem_stopped, 0U, 1U);

static int freq_hz;
static int frame_duration_us;
static int frames_per_sdu;
static int octets_per_frame;

/* pace LC3 with ISO (stream_sent_cb) */
static K_SEM_DEFINE(lc3_encoder_sem, 0U, TOTAL_BUF_NEEDED);

/* ------------------------------ Globals ---------------------------------- */

static const struct device *i2s_dev;

/* ------------------------------ Helpers ---------------------------------- */

static inline void downmix_stereo_block_to_mono(const int16_t *stereo, int16_t *mono)
{
    for (size_t i = 0; i < FRAMES_PER_BLOCK; i++) {
        int32_t L = stereo[2*i + 0];
        int32_t R = stereo[2*i + 1];
        mono[i] = (int16_t)((L + R) / 2);
    }
}

/* ------------------------------ RX thread -------------------------------- */

static void rx_thread_fn(void *p1, void *p2, void *p3)
{
    int err;
    uint8_t  stereo_buf[BLOCK_BYTES];
    int16_t  mono_frame[MAX_NUM_SAMPLES];

    printk("RX thread: capturing mic audio...\n");
    while (1) {
        size_t got = sizeof(stereo_buf);
        err = i2s_buf_read(i2s_dev, stereo_buf, &got);
        if (err == 0 && got == BLOCK_BYTES) {
            /* Downmix to mono (10 ms) and queue for LC3 */
            downmix_stereo_block_to_mono((const int16_t *)stereo_buf, mono_frame);
            if (k_msgq_put(&pcm_msgq, mono_frame, K_NO_WAIT) != 0) {
                int16_t drop[MAX_NUM_SAMPLES];
                (void)k_msgq_get(&pcm_msgq, drop, K_NO_WAIT);
                (void)k_msgq_put(&pcm_msgq, mono_frame, K_NO_WAIT);
            }
            /* Also queue stereo block for local playback (TX thread) */
            if (k_msgq_put(&tx_msgq, stereo_buf, K_NO_WAIT) != 0) {
                uint8_t drop2[BLOCK_BYTES];
                (void)k_msgq_get(&tx_msgq, drop2, K_NO_WAIT);
                (void)k_msgq_put(&tx_msgq, stereo_buf, K_NO_WAIT);
            }
        } else if (err == -EAGAIN || err == -EBUSY || err == -ETIMEDOUT) {
            k_usleep(200);
        } else {
            printk("i2s_buf_read error: %d\n", err);
            break;
        }
    }
}

/* ------------------------------ TX thread -------------------------------- */

static void tx_thread_fn(void *p1, void *p2, void *p3)
{
    int err;
    printk("TX thread: running\n");

    while (1) {
        uint8_t in[BLOCK_BYTES];
        (void)k_msgq_get(&tx_msgq, in, K_FOREVER);

        void *txblk;
        k_mem_slab_alloc(&tx_slab, &txblk, K_FOREVER);
        memcpy(txblk, in, BLOCK_BYTES);

        while ((err = i2s_write(i2s_dev, txblk, BLOCK_BYTES)) == -EAGAIN) {
            k_msleep(1);
        }
        if (err < 0) {
            printk("I2S TX write failed: %d\n", err);
            k_mem_slab_free(&tx_slab, &txblk);
            break;
        }
        /* success: driver frees txblk after sending */
    }
}

/* ------------------------- LC3 / BAP integration ------------------------- */

static void send_data(struct broadcast_source_stream *s)
{
    struct bt_bap_stream *stream = &s->stream;

    if (stopping) return;

    struct net_buf *buf = net_buf_alloc(&tx_pool, K_FOREVER);
    if (!buf) { printk("ISO buf alloc failed\n"); return; }
    net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

    /* Try to get a fresh 10 ms mono frame; otherwise keep previous data */
    int16_t frame[MAX_NUM_SAMPLES];
    if (k_msgq_get(&pcm_msgq, frame, K_MSEC(2)) == 0) {
        memcpy(send_pcm_data, frame, sizeof(frame));
    }
    /* else: keep last send_pcm_data contents (avoids popping to silence) */

    uint8_t lc3_encoded_buffer[CONFIG_BT_ISO_TX_MTU];
    int ret = lc3_encode(s->lc3_encoder,
                         LC3_PCM_FORMAT_S16,
                         send_pcm_data, 1,
                         octets_per_frame,
                         lc3_encoded_buffer);
    if (ret == -1) {
        printk("LC3 encode failed\n");
        net_buf_unref(buf);
        return;
    }

    net_buf_add_mem(buf, lc3_encoded_buffer, preset_active.qos.sdu);

    ret = bt_bap_stream_send(stream, buf, s->seq_num++);
    if (ret < 0) {
        printk("bt_bap_stream_send failed: %d\n", ret);
        net_buf_unref(buf);
        return;
    }

    if ((++s->sent_cnt % 1000U) == 0U) {
        printk("ISO sent: %u\n", s->sent_cnt);
    }
}

static void stream_started_cb(struct bt_bap_stream *stream)
{
    struct broadcast_source_stream *s =
        CONTAINER_OF(stream, struct broadcast_source_stream, stream);
    s->seq_num = 0U;
    s->sent_cnt = 0U;
}

static void stream_sent_cb(struct bt_bap_stream *stream)
{
    k_sem_give(&lc3_encoder_sem); /* pace LC3 at ISO cadence */
}

static struct bt_bap_stream_ops stream_ops = {
    .started = stream_started_cb,
    .sent    = stream_sent_cb,
};

static int setup_broadcast_source(struct bt_bap_broadcast_source **source)
{
    struct bt_bap_broadcast_source_stream_param
        stream_params[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
    struct bt_bap_broadcast_source_subgroup_param
        subgroup_param[CONFIG_BT_BAP_BROADCAST_SRC_SUBGROUP_COUNT];
    struct bt_bap_broadcast_source_param create_param = {0};

    const size_t streams_per_subgroup =
        ARRAY_SIZE(stream_params) / ARRAY_SIZE(subgroup_param);

    uint8_t left_loc[] = {
        BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
                            BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_LEFT))};
    uint8_t right_loc[] = {
        BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
                            BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_RIGHT))};

    for (size_t i = 0U; i < ARRAY_SIZE(subgroup_param); i++) {
        subgroup_param[i].params_count = streams_per_subgroup;
        subgroup_param[i].params       = stream_params + i * streams_per_subgroup;
        subgroup_param[i].codec_cfg    = &preset_active.codec_cfg;
    }
    for (size_t j = 0U; j < ARRAY_SIZE(stream_params); j++) {
        stream_params[j].stream   = &streams[j].stream;
        stream_params[j].data     = (j == 0) ? left_loc : right_loc;
        stream_params[j].data_len = (j == 0) ? sizeof(left_loc) : sizeof(right_loc);
        bt_bap_stream_cb_register(stream_params[j].stream, &stream_ops);
    }

    create_param.params_count = ARRAY_SIZE(subgroup_param);
    create_param.params       = subgroup_param;
    create_param.qos          = &preset_active.qos;
    create_param.encryption   = false;
    create_param.packing      = BT_ISO_PACKING_SEQUENTIAL;

    return bt_bap_broadcast_source_create(&create_param, source);
}

/* ------------------------------ LC3 thread ------------------------------- */

static void lc3_thread(void *a, void *b, void *c)
{
    const struct bt_audio_codec_cfg *cfg = &preset_active.codec_cfg;

    int ret = bt_audio_codec_cfg_get_freq(cfg);
    if (ret > 0) freq_hz = bt_audio_codec_cfg_freq_to_freq_hz(ret);
    else { printk("Codec freq not set\n"); return; }

    ret = bt_audio_codec_cfg_get_frame_dur(cfg);
    if (ret > 0) frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
    else { printk("Frame duration not set\n"); return; }

    octets_per_frame = bt_audio_codec_cfg_get_octets_per_frame(cfg);
    frames_per_sdu   = bt_audio_codec_cfg_get_frame_blocks_per_sdu(cfg, true);
    if (octets_per_frame <= 0) { printk("Octets/frame not set\n"); return; }

    for (size_t i = 0; i < ARRAY_SIZE(streams); i++) {
        streams[i].lc3_encoder = lc3_setup_encoder(frame_duration_us, freq_hz, 0,
                                                   &streams[i].lc3_encoder_mem);
        if (!streams[i].lc3_encoder) {
            printk("LC3 setup failed for stream %zu\n", i);
        }
    }

    /* seed with silence so first packet is valid even if RX is slow to start */
    memset(send_pcm_data, 0, sizeof(send_pcm_data));

    while (1) {
        k_sem_take(&lc3_encoder_sem, K_FOREVER);
        send_data(&streams[0]); /* send on first BIS */
    }
}

/* ------------------------------- main() ---------------------------------- */

/* dedicated stacks and TCBs for RX/TX/LC3 threads */
K_THREAD_STACK_DEFINE(rx_stack, 4096);
K_THREAD_STACK_DEFINE(tx_stack, 4096);
K_THREAD_STACK_DEFINE(encoder_stack, 4096);
static struct k_thread rx_thread;
static struct k_thread tx_thread;
static struct k_thread encoder_thread;

static void source_started_cb(struct bt_bap_broadcast_source *source)
{
    printk("Broadcast source %p started\n", source);
    k_sem_give(&sem_started);
}

static void source_stopped_cb(struct bt_bap_broadcast_source *source, uint8_t reason)
{
    printk("Broadcast source %p stopped (0x%02X)\n", source, reason);
    k_sem_give(&sem_stopped);
}

int main(void)
{
    static struct bt_bap_broadcast_source_cb broadcast_source_cb = {
        .started = source_started_cb,
        .stopped = source_stopped_cb,
    };

    int err;

    /* Init codec (board support) */
    err = max9867_init();
    if (err) { printk("MAX9867 init failed: %d\n", err); return 0; }

    i2s_dev = DEVICE_DT_GET(DT_ALIAS(i2s_node0));
    if (!device_is_ready(i2s_dev)) {
        printk("I2S not ready\n");
        return 0;
    }

    /* Configure RX and TX (same device). Codec is bit/frame clock master. */
    struct i2s_config rx_cfg = {
        .word_size      = 16,
        .channels       = CHANNELS,
        .format         = I2S_FMT_DATA_FORMAT_I2S,
        .options        = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE,
        .frame_clk_freq = SAMPLE_RATE_HZ,
        .mem_slab       = &rx_slab,
        .block_size     = BLOCK_BYTES,
        .timeout        = 1000,
    };
    struct i2s_config tx_cfg = {
        .word_size      = 16,
        .channels       = CHANNELS,
        .format         = I2S_FMT_DATA_FORMAT_I2S,
        .options        = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE,
        .frame_clk_freq = SAMPLE_RATE_HZ,
        .mem_slab       = &tx_slab,
        .block_size     = BLOCK_BYTES,
        .timeout        = 1000,
    };

    err = i2s_configure(i2s_dev, I2S_DIR_RX, &rx_cfg);
    if (err) { printk("I2S RX cfg failed: %d\n", err); return 0; }
    err = i2s_configure(i2s_dev, I2S_DIR_TX, &tx_cfg);
    if (err) { printk("I2S TX cfg failed: %d\n", err); return 0; }

    /* Prefill 2 silent TX blocks so TX is READY at start. */
    for (int i = 0; i < 2; i++) {
        void *txblk;
        k_mem_slab_alloc(&tx_slab, &txblk, K_FOREVER);
        memset(txblk, 0, BLOCK_BYTES);
        err = i2s_write(i2s_dev, txblk, BLOCK_BYTES);
        if (err < 0) {
            printk("TX prefill write failed: %d\n", err);
            k_mem_slab_free(&tx_slab, &txblk);
            return 0;
        }
    }

    /* Start BOTH directions together (prevents nrfx I2S state errors). */
    err = i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_START);
    if (err) { printk("I2S DIR_BOTH START failed: %d\n", err); return 0; }

    /* Start worker threads */
    k_tid_t rx_tid = k_thread_create(&rx_thread, rx_stack, K_THREAD_STACK_SIZEOF(rx_stack),
                                     rx_thread_fn, NULL, NULL, NULL,
                                     K_PRIO_PREEMPT(4), 0, K_NO_WAIT);
    k_thread_name_set(rx_tid, "i2s_rx");

    k_tid_t tx_tid = k_thread_create(&tx_thread, tx_stack, K_THREAD_STACK_SIZEOF(tx_stack),
                                     tx_thread_fn, NULL, NULL, NULL,
                                     K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_name_set(tx_tid, "i2s_tx");

    k_tid_t enc_tid = k_thread_create(&encoder_thread, encoder_stack, K_THREAD_STACK_SIZEOF(encoder_stack),
                                      lc3_thread, NULL, NULL, NULL,
                                      K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_name_set(enc_tid, "lc3");

    /* Bring up Bluetooth + BAP broadcast */
    err = bt_enable(NULL);
    if (err) { printk("Bluetooth init failed (err %d)\n", err); return 0; }
    printk("Bluetooth initialized\n");

    err = bt_bap_broadcast_source_register_cb(&broadcast_source_cb);
    if (err) { printk("Broadcast source cb reg failed: %d\n", err); return 0; }

    err = setup_broadcast_source(&broadcast_source);
    if (err) { printk("setup_broadcast_source failed: %d\n", err); return 0; }

    struct bt_le_ext_adv *adv;
    err = bt_le_ext_adv_create(BT_BAP_ADV_PARAM_BROADCAST_FAST, NULL, &adv);
    if (err) { printk("ext adv create failed: %d\n", err); return 0; }

    err = bt_le_per_adv_set_param(adv, BT_BAP_PER_ADV_PARAM_BROADCAST_FAST);
    if (err) { printk("per adv param failed: %d\n", err); return 0; }

    /* Extended advertising data (Broadcast ID) */
    uint32_t broadcast_id = 0x123456;
    NET_BUF_SIMPLE_DEFINE(ad_buf, BT_UUID_SIZE_16 + BT_AUDIO_BROADCAST_ID_SIZE);
    net_buf_simple_add_le16(&ad_buf, BT_UUID_BROADCAST_AUDIO_VAL);
    net_buf_simple_add_le24(&ad_buf, broadcast_id);

    struct bt_data ext_ad[2] = {
        {
            .type = BT_DATA_SVC_DATA16,
            .data_len = ad_buf.len,
            .data = ad_buf.data,
        },
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
                sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };
    err = bt_le_ext_adv_set_data(adv, ext_ad, 2, NULL, 0);
    if (err) { printk("set ext adv data failed: %d\n", err); return 0; }

    /* Periodic advertising data (BASE) */
    NET_BUF_SIMPLE_DEFINE(base_buf, 128);
    err = bt_bap_broadcast_source_get_base(broadcast_source, &base_buf);
    if (err) { printk("get BASE failed: %d\n", err); return 0; }

    struct bt_data per_ad = {
        .type = BT_DATA_SVC_DATA16,
        .data_len = base_buf.len,
        .data = base_buf.data,
    };
    err = bt_le_per_adv_set_data(adv, &per_ad, 1);
    if (err) { printk("set per adv data failed: %d\n", err); return 0; }

    err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) { printk("ext adv start failed: %d\n", err); return 0; }
    err = bt_le_per_adv_start(adv);
    if (err) { printk("per adv start failed: %d\n", err); return 0; }

    stopping = false;
    err = bt_bap_broadcast_source_start(broadcast_source, adv);
    if (err) { printk("broadcast start failed: %d\n", err); return 0; }

    k_sem_take(&sem_started, K_FOREVER);
    for (size_t i = 0; i < ARRAY_SIZE(streams); i++) {
        for (unsigned int j = 0U; j < BROADCAST_ENQUEUE_COUNT; j++) {
            stream_sent_cb(&streams[i].stream);
        }
    }
    printk("Broadcast started; mic → LC3 → BIS, and mic → speaker loopback\n");

    while (1) {
        k_sleep(K_SECONDS(1));
    }
    return 0;
}
