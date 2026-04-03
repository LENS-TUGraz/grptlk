#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "audio/audio.h"
#include "audio/drivers/audio_i2s.h"
#include "audio/sync/clk_sync.h"
#include "io/buttons.h"
#include "io/led.h"
#include "io/debug_gpio.h"

#if defined(CONFIG_GRPTLK_LC3_CODEC_T2)
#include "sw_codec_lc3.h"
#else
#include "lc3.h"
#endif
#include <zephyr/drivers/gpio.h>

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define PA_RETRY_COUNT	    6

#if (CONFIG_GRPTLK_UPLINK_BIS < 2) || (CONFIG_GRPTLK_UPLINK_BIS > CONFIG_BT_ISO_MAX_CHAN)
#error "CONFIG_GRPTLK_UPLINK_BIS must be in [2..CONFIG_BT_ISO_MAX_CHAN]"
#endif

#define DECODER_STACK_SIZE   6144
#define DECODER_PRIORITY     1
#define ENCODER_STACK_SIZE   4096
#define ENCODER_PRIORITY     2
#define TX_THREAD_STACK_SIZE 1024
#define TX_THREAD_PRIORITY   2

#define BT_LE_SCAN_CUSTOM                                                                          \
	BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, BT_LE_SCAN_OPT_NONE, BT_GAP_SCAN_FAST_INTERVAL,   \
			 BT_GAP_SCAN_FAST_WINDOW)

K_MSGQ_DEFINE(encoded_frame_q, sizeof(struct audio_encoded_frame), 2, 1);

static uint32_t ring_fifo[AUDIO_RING_NUM_BLKS][AUDIO_BLK_SAMPLES_MONO];
volatile uint16_t ring_prod_idx;
volatile uint16_t ring_cons_idx;

static struct audio_ring playback_ring = {
	.fifo = ring_fifo,
	.prod_idx = &ring_prod_idx,
	.cons_idx = &ring_cons_idx,
	.num_blks = AUDIO_RING_NUM_BLKS,
};

#define RING_NEXT(i)  (((i) + 1U) % AUDIO_RING_NUM_BLKS)
#define RING_FILLED() /* single-producer/single-consumer safe */                                   \
	((ring_prod_idx >= ring_cons_idx) ? (ring_prod_idx - ring_cons_idx)                        \
					  : (AUDIO_RING_NUM_BLKS - ring_cons_idx + ring_prod_idx))

static int16_t mic_pcm_shared[AUDIO_SAMPLES_PER_FRAME];
static K_SEM_DEFINE(mic_frame_sem, 0, 1);

static struct k_msgq *uplink_q;
static atomic_t uplink_path_ready = ATOMIC_INIT(0);

static atomic_t tx_in_progress = ATOMIC_INIT(0);

K_SEM_DEFINE(tx_sem, 0, 1);

static struct bt_bap_lc3_preset preset_active = BT_BAP_LC3_BROADCAST_PRESET_16_2_1(
	BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
	BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

#define GRPTLK_VENDOR_CODEC_ID	 BT_HCI_CODING_FORMAT_VS
#define GRPTLK_VENDOR_COMPANY_ID 0xDEAD
#define GRPTLK_VENDOR_VENDOR_ID	 0xBEEF
#if defined(CONFIG_GRPTLK_AUDIO_FRAME_10_MS)
#define GRPTLK_CODEC_INTERVAL_US 10000U
#define GRPTLK_CODEC_SDU_BYTES	 40U
#else
#define GRPTLK_CODEC_INTERVAL_US 5000U
#define GRPTLK_CODEC_SDU_BYTES	 20U
#endif
#define GRPTLK_DOWNLINK_APPENDIX_LEN	      4U
#define GRPTLK_MAX_UPLINK_BIS_BITMAP_BITS 30U

#if defined(CONFIG_GRPTLK_AUDIO_FRAME_5_MS)
static void override_preset_for_lc3plus_5ms(void)
{
	preset_active.qos.interval = GRPTLK_CODEC_INTERVAL_US;
	preset_active.qos.sdu = GRPTLK_CODEC_SDU_BYTES;
	preset_active.qos.rtn = 1U;
}
#endif

#if defined(CONFIG_GRPTLK_LC3_CODEC_T2)
/* T2 LC3 - uses global state, no handles needed */
#define LC3_PCM_BYTES_PER_FRAME (AUDIO_SAMPLES_PER_FRAME * sizeof(int16_t))
static uint16_t t2_pcm_bytes_req;
#else
/* Open LC3 - uses handles */
static lc3_decoder_t lc3_decoder;
static lc3_decoder_mem_16k_t lc3_decoder_mem;
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_16k_t lc3_encoder_mem;
#endif

static uint8_t big_actual_num_bis = CONFIG_BT_ISO_MAX_CHAN;
static uint8_t active_uplink_bis = CONFIG_GRPTLK_UPLINK_BIS - 1U;
#if defined(CONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT)
static uint8_t ptt_session_bis = 0xFFU;
static uint32_t ptt_wait_no_free_log_cnt;
#endif
static uint32_t big_sdu_interval_us = GRPTLK_CODEC_INTERVAL_US;
static uint16_t big_max_sdu;
static atomic_t downlink_codec_sdu_bytes = ATOMIC_INIT(GRPTLK_CODEC_SDU_BYTES);
static atomic_t downlink_appendix_bytes = ATOMIC_INIT(0);
static atomic_t downlink_uplink_bitmap = ATOMIC_INIT(0);
static atomic_t downlink_format_warned = ATOMIC_INIT(0);

static bool per_adv_found;
static bool per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t per_sid;
static uint32_t per_interval_us;
static atomic_t base_logged;
static atomic_t base_parse_warned;

static uint16_t get_downlink_codec_sdu_bytes(void)
{
	return (uint16_t)atomic_get(&downlink_codec_sdu_bytes);
}

static uint8_t get_downlink_appendix_bytes(void)
{
	return (uint8_t)atomic_get(&downlink_appendix_bytes);
}

static uint32_t get_downlink_uplink_bitmap(void)
{
	return (uint32_t)atomic_get(&downlink_uplink_bitmap);
}

#if defined(CONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT)
static bool select_random_uplink_bis_from_mask(uint32_t bis_mask, uint8_t *selected_idx)
{
	uint32_t candidate_count;
	uint32_t choice;

	if ((bis_mask == 0U) || (selected_idx == NULL)) {
		return false;
	}

	candidate_count = POPCOUNT(bis_mask);
	choice = sys_rand32_get() % candidate_count;

	for (uint8_t bit_idx = 0U; bit_idx < GRPTLK_MAX_UPLINK_BIS_BITMAP_BITS; bit_idx++) {
		if ((bis_mask & BIT(bit_idx)) == 0U) {
			continue;
		}

		if (choice == 0U) {
			*selected_idx = bit_idx + 1U;
			return true;
		}

		choice--;
	}

	return false;
}

static bool ptt_try_select_session_bis(void)
{
	const uint8_t num_uplink = (big_actual_num_bis > 1U) ? (big_actual_num_bis - 1U) : 0U;
	uint32_t candidate_mask;
	uint8_t selected_idx;

	if (ptt_session_bis != 0xFFU) {
		return true;
	}

	if ((atomic_get(&src_line_in_active) != 0) || (atomic_get(&ptt_active) == 0)) {
		return false;
	}

	if (num_uplink == 0U) {
		return false;
	}

	candidate_mask = BIT_MASK(num_uplink);

	if (get_downlink_appendix_bytes() == GRPTLK_DOWNLINK_APPENDIX_LEN) {
		candidate_mask &= ~get_downlink_uplink_bitmap();
		if (candidate_mask == 0U) {
			if ((ptt_wait_no_free_log_cnt++ % 200U) == 0U) {
				printk("[Uplink] Waiting for a free uplink BIS for PTT session\n");
			}
			return false;
		}
	}

	if (!select_random_uplink_bis_from_mask(candidate_mask, &selected_idx)) {
		return false;
	}

	ptt_session_bis = selected_idx;
	ptt_wait_no_free_log_cnt = 0U;
	printk("[Uplink] Selected BIS%d for PTT session\n", ptt_session_bis + 1U);

	return true;
}
#endif

static void format_downlink_appendix_bitmap(uint32_t appendix, char *bitmap, size_t len)
{
	const size_t bit_count =
		(len > 0U) ? MIN((size_t)GRPTLK_MAX_UPLINK_BIS_BITMAP_BITS, len - 1U) : 0U;

	for (size_t i = 0; i < bit_count; i++) {
		bitmap[i] = (appendix & BIT(i)) ? '1' : '0';
	}

	if (len > 0U) {
		bitmap[bit_count] = '\0';
	}
}

static void note_downlink_transport_overhead(uint16_t payload_bytes, const char *source)
{
	const uint16_t codec_sdu_bytes = get_downlink_codec_sdu_bytes();
	const uint16_t extra_bytes =
		(payload_bytes > codec_sdu_bytes) ? (payload_bytes - codec_sdu_bytes) : 0U;

	if ((extra_bytes == 0U) || (extra_bytes == GRPTLK_DOWNLINK_APPENDIX_LEN)) {
		atomic_set(&downlink_appendix_bytes, extra_bytes);
		if (extra_bytes == 0U) {
			atomic_set(&downlink_uplink_bitmap, 0);
		}
		return;
	}

	atomic_set(&downlink_appendix_bytes, 0);
	atomic_set(&downlink_uplink_bitmap, 0);
	if (atomic_cas(&downlink_format_warned, 0, 1)) {
		printk("Unexpected %s overhead: payload=%u codec=%u extra=%u\n", source,
		       payload_bytes, codec_sdu_bytes, extra_bytes);
	}
}

static void update_downlink_codec_sdu_bytes(uint16_t codec_sdu_bytes)
{
	if ((codec_sdu_bytes == 0U) || (codec_sdu_bytes > CONFIG_BT_ISO_TX_MTU)) {
		return;
	}

	atomic_set(&downlink_codec_sdu_bytes, codec_sdu_bytes);
	if (big_max_sdu > 0U) {
		note_downlink_transport_overhead(big_max_sdu, "BIGInfo");
	}
}

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, CONFIG_BT_ISO_MAX_CHAN);
static K_SEM_DEFINE(sem_big_sync_lost, 0, CONFIG_BT_ISO_MAX_CHAN);

static atomic_t big_chan_connected;
static atomic_t uplink_tx_active;

static bool iso_datapaths_setup;

static struct bt_iso_big *grptlk_big;

static struct k_timer bis1_activity_timer;
static struct k_work bis1_disconnect_work;

static void bis1_disconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	printk("BIS1 activity timeout — terminating BIG sync\n");
	k_timer_stop(&bis1_activity_timer);
	atomic_set(&uplink_path_ready, 0);
	if (grptlk_big != NULL) {
		int err = bt_iso_big_terminate(grptlk_big);

		if (err != 0) {
			printk("bt_iso_big_terminate failed: %d\n", err);
		}
	}
}

static void bis1_activity_timeout_handler(struct k_timer *timer)
{
	k_work_submit(&bis1_disconnect_work);
}

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, CONFIG_BT_ISO_MAX_CHAN * 1,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static struct bt_iso_chan bis_iso_chan[CONFIG_BT_ISO_MAX_CHAN];
static struct bt_iso_chan *bis[CONFIG_BT_ISO_MAX_CHAN];
static uint16_t seq_num;

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

static struct bt_iso_big_sync_param big_sync_param = {
	.bis_channels = bis,
	.num_bis = CONFIG_BT_ISO_MAX_CHAN,
	.bis_bitfield = BIT_MASK(CONFIG_BT_ISO_MAX_CHAN),
	.mse = 1,
	.sync_timeout = 100,
};

static void audio_rx_mono_frame(const int16_t *mono_frame)
{
	static uint32_t rx_frame_cnt;

	memcpy(mic_pcm_shared, mono_frame, sizeof(int16_t) * AUDIO_SAMPLES_PER_FRAME);
	k_sem_give(&mic_frame_sem);

	if ((rx_frame_cnt++ % 200U) == 0U) {
		int32_t sum = 0;
		for (int i = 0; i < 8; i++) {
			sum += mono_frame[i];
		}
		printk("[mic] frame=%u, samples[0-7].sum=%d\n", rx_frame_cnt, sum);
	}
}

static void decoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	struct audio_encoded_frame frame;
	int16_t mono_pcm[AUDIO_SAMPLES_PER_FRAME];
	static uint32_t rx_frame_cnt;
	static uint32_t downlink_appendix_log_cnt;
	static uint32_t rx_plc_cnt;	/* frames decoded via PLC (no real packet) */
	static uint32_t rx_dec_err_cnt; /* lc3_decode() hard failures */
	static uint32_t rx_frame_drop_cnt; /* whole decoded frame dropped */

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		int ret;

		k_msgq_get(&encoded_frame_q, &frame, K_FOREVER);
		const uint16_t codec_sdu_bytes = get_downlink_codec_sdu_bytes();

		debug_lc3_dec_set(1);

		const uint8_t *lc3_data = (frame.len > 0U) ? frame.data : NULL;
		const int lc3_len =
			(frame.len > 0U) ? MIN((int)frame.len, (int)codec_sdu_bytes) : 0;

		if (lc3_data == NULL) {
			rx_plc_cnt++;
		}

#if defined(CONFIG_GRPTLK_LC3_CODEC_T2)
		uint16_t pcm_len;
		bool bad_frame = (lc3_data == NULL);

		ret = sw_codec_lc3_dec_run(lc3_data, lc3_len, sizeof(mono_pcm),
					   0, mono_pcm, &pcm_len, bad_frame);
#else
		ret = lc3_decode(lc3_decoder, lc3_data, lc3_len, LC3_PCM_FORMAT_S16, mono_pcm, 1);
#endif
		if (ret < 0) {
			rx_dec_err_cnt++;
			memset(mono_pcm, 0, sizeof(mono_pcm));
		}

		debug_lc3_dec_set(0);

		if (frame.len == (codec_sdu_bytes + GRPTLK_DOWNLINK_APPENDIX_LEN)) {
			char uplink_bitmap[GRPTLK_MAX_UPLINK_BIS_BITMAP_BITS + 1U];
			uint32_t downlink_appendix = sys_get_le32(&frame.data[codec_sdu_bytes]);

			atomic_set(&downlink_uplink_bitmap, (atomic_val_t)downlink_appendix);

			if ((downlink_appendix_log_cnt++ % 100U) == 0U) {
				format_downlink_appendix_bitmap(downlink_appendix, uplink_bitmap,
								sizeof(uplink_bitmap));
				printk("[rx] downlink appendix uplinks=%s\n", uplink_bitmap);
			}
		}

		const uint16_t prewrite_fill = (uint16_t)RING_FILLED();
		const uint16_t free_blks = (AUDIO_RING_NUM_BLKS - 1U) - prewrite_fill;

		clk_sync_rx_notify(prewrite_fill);

		if (free_blks < AUDIO_BLKS_PER_FRAME) {
			rx_frame_drop_cnt++;
			continue;
		}

		for (uint8_t blk = 0; blk < AUDIO_BLKS_PER_FRAME; blk++) {
			const uint16_t prod_idx = ring_prod_idx;
			uint32_t *dst = ring_fifo[prod_idx];

			for (uint8_t s = 0; s < AUDIO_BLK_SAMPLES_MONO; s++) {
				int16_t sample = mono_pcm[blk * AUDIO_BLK_SAMPLES_MONO + s];

				dst[s] = ((uint32_t)(uint16_t)sample << 16) | (uint16_t)sample;
			}

			ring_prod_idx = RING_NEXT(prod_idx);
		}

		if ((rx_frame_cnt++ % 200U) == 0U) {
			printk("[rx] frame=%u plc=%u err=%u drop=%u ring=%u underrun=%u\n",
			       rx_frame_cnt, rx_plc_cnt, rx_dec_err_cnt, rx_frame_drop_cnt,
			       (uint32_t)RING_FILLED(), audio_underrun_count_reset());
			rx_plc_cnt = 0;
			rx_dec_err_cnt = 0;
			rx_frame_drop_cnt = 0;
		}
	}
}

static void encoder_thread_func(void *arg1, void *arg2, void *arg3)
{
	int16_t local_pcm[AUDIO_SAMPLES_PER_FRAME];
	uint8_t encoded_buf[CONFIG_BT_ISO_TX_MTU];
	const int octets_per_frame = preset_active.qos.sdu;
	static uint32_t enc_frame_cnt;
	static uint32_t enc_q_full_cnt;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		int ret;
		struct audio_encoded_frame ef = { 0 };

		k_sem_take(&mic_frame_sem, K_FOREVER);

		if (!atomic_get(&uplink_path_ready)) {
			continue;
		}

		memcpy(local_pcm, mic_pcm_shared, sizeof(local_pcm));

		static uint32_t enc_dbg_cnt;
		int32_t pcm_sum = 0;
		for (int i = 0; i < 8; i++) {
			pcm_sum += local_pcm[i];
		}
		if ((enc_dbg_cnt++ % 100U) == 0U) {
			printk("[enc] frame=%u, ptt=%ld, pcm[0-7].sum=%d\n", enc_dbg_cnt,
			       (long)atomic_get(&ptt_active), pcm_sum);
		}

		if (!atomic_get(&src_line_in_active) && atomic_get(&ptt_active) == 0) {
			continue;
		}

#if defined(CONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT)
		if ((atomic_get(&src_line_in_active) == 0) && (atomic_get(&ptt_active) != 0) &&
		    !ptt_try_select_session_bis()) {
			continue;
		}
#endif

		debug_lc3_enc_set(1);

#if defined(CONFIG_GRPTLK_LC3_CODEC_T2)
		uint16_t encoded_len;

		ret = sw_codec_lc3_enc_run(local_pcm, sizeof(local_pcm), LC3_USE_BITRATE_FROM_INIT,
					   0, sizeof(encoded_buf), encoded_buf, &encoded_len);
		if (ret) {
			printk("[enc] sw_codec_lc3_enc_run failed: %d\n", ret);
			debug_lc3_enc_set(0);
			continue;
		}
		/* Verify encoded size matches expected */
		if (encoded_len != octets_per_frame) {
			printk("[enc] LC3 encode size mismatch: got %u, expected %d\n",
			       encoded_len, octets_per_frame);
		}
#else
		ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, local_pcm, 1, octets_per_frame,
				 encoded_buf);
		if (ret < 0) {
			printk("[enc] lc3_encode error: %d\n", ret);
			debug_lc3_enc_set(0);
			continue;
		}
#endif

		memcpy(ef.data, encoded_buf, octets_per_frame);
		const uint8_t appendix_bytes = get_downlink_appendix_bytes();
		if (appendix_bytes > 0U) {
			memset(&ef.data[octets_per_frame], 0, appendix_bytes);
		}
		ef.len = octets_per_frame + appendix_bytes;

		ret = k_msgq_put(uplink_q, &ef, K_NO_WAIT);
		if (ret != 0) {
			/* Queue full — drop frame (clk_sync should prevent this) */
			enc_q_full_cnt++;
			if ((enc_q_full_cnt % 10U) == 1U) {
				printk("[enc] WARN: uplink_q full (drop #%u)\n", enc_q_full_cnt);
			}
		}

		debug_lc3_enc_set(0);

		if ((enc_frame_cnt++ % 200U) == 0U) {
			printk("[enc] frame=%u q_full=%u\n", enc_frame_cnt, enc_q_full_cnt);
			enc_q_full_cnt = 0;
		}
	}
}

static struct bt_iso_chan *iso_select_uplink_chan(void)
{
#if defined(CONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT)
	if ((atomic_get(&src_line_in_active) == 0) && (atomic_get(&ptt_active) != 0)) {
		if (!ptt_try_select_session_bis()) {
			return NULL;
		}
	}

	if (ptt_session_bis != 0xFFU) {
		active_uplink_bis = ptt_session_bis;
		return bis[active_uplink_bis];
	}

	active_uplink_bis = CONFIG_GRPTLK_UPLINK_BIS - 1U;
	return bis[active_uplink_bis];

#elif defined(CONFIG_GRPTLK_UPLINK_RANDOM)
	uint8_t num_uplink = (big_actual_num_bis > 1U) ? (big_actual_num_bis - 1U) : 0U;

	if (num_uplink == 0U) {
		return NULL;
	}
	uint8_t idx = (uint8_t)(sys_rand32_get() % num_uplink) + 1U;

	active_uplink_bis = idx;

	return bis[active_uplink_bis];
#else
	active_uplink_bis = CONFIG_GRPTLK_UPLINK_BIS - 1U;
	return bis[active_uplink_bis];
#endif
}

#if defined(CONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT)
void ptt_session_bis_reset(void)
{
	ptt_session_bis = 0xFFU;
	ptt_wait_no_free_log_cnt = 0U;
}
#endif

static int uplink_send_next(struct bt_iso_chan *chan, const struct audio_encoded_frame *ef);

static void tx_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		struct bt_iso_chan *ul_chan;
		struct audio_encoded_frame ef;
		int ret;

		ret = k_msgq_get(uplink_q, &ef, K_MSEC(10));
		if (ret != 0) {
			if (k_sem_take(&tx_sem, K_NO_WAIT) == 0) {
				continue;
			}
			continue;
		}

		while (atomic_get(&tx_in_progress) != 0) {
			k_sleep(K_USEC(100));
		}

		ul_chan = iso_select_uplink_chan();
		if (ul_chan != NULL) {
			(void)uplink_send_next(ul_chan, &ef);
		}
	}
}

static int uplink_send_next(struct bt_iso_chan *chan, const struct audio_encoded_frame *ef)
{
	struct net_buf *buf;
	int err;

	if (ef->len == 0) {
		printk("[send] skipped: ef->len=0\n");
		return -ENODATA;
	}

	buf = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
	if (!buf) {
		static uint32_t pool_err_cnt;

		if (atomic_get(&uplink_tx_active) != 0 && (pool_err_cnt++ % 50U) == 0U) {
			printk("iso_tx: net_buf pool exhausted (cnt=%u)\n", pool_err_cnt);
		}
		return -ENOMEM;
	}

	if (!atomic_get(&src_line_in_active) && atomic_get(&ptt_active) == 0) {
		net_buf_unref(buf);
		return 0;
	}

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	net_buf_add_mem(buf, ef->data, ef->len);

	atomic_set(&tx_in_progress, 1);

	err = bt_iso_chan_send(chan, buf, seq_num);
	if (err < 0) {
		printk("bt_iso_chan_send failed on %p: %d\n", chan, err);
		net_buf_unref(buf);
		return err;
	}

	atomic_set(&uplink_tx_active, 1);
	seq_num++;
	return 0;
}

static void iso_sent(struct bt_iso_chan *chan)
{
	debug_iso_sent_set(1);

	if (chan == bis[active_uplink_bis]) {
		atomic_set(&tx_in_progress, 0);
	}

	debug_iso_sent_set(0);
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	if (chan != bis[0]) {
		return;
	}

	debug_iso_recv_set(1);

	{
		struct audio_encoded_frame ef;

		memset(&ef, 0, sizeof(ef));

		if ((info->flags & BT_ISO_FLAGS_TS) != 0U) {
			ef.sdu_ref_us = info->ts;
		}

		if ((buf != NULL) && (buf->len <= sizeof(ef.data)) &&
		    ((info->flags & BT_ISO_FLAGS_VALID) != 0U)) {
			ef.len = (uint16_t)buf->len;
			memcpy(ef.data, buf->data, buf->len);
			note_downlink_transport_overhead(ef.len, "RX frame");
			k_timer_start(&bis1_activity_timer, K_MSEC(500), K_NO_WAIT);
		} else {
			ef.len = 0U;
		}
		k_msgq_put(&encoded_frame_q, &ef, K_NO_WAIT);
	}

	debug_iso_recv_set(0);

	iso_datapaths_setup = false;
}

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);
	atomic_inc(&big_chan_connected);
	k_sem_give(&sem_big_sync);
	if (chan == bis[0]) {
		k_timer_start(&bis1_activity_timer, K_MSEC(500), K_NO_WAIT);
	}
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected with reason 0x%02x\n", chan, reason);
	atomic_set(&uplink_path_ready, 0);
	if (chan == bis[active_uplink_bis]) {
		atomic_set(&uplink_tx_active, 0);
	}
	if (chan == bis[0]) {
		k_timer_stop(&bis1_activity_timer);
	}
	if (reason == BT_HCI_ERR_CONN_FAIL_TO_ESTAB || reason == BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
		k_sem_give(&sem_big_sync);
	} else {
		k_sem_give(&sem_big_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.connected = iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
	.recv = iso_recv,
};

static bool base_pull_len_prefixed_blob(struct net_buf_simple *buf, uint8_t *len_out,
					const uint8_t **data_out)
{
	uint8_t len;
	const uint8_t *data;

	if (buf->len < sizeof(len)) {
		return false;
	}

	len = net_buf_simple_pull_u8(buf);
	if (buf->len < len) {
		return false;
	}

	data = buf->data;

	if (len_out != NULL) {
		*len_out = len;
	}

	if (data_out != NULL) {
		*data_out = data;
	}

	net_buf_simple_pull_mem(buf, len);

	return true;
}

static bool base_is_grptlk_vendor_codec(uint8_t codec_id, uint16_t cid, uint16_t vid)
{
	return (codec_id == GRPTLK_VENDOR_CODEC_ID) && (cid == GRPTLK_VENDOR_COMPANY_ID) &&
	       (vid == GRPTLK_VENDOR_VENDOR_ID);
}

static bool base_extract_octets_per_frame(const uint8_t *codec_cfg, uint8_t codec_cfg_len,
					  uint16_t *octets_per_frame)
{
	size_t pos = 0U;

	while (pos < codec_cfg_len) {
		uint8_t len;
		uint8_t type;
		uint8_t value_len;

		len = codec_cfg[pos++];
		if ((len == 0U) || (pos + len > codec_cfg_len)) {
			return false;
		}

		type = codec_cfg[pos++];
		value_len = len - 1U;

		if (type == BT_AUDIO_CODEC_CFG_FRAME_LEN && value_len == sizeof(uint16_t)) {
			*octets_per_frame = sys_get_le16(&codec_cfg[pos]);
			return true;
		}

		pos += value_len;
	}

	return false;
}

static int base_log_service_data(const struct bt_data *data)
{
	struct net_buf_simple buf;
	uint32_t presentation_delay_us;
	uint8_t subgroup_count;

	if ((data == NULL) || (data->type != BT_DATA_SVC_DATA16) ||
	    (data->data_len < (BT_UUID_SIZE_16 + 3U + 1U))) {
		return -ENOENT;
	}

	net_buf_simple_init_with_data(&buf, (void *)data->data, data->data_len);
	if (net_buf_simple_pull_le16(&buf) != BT_UUID_BASIC_AUDIO_VAL) {
		return -ENOENT;
	}

	if (buf.len < 4U) {
		return -EINVAL;
	}

	presentation_delay_us = net_buf_simple_pull_le24(&buf);
	subgroup_count = net_buf_simple_pull_u8(&buf);
	if (subgroup_count == 0U) {
		return -EINVAL;
	}

	printk("BASE: presentation_delay=%u us, num_subgroups=%u\n", presentation_delay_us,
	       subgroup_count);

	for (uint8_t subgroup_idx = 0U; subgroup_idx < subgroup_count; subgroup_idx++) {
		uint8_t bis_count;
		uint8_t codec_id;
		uint8_t codec_cfg_len;
		uint8_t meta_len;
		const uint8_t *codec_cfg;
		uint16_t cid;
		uint16_t vid;
		uint32_t bis_bitfield = 0U;
		const char *codec_note = "";

		if (buf.len < 1U + 1U + 2U + 2U + 1U + 1U) {
			return -EINVAL;
		}

		bis_count = net_buf_simple_pull_u8(&buf);
		if (bis_count == 0U) {
			return -EINVAL;
		}

		codec_id = net_buf_simple_pull_u8(&buf);
		cid = net_buf_simple_pull_le16(&buf);
		vid = net_buf_simple_pull_le16(&buf);

		if (!base_pull_len_prefixed_blob(&buf, &codec_cfg_len, &codec_cfg) ||
		    !base_pull_len_prefixed_blob(&buf, &meta_len, NULL)) {
			return -EINVAL;
		}

		{
			uint16_t octets_per_frame;

			if (base_extract_octets_per_frame(codec_cfg, codec_cfg_len, &octets_per_frame)) {
				update_downlink_codec_sdu_bytes(octets_per_frame);
			}
		}

		for (uint8_t bis_idx = 0U; bis_idx < bis_count; bis_idx++) {
			uint8_t bis_index;
			uint8_t bis_codec_cfg_len;

			if (buf.len < 2U) {
				return -EINVAL;
			}

			bis_index = net_buf_simple_pull_u8(&buf);
			bis_codec_cfg_len = net_buf_simple_pull_u8(&buf);
			if (buf.len < bis_codec_cfg_len) {
				return -EINVAL;
			}

			net_buf_simple_pull_mem(&buf, bis_codec_cfg_len);

			if ((bis_index > 0U) && (bis_index <= 32U)) {
				bis_bitfield |= (uint32_t)1U << (bis_index - 1U);
			}
		}

		if (base_is_grptlk_vendor_codec(codec_id, cid, vid)) {
			codec_note = " (GRPTLK vendor codec)";
		} else if (codec_id == BT_HCI_CODING_FORMAT_VS) {
			codec_note = " (vendor specific)";
		}

		printk("BASE subgroup %u: codec_id=0x%02x%s, cid=0x%04x, vid=0x%04x, "
		       "codec_cfg_len=%u, meta_len=%u, num_bis=%u, bis_bitfield=0x%08x\n",
		       subgroup_idx + 1U, codec_id, codec_note, cid, vid, codec_cfg_len, meta_len,
		       bis_count, bis_bitfield);
	}

	return (buf.len == 0U) ? 0 : -EINVAL;
}

static bool pa_decode_base(struct bt_data *data, void *user_data)
{
	int err;

	ARG_UNUSED(user_data);

	err = base_log_service_data(data);
	if (err == -ENOENT) {
		return true;
	}

	if (err != 0) {
		if (atomic_cas(&base_parse_warned, 0, 1)) {
			printk("BASE parse failed: %d\n", err);
		}
		return false;
	}

	atomic_set(&base_logged, 1);

	return false;
}

static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
{
	ARG_UNUSED(buf);
	if (!per_adv_found && info->interval) {
		per_adv_found = true;
		per_sid = info->sid;
		per_interval_us = BT_CONN_INTERVAL_TO_US(info->interval);
		bt_addr_le_copy(&per_addr, info->addr);
		k_sem_give(&sem_per_adv);
	}
}

static struct bt_le_scan_cb scan_callbacks = {.recv = scan_recv};

static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
	ARG_UNUSED(sync);
	ARG_UNUSED(info);
	k_sem_give(&sem_per_sync);
}

static void pa_recv_cb(struct bt_le_per_adv_sync *sync,
		       const struct bt_le_per_adv_sync_recv_info *info, struct net_buf_simple *buf)
{
	struct net_buf_simple ad_copy = *buf;

	ARG_UNUSED(sync);
	ARG_UNUSED(info);

	if (atomic_get(&base_logged) != 0) {
		return;
	}

	bt_data_parse(&ad_copy, pa_decode_base, NULL);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	ARG_UNUSED(sync);
	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
	       bt_le_per_adv_sync_get_index(sync), le_addr);
	per_adv_lost = true;
	k_sem_give(&sem_per_sync_lost);
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync, const struct bt_iso_biginfo *biginfo)
{
	ARG_UNUSED(sync);
	big_actual_num_bis = MIN(biginfo->num_bis, (uint8_t)CONFIG_BT_ISO_MAX_CHAN);
	big_sdu_interval_us = biginfo->sdu_interval;
	big_max_sdu = biginfo->max_sdu;
	note_downlink_transport_overhead(big_max_sdu, "BIGInfo");
	if (grptlk_big == NULL) {
		printk("BIG: %u BIS(es), sdu_interval=%u us, max_sdu=%u B, syncing to %u\n",
		       biginfo->num_bis, big_sdu_interval_us, big_max_sdu, big_actual_num_bis);
	}
	k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.recv = pa_recv_cb,
	.term = term_cb,
	.biginfo = biginfo_cb,
};

static void reset_semaphores(void)
{
	k_sem_reset(&sem_per_adv);
	k_sem_reset(&sem_per_sync);
	k_sem_reset(&sem_per_sync_lost);
	k_sem_reset(&sem_per_big_info);
	k_sem_reset(&sem_big_sync);
	k_sem_reset(&sem_big_sync_lost);
	atomic_set(&base_logged, 0);
	atomic_set(&base_parse_warned, 0);
	atomic_set(&downlink_codec_sdu_bytes, GRPTLK_CODEC_SDU_BYTES);
	atomic_set(&downlink_appendix_bytes, 0);
	atomic_set(&downlink_uplink_bitmap, 0);
	atomic_set(&downlink_format_warned, 0);
	big_max_sdu = 0U;
}

static bool bis_channels_idle(void)
{
	for (uint8_t i = 0U; i < CONFIG_BT_ISO_MAX_CHAN; i++) {
		if (bis[i]->iso != NULL) {
			return false;
		}
	}
	return true;
}

static void wait_bis_channels_idle(k_timeout_t timeout)
{
	int64_t deadline_ms = k_uptime_get() + k_ticks_to_ms_floor64(timeout.ticks);

	while (!bis_channels_idle()) {
		if (k_uptime_get() >= deadline_ms) {
			break;
		}
		(void)k_sem_take(&sem_big_sync_lost, K_MSEC(100));
	}
}

static void bis_channels_init(void)
{
	for (uint8_t i = 0U; i < CONFIG_BT_ISO_MAX_CHAN; i++) {
		bis_iso_chan[i].ops = &iso_ops;
		bis_iso_chan[i].qos = &bis_iso_qos;
		bis[i] = &bis_iso_chan[i];
	}
}

static int setup_iso_datapaths(void)
{
	int err;

	atomic_set(&uplink_path_ready, 0);

	for (uint8_t i = 0U; i < big_sync_param.num_bis; i++) {
		struct bt_iso_chan *chan = bis[i];
		const struct bt_iso_chan_path hci_path = {
			.pid = BT_ISO_DATA_PATH_HCI,
			.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
		};
		uint8_t dir = (i == 0U) ? BT_HCI_DATAPATH_DIR_CTLR_TO_HOST
					: BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;

		printk("Setting data path chan %u...\n", i);
		err = bt_iso_setup_data_path(chan, dir, &hci_path);
		if (err == -EACCES) {
			printk("Data path chan %u busy, removing stale path and retrying...\n", i);
			(void)bt_iso_remove_data_path(chan, BT_HCI_DATAPATH_DIR_CTLR_TO_HOST);
			(void)bt_iso_remove_data_path(chan, BT_HCI_DATAPATH_DIR_HOST_TO_CTLR);
			err = bt_iso_setup_data_path(chan, dir, &hci_path);
		}
		if (err) {
			printk("Failed to setup ISO data path for chan %u: %d\n", i, err);
			return err;
		}
		printk("Data path set chan %u.\n", i);
	}
	clk_sync_reset();

	seq_num = 0U;
	atomic_set(&uplink_tx_active, 0);
	k_msgq_purge(&encoded_frame_q);
	k_msgq_purge(uplink_q); /* Purge uplink queue too */
	ring_prod_idx = 0;
	ring_cons_idx = 0;
	memset(ring_fifo, 0, sizeof(ring_fifo));
	audio_playback_reset();
	while (k_sem_take(&mic_frame_sem, K_NO_WAIT) == 0) {
	}

	iso_datapaths_setup = true;
	atomic_set(&uplink_path_ready, 1);
	return 0;
}

K_THREAD_STACK_DEFINE(decoder_stack, DECODER_STACK_SIZE);
static struct k_thread decoder_thread_data;

K_THREAD_STACK_DEFINE(encoder_stack, ENCODER_STACK_SIZE);
static struct k_thread encoder_thread_data;

K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);
static struct k_thread tx_thread_data;

static int lc3_workers_start(void)
{
#if defined(CONFIG_GRPTLK_LC3_CODEC_T2)
	/* T2 LC3 init */
	int err;

	err = sw_codec_lc3_init(NULL, NULL, preset_active.qos.interval);
	if (err) {
		printk("sw_codec_lc3_init failed: %d\n", err);
		return err;
	}

	err = sw_codec_lc3_enc_init(AUDIO_SAMPLE_RATE_HZ, 16, preset_active.qos.interval,
				    32000, 1, &t2_pcm_bytes_req);
	if (err) {
		printk("sw_codec_lc3_enc_init failed: %d\n", err);
		return err;
	}

	err = sw_codec_lc3_dec_init(AUDIO_SAMPLE_RATE_HZ, 16, preset_active.qos.interval, 1);
	if (err) {
		printk("sw_codec_lc3_dec_init failed: %d\n", err);
		return err;
	}
#else
	lc3_decoder = lc3_setup_decoder(preset_active.qos.interval, AUDIO_SAMPLE_RATE_HZ, 0,
					&lc3_decoder_mem);
	if (lc3_decoder == NULL) {
		printk("ERROR: Failed to setup LC3 decoder\n");
		return -EIO;
	}

	lc3_encoder = lc3_setup_encoder(preset_active.qos.interval, AUDIO_SAMPLE_RATE_HZ, 0,
					&lc3_encoder_mem);
	if (lc3_encoder == NULL) {
		printk("ERROR: Failed to setup LC3 encoder\n");
		return -EIO;
	}
#endif

	k_thread_create(&decoder_thread_data, decoder_stack, K_THREAD_STACK_SIZEOF(decoder_stack),
			decoder_thread_func, NULL, NULL, NULL, DECODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&decoder_thread_data, "lc3_decoder");

	k_thread_create(&encoder_thread_data, encoder_stack, K_THREAD_STACK_SIZEOF(encoder_stack),
			encoder_thread_func, NULL, NULL, NULL, ENCODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&encoder_thread_data, "lc3_encoder");

	k_thread_create(&tx_thread_data, tx_thread_stack, K_THREAD_STACK_SIZEOF(tx_thread_stack),
			tx_thread, NULL, NULL, NULL, TX_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "iso_tx");

	return 0;
}

int main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	uint32_t sem_timeout_us;
	int err;
	int led_err;

#if defined(CONFIG_GRPTLK_AUDIO_FRAME_5_MS)
	override_preset_for_lc3plus_5ms();
#endif

	printk("Starting GRPTLK Receiver\n");
#if defined(CONFIG_GRPTLK_LC3_CODEC_T2)
	printk("LC3 codec: T2 Software LC3\n");
#else
	printk("LC3 codec: Open-source liblc3\n");
#endif
#if defined(CONFIG_GRPTLK_UPLINK_RANDOM)
	printk("Config: downlink=BIS1, uplink=random from BIS2..BIS%u\n", CONFIG_BT_ISO_MAX_CHAN);
#else
	printk("Config: downlink=BIS1, uplink=static BIS%u\n", CONFIG_GRPTLK_UPLINK_BIS);
#endif

	led_err = led_init();
	if (led_err) {
		printk("led_init failed: %d\n", led_err);
	} else {
		(void)led_set_receiver_synced(false);
	}

	err = buttons_init(&tx_sem);
	if (err) {
		printk("buttons_init failed: %d\n", err);
		return err;
	}

	/* Debug GPIO (optional, for logic analyzer timing analysis) */
	(void)debug_gpio_init();

	k_timer_init(&bis1_activity_timer, bis1_activity_timeout_handler, NULL);
	k_work_init(&bis1_disconnect_work, bis1_disconnect_work_handler);

	bis_channels_init();

	err = audio_init(&playback_ring, audio_rx_mono_frame);
	if (err) {
		printk("audio_init failed: %d\n", err);
		return err;
	}

	err = audio_start();
	if (err) {
		printk("audio_start failed: %d\n", err);
		return err;
	}

	clk_sync_init();

	uplink_q = clk_sync_get_uplink_q();

	err = lc3_workers_start();
	if (err) {
		return err;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	bt_le_scan_cb_register(&scan_callbacks);
	bt_le_per_adv_sync_cb_register(&sync_callbacks);

	do {
		reset_semaphores();
		per_adv_lost = false;
		if (led_err == 0) {
			(void)led_set_receiver_synced(false);
		}

		printk("Start scanning...\n");
		err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
		if (err) {
			printk("bt_le_scan_start failed: %d\n", err);
			return err;
		}

		per_adv_found = false;
		printk("Waiting for periodic advertising...\n");
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err) {
			printk("sem_per_adv failed: %d\n", err);
			return err;
		}

		printk("Found periodic advertising.\n");
		err = bt_le_scan_stop();
		if (err) {
			printk("bt_le_scan_stop failed: %d\n", err);
			return err;
		}

		bt_addr_le_copy(&sync_create_param.addr, &per_addr);
		sync_create_param.options = 0;
		sync_create_param.sid = per_sid;
		sync_create_param.skip = 0;
		sync_create_param.timeout =
			(per_interval_us * PA_RETRY_COUNT) / (10 * USEC_PER_MSEC);
		sem_timeout_us = per_interval_us * PA_RETRY_COUNT;

		printk("Creating Periodic Advertising Sync...\n");
		err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
		if (err) {
			printk("bt_le_per_adv_sync_create failed: %d\n", err);
			return err;
		}

		printk("Waiting for periodic sync...\n");
		err = k_sem_take(&sem_per_sync, K_USEC(sem_timeout_us));
		if (err) {
			printk("Periodic sync timeout\n");
			(void)bt_le_per_adv_sync_delete(sync);
			continue;
		}

		printk("Waiting for BIG info...\n");
		err = k_sem_take(&sem_per_big_info, K_USEC(sem_timeout_us));
		if (err) {
			printk("BIG info timeout\n");
			if (!per_adv_lost) {
				(void)bt_le_per_adv_sync_delete(sync);
			}
			continue;
		}

		preset_active.qos.interval = big_sdu_interval_us;
		preset_active.qos.sdu = GRPTLK_CODEC_SDU_BYTES;

#if defined(CONFIG_GRPTLK_LC3_CODEC_T2)
		/* Re-init T2 LC3 for new parameters */
		err = sw_codec_lc3_init(NULL, NULL, preset_active.qos.interval);
		if (err) {
			printk("sw_codec_lc3_init re-init failed: %d\n", err);
			goto per_sync_lost_check;
		}

		err = sw_codec_lc3_enc_init(AUDIO_SAMPLE_RATE_HZ, 16, preset_active.qos.interval,
					    32000, 1, &t2_pcm_bytes_req);
		if (err) {
			printk("sw_codec_lc3_enc_init re-init failed: %d\n", err);
			goto per_sync_lost_check;
		}

		err = sw_codec_lc3_dec_init(AUDIO_SAMPLE_RATE_HZ, 16, preset_active.qos.interval, 1);
		if (err) {
			printk("sw_codec_lc3_dec_init re-init failed: %d\n", err);
			goto per_sync_lost_check;
		}
#else
		lc3_decoder = lc3_setup_decoder(preset_active.qos.interval, AUDIO_SAMPLE_RATE_HZ, 0,
						&lc3_decoder_mem);
		if (lc3_decoder == NULL) {
			printk("ERROR: lc3_setup_decoder failed\n");
			goto per_sync_lost_check;
		}
		lc3_encoder = lc3_setup_encoder(preset_active.qos.interval, AUDIO_SAMPLE_RATE_HZ, 0,
						&lc3_encoder_mem);
		if (lc3_encoder == NULL) {
			printk("ERROR: lc3_setup_encoder failed\n");
			goto per_sync_lost_check;
		}
#endif

big_sync_create:
		if (!bis_channels_idle()) {
			printk("Waiting for previous BIG channel cleanup...\n");
			wait_bis_channels_idle(K_SECONDS(2));
		}
		if (!bis_channels_idle()) {
			printk("Previous BIG channels still allocated, waiting for fresh "
			       "BIGInfo\n");
			goto per_sync_lost_check;
		}

		big_sync_param.num_bis = big_actual_num_bis;
		big_sync_param.bis_bitfield = BIT_MASK(big_actual_num_bis);
		printk("Create BIG Sync (num_bis=%u, bitfield=0x%x)...\n", big_sync_param.num_bis,
		       big_sync_param.bis_bitfield);

		atomic_set(&big_chan_connected, 0);
		err = bt_iso_big_sync(sync, &big_sync_param, &grptlk_big);
		if (err) {
			printk("bt_iso_big_sync failed: %d\n", err);
			if (err == -EALREADY) {
				wait_bis_channels_idle(K_SECONDS(2));
				goto per_sync_lost_check;
			}
			return err;
		}

		for (uint8_t i = 0U; i < big_actual_num_bis; i++) {
			printk("Waiting for BIG sync chan %u...\n", i);
			err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
			if (err) {
				break;
			}
			printk("BIG sync chan %u successful.\n", i);
		}

		if (err || (int)atomic_get(&big_chan_connected) < (int)big_actual_num_bis) {
			printk("BIG sync failed (err %d, connected %d/%u)\n", err,
			       (int)atomic_get(&big_chan_connected), big_actual_num_bis);
			err = bt_iso_big_terminate(grptlk_big);
			if ((err != 0) && (err != -EINVAL) && (err != -EIO)) {
				printk("bt_iso_big_terminate failed: %d\n", err);
				return err;
			}
			wait_bis_channels_idle(K_SECONDS(2));
			goto per_sync_lost_check;
		}

		printk("BIG sync established.\n");
		err = setup_iso_datapaths();
		if (err) {
			int term_err = bt_iso_big_terminate(grptlk_big);

			if ((term_err != 0) && (term_err != -EINVAL) && (term_err != -EIO)) {
				printk("bt_iso_big_terminate failed: %d\n", term_err);
				return term_err;
			}
			wait_bis_channels_idle(K_SECONDS(2));
			goto per_sync_lost_check;
		}

		if (led_err == 0) {
			(void)led_set_receiver_synced(true);
		}

		for (uint8_t i = 0U; i < big_actual_num_bis; i++) {
			printk("Waiting for BIG sync lost chan %u...\n", i);
			err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
			if (err) {
				printk("sem_big_sync_lost failed: %d\n", err);
				return err;
			}
			printk("BIG sync lost chan %u.\n", i);
		}

		printk("BIG sync lost.\n");
		if (led_err == 0) {
			(void)led_set_receiver_synced(false);
		}

per_sync_lost_check:
		printk("Checking for periodic sync lost...\n");
		err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
		if (err) {
			printk("Waiting for fresh BIGInfo...\n");
			k_sem_reset(&sem_per_big_info);
			err = k_sem_take(&sem_per_big_info, K_SECONDS(5));
			if (err == 0) {
				goto big_sync_create;
			}
			printk("No BIGInfo — waiting for PA sync to terminate...\n");
			(void)k_sem_take(&sem_per_sync_lost, K_FOREVER);
		}

		printk("Periodic sync lost.\n");
	} while (true);
}
