#include "audio_i2s.h"

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <nrfx_clock.h>
#include <nrfx_i2s.h>

LOG_MODULE_REGISTER(audio_i2s, CONFIG_AUDIO_I2S_LOG_LEVEL);

#define I2S_NL DT_NODELABEL(i2s0)

/*
 * HFCLKAUDIO FREQ_VALUE formula (nRF5340 PS §5.5.3.2.7):
 *   FREQ_VALUE = 2^16 × ((12 × f_out / 32 MHz) − 4),  f_out = 12.288 MHz
 *   → FREQ_VALUE = 0x9BA6
 */
#define HFCLKAUDIO_12_288_MHZ 0x9BA6

#define AUDIO_I2S_CFG_RATIO  NRF_I2S_RATIO_384X
#define AUDIO_I2S_CFG_MCK    0x66666000
#define AUDIO_I2S_CFG_BYPASS false

enum audio_i2s_state {
	AUDIO_I2S_STATE_UNINIT,
	AUDIO_I2S_STATE_IDLE,
	AUDIO_I2S_STATE_STARTED,
};

static enum audio_i2s_state state = AUDIO_I2S_STATE_UNINIT;

PINCTRL_DT_DEFINE(I2S_NL);

static nrfx_i2s_t i2s_inst = NRFX_I2S_INSTANCE(0);

static const nrfx_i2s_config_t i2s_cfg = {
	/* Pin configuration comes from board pinctrl DTS — skip nrfx defaults. */
	.skip_gpio_cfg = true,
	.skip_psel_cfg = true,
	.irq_priority = DT_IRQ(I2S_NL, priority),
	.mode = NRF_I2S_MODE_MASTER,
	.format = NRF_I2S_FORMAT_I2S,
	.alignment = NRF_I2S_ALIGN_LEFT,
	.ratio = AUDIO_I2S_CFG_RATIO,
	.mck_setup = AUDIO_I2S_CFG_MCK,
	.sample_width = NRF_I2S_SWIDTH_16BIT,
	.channels = NRF_I2S_CHANNELS_STEREO,
#if NRF_I2S_HAS_CLKCONFIG
	.clksrc = NRF_I2S_CLKSRC_ACLK,
	.enable_bypass = AUDIO_I2S_CFG_BYPASS,
#endif
};

static audio_i2s_blk_cb_t blk_cb;

static void i2s_comp_handler(nrfx_i2s_buffers_t const *released_bufs, uint32_t status)
{
	if (status != NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
		return;
	}

	if (released_bufs && blk_cb && (released_bufs->p_rx_buffer || released_bufs->p_tx_buffer)) {
		blk_cb(released_bufs->p_rx_buffer, released_bufs->p_tx_buffer);
	}
}

void audio_i2s_blk_cb_register(audio_i2s_blk_cb_t cb)
{
	blk_cb = cb;
}

int audio_i2s_init(void)
{
	nrfx_err_t nrfx_err;
	int err;

	if (state != AUDIO_I2S_STATE_UNINIT) {
		return 0;
	}

	/* Start HFCLKAUDIO (ACLK) at 12.288 MHz. */
	nrfx_clock_hfclkaudio_config_set(HFCLKAUDIO_12_288_MHZ);
	NRF_CLOCK->EVENTS_HFCLKAUDIOSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKAUDIOSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKAUDIOSTARTED == 0) {
		k_sleep(K_MSEC(1));
	}
	LOG_DBG("ACLK started (12.288 MHz)");

	err = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NL), PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("pinctrl apply failed: %d", err);
		return err;
	}

	IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_isr, nrfx_i2s_0_irq_handler, 0);
	irq_enable(DT_IRQN(I2S_NL));

	nrfx_err = nrfx_i2s_init(&i2s_inst, &i2s_cfg, i2s_comp_handler);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_i2s_init failed: 0x%08x", nrfx_err);
		return -EIO;
	}

	LOG_DBG("I2S initialized (master, ACLK, ratio=384x, 16-bit stereo)");
	state = AUDIO_I2S_STATE_IDLE;
	return 0;
}

int audio_i2s_start(uint32_t *rx_buf_initial, uint32_t *tx_buf_initial)
{
	nrfx_i2s_buffers_t bufs;
	nrfx_err_t nrfx_err;

	if (state != AUDIO_I2S_STATE_IDLE) {
		LOG_ERR("audio_i2s_start: not idle (state=%d)", state);
		return -EINVAL;
	}
	if (rx_buf_initial == NULL && tx_buf_initial == NULL) {
		LOG_ERR("audio_i2s_start: both RX and TX buffers are NULL");
		return -EINVAL;
	}

	bufs.p_rx_buffer = rx_buf_initial;
	bufs.p_tx_buffer = tx_buf_initial;
	bufs.buffer_size = AUDIO_I2S_SAMPLES_PER_BLOCK;

	nrfx_err = nrfx_i2s_start(&i2s_inst, &bufs, 0);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_i2s_start failed: 0x%08x", nrfx_err);
		return -EIO;
	}

	state = AUDIO_I2S_STATE_STARTED;
	return 0;
}

int audio_i2s_set_next_buf(uint32_t *rx_buf, uint32_t *tx_buf)
{
	nrfx_i2s_buffers_t bufs;
	nrfx_err_t nrfx_err;

	if (state != AUDIO_I2S_STATE_STARTED) {
		LOG_ERR("audio_i2s_set_next_buf: not started (state=%d)", state);
		return -EINVAL;
	}
	if (rx_buf == NULL && tx_buf == NULL) {
		LOG_ERR("audio_i2s_set_next_buf: both RX and TX buffers are NULL");
		return -EINVAL;
	}

	bufs.p_rx_buffer = rx_buf;
	bufs.p_tx_buffer = tx_buf;
	bufs.buffer_size = AUDIO_I2S_SAMPLES_PER_BLOCK;

	nrfx_err = nrfx_i2s_next_buffers_set(&i2s_inst, &bufs);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_i2s_next_buffers_set failed: 0x%08x", nrfx_err);
		return -EIO;
	}

	return 0;
}

void audio_i2s_stop(void)
{
	if (state == AUDIO_I2S_STATE_STARTED) {
		nrfx_i2s_stop(&i2s_inst);
		state = AUDIO_I2S_STATE_IDLE;
		LOG_DBG("I2S stopped");
	}
}
