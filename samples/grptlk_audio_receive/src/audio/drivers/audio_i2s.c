/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "audio_i2s.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <nrfx_clock.h>
#include <nrfx_i2s.h>

#define I2S_NL DT_NODELABEL(i2s0)

/*
 * Calculation from nRF5340 Audio reference:
 * FREQ_VALUE = 2^16 * ((12 * f_out / 32M) - 4), f_out = 12.288 MHz
 */
#define HFCLKAUDIO_12_288_MHZ 0x9BA6

enum audio_i2s_state {
	AUDIO_I2S_STATE_UNINIT,
	AUDIO_I2S_STATE_IDLE,
	AUDIO_I2S_STATE_STARTED,
};

static enum audio_i2s_state state = AUDIO_I2S_STATE_UNINIT;

PINCTRL_DT_DEFINE(I2S_NL);

static nrfx_i2s_t i2s_inst = NRFX_I2S_INSTANCE(0);

static nrfx_i2s_config_t cfg = {
	/* Pins come from board pinctrl DTS. */
	.skip_gpio_cfg = true,
	.skip_psel_cfg = true,
	.irq_priority = DT_IRQ(I2S_NL, priority),
	.mode = NRF_I2S_MODE_MASTER,
	.format = NRF_I2S_FORMAT_I2S,
	.alignment = NRF_I2S_ALIGN_LEFT,
	.ratio = NRF_I2S_RATIO_384X,
	.mck_setup = 0x66666000,
	.sample_width = NRF_I2S_SWIDTH_16BIT,
	.channels = NRF_I2S_CHANNELS_STEREO,
#if NRF_I2S_HAS_CLKCONFIG
	.clksrc = NRF_I2S_CLKSRC_ACLK,
	.enable_bypass = false,
#endif
};

static audio_i2s_blk_cb_t blk_cb;
static uint32_t s_words_per_block = AUDIO_I2S_WORDS_PER_BLOCK_DEFAULT;

static void i2s_comp_handler(nrfx_i2s_buffers_t const *released_bufs, uint32_t status)
{
	if ((status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) && released_bufs && blk_cb &&
	    (released_bufs->p_rx_buffer || released_bufs->p_tx_buffer)) {
		blk_cb(released_bufs->p_rx_buffer, released_bufs->p_tx_buffer);
	}
}

void audio_i2s_blk_cb_register(audio_i2s_blk_cb_t cb)
{
	blk_cb = cb;
}

void audio_i2s_set_block_size(uint32_t words_per_block)
{
	s_words_per_block = words_per_block;
}

int audio_i2s_init(void)
{
	nrfx_err_t ret;
	int err;

	if (state != AUDIO_I2S_STATE_UNINIT) {
		return 0;
	}

	nrfx_clock_hfclkaudio_config_set(HFCLKAUDIO_12_288_MHZ);
	NRF_CLOCK->EVENTS_HFCLKAUDIOSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKAUDIOSTART = 1;

	/* Wait for ACLK to start */
	while (NRF_CLOCK->EVENTS_HFCLKAUDIOSTARTED == 0) {
		k_sleep(K_MSEC(1));
	}

	err = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NL), PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return -EIO;
	}

	IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_isr, nrfx_i2s_0_irq_handler, 0);
	irq_enable(DT_IRQN(I2S_NL));

	ret = nrfx_i2s_init(&i2s_inst, &cfg, i2s_comp_handler);
	if (ret != NRFX_SUCCESS) {
		return -EIO;
	}

	state = AUDIO_I2S_STATE_IDLE;
	return 0;
}

int audio_i2s_start(uint32_t *rx_buf_initial, uint32_t *tx_buf_initial)
{
	nrfx_i2s_buffers_t i2s_buf;
	nrfx_err_t ret;

	if (state != AUDIO_I2S_STATE_IDLE ||
	    (rx_buf_initial == NULL && tx_buf_initial == NULL)) {
		return -EINVAL;
	}

	i2s_buf.p_rx_buffer = rx_buf_initial;
	i2s_buf.p_tx_buffer = tx_buf_initial;
	i2s_buf.buffer_size = s_words_per_block;

	ret = nrfx_i2s_start(&i2s_inst, &i2s_buf, 0);
	if (ret != NRFX_SUCCESS) {
		return -EIO;
	}

	state = AUDIO_I2S_STATE_STARTED;
	return 0;
}

int audio_i2s_set_next_buf(uint32_t *rx_buf, uint32_t *tx_buf)
{
	nrfx_i2s_buffers_t i2s_buf;
	nrfx_err_t ret;

	if (state != AUDIO_I2S_STATE_STARTED || (rx_buf == NULL && tx_buf == NULL)) {
		return -EINVAL;
	}

	i2s_buf.p_rx_buffer = rx_buf;
	i2s_buf.p_tx_buffer = tx_buf;
	i2s_buf.buffer_size = s_words_per_block;

	ret = nrfx_i2s_next_buffers_set(&i2s_inst, &i2s_buf);
	if (ret != NRFX_SUCCESS) {
		return -EIO;
	}

	return 0;
}

void audio_i2s_stop(void)
{
	if (state == AUDIO_I2S_STATE_STARTED) {
		nrfx_i2s_stop(&i2s_inst);
		state = AUDIO_I2S_STATE_IDLE;
	}
}
