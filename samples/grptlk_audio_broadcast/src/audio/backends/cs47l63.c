#include "audio/backend.h"

#include <errno.h>
#include <stdbool.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "cs47l63.h"
#include "audio/drivers/cs47l63_comm.h"
#include "cs47l63_reg_conf.h"

#if !DT_HAS_COMPAT_STATUS_OKAY(cirrus_cs47l63)
#error "No cirrus,cs47l63 node available. Use nrf5340_audio_dk/nrf5340/cpuapp."
#endif

static cs47l63_t codec_driver;

static int codec_reg_conf_write(const uint32_t config[][2], size_t num_of_regs)
{
	for (size_t i = 0; i < num_of_regs; i++) {
		const uint32_t reg = config[i][0];
		const uint32_t value = config[i][1];
		uint32_t ret;

		if (reg == SPI_BUSY_WAIT) {
			k_busy_wait(value);
			continue;
		}

		ret = cs47l63_write_reg(&codec_driver, reg, value);
		if (ret != CS47L63_STATUS_OK) {
			printk("CS47L63 write failed: reg=0x%08x val=0x%08x ret=%u\n", reg, value,
			       ret);
			return -EIO;
		}
	}

	return 0;
}

int audio_backend_init(struct audio_backend_config *config)
{
	int err;

	if (config == NULL) {
		return -EINVAL;
	}

	err = cs47l63_comm_init(&codec_driver);
	if (err) {
		printk("cs47l63_comm_init failed: %d\n", err);
		return err;
	}

	err = codec_reg_conf_write(soft_reset, ARRAY_SIZE(soft_reset));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(clock_configuration, ARRAY_SIZE(clock_configuration));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(GPIO_configuration, ARRAY_SIZE(GPIO_configuration));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(asp1_enable, ARRAY_SIZE(asp1_enable));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(pdm_mic_enable_configure, ARRAY_SIZE(pdm_mic_enable_configure));
	if (err) {
		return err;
	}

	err = codec_reg_conf_write(output_enable, ARRAY_SIZE(output_enable));
	if (err) {
		return err;
	}

	err = cs47l63_write_reg(&codec_driver, CS47L63_OUT1L_VOLUME_1,
				OUT_VOLUME_DEFAULT | VOLUME_UPDATE_BIT);
	if (err != CS47L63_STATUS_OK) {
		printk("CS47L63 output volume set failed: %d\n", err);
		return -EIO;
	}

#if IS_ENABLED(CONFIG_GRPTLK_AUDIO_SOURCE_LINE_IN)
	config->capture_channel_hint = AUDIO_BACKEND_CAPTURE_CHANNEL_LEFT;
#else
	config->capture_channel_hint = AUDIO_BACKEND_CAPTURE_CHANNEL_AUTO;
#endif
	config->capture_enabled = true;
	printk("[audio] codec=cs47l63 source=pdm\n");

	return 0;
}

int audio_backend_prepare_stream_start(void)
{
	int err;

	err = codec_reg_conf_write(FLL_toggle, ARRAY_SIZE(FLL_toggle));
	if (err) {
		printk("Codec FLL toggle failed: %d\n", err);
		return err;
	}

	return 0;
}

int audio_backend_volume_adjust(int8_t step_db)
{
	uint32_t vol;
	int ret;

	ret = cs47l63_read_reg(&codec_driver, CS47L63_OUT1L_VOLUME_1, &vol);
	if (ret != CS47L63_STATUS_OK) {
		return -EIO;
	}

	int32_t new_vol = (int32_t)(vol & 0xFFU) + (int32_t)(step_db * 2);

	if (new_vol < 0) {
		new_vol = 0;
	} else if (new_vol > MAX_VOLUME_REG_VAL) {
		new_vol = MAX_VOLUME_REG_VAL;
	}

	ret = cs47l63_write_reg(&codec_driver, CS47L63_OUT1L_VOLUME_1,
				(uint32_t)new_vol | VOLUME_UPDATE_BIT);
	if (ret != CS47L63_STATUS_OK) {
		return -EIO;
	}

	printk("[VOL] %+d dB -> reg=0x%02x (%d dB)\n", step_db, (uint8_t)new_vol,
	       (int)(new_vol / 2) - MAX_VOLUME_DB);
	return 0;
}

int audio_backend_input_source_switch(bool use_line_in, int *capture_channel_hint)
{
	int err;

	if (capture_channel_hint == NULL) {
		return -EINVAL;
	}

	if (use_line_in) {
		err = codec_reg_conf_write(line_in_enable, ARRAY_SIZE(line_in_enable));
		if (err) {
			return err;
		}
		*capture_channel_hint = AUDIO_BACKEND_CAPTURE_CHANNEL_LEFT;
		printk("[audio] codec=cs47l63 source=line-in\n");
	} else {
		err = codec_reg_conf_write(pdm_mic_enable_configure,
					   ARRAY_SIZE(pdm_mic_enable_configure));
		if (err) {
			return err;
		}
		*capture_channel_hint = AUDIO_BACKEND_CAPTURE_CHANNEL_AUTO;
		printk("[audio] codec=cs47l63 source=pdm\n");
	}

	return 0;
}
