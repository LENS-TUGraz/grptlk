#include "audio/backend.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#include "max9867_regs.h"

#if !DT_HAS_COMPAT_STATUS_OKAY(maxim_max9867)
#error "No maxim,max9867 node available. Build with FILE_SUFFIX=max9867 (or the legacy le_audio_playground alias) when CONFIG_GRPTLK_AUDIO_CODEC_MAX9867=y."
#endif

#define MAX9867_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(maxim_max9867)

static const struct i2c_dt_spec codec_i2c = I2C_DT_SPEC_GET(MAX9867_NODE);
static uint8_t current_volume_reg = MAX9867_VOL_DEFAULT_REG;
static bool codec_stream_configured;

static const int16_t volume_half_db_table[MAX9867_VOL_MAX_REG + 1] = {
	12,  11,  10,	9,    8,    7,	  6,	4,    2,    0,	  -2,	-4,   -6,  -8,
	-10, -12, -16,	-20,  -24,  -28,  -32,	-36,  -40,  -44,  -52,	-60,  -68, -76,
	-84, -92, -100, -108, -116, -124, -132, -140, -148, -156, -164, -168,
};

static int max9867_reg_read(uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte_dt(&codec_i2c, reg, val);
}

static int max9867_reg_write(uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte_dt(&codec_i2c, reg, val);
}

static int max9867_playback_volume_write(uint8_t reg_val)
{
	int err;

	err = max9867_reg_write(MAX9867_REG_L_VOL, reg_val);
	if (err) {
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_R_VOL, reg_val);
	if (err) {
		return err;
	}

	current_volume_reg = reg_val;
	return 0;
}

static uint8_t max9867_volume_reg_find_closest(int16_t target_half_db)
{
	uint8_t best_reg = MAX9867_VOL_DEFAULT_REG;
	int32_t best_delta = INT32_MAX;

	for (uint8_t reg = MAX9867_VOL_MIN_REG; reg <= MAX9867_VOL_MAX_REG; reg++) {
		int32_t diff = volume_half_db_table[reg] - target_half_db;
		int32_t delta = (diff < 0) ? -diff : diff;

		if (delta < best_delta) {
			best_delta = delta;
			best_reg = reg;
		}
	}

	return best_reg;
}

static int max9867_revision_check(void)
{
	uint8_t rev;
	int err;

	err = max9867_reg_read(MAX9867_REG_REVISION, &rev);
	if (err) {
		printk("MAX9867 revision read failed: %d\n", err);
		return err;
	}

	if (rev != MAX9867_REVISION_ID) {
		printk("Unexpected MAX9867 revision: 0x%02x\n", rev);
		return -EIO;
	}

	return 0;
}

static int max9867_stream_path_configure(void)
{
	int err;
	uint8_t status = 0U;
	uint8_t adc_lvl_reg = MAX9867_ADC_LVL_0_DB_BOTH;

	/* Fixed MAX9867 capture path: analog electret mic on MICRP, presented on
	 * the right ADC/I2S channel. */
	err = max9867_reg_write(MAX9867_REG_PWR_MGMT, MAX9867_PWR_MGMT_SHUTDOWN);
	if (err) {
		printk("MAX9867 power-down write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_SYS_CLK, MAX9867_SYS_CLK_PSCLK_10_TO_20_MHZ);
	if (err) {
		printk("MAX9867 SYS_CLK write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_STEREO_CLK_HI, MAX9867_STEREO_CLK_PLL_EN);
	if (err) {
		printk("MAX9867 STEREO_CLK_HI write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_STEREO_CLK_LOW, MAX9867_STEREO_CLK_LOW_DEFAULT);
	if (err) {
		printk("MAX9867 STEREO_CLK_LOW write failed: %d\n", err);
		return err;
	}

	k_sleep(K_MSEC(100));
	(void)max9867_reg_read(MAX9867_REG_STATUS, &status);

	err = max9867_reg_write(MAX9867_REG_IF_MODE_ONE, MAX9867_IF_MODE_I2S);
	if (err) {
		printk("MAX9867 IF_MODE_ONE write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_IF_MODE_TWO, MAX9867_IF_MODE_SLAVE_BSEL64);
	if (err) {
		printk("MAX9867 IF_MODE_TWO write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_CODEC_FILTERS, MAX9867_CODEC_FILTERS_DEFAULT);
	if (err) {
		printk("MAX9867 CODEC_FILTERS write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_SIDETONE, MAX9867_SIDETONE_DISABLED);
	if (err) {
		printk("MAX9867 SIDETONE write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_DAC_LVL, MAX9867_DAC_LVL);
	if (err) {
		printk("MAX9867 DAC_LVL write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_ADC_LVL, adc_lvl_reg);
	if (err) {
		printk("MAX9867 ADC_LVL write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_L_MIC_GAIN, MAX9867_L_MIC_GAIN);
	if (err) {
		printk("MAX9867 L_MIC_GAIN write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_R_MIC_GAIN, MAX9867_R_MIC_GAIN_MICRP);
	if (err) {
		printk("MAX9867 R_MIC_GAIN write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_MICROPHONE, MAX9867_DIGMIC_DISABLED);
	if (err) {
		printk("MAX9867 MICROPHONE write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_ADC_INPUT, MAX9867_ADC_INPUT_RIGHT_MIC);
	if (err) {
		printk("MAX9867 ADC_INPUT write failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_MODE, MAX9867_MODE_DEFAULT);
	if (err) {
		printk("MAX9867 MODE write failed: %d\n", err);
		return err;
	}

	err = max9867_playback_volume_write(current_volume_reg);
	if (err) {
		printk("MAX9867 volume init failed: %d\n", err);
		return err;
	}

	err = max9867_reg_write(MAX9867_REG_PWR_MGMT,
				MAX9867_PWR_MGMT_PLAYBACK_CAPTURE_ANALOG_RIGHT);
	if (err) {
		printk("MAX9867 power-up write failed: %d\n", err);
		return err;
	}

	printk("MAX9867 mic path: analog-right (MICRP), adc_lvl=0x%02x, "
	       "l_mic_gain=0x%02x r_mic_gain=0x%02x, capture=right\n",
	       adc_lvl_reg, MAX9867_L_MIC_GAIN, MAX9867_R_MIC_GAIN_MICRP);

	if ((status & 0x02U) == 0U) {
		printk("MAX9867 PLL not locked before stream enable (status=0x%02x)\n", status);
	}

	return 0;
}

int audio_backend_init(struct audio_backend_config *config)
{
	int err;

	if (config == NULL) {
		return -EINVAL;
	}

	if (!i2c_is_ready_dt(&codec_i2c)) {
		printk("MAX9867 I2C bus not ready\n");
		return -ENODEV;
	}

	err = max9867_revision_check();
	if (err) {
		return err;
	}

	config->capture_enabled = true;
	config->capture_channel_hint = AUDIO_BACKEND_CAPTURE_CHANNEL_RIGHT;
	codec_stream_configured = false;
	printk("[audio] codec=max9867 source=micrp capture=right\n");

	return 0;
}

int audio_backend_prepare_stream_start(void)
{
	int err;

	if (codec_stream_configured) {
		return 0;
	}

	err = max9867_stream_path_configure();
	if (err == 0) {
		codec_stream_configured = true;
	}

	return err;
}

int audio_backend_volume_adjust(int8_t step_db)
{
	int16_t target_half_db;
	uint8_t new_reg;
	int err;

	target_half_db = volume_half_db_table[current_volume_reg] + ((int16_t)step_db * 2);
	new_reg = max9867_volume_reg_find_closest(target_half_db);

	err = max9867_playback_volume_write(new_reg);
	if (err) {
		return err;
	}

	printk("[VOL] %+d dB -> reg=0x%02x\n", step_db, new_reg);
	return 0;
}

int audio_backend_input_source_switch(bool use_line_in, int *capture_channel_hint)
{
	ARG_UNUSED(use_line_in);
	ARG_UNUSED(capture_channel_hint);

	return -ENOTSUP;
}
