#include "max9867.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
// #include <stdlib.h>

#define MAX9867_DAC_GAIN

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MAX9867, CONFIG_MAX9867_LOG_LEVEL);

static const struct device *i2c_dev = DEVICE_DT_GET(DT_ALIAS(i2c_codec));
static uint8_t dac_lvl_reg = 0x0F;

static int reg_read(uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte(i2c_dev, MAX9867_I2C_ADDR, reg, val);
}

static int reg_write(uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte(i2c_dev, MAX9867_I2C_ADDR, reg, val);
}

static void dump_reg(const char *tag, uint8_t reg, const char *name)
{
	uint8_t val;
	int err;

	err = reg_read(reg, &val);
	if (err) {
		LOG_ERR("%s %s (0x%02x) read failed: %d", tag, name, reg, err);
		return;
	}

	LOG_INF("%s %s (0x%02x) = 0x%02x", tag, name, reg, val);
}

int max9867_init()
{
	int err;

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("MAX9867 is not ready!");
		return -ENXIO;
	}

	uint8_t codec_id;
	err = reg_read(MAX9867_REG_REVISION, &codec_id);
	if (err || codec_id != 0x42) {
		LOG_ERR("Failed to read MAX9867 ID!");
		return -EIO;
	}
	LOG_DBG("MAX9867 found!");

	err = reg_write(MAX9867_REG_PWR_MGMT, 0x00);
	if (err) {
		LOG_ERR("Failed to clear power management register!");
		return -EFAULT;
	}
	LOG_DBG("Power management register cleared!");

#if CONFIG_GRPTLK_I2S_CODEC_MASTER
	/* MAX9867 as I2S master (HWN001): codec generates BCLK/LRCK, nRF is slave.
	 * MCLK 20-40 MHz range, PLL disabled, NI divider for 16 kHz LRCLK. */
	err = reg_write(MAX9867_REG_SYS_CLK, 0x20);
	if (err) {
		LOG_ERR("Failed to set system clock register!");
		return -EFAULT;
	}
	err = reg_write(MAX9867_REG_STEREO_CLK_HI, 0x20);
	if (err) {
		LOG_ERR("Failed to disable codec PLL!");
		return -EFAULT;
	}
	err = reg_write(MAX9867_REG_STEREO_CLK_LOW, 0x00);
	if (err) {
		LOG_ERR("Failed to set stereo clock low register!");
		return -EFAULT;
	}
	LOG_DBG("PLL disabled, NI divider for 16 kHz LRCLK (master mode)");

	err = reg_write(MAX9867_REG_IF_MODE_ONE, 0x90);
	if (err) {
		LOG_ERR("Failed to set audio interface register one (master mode)!");
		return -EFAULT;
	}
	err = reg_write(MAX9867_REG_IF_MODE_TWO, 0x01);
	if (err) {
		LOG_ERR("Failed to set audio interface register two (mono, 64x BCLK)!");
		return -EFAULT;
	}
	LOG_DBG("MAX9867 I2S master, BCLK 64x LRCLK");

	err = reg_write(MAX9867_REG_SIDETONE, 0x80);
	if (err) {
		LOG_ERR("Failed to disable sidetone!");
		return -EFAULT;
	}
#else
	/* MAX9867 as used on RBV2H hello_max9867: nRF5340 generates BCLK/LRCK,
	 * codec runs in slave mode with PLL enabled and the RBV2H-specific
	 * headset/mic analog routing. */
	err = reg_write(MAX9867_REG_SYS_CLK, 0x10);
	if (err) {
		LOG_ERR("Failed to set system clock register!");
		return -EFAULT;
	}
	err = reg_write(MAX9867_REG_STEREO_CLK_HI, 0x80);
	if (err) {
		LOG_ERR("Failed to set stereo clock high register!");
		return -EFAULT;
	}
	err = reg_write(MAX9867_REG_STEREO_CLK_LOW, 0x00);
	if (err) {
		LOG_ERR("Failed to set stereo clock low register!");
		return -EFAULT;
	}
	LOG_DBG("RBV2H slave clocks configured");

	err = reg_write(MAX9867_REG_IF_MODE_ONE, 0x10);
	if (err) {
		LOG_ERR("Failed to set audio interface register one (slave mode)!");
		return -EFAULT;
	}
	/* DMONO=1 mixes SDIN stereo to both DACs on rbv2h. */
	err = reg_write(MAX9867_REG_IF_MODE_TWO, 0x08);
	if (err) {
		LOG_ERR("Failed to set audio interface register two!");
		return -EFAULT;
	}
	LOG_DBG("MAX9867 I2S slave, RBV2H audio interface mode");

	/* DSTS=00 disables digital sidetone source selection. */
	err = reg_write(MAX9867_REG_SIDETONE, 0x00);
	if (err) {
		LOG_ERR("Failed to set sidetone register!");
		return -EFAULT;
	}
#endif /* CONFIG_GRPTLK_I2S_CODEC_MASTER */

	/* Keep the rbv2h codec on the known-good DAC gain setting from hello_max9867. */
	dac_lvl_reg = 0x0F;
	err = reg_write(MAX9867_REG_DAC_LVL, dac_lvl_reg);
	if (err) {
		LOG_ERR("Failed to set DAC gain!");
		return -EFAULT;
	}

	err = reg_write(MAX9867_REG_ADC_LVL, 0x30);
	if (err) {
		LOG_ERR("Failed to set ADC gain!");
		return -EFAULT;
	}
	LOG_DBG("Audio interface registers set!");

	err = reg_write(MAX9867_REG_L_MIC_GAIN, 0x26);
	if (err) {
		LOG_ERR("Failed to set Left Analog Mic register!");
		return -EFAULT;
	}
	LOG_DBG("Left Analog Mic register set!");

#if CONFIG_GRPTLK_I2S_CODEC_MASTER
	err = reg_write(MAX9867_REG_ADC_IN, 0x50);
#else
	err = reg_write(MAX9867_REG_ADC_IN, 0x40);
#endif
	if (err) {
		LOG_ERR("Failed to set ADC register!");
		return -EFAULT;
	}
	LOG_DBG("ADC register set!");

	err = reg_write(MAX9867_REG_MIC, 0x00);
	if (err) {
		LOG_ERR("Failed to disable digital mic!");
		return -EFAULT;
	}
	LOG_DBG("Digital mic disabled!");

#if CONFIG_GRPTLK_I2S_CODEC_MASTER
	err = reg_write(MAX9867_REG_MODE, 0x17);
#else
	/* RBV2H headset jack is wired for common-return playback, not stereo differential. */
	err = reg_write(MAX9867_REG_MODE, 0x00);
#endif
	if (err) {
		LOG_ERR("Failed to set mode configuration register!");
		return -EFAULT;
	}
	LOG_DBG("Mode configuration registers set!");

	err = reg_write(MAX9867_REG_L_VOL, 0x17);
	if (err) {
		LOG_ERR("Failed to set left volume register!");
		return -EFAULT;
	}
	LOG_DBG("Left volume register set!");

	err = reg_write(MAX9867_REG_R_VOL, 0x17);
	if (err) {
		LOG_ERR("Failed to set right volume register!");
		return -EFAULT;
	}

#if CONFIG_GRPTLK_I2S_CODEC_MASTER
	err = reg_write(MAX9867_REG_PWR_MGMT, 0x8F);
#else
	err = reg_write(MAX9867_REG_PWR_MGMT, 0x8E);
#endif
	if (err) {
		LOG_ERR("Failed to toggle power management register!");
		return -EFAULT;
	}
	LOG_DBG("Power management register set and toggled!");

	/* Apply the app's maximum startup volume using the fixed scaling logic. */
	err = max9867_set_volume(13668, 13668);
	if (err) {
		LOG_ERR("Failed to set startup volume!");
		return -EFAULT;
	}

	(void)max9867_dump_state("post-init");

	return 0;
}

int max9867_status()
{
	uint8_t status;
	int err = reg_read(MAX9867_REG_STATUS, &status);
	if (err) {
		LOG_ERR("Failed to read status register!");
		return -EIO;
	}
	LOG_DBG("Status: %x\n", status);
	return 0;
}

int max9867_dump_state(const char *tag)
{
	if (tag == NULL) {
		tag = "dump";
	}

	dump_reg(tag, MAX9867_REG_STATUS, "STATUS");
	dump_reg(tag, MAX9867_REG_JKSNS, "JKSNS");
	dump_reg(tag, MAX9867_REG_INOUT_EN, "INOUT_EN");
	dump_reg(tag, MAX9867_REG_SYS_CLK, "SYS_CLK");
	dump_reg(tag, MAX9867_REG_STEREO_CLK_HI, "STEREO_CLK_HI");
	dump_reg(tag, MAX9867_REG_STEREO_CLK_LOW, "STEREO_CLK_LOW");
	dump_reg(tag, MAX9867_REG_IF_MODE_ONE, "IF_MODE_ONE");
	dump_reg(tag, MAX9867_REG_IF_MODE_TWO, "IF_MODE_TWO");
	dump_reg(tag, MAX9867_REG_CODEC_FILTERS, "CODEC_FILTERS");
	dump_reg(tag, MAX9867_REG_SIDETONE, "SIDETONE");
	dump_reg(tag, MAX9867_REG_DAC_LVL, "DAC_LVL");
	dump_reg(tag, MAX9867_REG_ADC_LVL, "ADC_LVL");
	dump_reg(tag, MAX9867_REG_L_VOL, "L_VOL");
	dump_reg(tag, MAX9867_REG_R_VOL, "R_VOL");
	dump_reg(tag, MAX9867_REG_L_MIC_GAIN, "L_MIC_GAIN");
	dump_reg(tag, MAX9867_REG_R_MIC_GAIN, "R_MIC_GAIN");
	dump_reg(tag, MAX9867_REG_ADC_IN, "ADC_IN");
	dump_reg(tag, MAX9867_REG_MIC, "MIC");
	dump_reg(tag, MAX9867_REG_MODE, "MODE");
	dump_reg(tag, MAX9867_REG_PWR_MGMT, "PWR_MGMT");

	return 0;
}

int max9867_set_volume(int16_t vol, int16_t max_vol)
{
	int err;
	int32_t clamped_vol = vol;
	int32_t full_scale = max_vol;
	int32_t scaled_vol;
	uint8_t register_vol;

	if (full_scale <= 0) {
		full_scale = 0x7FFF;
	}

	if (clamped_vol < 0) {
		clamped_vol = 0;
	} else if (clamped_vol > full_scale) {
		clamped_vol = full_scale;
	}

	scaled_vol = (clamped_vol * 40 + (full_scale / 2)) / full_scale;
	register_vol = (uint8_t)(40 - scaled_vol);

	err = reg_write(MAX9867_REG_L_VOL, register_vol);
	if (err) {
		LOG_ERR("Failed to set left volume register!");
		return -EFAULT;
	}
	LOG_DBG("Left volume register set!");

	err = reg_write(MAX9867_REG_R_VOL, register_vol);
	if (err) {
		LOG_ERR("Failed to set right volume register!");
		return -EFAULT;
	}
	LOG_DBG("Right volume register set!");
	LOG_INF("set_volume vol=%d/%d reg=0x%02x", vol, max_vol, register_vol);
	return 0;
}

int max9867_set_mute(bool mute)
{
	int err;
	uint8_t register_mute = (mute ? BIT(6) : 0U) | (dac_lvl_reg & 0x3F);

	err = reg_write(MAX9867_REG_DAC_LVL, register_mute);
	if (err) {
		LOG_ERR("Failed to set left volume register!");
		return -EFAULT;
	}
	dac_lvl_reg = register_mute;
	LOG_DBG("Mute set to %d!", mute);
	LOG_INF("set_mute mute=%d dac_lvl=0x%02x", mute, register_mute);

	return 0;
}
