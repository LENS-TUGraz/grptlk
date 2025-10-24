#include "max9867.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
// #include <stdlib.h>

#define MAX9867_DAC_GAIN

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MAX9867, CONFIG_MAX9867_LOG_LEVEL);

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c20));

static int reg_read(uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte(i2c_dev, MAX9867_I2C_ADDR, reg, val);
}

static int reg_write(uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte(i2c_dev, MAX9867_I2C_ADDR, reg, val);
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

	err = reg_write(MAX9867_REG_SYS_CLK, 0x20);
	if (err) {
		LOG_ERR("Failed to diable the master clock and set the sample rate!");
		return -EFAULT;
	}
	LOG_DBG("Master clock set between 20 and 40 MHz and Normal Mode!");

	err = reg_write(MAX9867_REG_STEREO_CLK_HI, 0x20);
	if (err) {
		LOG_ERR("Failed to disable the codec PLL!");
		return -EFAULT;
	}
	err = reg_write(MAX9867_REG_STEREO_CLK_LOW, 0x00);
	if (err) {
		LOG_ERR("Failed to disable the rapid lock mode!");
		return -EFAULT;
	}
	LOG_DBG("Disabled the codec PLL and set NI divider bits to a 16kHz LRCLK!");

	err = reg_write(MAX9867_REG_IF_MODE_ONE, 0x90);
	if (err) {
		LOG_ERR("Failed to set audio interface register one!");
		return -EFAULT;
	}
	err = reg_write(MAX9867_REG_IF_MODE_TWO, 0x01);
	if (err) {
		LOG_ERR("Failed to set audio interface register two (MONO OUT ONLY)!");
		return -EFAULT;
	}
	LOG_DBG("Set master mode and BCLK to 64x LRCLK");

	err = reg_write(MAX9867_REG_SIDETONE, 0x46);
	if (err) {
		LOG_ERR("Failed to disable sidetone!");
		return -EFAULT;
	}
	// set internal DAC gain to -15dB -> audio is too loud without this
	err = reg_write(MAX9867_REG_DAC_LVL, 0x0F);
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

	err = reg_write(MAX9867_REG_ADC_IN, 0x50);
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

	err = reg_write(MAX9867_REG_MODE, 0x4C);
	if (err) {
		LOG_ERR("Failed to set mode configuration register!");
		return -EFAULT;
	}
	LOG_DBG("Mode configuration registers set to capacitorless!");

	err = reg_write(MAX9867_REG_L_VOL, 0x11);
	if (err) {
		LOG_ERR("Failed to set left volume register!");
		return -EFAULT;
	}
	LOG_DBG("Left volume register set!");

	err = reg_write(MAX9867_REG_R_VOL, 0x11);
	if (err) {
		LOG_ERR("Failed to set right volume register!");
		return -EFAULT;
	}

	err = reg_write(MAX9867_REG_PWR_MGMT, 0x8F);
	if (err) {
		LOG_ERR("Failed to toggle power management register!");
		return -EFAULT;
	}
	LOG_DBG("Power management register set and toggled!");

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

int max9867_set_volume(int16_t vol, int16_t max_vol)
{
	int err;
	uint8_t scaled_vol = (vol * 41) / 0x7FFF;
	uint8_t register_vol = 40 - scaled_vol;

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
	return 0;
}

int max9867_set_mute(bool mute)
{
	int err;
	uint8_t register_mute = (mute << 6);

	err = reg_write(MAX9867_REG_DAC_LVL, register_mute);
	if (err) {
		LOG_ERR("Failed to set left volume register!");
		return -EFAULT;
	}
	LOG_DBG("Mute set to %d!", mute);

	return 0;
}