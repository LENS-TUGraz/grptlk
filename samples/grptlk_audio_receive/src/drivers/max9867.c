#include "max9867.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(MAX9867, CONFIG_MAX9867_LOG_LEVEL);

/*
 * The i2c-codec alias is defined in the board overlay and resolves to the
 * I2C controller the MAX9867 is wired to (i2c1 on RBV2H).
 */
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_ALIAS(i2c_codec));

/*
 * Shadow of the DAC_LVL register. Kept in sync on every write so that
 * max9867_set_mute() can toggle the mute bit without clobbering the gain.
 */
static uint8_t dac_lvl_shadow = 0x0F;

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
		LOG_ERR("[%s] %s (0x%02x) read failed: %d", tag, name, reg, err);
		return;
	}

	LOG_INF("[%s] %s (0x%02x) = 0x%02x", tag, name, reg, val);
}

int max9867_init(void)
{
	int err;
	uint8_t id;

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENXIO;
	}

	err = reg_read(MAX9867_REG_REVISION, &id);
	if (err || id != MAX9867_REVISION_VAL) {
		LOG_ERR("MAX9867 not found (id=0x%02x, err=%d)", id, err);
		return -EIO;
	}
	LOG_DBG("MAX9867 found (rev=0x%02x)", id);

	/* Power down before reconfiguring. */
	err = reg_write(MAX9867_REG_PWR_MGMT, 0x00);
	if (err) {
		LOG_ERR("Failed to power down");
		return err;
	}

	/*
	 * Clock: PSCLK=01 (MCLK 10–20 MHz range), PLL enabled.
	 * nRF5340 generates BCLK/LRCK; MAX9867 is I2S slave.
	 */
	err = reg_write(MAX9867_REG_SYS_CLK, 0x10);
	if (err) {
		LOG_ERR("Failed to set SYS_CLK");
		return err;
	}

	/* NI[14:8]=0x80 enables PLL; rapid-lock disabled. */
	err = reg_write(MAX9867_REG_STEREO_CLK_HI, 0x80);
	if (err) {
		LOG_ERR("Failed to set STEREO_CLK_HI");
		return err;
	}
	err = reg_write(MAX9867_REG_STEREO_CLK_LOW, 0x00);
	if (err) {
		LOG_ERR("Failed to set STEREO_CLK_LOW");
		return err;
	}
	LOG_DBG("PLL enabled");

	/*
	 * Interface: I2S slave, 16-bit, BCLK=32×LRCLK.
	 * IF_MODE_ONE 0x10 = WCI format, 16-bit word length.
	 * IF_MODE_TWO 0x00 = stereo, normal BCLK.
	 */
	err = reg_write(MAX9867_REG_IF_MODE_ONE, 0x10);
	if (err) {
		LOG_ERR("Failed to set IF_MODE_ONE");
		return err;
	}
	err = reg_write(MAX9867_REG_IF_MODE_TWO, 0x00);
	if (err) {
		LOG_ERR("Failed to set IF_MODE_TWO");
		return err;
	}
	LOG_DBG("Audio interface configured (I2S slave, 16-bit)");

	/* Sidetone: disabled (0x00). */
	err = reg_write(MAX9867_REG_SIDETONE, 0x00);
	if (err) {
		LOG_ERR("Failed to set SIDETONE");
		return err;
	}

	/* DAC level: 0 dB attenuation (0x00) — maximum DAC output. */
	dac_lvl_shadow = 0x00;
	err = reg_write(MAX9867_REG_DAC_LVL, dac_lvl_shadow);
	if (err) {
		LOG_ERR("Failed to set DAC_LVL");
		return err;
	}

	/* ADC level: +12 dB gain on both channels (0x30). */
	err = reg_write(MAX9867_REG_ADC_LVL, 0x30);
	if (err) {
		LOG_ERR("Failed to set ADC_LVL");
		return err;
	}

	/* Left mic: PA gain = +20 dB, MIC_IN_DIFF routed to ADC (0x26). */
	err = reg_write(MAX9867_REG_L_MIC_GAIN, 0x26);
	if (err) {
		LOG_ERR("Failed to set L_MIC_GAIN");
		return err;
	}

	/* ADC input: left=MIC, right=MIC (0x40). */
	err = reg_write(MAX9867_REG_ADC_IN, 0x40);
	if (err) {
		LOG_ERR("Failed to set ADC_IN");
		return err;
	}

	/* MIC: analog mic selected (digital mic disabled, 0x00). */
	err = reg_write(MAX9867_REG_MIC, 0x00);
	if (err) {
		LOG_ERR("Failed to set MIC");
		return err;
	}

	/*
	 * Mode: 0x00 = stereo differential output.
	 * RBV2H headset jack is wired for common-return (non-differential),
	 * so this setting works correctly with the board's output stage.
	 */
	err = reg_write(MAX9867_REG_MODE, 0x00);
	if (err) {
		LOG_ERR("Failed to set MODE");
		return err;
	}

	/* Headphone volume: register 0x00 = 0 dB (maximum). */
	err = reg_write(MAX9867_REG_L_VOL, 0x00);
	if (err) {
		LOG_ERR("Failed to set L_VOL");
		return err;
	}
	err = reg_write(MAX9867_REG_R_VOL, 0x00);
	if (err) {
		LOG_ERR("Failed to set R_VOL");
		return err;
	}

	/*
	 * Power up: SHDN=1, LNLEN=1, LNREN=1, DALEN=1 (0x8E).
	 * ADC power (ADLEN/ADREN) left off — enabled on-demand via ADC_IN.
	 */
	err = reg_write(MAX9867_REG_PWR_MGMT, 0x8E);
	if (err) {
		LOG_ERR("Failed to set PWR_MGMT");
		return err;
	}
	LOG_DBG("MAX9867 powered up");

	/* Set the startup volume to maximum (0 dB relative to full scale). */
	err = max9867_set_volume(32767, 32767);
	if (err) {
		LOG_ERR("Failed to set startup volume");
		return err;
	}

	(void)max9867_dump_state("post-init");

	return 0;
}

int max9867_status(void)
{
	uint8_t status;
	int err;

	err = reg_read(MAX9867_REG_STATUS, &status);
	if (err) {
		LOG_ERR("Failed to read STATUS");
		return err;
	}

	LOG_DBG("STATUS = 0x%02x", status);
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
	uint8_t reg_val;

	if (full_scale <= 0) {
		full_scale = 0x7FFF;
	}

	if (clamped_vol < 0) {
		clamped_vol = 0;
	} else if (clamped_vol > full_scale) {
		clamped_vol = full_scale;
	}

	/*
	 * Map [0, full_scale] linearly onto register range [40, 0]:
	 *   reg 0x00 = 0 dB, reg 0x28 (40) = -39 dB attenuation.
	 * Round to nearest by adding half the denominator before dividing.
	 */
	scaled_vol = (clamped_vol * 40 + (full_scale / 2)) / full_scale;
	reg_val = (uint8_t)(40 - scaled_vol);

	err = reg_write(MAX9867_REG_L_VOL, reg_val);
	if (err) {
		LOG_ERR("Failed to set L_VOL");
		return err;
	}

	err = reg_write(MAX9867_REG_R_VOL, reg_val);
	if (err) {
		LOG_ERR("Failed to set R_VOL");
		return err;
	}

	LOG_INF("set_volume: vol=%d/%d reg=0x%02x", vol, max_vol, reg_val);
	return 0;
}

int max9867_set_mute(bool mute)
{
	int err;
	uint8_t reg_val;

	/* Set or clear the MUTE bit (bit 6) while preserving the gain bits. */
	reg_val = (mute ? BIT(6) : 0U) | (dac_lvl_shadow & 0x3F);

	err = reg_write(MAX9867_REG_DAC_LVL, reg_val);
	if (err) {
		LOG_ERR("Failed to set DAC_LVL (mute=%d)", mute);
		return err;
	}

	dac_lvl_shadow = reg_val;
	LOG_INF("set_mute: mute=%d dac_lvl=0x%02x", mute, reg_val);
	return 0;
}
