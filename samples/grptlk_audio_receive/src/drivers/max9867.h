#ifndef GRPTLK_MAX9867_H_
#define GRPTLK_MAX9867_H_

#include <stdbool.h>
#include <stdint.h>

/** MAX9867 I2C address (ADDR pin tied low). */
#define MAX9867_I2C_ADDR 0x18

/* Status / Jack-sense */
#define MAX9867_REG_STATUS   0x00
#define MAX9867_REG_JKSNS    0x01
#define MAX9867_REG_AUX_HI   0x02
#define MAX9867_REG_AUX_LOW  0x03
#define MAX9867_REG_INOUT_EN 0x04

/* Clock control */
#define MAX9867_REG_SYS_CLK	   0x05
#define MAX9867_REG_STEREO_CLK_HI  0x06
#define MAX9867_REG_STEREO_CLK_LOW 0x07

/* Digital audio interface */
#define MAX9867_REG_IF_MODE_ONE 0x08
#define MAX9867_REG_IF_MODE_TWO 0x09

/* Digital filtering */
#define MAX9867_REG_CODEC_FILTERS 0x0A

/* Level control */
#define MAX9867_REG_SIDETONE   0x0B
#define MAX9867_REG_DAC_LVL    0x0C
#define MAX9867_REG_ADC_LVL    0x0D
#define MAX9867_REG_L_LIN_LVL  0x0E
#define MAX9867_REG_R_LIN_LVL  0x0F
#define MAX9867_REG_L_VOL      0x10
#define MAX9867_REG_R_VOL      0x11
#define MAX9867_REG_L_MIC_GAIN 0x12
#define MAX9867_REG_R_MIC_GAIN 0x13
#define MAX9867_REG_ADC_IN     0x14
#define MAX9867_REG_MIC	       0x15
#define MAX9867_REG_MODE       0x16

/* Power management */
#define MAX9867_REG_PWR_MGMT 0x17

/* Read-only revision register — reads 0x42 on a valid device. */
#define MAX9867_REG_REVISION 0xFF
#define MAX9867_REVISION_VAL 0x42

/**
 * @brief Initialize and configure the MAX9867 for RBV2H operation.
 *
 * Must be called once after I2C is ready. Verifies device presence,
 * sets all required registers, and enables the codec.
 *
 * @return 0 on success, negative errno on failure.
 */
int max9867_init(void);

/**
 * @brief Read and log the STATUS register.
 *
 * @return 0 on success, negative errno on failure.
 */
int max9867_status(void);

/**
 * @brief Log the value of every user-accessible register.
 *
 * @param tag  Short label prepended to each log line (e.g. "post-init").
 *             If NULL, defaults to "dump".
 *
 * @return 0 on success, negative errno on failure.
 */
int max9867_dump_state(const char *tag);

/**
 * @brief Set the headphone output volume.
 *
 * Maps @p vol in the range [0, @p max_vol] linearly onto the codec's
 * 0 dB … -39 dB attenuation range (register values 0x00 … 0x28).
 *
 * @param vol      Current volume level.
 * @param max_vol  Value that represents 100 % (0 dB). If <= 0, 0x7FFF is used.
 *
 * @return 0 on success, negative errno on failure.
 */
int max9867_set_volume(int16_t vol, int16_t max_vol);

/**
 * @brief Mute or un-mute the DAC output.
 *
 * The DAC gain register value is preserved across mute/un-mute cycles.
 *
 * @param mute  true to mute, false to un-mute.
 *
 * @return 0 on success, negative errno on failure.
 */
int max9867_set_mute(bool mute);

#endif /* GRPTLK_MAX9867_H_ */
