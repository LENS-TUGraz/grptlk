#ifndef _MAX9867_H_
#define _MAX9867_H_

#include <stdbool.h>
#include <stdint.h>

#define MAX9867_I2C_ADDR 0x18

/* Status Register */
#define MAX9867_REG_STATUS 0x00
#define MAX9867_REG_JKSNS 0x01
#define MAX9867_REG_AUX_HI 0x02
#define MAX9867_REG_AUX_LOW 0x03
#define MAX9867_REG_INOUT_EN 0x04

/* Clock Control */
#define MAX9867_REG_SYS_CLK 0x05
#define MAX9867_REG_STEREO_CLK_HI 0x06
#define MAX9867_REG_STEREO_CLK_LOW 0x07

/* Digital Audio Interface */
#define MAX9867_REG_IF_MODE_ONE 0x08
#define MAX9867_REG_IF_MODE_TWO 0x09

/* Digital Filtering */
#define MAX9867_REG_CODEC_FILTERS 0x0A

/* Level Control */
#define MAX9867_REG_SIDETONE 0x0B
#define MAX9867_REG_DAC_LVL 0x0C
#define MAX9867_REG_ADC_LVL 0x0D
#define MAX9867_REG_L_LIN_LVL 0x0E
#define MAX9867_REG_R_LIN_LVL 0x0F
#define MAX9867_REG_L_VOL 0x10
#define MAX9867_REG_R_VOL 0x11
#define MAX9867_REG_L_MIC_GAIN 0x12
#define MAX9867_REG_R_MIC_GAIN 0x13
#define MAX9867_REG_ADC_IN 0x14
#define MAX9867_REG_MIC 0x15
#define MAX9867_REG_MODE 0x16

/* Power Management */
#define MAX9867_REG_PWR_MGMT 0x17
#define MAX9867_REG_REVISION 0xFF

int max9867_init();
int max9867_status();
int max9867_set_volume(int16_t vol, int16_t max_vol);
int max9867_set_mute(bool mute);

#endif /* _MAX9867_H_ */