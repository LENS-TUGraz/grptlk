#ifndef GRPTLK_MAX9867_REGS_H_
#define GRPTLK_MAX9867_REGS_H_

/* Register addresses */
#define MAX9867_REG_STATUS	   0x00
#define MAX9867_REG_SYS_CLK	   0x05
#define MAX9867_REG_STEREO_CLK_HI  0x06
#define MAX9867_REG_STEREO_CLK_LOW 0x07
#define MAX9867_REG_IF_MODE_ONE	   0x08
#define MAX9867_REG_IF_MODE_TWO	   0x09
#define MAX9867_REG_CODEC_FILTERS  0x0A
#define MAX9867_REG_SIDETONE	   0x0B
#define MAX9867_REG_DAC_LVL	   0x0C
#define MAX9867_REG_ADC_LVL	   0x0D
#define MAX9867_REG_L_VOL	   0x10
#define MAX9867_REG_R_VOL	   0x11
#define MAX9867_REG_L_MIC_GAIN	   0x12
#define MAX9867_REG_R_MIC_GAIN	   0x13
#define MAX9867_REG_ADC_INPUT	   0x14
#define MAX9867_REG_MICROPHONE	   0x15
#define MAX9867_REG_MODE	   0x16
#define MAX9867_REG_PWR_MGMT	   0x17
#define MAX9867_REG_REVISION	   0xFF

/* Fixed values for this sample */
#define MAX9867_REVISION_ID			       0x42 /* expected chip revision */
#define MAX9867_PWR_MGMT_SHUTDOWN		       0x00 /* all blocks off */
#define MAX9867_SYS_CLK_PSCLK_10_TO_20_MHZ	       0x10 /* PSCLK = 10-20 MHz */
#define MAX9867_STEREO_CLK_PLL_EN		       0x80 /* enable PLL */
#define MAX9867_STEREO_CLK_LOW_DEFAULT		       0x00 /* default low byte */
#define MAX9867_IF_MODE_I2S			       0x10 /* I2S bus format */
#define MAX9867_IF_MODE_SLAVE_BSEL64		       0x10 /* slave, 64 BCLKs */
#define MAX9867_CODEC_FILTERS_DEFAULT		       0x00 /* default filters */
#define MAX9867_SIDETONE_DISABLED		       0x00 /* sidetone off */
#define MAX9867_DAC_LVL				       0x30 /* DAC level +18 dB */
#define MAX9867_ADC_LVL_0_DB_BOTH		       0x33 /* left 0 dB, right 0 dB */
#define MAX9867_L_MIC_GAIN			       0x34 /* preamp 0 dB, PGA 0 dB */
#define MAX9867_R_MIC_GAIN_MICRP		       0x2E /* preamp 0 dB, PGA +6 dB */
#define MAX9867_DIGMIC_DISABLED			       0x00 /* use analog mic path */
#define MAX9867_ADC_INPUT_RIGHT_MIC		       0x10 /* right ADC = mic */
#define MAX9867_MODE_DEFAULT			       0x02 /* current stream mode */
#define MAX9867_PWR_MGMT_PLAYBACK_CAPTURE_ANALOG_RIGHT 0x8F /* enable DAC/ADC L+R */

#define MAX9867_VOL_MIN_REG	0x00
#define MAX9867_VOL_MAX_REG	0x27
#define MAX9867_VOL_DEFAULT_REG 0x15 /* default playback volume = -18 dB */

#endif /* GRPTLK_MAX9867_REGS_H_ */
