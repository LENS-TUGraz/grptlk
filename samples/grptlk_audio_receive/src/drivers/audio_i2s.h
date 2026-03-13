#ifndef GRPTLK_AUDIO_I2S_H_
#define GRPTLK_AUDIO_I2S_H_

#include <stdint.h>

/** Audio stream parameters — must match the LC3 codec configuration. */
#define AUDIO_I2S_SAMPLE_RATE_HZ    16000U
#define AUDIO_I2S_FRAME_DURATION_MS 5U
#define AUDIO_I2S_CHANNELS	    2U
#define AUDIO_I2S_BITS_PER_SAMPLE   16U

/** Number of PCM samples (per channel) in one 5 ms I2S block. */
#define AUDIO_I2S_SAMPLES_PER_BLOCK                                                                \
	((AUDIO_I2S_SAMPLE_RATE_HZ * AUDIO_I2S_FRAME_DURATION_MS) / 1000U)

/** Size of one I2S DMA block in bytes (both channels, 16-bit samples). */
#define AUDIO_I2S_BLOCK_BYTES                                                                      \
	(AUDIO_I2S_SAMPLES_PER_BLOCK * AUDIO_I2S_CHANNELS * (AUDIO_I2S_BITS_PER_SAMPLE / 8U))

/** Number of uint32_t words in one I2S DMA block.
 *  For 16-bit stereo each word holds one L+R sample pair. */
#define AUDIO_I2S_WORDS_PER_BLOCK (AUDIO_I2S_BLOCK_BYTES / sizeof(uint32_t))

/**
 * @brief I2S block completion callback.
 *
 * Called from the nrfx I2S IRQ handler when a DMA block transfer completes.
 * The caller must call audio_i2s_set_next_buf() before returning to keep
 * the stream running without underrun.
 *
 * @param rx_buf_released  RX buffer that was just filled (NULL if RX not used).
 * @param tx_buf_released  TX buffer that was just consumed (NULL if TX not used).
 */
typedef void (*audio_i2s_blk_cb_t)(uint32_t *rx_buf_released, const uint32_t *tx_buf_released);

/**
 * @brief Register the block completion callback.
 *
 * Must be called before audio_i2s_start().
 *
 * @param cb  Callback function pointer.
 */
void audio_i2s_blk_cb_register(audio_i2s_blk_cb_t cb);

/**
 * @brief Initialize the I2S peripheral and start the ACLK.
 *
 * Applies pinctrl, connects the IRQ, and initializes the nrfx I2S driver.
 * Safe to call multiple times — subsequent calls are no-ops.
 *
 * @return 0 on success, negative errno on failure.
 */
int audio_i2s_init(void);

/**
 * @brief Start streaming with the provided initial DMA buffers.
 *
 * At least one of @p rx_buf_initial or @p tx_buf_initial must be non-NULL.
 * Each buffer must be AUDIO_I2S_SAMPLES_PER_BLOCK words (uint32_t) in size.
 *
 * @param rx_buf_initial  First RX buffer (or NULL to disable RX).
 * @param tx_buf_initial  First TX buffer (or NULL to disable TX).
 *
 * @return 0 on success, negative errno on failure.
 */
int audio_i2s_start(uint32_t *rx_buf_initial, uint32_t *tx_buf_initial);

/**
 * @brief Queue the next DMA buffers while streaming.
 *
 * Must be called from within the audio_i2s_blk_cb_t callback to avoid
 * underrun. At least one of @p rx_buf or @p tx_buf must be non-NULL.
 *
 * @param rx_buf  Next RX buffer (or NULL).
 * @param tx_buf  Next TX buffer (or NULL).
 *
 * @return 0 on success, negative errno on failure.
 */
int audio_i2s_set_next_buf(uint32_t *rx_buf, uint32_t *tx_buf);

/**
 * @brief Stop streaming and put the I2S peripheral back to idle.
 *
 * Safe to call when already stopped.
 */
void audio_i2s_stop(void);

#endif /* GRPTLK_AUDIO_I2S_H_ */
