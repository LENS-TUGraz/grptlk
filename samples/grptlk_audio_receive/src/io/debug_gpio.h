#ifndef GRPTLK_DEBUG_GPIO_H_
#define GRPTLK_DEBUG_GPIO_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_GRPTLK_DEBUG_GPIO)
extern const struct gpio_dt_spec debug_iso_recv;
extern const struct gpio_dt_spec debug_lc3_dec;
extern const struct gpio_dt_spec debug_lc3_enc;
extern const struct gpio_dt_spec debug_iso_sent;
#endif

int debug_gpio_init(void);

static inline void debug_iso_recv_set(int val)
{
	IF_ENABLED(CONFIG_GRPTLK_DEBUG_GPIO, (gpio_pin_set_dt(&debug_iso_recv, val);))
}

static inline void debug_lc3_dec_set(int val)
{
	IF_ENABLED(CONFIG_GRPTLK_DEBUG_GPIO, (gpio_pin_set_dt(&debug_lc3_dec, val);))
}

static inline void debug_lc3_enc_set(int val)
{
	IF_ENABLED(CONFIG_GRPTLK_DEBUG_GPIO, (gpio_pin_set_dt(&debug_lc3_enc, val);))
}

static inline void debug_iso_sent_set(int val)
{
	IF_ENABLED(CONFIG_GRPTLK_DEBUG_GPIO, (gpio_pin_set_dt(&debug_iso_sent, val);))
}

#ifdef __cplusplus
}
#endif

#endif /* GRPTLK_DEBUG_GPIO_H_ */
