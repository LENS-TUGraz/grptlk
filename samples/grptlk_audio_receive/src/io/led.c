#include "led.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#if IS_ENABLED(CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP) &&                                    \
	DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(led2)) &&                                                 \
	DT_NODE_HAS_STATUS_OKAY(DT_GPIO_CTLR(DT_ALIAS(led2), gpios))
#define RECEIVE_LED_AVAILABLE 1
static const struct gpio_dt_spec receive_led = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#else
#define RECEIVE_LED_AVAILABLE 0
#endif

#if RECEIVE_LED_AVAILABLE
static bool led_ready;

static int led_set_synced_state(bool synced)
{
	return gpio_pin_set_dt(&receive_led, synced ? 1 : 0);
}
#endif

int led_init(void)
{
#if !RECEIVE_LED_AVAILABLE
	return 0;
#else
	int err;

	if (led_ready) {
		return 0;
	}

	if (!gpio_is_ready_dt(&receive_led)) {
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&receive_led, GPIO_OUTPUT_INACTIVE);
	if (err) {
		return err;
	}

	err = led_set_synced_state(false);
	if (err) {
		return err;
	}

	led_ready = true;
	return 0;
#endif
}

int led_set_receiver_synced(bool synced)
{
#if !RECEIVE_LED_AVAILABLE
	(void)synced;
	return 0;
#else
	int err;

	if (!led_ready) {
		err = led_init();
		if (err) {
			return err;
		}
	}

	return led_set_synced_state(synced);
#endif
}
