#include "led.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#if CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP && \
	DT_NODE_HAS_STATUS(DT_ALIAS(led2), okay) && \
	DT_NODE_HAS_STATUS(DT_GPIO_CTLR(DT_ALIAS(led2), gpios), okay)
#define BROADCAST_LED_AVAILABLE 1
static const struct gpio_dt_spec broadcast_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#elif CONFIG_BOARD_HWN001_NRF54L15_CPUAPP && \
	DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay) && \
	DT_NODE_HAS_STATUS(DT_GPIO_CTLR(DT_ALIAS(led0), gpios), okay)
#define BROADCAST_LED_AVAILABLE 1
static const struct gpio_dt_spec broadcast_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#define BROADCAST_LED_AVAILABLE 0
#endif

#if BROADCAST_LED_AVAILABLE
static bool led_ready;

static int led_set_running_state(bool running)
{
#if CONFIG_BOARD_HWN001_NRF54L15_CPUAPP
	/* HWN001 LED0 electrical level is inverted relative to observed behavior. */
	bool logical_state = running;

	if ((broadcast_led.dt_flags & GPIO_ACTIVE_LOW) == 0U) {
		logical_state = !logical_state;
	}

	return gpio_pin_set_dt(&broadcast_led, logical_state ? 1 : 0);
#else
	return gpio_pin_set_dt(&broadcast_led, running ? 1 : 0);
#endif
}
#endif

int led_init(void)
{
#if !BROADCAST_LED_AVAILABLE
	return 0;
#else
	int err;

	if (led_ready) {
		return 0;
	}

	if (!gpio_is_ready_dt(&broadcast_led)) {
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&broadcast_led, GPIO_OUTPUT_INACTIVE);
	if (err) {
		return err;
	}

	err = led_set_running_state(false);
	if (err) {
		return err;
	}

	led_ready = true;
	return 0;
#endif
}

int led_set_broadcast_running(bool running)
{
#if !BROADCAST_LED_AVAILABLE
	(void)running;
	return 0;
#else
	int err;

	if (!led_ready) {
		err = led_init();
		if (err) {
			return err;
		}
	}

	return led_set_running_state(running);
#endif
}
