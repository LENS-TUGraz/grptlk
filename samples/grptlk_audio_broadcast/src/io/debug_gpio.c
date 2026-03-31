#include "debug_gpio.h"

#if defined(CONFIG_GRPTLK_DEBUG_GPIO)

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

const struct gpio_dt_spec debug_iso_sent = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)), .pin = 31, .dt_flags = 0};
const struct gpio_dt_spec debug_iso_recv = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio1)), .pin = 0, .dt_flags = 0};
const struct gpio_dt_spec debug_lc3_dec = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio1)), .pin = 1, .dt_flags = 0};
const struct gpio_dt_spec debug_lc3_enc = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio1)), .pin = 14, .dt_flags = 0};

int debug_gpio_init(void)
{
	int err;

	if (!device_is_ready(debug_iso_recv.port)) {
		printk("Debug ISO recv GPIO not ready\n");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&debug_iso_recv, GPIO_OUTPUT_LOW);
	if (err) {
		printk("Debug ISO recv GPIO configure failed: %d\n", err);
		return err;
	}

	if (!device_is_ready(debug_lc3_dec.port)) {
		printk("Debug LC3 decode GPIO not ready\n");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&debug_lc3_dec, GPIO_OUTPUT_LOW);
	if (err) {
		printk("Debug LC3 decode GPIO configure failed: %d\n", err);
		return err;
	}

	if (!device_is_ready(debug_lc3_enc.port)) {
		printk("Debug LC3 encode GPIO not ready\n");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&debug_lc3_enc, GPIO_OUTPUT_LOW);
	if (err) {
		printk("Debug LC3 encode GPIO configure failed: %d\n", err);
		return err;
	}

	if (!device_is_ready(debug_iso_sent.port)) {
		printk("Debug ISO sent GPIO not ready\n");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&debug_iso_sent, GPIO_OUTPUT_LOW);
	if (err) {
		printk("Debug ISO sent GPIO configure failed: %d\n", err);
		return err;
	}

	printk("Debug GPIO initialized: D2=iso_recv, D3=lc3_dec, D4=lc3_enc, P1.14=iso_sent\n");
	return 0;
}

#else

int debug_gpio_init(void)
{
	return 0;
}

#endif
