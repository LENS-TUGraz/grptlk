#include "io/battery.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <string.h>

#define BATT_VBAT_DT_NODE  DT_PATH(zephyr_user)
#define BATT_LIPO1_NODE    DT_ALIAS(lipo1)
#define BATT_LIPO2_NODE    DT_ALIAS(lipo2)
#define BATT_CHARGE_LED    DT_ALIAS(led0)

#if IS_ENABLED(CONFIG_GRPTLK_BATTERY) && \
	DT_NODE_HAS_STATUS(BATT_VBAT_DT_NODE, okay) && \
	DT_NODE_HAS_PROP(BATT_VBAT_DT_NODE, io_channels) && \
	DT_NODE_HAS_STATUS(BATT_LIPO1_NODE, okay) && \
	DT_NODE_HAS_STATUS(BATT_LIPO2_NODE, okay)

#define BATT_HAS_CHARGE_LED DT_NODE_HAS_STATUS(BATT_CHARGE_LED, okay)

/* R8 = 787k (top from VBAT), R10 = 2M (bottom to GND).
 * VBAT = V_pin * (R8 + R10) / R10 = V_pin * 2787 / 2000.
 */
#define BATT_DIV_NUM 2787
#define BATT_DIV_DEN 2000

#define BATT_FULL_MV  4200
#define BATT_EMPTY_MV 3300

static const struct adc_dt_spec adc_vbat = ADC_DT_SPEC_GET(BATT_VBAT_DT_NODE);
static const struct gpio_dt_spec stat1_in = GPIO_DT_SPEC_GET(BATT_LIPO1_NODE, gpios);
static const struct gpio_dt_spec stat2_in = GPIO_DT_SPEC_GET(BATT_LIPO2_NODE, gpios);
#if BATT_HAS_CHARGE_LED
static const struct gpio_dt_spec charge_led = GPIO_DT_SPEC_GET(BATT_CHARGE_LED, gpios);
#endif

static int16_t adc_buf;
static struct k_work_delayable batt_work;

static const char *charger_state(int s1, int s2)
{
	/* MCP73833 STAT pins are open-drain with pullup:
	 * pin == 0 → asserted by charger, pin == 1 → Hi-Z (deasserted).
	 */
	if (s1 == 0 && s2 == 1) {
		return "charging";
	}
	if (s1 == 1 && s2 == 0) {
		return "done";
	}
	if (s1 == 0 && s2 == 0) {
		return "fault";
	}
	return "standby";
}

static int battery_sample(int *vbat_mv_out, const char **state_out)
{
	struct adc_sequence seq = {
		.buffer = &adc_buf,
		.buffer_size = sizeof(adc_buf),
	};
	int err;
	int32_t pin_mv;

	err = adc_sequence_init_dt(&adc_vbat, &seq);
	if (err) {
		return err;
	}

	err = adc_read_dt(&adc_vbat, &seq);
	if (err) {
		return err;
	}

	pin_mv = (int32_t)adc_buf;
	err = adc_raw_to_millivolts_dt(&adc_vbat, &pin_mv);
	if (err) {
		return err;
	}
	if (pin_mv < 0) {
		pin_mv = 0;
	}

	*vbat_mv_out = (int)((pin_mv * BATT_DIV_NUM) / BATT_DIV_DEN);
	*state_out = charger_state(gpio_pin_get_dt(&stat1_in),
				   gpio_pin_get_dt(&stat2_in));
	return 0;
}

static void battery_log(void)
{
	int vbat_mv = 0;
	const char *state = "?";
	int err = battery_sample(&vbat_mv, &state);

	if (err) {
		printk("[BATT] sample failed: %d\n", err);
		return;
	}

	int pct = ((vbat_mv - BATT_EMPTY_MV) * 100) / (BATT_FULL_MV - BATT_EMPTY_MV);

	pct = CLAMP(pct, 0, 100);

	printk("[BATT] VBAT=%d.%03dV (%d%%) | charger=%s\n",
	       vbat_mv / 1000, vbat_mv % 1000, pct, state);

#if BATT_HAS_CHARGE_LED
	gpio_pin_set_dt(&charge_led, strcmp(state, "charging") == 0);
#endif
}

static void battery_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	battery_log();
	k_work_schedule(&batt_work, K_SECONDS(CONFIG_GRPTLK_BATTERY_PERIOD_S));
}

int battery_init(void)
{
	int err;

	if (!adc_is_ready_dt(&adc_vbat)) {
		printk("[BATT] ADC not ready\n");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&stat1_in) || !gpio_is_ready_dt(&stat2_in)) {
		printk("[BATT] STAT GPIOs not ready\n");
		return -ENODEV;
	}

	err = adc_channel_setup_dt(&adc_vbat);
	if (err) {
		printk("[BATT] adc_channel_setup failed: %d\n", err);
		return err;
	}
	err = gpio_pin_configure_dt(&stat1_in, GPIO_INPUT);
	if (err) {
		return err;
	}
	err = gpio_pin_configure_dt(&stat2_in, GPIO_INPUT);
	if (err) {
		return err;
	}

#if BATT_HAS_CHARGE_LED
	if (gpio_is_ready_dt(&charge_led)) {
		(void)gpio_pin_configure_dt(&charge_led, GPIO_OUTPUT_INACTIVE);
	}
#endif

	k_work_init_delayable(&batt_work, battery_work_handler);

	battery_log();
	k_work_schedule(&batt_work, K_SECONDS(CONFIG_GRPTLK_BATTERY_PERIOD_S));
	return 0;
}

#else

int battery_init(void)
{
	return 0;
}

#endif
