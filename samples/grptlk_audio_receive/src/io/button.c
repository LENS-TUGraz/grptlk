#include "button.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(button, CONFIG_BUTTON_LOG_LEVEL);

#define VOLUME_STEP_DB 3

/* ─── Board layout detection ───────────────────────────────────────────────
 *
 * RBV2H assigns sw1 to PTT/mute; the nRF5340 Audio DK puts volume buttons on
 * sw0+sw1 and PTT on sw2.
 */
#if defined(CONFIG_BOARD_RBV2H_NRF5340_CPUAPP)
#define RBV2H_BUTTON_LAYOUT 1
#else
#define RBV2H_BUTTON_LAYOUT 0
#endif

/* ─── Volume buttons (nRF5340 Audio DK: sw0 = down, sw1 = up) ──────────── */
#if !RBV2H_BUTTON_LAYOUT && DT_NODE_HAS_STATUS(DT_ALIAS(sw0), okay) &&                             \
	DT_NODE_HAS_STATUS(DT_ALIAS(sw1), okay)
#define VOL_BTN_AVAILABLE 1
static const struct gpio_dt_spec vol_dn_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec vol_up_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static struct gpio_callback vol_dn_cb_data;
static struct gpio_callback vol_up_cb_data;
static struct k_work vol_work;
static atomic_t vol_pending_step = ATOMIC_INIT(0);
#else
#define VOL_BTN_AVAILABLE 0
#endif

/* ─── PTT button ────────────────────────────────────────────────────────── */
#if RBV2H_BUTTON_LAYOUT
#define PTT_AVAILABLE 1
static const struct gpio_dt_spec ptt_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const char *const ptt_btn_name = "sw1/mute";
#elif DT_NODE_HAS_STATUS(DT_ALIAS(sw2), okay)
#define PTT_AVAILABLE 1
static const struct gpio_dt_spec ptt_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static const char *const ptt_btn_name = "sw2";
#else
#define PTT_AVAILABLE 0
#endif

#if PTT_AVAILABLE
static struct gpio_callback ptt_cb_data;
static struct k_work_delayable ptt_debounce_work;
#endif

/* ─── PTT-lock button + LED (sw3 = toggle, led0 = status) ──────────────── */
#if DT_NODE_HAS_STATUS(DT_ALIAS(sw3), okay) && DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
#define PTT_LOCK_AVAILABLE 1
static const struct gpio_dt_spec ptt_lock_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);
static const struct gpio_dt_spec ptt_lock_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static struct gpio_callback ptt_lock_cb_data;
static atomic_t ptt_lock_active = ATOMIC_INIT(0);
static struct k_work ptt_lock_toggle_work;
static struct k_work_delayable ptt_lock_debounce_work;
#else
#define PTT_LOCK_AVAILABLE 0
#endif

/* ─── Shared state ──────────────────────────────────────────────────────── */

/* PTT_AVAILABLE=0 means no button — default to always transmitting. */
#if PTT_AVAILABLE
static atomic_t ptt_active = ATOMIC_INIT(0);
#else
static atomic_t ptt_active = ATOMIC_INIT(1);
#endif

static int (*s_vol_adjust_cb)(int8_t step_db);
static button_ptt_activated_cb_t s_ptt_activated_cb;

/* ─── Volume button handlers ────────────────────────────────────────────── */
#if VOL_BTN_AVAILABLE

static void vol_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int step = (int)atomic_set(&vol_pending_step, 0);

	if (step != 0 && s_vol_adjust_cb != NULL) {
		s_vol_adjust_cb((int8_t)step);
	}
}

static void vol_dn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	LOG_DBG("vol down pressed");
	atomic_add(&vol_pending_step, -VOLUME_STEP_DB);
	k_work_submit(&vol_work);
}

static void vol_up_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	LOG_DBG("vol up pressed");
	atomic_add(&vol_pending_step, VOLUME_STEP_DB);
	k_work_submit(&vol_work);
}

static int vol_buttons_init(void)
{
	int err;

	k_work_init(&vol_work, vol_work_handler);

	if (!gpio_is_ready_dt(&vol_dn_btn) || !gpio_is_ready_dt(&vol_up_btn)) {
		LOG_ERR("volume button GPIO not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&vol_dn_btn, GPIO_INPUT);
	if (err) {
		return err;
	}
	err = gpio_pin_configure_dt(&vol_up_btn, GPIO_INPUT);
	if (err) {
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&vol_dn_btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&vol_up_btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		return err;
	}

	gpio_init_callback(&vol_dn_cb_data, vol_dn_isr, BIT(vol_dn_btn.pin));
	gpio_init_callback(&vol_up_cb_data, vol_up_isr, BIT(vol_up_btn.pin));
	gpio_add_callback(vol_dn_btn.port, &vol_dn_cb_data);
	gpio_add_callback(vol_up_btn.port, &vol_up_cb_data);

	LOG_INF("volume buttons: sw0=down, sw1=up, step=%d dB", VOLUME_STEP_DB);
	return 0;
}
#endif /* VOL_BTN_AVAILABLE */

/* ─── PTT button handlers ───────────────────────────────────────────────── */
#if PTT_AVAILABLE

static void ptt_debounce_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int val = gpio_pin_get_dt(&ptt_btn);

	if (val > 0) {
		if (atomic_get(&ptt_active) == 0) {
			atomic_set(&ptt_active, 1);
			LOG_INF("PTT pressed");
			if (s_ptt_activated_cb != NULL) {
				s_ptt_activated_cb();
			}
		}
	} else {
		if (atomic_get(&ptt_active) == 1) {
			atomic_set(&ptt_active, 0);
			LOG_INF("PTT released");
		}
	}
}

static void ptt_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	k_work_reschedule(&ptt_debounce_work, K_MSEC(50));
}

static int ptt_button_init(void)
{
	int err;

	if (!gpio_is_ready_dt(&ptt_btn)) {
		LOG_ERR("PTT button GPIO not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&ptt_btn, GPIO_INPUT);
	if (err) {
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&ptt_btn, GPIO_INT_EDGE_BOTH);
	if (err) {
		return err;
	}

	k_work_init_delayable(&ptt_debounce_work, ptt_debounce_handler);
	gpio_init_callback(&ptt_cb_data, ptt_isr, BIT(ptt_btn.pin));
	gpio_add_callback(ptt_btn.port, &ptt_cb_data);

	atomic_set(&ptt_active, 0);
	LOG_INF("PTT init: %s ready, TX disabled at boot", ptt_btn_name);
	return 0;
}
#endif /* PTT_AVAILABLE */

/* ─── PTT-lock handlers ─────────────────────────────────────────────────── */
#if PTT_LOCK_AVAILABLE

static void ptt_lock_toggle_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (atomic_get(&ptt_lock_active)) {
		atomic_set(&ptt_lock_active, 0);
		gpio_pin_set_dt(&ptt_lock_led, 0);
		atomic_set(&ptt_active, 0);
		LOG_INF("PTT-lock disabled — PTT mode active");
	} else {
		atomic_set(&ptt_lock_active, 1);
		gpio_pin_set_dt(&ptt_lock_led, 1);
		atomic_set(&ptt_active, 1);
		LOG_INF("PTT-lock enabled — always transmitting");
		if (s_ptt_activated_cb != NULL) {
			s_ptt_activated_cb();
		}
	}
}

static void ptt_lock_debounce_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (gpio_pin_get_dt(&ptt_lock_btn) > 0) {
		k_work_submit(&ptt_lock_toggle_work);
	}
}

static void ptt_lock_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	k_work_reschedule(&ptt_lock_debounce_work, K_MSEC(50));
}

static int ptt_lock_button_init(void)
{
	int err;

	k_work_init(&ptt_lock_toggle_work, ptt_lock_toggle_work_handler);
	k_work_init_delayable(&ptt_lock_debounce_work, ptt_lock_debounce_handler);

	if (!gpio_is_ready_dt(&ptt_lock_btn) || !gpio_is_ready_dt(&ptt_lock_led)) {
		LOG_ERR("PTT-lock button/LED GPIO not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&ptt_lock_btn, GPIO_INPUT);
	if (err) {
		return err;
	}

	err = gpio_pin_configure_dt(&ptt_lock_led, GPIO_OUTPUT_INACTIVE);
	if (err) {
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&ptt_lock_btn, GPIO_INT_EDGE_BOTH);
	if (err) {
		return err;
	}

	gpio_init_callback(&ptt_lock_cb_data, ptt_lock_isr, BIT(ptt_lock_btn.pin));
	gpio_add_callback(ptt_lock_btn.port, &ptt_lock_cb_data);

	LOG_INF("PTT-lock init: sw3=toggle, led0=indicator");
	return 0;
}
#endif /* PTT_LOCK_AVAILABLE */

/* ─── Public API ────────────────────────────────────────────────────────── */

bool button_ptt_is_active(void)
{
	return atomic_get(&ptt_active) != 0;
}

int button_init(int (*vol_adjust_cb)(int8_t step_db), button_ptt_activated_cb_t ptt_activated_cb)
{
	int err;

	s_vol_adjust_cb = vol_adjust_cb;
	s_ptt_activated_cb = ptt_activated_cb;

#if VOL_BTN_AVAILABLE
	err = vol_buttons_init();
	if (err) {
		return err;
	}
#else
	LOG_INF("volume buttons: not available on this board");
#endif

#if PTT_AVAILABLE
	err = ptt_button_init();
	if (err) {
		return err;
	}
#else
	LOG_INF("PTT: no dedicated button, always transmitting");
#endif

#if PTT_LOCK_AVAILABLE
	err = ptt_lock_button_init();
	if (err) {
		return err;
	}
#else
	LOG_INF("PTT-lock: sw3/led0 not available on this board");
#endif

	return 0;
}
