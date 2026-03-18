/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Button inputs: PTT, PTT-lock, volume up/down.
 * See buttons.h for the design rationale.
 */

#include "io/buttons.h"
#include "audio/audio.h"
#include "audio/sync/clk_sync.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/* Defined in main.c — flushes stale pre-PTT frames from lc3_tx_q. */
extern void uplink_tx_flush(void);

/* Volume step in dB per button press. */
#define VOLUME_STEP_DB 3

/* Semaphore pointer set by buttons_init() — shared with PTT and PTT-lock. */
static struct k_sem *uplink_tx_sem;

/* =========================================================================
 * ptt_active — exported; read by iso_sent() and iso_recv() in main.c.
 *
 * Default: 1 (always TX) if no PTT button is present on this board.
 * ========================================================================= */
#if DT_NODE_HAS_STATUS(DT_ALIAS(sw2), okay)
atomic_t ptt_active = ATOMIC_INIT(0); /* start silent; button arms TX */
#else
atomic_t ptt_active = ATOMIC_INIT(1); /* no button — always transmitting */
#endif

/* =========================================================================
 * PTT (push-to-talk)  — sw2
 *
 * Edge-both interrupt.  On press: set ptt_active=1 and give tx_sem to
 * kick-start the uplink send chain.  On release: clear ptt_active=0 so
 * iso_sent() stops re-arming tx_sem and the chain starves naturally.
 * ========================================================================= */
#if DT_NODE_HAS_STATUS(DT_ALIAS(sw2), okay)
#define PTT_AVAILABLE 1
static const struct gpio_dt_spec ptt_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static struct gpio_callback ptt_cb_data;

static void ptt_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (gpio_pin_get_dt(&ptt_btn) > 0) {
		atomic_set(&ptt_active, 1);
		uplink_tx_flush();  /* discard stale pre-PTT frames */
		clk_sync_reset();   /* restart lock detection (freq preserved) */
		printk("[PTT] pressed\n");
		k_sem_give(uplink_tx_sem);
	} else {
		atomic_set(&ptt_active, 0);
		printk("[PTT] released\n");
	}
}

static int ptt_init(void)
{
	int err;

	if (!gpio_is_ready_dt(&ptt_btn)) {
		printk("PTT button GPIO not ready\n");
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&ptt_btn, GPIO_INPUT);
	if (err) {
		printk("PTT button configure failed: %d\n", err);
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&ptt_btn, GPIO_INT_EDGE_BOTH);
	if (err) {
		printk("PTT interrupt configure failed: %d\n", err);
		return err;
	}
	gpio_init_callback(&ptt_cb_data, ptt_isr, BIT(ptt_btn.pin));
	gpio_add_callback(ptt_btn.port, &ptt_cb_data);

	printk("PTT init: sw2 ready, mic TX disabled at boot\n");
	return 0;
}
#else
#define PTT_AVAILABLE 0
static int ptt_init(void)
{
	printk("PTT: no sw2 alias — always transmitting\n");
	return 0;
}
#endif /* PTT sw2 */

/* =========================================================================
 * PTT-lock  — sw3 (toggle button) + led0 (status LED)
 *
 * Each press toggles between locked (always TX) and unlocked (PTT mode).
 * The SPI-backed LED write is deferred to a work item so it is safe to
 * call from the GPIO ISR.
 * ========================================================================= */
#if DT_NODE_HAS_STATUS(DT_ALIAS(sw3), okay) && DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
#define PTT_LOCK_AVAILABLE 1
static const struct gpio_dt_spec ptt_lock_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);
static const struct gpio_dt_spec ptt_lock_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static struct gpio_callback ptt_lock_cb_data;
static atomic_t ptt_lock_active = ATOMIC_INIT(0);
static struct k_work ptt_lock_work;

static void ptt_lock_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (atomic_get(&ptt_lock_active)) {
		/* Unlock: back to PTT mode. */
		atomic_set(&ptt_lock_active, 0);
		gpio_pin_set_dt(&ptt_lock_led, 0);
		atomic_set(&ptt_active, 0);
		uplink_tx_flush();  /* discard stale encoded frames */
		printk("[PTT-LOCK] disabled — PTT mode active\n");
	} else {
		/* Lock: latch TX on. */
		atomic_set(&ptt_lock_active, 1);
		gpio_pin_set_dt(&ptt_lock_led, 1);
		atomic_set(&ptt_active, 1);
		uplink_tx_flush();  /* discard stale pre-PTT frames */
		clk_sync_reset();   /* restart lock detection (freq preserved) */
		k_sem_give(uplink_tx_sem);
		printk("[PTT-LOCK] enabled — always transmitting\n");
	}
}

static void ptt_lock_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	if (gpio_pin_get_dt(&ptt_lock_btn) > 0) {
		k_work_submit(&ptt_lock_work);
	}
}

static int ptt_lock_init(void)
{
	int err;

	k_work_init(&ptt_lock_work, ptt_lock_work_handler);

	if (!gpio_is_ready_dt(&ptt_lock_btn) || !gpio_is_ready_dt(&ptt_lock_led)) {
		printk("PTT-lock GPIO not ready\n");
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&ptt_lock_btn, GPIO_INPUT);
	if (err) {
		printk("PTT-lock button configure failed: %d\n", err);
		return err;
	}
	err = gpio_pin_configure_dt(&ptt_lock_led, GPIO_OUTPUT_INACTIVE);
	if (err) {
		printk("PTT-lock LED configure failed: %d\n", err);
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&ptt_lock_btn, GPIO_INT_EDGE_BOTH);
	if (err) {
		printk("PTT-lock interrupt configure failed: %d\n", err);
		return err;
	}
	gpio_init_callback(&ptt_lock_cb_data, ptt_lock_isr, BIT(ptt_lock_btn.pin));
	gpio_add_callback(ptt_lock_btn.port, &ptt_lock_cb_data);

	printk("PTT-lock init: sw3=toggle, led0=status\n");
	return 0;
}
#else
#define PTT_LOCK_AVAILABLE 0
static int ptt_lock_init(void)
{
	printk("PTT-lock: sw3/led0 not available on this board\n");
	return 0;
}
#endif /* PTT-lock sw3+led0 */

/* =========================================================================
 * Volume buttons  — sw0 (down) + sw1 (up)
 *
 * The CS47L63 volume register is accessed via SPI so the actual adjustment
 * must happen outside the ISR.  Presses accumulate into vol_pending_step
 * (atomic) and a work item drains it.
 * ========================================================================= */
#if DT_NODE_HAS_STATUS(DT_ALIAS(sw0), okay) && DT_NODE_HAS_STATUS(DT_ALIAS(sw1), okay)
#define VOL_BTN_AVAILABLE 1
static const struct gpio_dt_spec vol_dn_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec vol_up_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static struct gpio_callback vol_dn_cb_data;
static struct gpio_callback vol_up_cb_data;
static struct k_work vol_work;
static atomic_t vol_pending_step = ATOMIC_INIT(0);

static void vol_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int step = (int)atomic_set(&vol_pending_step, 0);

	if (step != 0) {
		audio_volume_adjust((int8_t)step);
	}
}

static void vol_dn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	printk("[VOL] down\n");
	atomic_add(&vol_pending_step, -VOLUME_STEP_DB);
	k_work_submit(&vol_work);
}

static void vol_up_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	printk("[VOL] up\n");
	atomic_add(&vol_pending_step, VOLUME_STEP_DB);
	k_work_submit(&vol_work);
}

static int vol_buttons_init(void)
{
	int err;

	k_work_init(&vol_work, vol_work_handler);

	if (!gpio_is_ready_dt(&vol_dn_btn) || !gpio_is_ready_dt(&vol_up_btn)) {
		printk("Volume button GPIO not ready\n");
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

	printk("Volume buttons: sw0=down, sw1=up, step=%d dB\n", VOLUME_STEP_DB);
	return 0;
}
#else
#define VOL_BTN_AVAILABLE 0
static int vol_buttons_init(void)
{
	printk("Volume buttons: sw0/sw1 not available on this board\n");
	return 0;
}
#endif /* Volume sw0+sw1 */

/* =========================================================================
 * Public API
 * ========================================================================= */

int buttons_init(struct k_sem *tx_sem)
{
	int err;

	uplink_tx_sem = tx_sem;

	err = ptt_init();
	if (err) {
		return err;
	}

	err = ptt_lock_init();
	if (err) {
		return err;
	}

	err = vol_buttons_init();
	if (err) {
		return err;
	}

	return 0;
}
