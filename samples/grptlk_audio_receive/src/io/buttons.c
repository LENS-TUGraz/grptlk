#include "io/buttons.h"
#include "audio/audio.h"
#include "audio/sync/clk_sync.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define VOLUME_STEP_DB 3

static struct k_sem *uplink_tx_sem;

/* LINE-IN mode flag: set when audio source is LINE-IN jack.
 * Causes PTT and PTT-lock to be no-ops. */
atomic_t src_line_in_active = ATOMIC_INIT(0);

#if DT_NODE_HAS_STATUS(DT_ALIAS(sw2), okay)
atomic_t ptt_active = ATOMIC_INIT(0);
#else
atomic_t ptt_active = ATOMIC_INIT(1);
#endif

/* PTT-lock: sw3 (toggle) + led0 (status). SPI-backed LED write deferred
 * to work item for ISR safety.
 * Declared before PTT block so ptt_isr can observe lock state. */
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
	if (atomic_get(&src_line_in_active)) {
		return; /* PTT-lock is MIC-only */
	}
	if (atomic_get(&ptt_lock_active)) {
		atomic_set(&ptt_lock_active, 0);
		gpio_pin_set_dt(&ptt_lock_led, 0);
		atomic_set(&ptt_active, 0);
#if defined(CONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT)
		extern void ptt_session_bis_reset(void);
		ptt_session_bis_reset();
#endif
		printk("[PTT-LOCK] disabled\n");
	} else {
		atomic_set(&ptt_lock_active, 1);
		gpio_pin_set_dt(&ptt_lock_led, 1);
		atomic_set(&ptt_active, 1);
		clk_sync_reset();
		k_sem_give(uplink_tx_sem);
		printk("[PTT-LOCK] enabled\n");
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

	printk("PTT-lock: sw3=toggle, led0=status\n");
	return 0;
}
#else
#define PTT_LOCK_AVAILABLE 0
static int ptt_lock_init(void)
{
	printk("PTT-lock: sw3/led0 not available\n");
	return 0;
}
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(sw2), okay)
#define PTT_AVAILABLE 1
static const struct gpio_dt_spec ptt_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static struct gpio_callback ptt_cb_data;

static void ptt_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (atomic_get(&src_line_in_active)) {
		return; /* PTT is MIC-only */
	}

#if PTT_LOCK_AVAILABLE
	if (atomic_get(&ptt_lock_active)) {
		return; /* lock owns ptt_active; sw2 is a no-op while engaged */
	}
#endif

	if (gpio_pin_get_dt(&ptt_btn) > 0) {
		atomic_set(&ptt_active, 1);
		clk_sync_reset();
		printk("[PTT] pressed\n");
		k_sem_give(uplink_tx_sem);
	} else {
		atomic_set(&ptt_active, 0);
#if defined(CONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT)
		extern void ptt_session_bis_reset(void);
		ptt_session_bis_reset();
#endif
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

	printk("PTT init: sw2 ready, mic disabled at boot\n");
	return 0;
}
#else
#define PTT_AVAILABLE 0
static int ptt_init(void)
{
	printk("PTT: no sw2 alias - always TX\n");
	return 0;
}
#endif

/* Volume buttons: sw0 (down) + sw1 (up). CS47L63 volume accessed via SPI,
 * so adjustments deferred to work queue. */
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

	printk("Volume: sw0=down, sw1=up, step=%d dB\n", VOLUME_STEP_DB);
	return 0;
}
#else
#define VOL_BTN_AVAILABLE 0
static int vol_buttons_init(void)
{
	printk("Volume buttons: sw0/sw1 not available\n");
	return 0;
}
#endif

/* Source toggle: sw4 (BTN5) + led1. Deferred to work queue (SPI access). */
#if defined(CONFIG_GRPTLK_AUDIO_CODEC_CIRRUS) && DT_NODE_HAS_STATUS(DT_ALIAS(sw4), okay) && \
	DT_NODE_HAS_STATUS(DT_ALIAS(led1), okay)
#define SRC_TOGGLE_AVAILABLE 1
static const struct gpio_dt_spec src_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw4), gpios);
static const struct gpio_dt_spec src_led = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static struct gpio_callback src_cb_data;
static struct k_work src_toggle_work;

static void src_toggle_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (atomic_get(&src_line_in_active)) {
		atomic_set(&src_line_in_active, 0);
		gpio_pin_set_dt(&src_led, 0);
		audio_input_source_switch(false);
	} else {
		atomic_set(&src_line_in_active, 1);
		gpio_pin_set_dt(&src_led, 1);
		/* Reset PTT state — buttons are no-ops in LINE-IN mode */
		atomic_set(&ptt_active, 0);
#if PTT_LOCK_AVAILABLE
		if (atomic_get(&ptt_lock_active)) {
			atomic_set(&ptt_lock_active, 0);
			gpio_pin_set_dt(&ptt_lock_led, 0);
		}
#endif
		audio_input_source_switch(true);
	}
}

static void src_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	k_work_submit(&src_toggle_work);
}

static int src_toggle_init(void)
{
	int err;

	k_work_init(&src_toggle_work, src_toggle_work_handler);

	if (!gpio_is_ready_dt(&src_btn) || !gpio_is_ready_dt(&src_led)) {
		printk("SRC toggle GPIO not ready\n");
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&src_btn, GPIO_INPUT);
	if (err) {
		return err;
	}
	err = gpio_pin_configure_dt(&src_led, GPIO_OUTPUT_INACTIVE);
	if (err) {
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&src_btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		return err;
	}
	gpio_init_callback(&src_cb_data, src_isr, BIT(src_btn.pin));
	gpio_add_callback(src_btn.port, &src_cb_data);

	printk("SRC toggle: sw4=toggle, led1=line-in indicator\n");
	return 0;
}
#else
#define SRC_TOGGLE_AVAILABLE 0
static int src_toggle_init(void)
{
	printk("SRC toggle: unavailable on selected codec\n");
	return 0;
}
#endif

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

	err = src_toggle_init();
	if (err) {
		return err;
	}

	return 0;
}
