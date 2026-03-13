#include "qdec.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(qdec, CONFIG_QDEC_LOG_LEVEL);

/* ─── QDEC availability ─────────────────────────────────────────────────── */
#if DT_HAS_ALIAS(qdec0) && DT_NODE_HAS_STATUS(DT_ALIAS(qdec0), okay)
#define HAS_QDEC   1
#define QDEC_STEPS DT_PROP(DT_ALIAS(qdec0), steps)
static const struct device *const qdec = DEVICE_DT_GET(DT_ALIAS(qdec0));
#else
#define HAS_QDEC 0
#endif

#if HAS_QDEC

K_THREAD_STACK_DEFINE(vol_stack, 1024);
static struct k_thread vol_thread_data;
static int (*s_vol_adjust_cb)(int8_t step_db);

static void volume_thread_fn(void *p1, void *p2, void *p3)
{
	struct sensor_value val;
	int err;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		err = sensor_sample_fetch(qdec);
		if (err) {
			k_sleep(K_MSEC(50));
			continue;
		}

		err = sensor_channel_get(qdec, SENSOR_CHAN_ROTATION, &val);
		if (err) {
			k_sleep(K_MSEC(50));
			continue;
		}

		if (val.val1 != 0) {
			int32_t rotation = (val.val1 < 0) ? -val.val1 : val.val1;
			int32_t detents = DIV_ROUND_CLOSEST(rotation * QDEC_STEPS, 360);

			if (detents == 0) {
				detents = 1;
			}

			/* Knob CW → val1 positive → volume down. */
			int8_t step_db = (int8_t)((val.val1 < 0 ? 1 : -1) * detents);

			LOG_DBG("qdec %s (%ld deg, %ld detents, %d dB)",
				val.val1 < 0 ? "up" : "down", (long)val.val1, (long)detents,
				step_db);
			s_vol_adjust_cb(step_db);
		}

		k_sleep(K_MSEC(25));
	}
}

#endif /* HAS_QDEC */

int qdec_start(int (*vol_adjust_cb)(int8_t step_db))
{
#if !HAS_QDEC
	ARG_UNUSED(vol_adjust_cb);
	LOG_INF("no QDEC on this board — volume knob not available");
	return 0;
#else
	if (!device_is_ready(qdec)) {
		LOG_ERR("QDEC device not ready");
		return -ENODEV;
	}

	s_vol_adjust_cb = vol_adjust_cb;

	k_thread_create(&vol_thread_data, vol_stack, K_THREAD_STACK_SIZEOF(vol_stack),
			volume_thread_fn, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
	k_thread_name_set(&vol_thread_data, "qdec_vol");

	LOG_INF("QDEC volume thread started");
	return 0;
#endif
}
