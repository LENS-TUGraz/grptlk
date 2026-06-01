#include "io/encoder.h"
#include "audio/audio.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#define QDEC_NODE DT_ALIAS(qdec0)

#if DT_NODE_HAS_STATUS(QDEC_NODE, okay) && IS_ENABLED(CONFIG_GRPTLK_ENCODER)

static const struct device *const qdec_dev = DEVICE_DT_GET(QDEC_NODE);
static struct k_work enc_work;
static atomic_t enc_pending_db = ATOMIC_INIT(0);
static int32_t enc_deg_accum;

static void enc_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int db = (int)atomic_set(&enc_pending_db, 0);

	if (db != 0) {
		audio_volume_adjust((int8_t)db);
	}
}

static void qdec_trig_cb(const struct device *dev, const struct sensor_trigger *trig)
{
	ARG_UNUSED(trig);
	struct sensor_value val;
	int ret;

	ret = sensor_sample_fetch(dev);
	if (ret < 0) {
		return;
	}
	ret = sensor_channel_get(dev, SENSOR_CHAN_ROTATION, &val);
	if (ret < 0) {
		return;
	}

	enc_deg_accum += val.val1;
	int steps = enc_deg_accum / CONFIG_GRPTLK_ENCODER_DEG_PER_STEP;

	if (steps == 0) {
		return;
	}
	enc_deg_accum -= steps * CONFIG_GRPTLK_ENCODER_DEG_PER_STEP;

#if IS_ENABLED(CONFIG_GRPTLK_ENCODER_INVERT)
	steps = -steps;
#endif

	atomic_add(&enc_pending_db, steps * CONFIG_GRPTLK_ENCODER_VOLUME_STEP_DB);
	k_work_submit(&enc_work);
}

int encoder_init(void)
{
	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ROTATION,
	};
	int err;

	if (!device_is_ready(qdec_dev)) {
		printk("Encoder: qdec0 not ready\n");
		return -ENODEV;
	}

	k_work_init(&enc_work, enc_work_handler);

	err = sensor_trigger_set(qdec_dev, &trig, qdec_trig_cb);
	if (err) {
		printk("Encoder: trigger set failed: %d\n", err);
		return err;
	}

	printk("Encoder: qdec0 ready, %d deg/step, %d dB/detent%s\n",
	       CONFIG_GRPTLK_ENCODER_DEG_PER_STEP,
	       CONFIG_GRPTLK_ENCODER_VOLUME_STEP_DB,
	       IS_ENABLED(CONFIG_GRPTLK_ENCODER_INVERT) ? " (inverted)" : "");
	return 0;
}

#else

int encoder_init(void)
{
	printk("Encoder: disabled (no qdec0 alias or CONFIG_GRPTLK_ENCODER=n)\n");
	return 0;
}

#endif
