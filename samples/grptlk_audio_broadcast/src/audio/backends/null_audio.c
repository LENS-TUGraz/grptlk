/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "audio/audio.h"

#include <errno.h>
#include <stdbool.h>
#include <zephyr/sys/util.h>

static bool is_initialized;
static bool is_started;

int audio_init(struct audio_ring *ring, audio_rx_cb_t rx_cb)
{
	ARG_UNUSED(ring);
	ARG_UNUSED(rx_cb);

	if (!is_initialized) {
		is_initialized = true;
		is_started = false;
	}

	return 0;
}

int audio_start(void)
{
	if (!is_initialized) {
		return -EPERM;
	}

	if (is_started) {
		return 0;
	}

	is_started = true;
	return 0;
}

int audio_volume_adjust(int8_t step_db)
{
	ARG_UNUSED(step_db);

	if (!is_started) {
		return -EPERM;
	}

	return -ENOTSUP;
}

int audio_input_source_switch(bool use_line_in)
{
	ARG_UNUSED(use_line_in);

	if (!is_started) {
		return -EPERM;
	}

	return -ENOTSUP;
}

uint32_t audio_underrun_count_reset(void)
{
	return 0U;
}

uint32_t audio_served_count_reset(void)
{
	return 0U;
}

uint32_t audio_max_consec_underrun_reset(void)
{
	return 0U;
}
