/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "audio/sync/clk_sync.h"

#include <zephyr/sys/util.h>

int clk_sync_init(uint32_t blk_period_us)
{
	ARG_UNUSED(blk_period_us);
	return 0;
}

void clk_sync_anchor_notify(uint32_t ts_us)
{
	ARG_UNUSED(ts_us);
}

void clk_sync_i2s_notify(uint32_t ts_us)
{
	ARG_UNUSED(ts_us);
}

void clk_sync_reset(void)
{
}

bool clk_sync_is_locked(void)
{
	/* Relay-only mode has no local audio clock to converge, so report ready. */
	return true;
}
