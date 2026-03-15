/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Button inputs for the GRPTLK receiver.
 *
 * Three logical button groups are supported, all optional and detected at
 * compile time from devicetree aliases:
 *
 *   PTT (push-to-talk)   sw2
 *     Press to activate uplink TX; release to stop.
 *     If sw2 is absent the receiver always transmits (ptt_active = 1).
 *
 *   PTT-lock             sw3 + led0
 *     Toggle to latch uplink TX on/off without holding the button.
 *     LED (led0) lights when lock is active.
 *
 *   Volume               sw0 (down) + sw1 (up)
 *     Adjust speaker volume ±VOLUME_STEP_DB per press.
 *     SPI write is deferred to a work queue — safe to call from ISR context.
 *
 * Usage from main.c:
 *
 *   extern atomic_t ptt_active;   // read by iso_sent / iso_recv
 *
 *   buttons_init(&tx_sem);        // pass the uplink TX semaphore
 */

#ifndef GRPTLK_BUTTONS_H_
#define GRPTLK_BUTTONS_H_

#include <zephyr/kernel.h>

/*
 * ptt_active — atomic flag exported to main.c for use in ISO callbacks.
 *   0: uplink chain starved (iso_sent does not re-arm tx_sem)
 *   1: uplink chain self-sustains via iso_sent → tx_sem → iso_tx
 */
extern atomic_t ptt_active;

/*
 * Initialise all available button groups.
 *
 * tx_sem: the semaphore that arms the iso_tx uplink send chain.  The PTT and
 *         PTT-lock handlers call k_sem_give(tx_sem) to start transmission.
 *
 * Returns 0 on success.  Individual button groups that are absent on the
 * board are silently skipped; only hard GPIO errors are returned.
 */
int buttons_init(struct k_sem *tx_sem);

#endif /* GRPTLK_BUTTONS_H_ */
