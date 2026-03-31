#ifndef GRPTLK_BUTTONS_H_
#define GRPTLK_BUTTONS_H_

#include <zephyr/kernel.h>

extern atomic_t ptt_active;

int buttons_init(struct k_sem *tx_sem);

#endif /* GRPTLK_BUTTONS_H_ */
