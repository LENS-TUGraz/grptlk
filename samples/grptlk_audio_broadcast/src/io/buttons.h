#ifndef GRPTLK_BUTTONS_H_
#define GRPTLK_BUTTONS_H_

#include <zephyr/kernel.h>

extern atomic_t ptt_active;
extern atomic_t src_line_in_active;

#if defined(CONFIG_GRPTLK_PTT_VAD)
extern atomic_t ptt_vad_active;
#endif

#endif /* GRPTLK_BUTTONS_H_ */
