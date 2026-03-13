#ifndef GRPTLK_BUTTON_H_
#define GRPTLK_BUTTON_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Callback invoked when PTT transitions to active (pressed / lock-on).
 *
 * Called from a work queue context, not from ISR.
 */
typedef void (*button_ptt_activated_cb_t)(void);

/**
 * @brief Initialise all board buttons (volume, PTT, PTT-lock).
 *
 * Board availability is determined at compile time via DT aliases:
 *   - sw0/sw1   — volume-down / volume-up  (nRF5340 Audio DK only)
 *   - sw1       — PTT/mute                 (RBV2H)
 *   - sw2       — PTT                      (nRF5340 Audio DK)
 *   - sw3/led0  — PTT-lock toggle / LED    (both boards, if present)
 *
 * @param vol_adjust_cb  Called with ±step_db when a volume button is pressed.
 *                       May be NULL if no volume buttons are available.
 * @param ptt_activated_cb  Called when PTT becomes active. May be NULL.
 *
 * @return 0 on success, negative errno on failure.
 */
int button_init(int (*vol_adjust_cb)(int8_t step_db), button_ptt_activated_cb_t ptt_activated_cb);

/**
 * @brief Return true if PTT is currently active (pressed or locked on).
 *
 * Safe to call from any context.
 */
bool button_ptt_is_active(void);

#endif /* GRPTLK_BUTTON_H_ */
