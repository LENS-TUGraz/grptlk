#ifndef GRPTLK_QDEC_H_
#define GRPTLK_QDEC_H_

#include <stdint.h>

/**
 * @brief Start the QDEC volume-control thread.
 *
 * Polls the quadrature encoder aliased as "qdec0" and calls @p vol_adjust_cb
 * with a ±dB step whenever the knob moves.  If no qdec0 alias is present the
 * function returns 0 immediately and no thread is created.
 *
 * @param vol_adjust_cb  Called with the volume delta in dB (+3 per detent up,
 *                       -3 per detent down).
 *
 * @return 0 on success, negative errno on failure.
 */
int qdec_start(int (*vol_adjust_cb)(int8_t step_db));

#endif /* GRPTLK_QDEC_H_ */
