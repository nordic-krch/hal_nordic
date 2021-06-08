/*$$$LICENCE_NORDIC_STANDARD<2021>$$$*/

#ifndef NRFX_UTILS_H__
#define NRFX_UTILS_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Function is used to initilize allocator mask.
 *
 * Alternatively mask can be set to init value by direct assignment.
 *
 * @param[out] p_mask Mask to be initilized.
 * @param[in]  init_mask Mask with available flags, e.g. available flag 0, bit 0 set.
 */
static inline void nrfx_flag32_init(nrfx_atomic_t *p_mask, uint32_t init_mask)
{
	*p_mask = init_mask;
}

/**
 * @brief Function checks if given flag is allocated.
 *
 * Note that this check may not be valid if context is preempted and state is changed.
 *
 * @param[in] mask Mask.
 * @param[in] flag_id Flag ID.
 *
 * @retval True flag is allocated.
 * @retval False flag is available.
 */
bool nrfx_flag32_is_allocated(nrfx_atomic_t mask, uint8_t flag_id);

/**
 * @brief Function for allocating a flag in the mask.
 *
 * Function is thread safe. Bit mask should be initiliazed with bits set for
 * each available flag.
 *
 * @param[in,out] p_mask Mask with available flags set. On successful allocation flag is cleared.
 * @param[out]    p_flag Index of the allocated flag.
 *
 * @retval NRFX_SUCCESS      Allocation was successful.
 * @retval NRFX_ERROR_NO_MEM No resource available.
 */
nrfx_err_t nrfx_flag32_alloc(nrfx_atomic_t *p_mask, uint8_t *p_flag);

/**
 * @brief Function for freeing a flag allocated with @ref nrfx_flag32_alloc.
 *
 * Function is thread safe.
 *
 * @param[in,out] p_mask Mask with available flags set. On successful allocation flag is set.
 * @param[in]     flag   Flag index.
 *
 * @retval NRFX_SUCCESS             Freeing was successful.
 * @retval NRFX_ERROR_INVALID_PARAM Flag was not allocated.
 */
nrfx_err_t nrfx_flag32_free(nrfx_atomic_t *p_mask, uint8_t flag);

#ifdef __cplusplus
}
#endif

#endif // NRFX_UTILS_H__
