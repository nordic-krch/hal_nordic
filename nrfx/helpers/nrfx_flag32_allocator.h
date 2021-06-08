/*$$$LICENCE_NORDIC_STANDARD<2021>$$$*/

#ifndef NRFX_FLAG32_ALLOCATOR_H__
#define NRFX_FLAG32_ALLOCATOR_H__

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_flag32_allocator Generic flag allocator
 * @{
 * @ingroup nrfx
 * @brief Generic flag allocator.
 */

/**
 * @brief Function for initializing allocator mask.
 *
 * Initialization value contains mask where each bit indicates availablility of
 * a given flag, e.g. init value 0x0000000A indicate that flag 3 and 1 (counting
 * from 0) can be allocated.
 *
 * Alternatively, mask can be set to init value by direct assignment.
 *
 * @param[out] p_mask    Mask to be initilized.
 * @param[in]  init_mask Mask with pool of available flags where bit being set means that
 *                       flag is free and can be allocated.
 */
__STATIC_INLINE void nrfx_flag32_init(nrfx_atomic_t *p_mask, uint32_t init_mask)
{
	*p_mask = init_mask;
}

/**
 * @brief Function for checking if given flag is allocated.
 *
 * @note This check may not be valid if context is preempted and state is changed.
 *
 * @param[in] mask   Mask.
 * @param[in] bitpos Flag bit position.
 *
 * @return True if specified flag is allocated, false otherwise.
 */
bool nrfx_flag32_is_allocated(nrfx_atomic_t mask, uint8_t bitpos);

/**
 * @brief Function for allocating a flag in the mask.
 *
 * @note Function is thread safe, it uses @ref NRFX_ATOMIC_CAS macro. No further
 * synchronization mechanism is needed, provided the macro is properly implemented
 * (see @ref nrfx_glue).
 *
 * Mask must be initilized before first allocation. Flags are allocated from the
 * highest bit position, e.g. if mask is set to 0x0000000A, 3 is returned and bit 3
 * is cleared in the mask. Mask is set to 0x00000002 on return after successful allocation.
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
 * @note Function is thread safe, it uses @ref NRFX_ATOMIC_CAS macro. No further
 * synchronization mechanism is needed, provided the macro is properly implemented
 * (see @ref nrfx_glue).
 *
 * @param[in,out] p_mask Mask with available flags set. On successful allocation flag is set.
 * @param[in]     flag   Flag index.
 *
 * @retval NRFX_SUCCESS             Freeing was successful.
 * @retval NRFX_ERROR_INVALID_PARAM Flag was not allocated.
 */
nrfx_err_t nrfx_flag32_free(nrfx_atomic_t *p_mask, uint8_t flag);

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRFX_FLAG32_ALLOCATOR_H__
