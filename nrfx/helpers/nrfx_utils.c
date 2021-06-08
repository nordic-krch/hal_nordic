/*$$$LICENCE_NORDIC_STANDARD<2021>$$$*/
#include <helpers/nrfx_utils.h>

bool nrfx_flag32_is_allocated(nrfx_atomic_t mask, uint8_t flag_id)
{
    return (mask & NRFX_BIT(flag_id)) ? false : true;
}

nrfx_err_t nrfx_flag32_alloc(nrfx_atomic_t *p_mask, uint8_t *p_flag)
{
    int8_t idx;
    uint32_t new_mask, prev_mask;

    do {
        prev_mask = *p_mask;
        idx = 31 - __CLZ(prev_mask);
        if (idx < 0) {
            return NRFX_ERROR_NO_MEM;
        }

        new_mask = prev_mask & ~NRFX_BIT(idx);
    } while (!NRFX_ATOMIC_CAS(p_mask, prev_mask, new_mask));

    *p_flag = idx;

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_flag32_free(nrfx_atomic_t *p_mask, uint8_t flag)
{
    uint32_t new_mask, prev_mask;

    if((NRFX_BIT(flag) & *p_mask))
    {
        return NRFX_ERROR_INVALID_PARAM;
    }

    do {
        prev_mask = *p_mask;
        new_mask = prev_mask | NRFX_BIT(flag);
    } while (!NRFX_ATOMIC_CAS(p_mask, prev_mask, new_mask));

    return NRFX_SUCCESS;
}

