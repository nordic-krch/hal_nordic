/*$$$LICENCE_NORDIC_STANDARD<2021>$$$*/
#include <nrfx.h>

#if NRFX_CHECK(NRFX_GPIOTE_ENABLED)

#include <nrfx_gpiote.h>
#include <helpers/nrfx_utils.h>
#include "nrf_bitmask.h"
#include <string.h>

#define NRFX_LOG_MODULE GPIOTE
#include <nrfx_log.h>

#if (GPIO_COUNT == 1)
#define MAX_PIN_NUMBER 32
#elif (GPIO_COUNT == 2)
#define MAX_PIN_NUMBER (32 + P1_PIN_NUM)
#else
#error "Not supported."
#endif

#define UNALLOCATED_HANDLER_ADDRESS ((nrfx_gpiote_evt_handler_t)UINT32_MAX)
#define ALLOCATED_HANDLER_ADDRESS   ((nrfx_gpiote_evt_handler_t)(UINT32_MAX-1))
#define PIN_NOT_USED                (-1)
#define PIN_USED                    (-2)
#define NO_CHANNELS                 (-1)
#define POLARITY_FIELD_POS          (6)
#define POLARITY_FIELD_MASK         (0xC0)

/* Check if every pin can be encoded on provided number of bits. */
NRFX_STATIC_ASSERT(MAX_PIN_NUMBER <= (1 << POLARITY_FIELD_POS));

#define NRFX_GPIOTE_DIR_MSK (NRFX_GPIOTE_INPUT | NRFX_GPIOTE_OUTPUT)

#define NRFX_GPIOTE_INT_CHAN_GET(flags) \
    ((flags >> NRFX_GPIOTE_INT_CHAN_SHIFT) & NRFX_GPIOTE_INT_CHAN_MASK)

/*
 * +-----------+--------------+----------------+--------------------+----------------------+
 * | bit 0     | bit 1        | bit 2          | bit 3              | bits 4-7             |
 * +-----------+--------------+----------------+--------------------+----------------------+
 * | 1: output | input only   | input only     | input:             | 0xF - free pin       |
 * | 0: input  |              |                | 1: use event       |                      |
 * |           | 1:Active Low | 1: Active High | output:            | input:               |
 * |           | 0:no         | 0: no          | 1: use task        | 0xE - no handler     |
 * |           |              |                |                    | 0x0-0xD - handler ID |
 * |           |              |                |                    | output:              |
 * |           |              |                |                    | 0x0-0xe task ID      |
 * +-----------+--------------+----------------+--------------------+----------------------+
 */
#define PIN_FLAG_DIR_MASK NRFX_BIT(0)
#define PIN_FLAG_OUTPUT PIN_FLAG_DIR_MASK
#define PIN_FLAG_INPUT 0

#define PIN_FLAG_ACTIVE_LOW NRFX_BIT(1)
#define PIN_FLAG_ACTIVE_HIGH NRFX_BIT(2)
#define PIN_FLAG_ACTIVE_MASK (PIN_FLAG_ACTIVE_LOW | PIN_FLAG_ACTIVE_HIGH)
#define PIN_FLAG_ACTIVE_BOTH PIN_FLAG_ACTIVE_MASK

/* Valid only if input flag is set, 1: task, event used, 0: port, just output */
#define PIN_FLAG_TE NRFX_BIT(3)

/* If pin set as output then keep_cfg flag is interpretted as indication if task is used. */

#define PIN_HANDLER_SHIFT 4
#define PIN_HANDLER_BITS 4
#define PIN_HANDLER_MASK (NRFX_BIT_MASK(PIN_HANDLER_BITS) << PIN_HANDLER_SHIFT)
#define PIN_FLAG_HANDLER(x) (x << PIN_HANDLER_SHIFT)
#define PIN_FLAG_TASK(x) PIN_FLAG_HANDLER(x)

#define PIN_GET_HANDLER_ID(flags) ((flags & PIN_HANDLER_MASK) >> PIN_HANDLER_SHIFT)
#define PIN_GET_TASK_ID(flags) PIN_GET_HANDLER_ID(flags)

/* Indicating that pin is not used. */
#define PIN_FLAG_UNUSED (NRFX_BIT_MASK(PIN_HANDLER_BITS))

/* Pin in use but no handler attached. */
#define PIN_FLAG_NO_HANDLER (PIN_FLAG_UNUSED - 1)

#define PIN_HANDLER_MAX_COUNT (PIN_FLAG_NO_HANDLER - 1)

#define NRFX_GPIOTE_CONFIG_NUM_OF_EVT_HANDLERS 4

typedef struct
{
    nrfx_gpiote_evt_handler_t handlers[NRFX_GPIOTE_CONFIG_NUM_OF_EVT_HANDLERS];
    nrfx_gpiote_evt_handler_t global_handler;
    uint8_t                   pin_flags[MAX_PIN_NUMBER];
    nrfx_atomic_t             allocated_channels_mask;
    nrfx_atomic_t             allocated_evt_handlers;
    uint8_t                   level_pins[((MAX_PIN_NUMBER)+7) / 8];
    uint8_t                   skip_config[((MAX_PIN_NUMBER)+7) / 8];
#if !defined(NRF_GPIO_LATCH_PRESENT)
    uint32_t                  port_pins[GPIO_COUNT];
#endif
    nrfx_drv_state_t          state;
} gpiote_control_block_t;

static gpiote_control_block_t m_cb;

/** @brief Checks if pin is in use by the driver.
 *
 * @param pin Absolute pin.
 * @return True if pin is in use.
 */
static bool pin_in_use(uint32_t pin)
{
    return m_cb.pin_flags[pin] != PIN_FLAG_HANDLER(PIN_FLAG_UNUSED);
}

/** @brief Check if Task/Event is used.
 *
 * Assuming that pin is in use.
 *
 * @param pin Absolute pin.
 * @return True if pin uses GPIOTE task/event.
 */
static bool pin_in_use_by_te(uint32_t pin)
{
    return (m_cb.pin_flags[pin] & PIN_FLAG_TE) ? true : false;
}

/** @brief Check if port is used.
 *
 * Assuming that pin is in use.
 *
 * @param pin Absolute pin.
 * @return True if pin is using PORT sensing.
 */
static bool pin_in_use_by_port(uint32_t pin)
{
    return (m_cb.pin_flags[pin] & (PIN_FLAG_DIR_MASK | PIN_FLAG_TE)) == PIN_FLAG_INPUT;
}


/** @brief Check if pin is output.
 *
 * Assuming that pin is in use.
 *
 * @param pin Absolute pin.
 * @return True if pin is output.
 */
static bool pin_is_output(uint32_t pin)
{
    return (m_cb.pin_flags[pin] & PIN_FLAG_DIR_MASK) == PIN_FLAG_OUTPUT;
}

/** @brief Check if pin is output controlled by GPIOTE task.
 *
 * @param pin Absolute pin.
 * @return True if pin is task output.
 */
static bool pin_is_task_output(uint32_t pin)
{
    return pin_is_output(pin) && pin_in_use_by_te(pin);
}


/** @brief Check if pin is output controlled by GPIO.
 *
 * @param pin Absolute pin.
 * @return True if pin is GPIO output.
 */
static bool pin_is_raw_output(uint32_t pin)
{
    return pin_is_output(pin) && !pin_in_use_by_te(pin);
}

/** @brief Check if pin is used by the driver and configured as input.
 *
 * @param pin Absolute pin.
 * @return True if pin is configured as input.
 */
static bool pin_is_input(uint32_t pin)
{
    return pin_in_use(pin) && !pin_is_output(pin);
}

static uint32_t pin_ch_get(nrfx_gpiote_pin_t pin)
{
   for (uint32_t i = 0; i < GPIOTE_CH_NUM; i++)
   {
       if (nrf_gpiote_event_pin_get(NRF_GPIOTE, i) == pin)
       {
           return i;
       }
   }

   NRFX_ASSERT(0);
   return 0;
}

/** @brief Uninitialize pin and free TE if used.
 *
 * Pin state is restored to the default state unless marked differently
 */
static nrfx_err_t pin_uninit(nrfx_gpiote_pin_t pin, uint32_t flags)
{
    nrfx_err_t err = NRFX_SUCCESS;

    if (!pin_in_use(pin))
    {
        return NRFX_ERROR_INVALID_PARAM;
    }


    if (pin_in_use_by_te(pin))
    {
        uint32_t ch = pin_ch_get(pin);

        /* te to default */
        nrf_gpiote_te_default(NRF_GPIOTE, ch);
        err = nrfx_flag32_free(&m_cb.allocated_channels_mask, ch);
        if (err != NRFX_SUCCESS)
        {
            return err;
        }
    }
    else
    {
#if !defined(NRF_GPIO_LATCH_PRESENT)
        nrf_bitmask_bit_clear(pin, (uint8_t *)m_cb.port_pins);
#endif
        nrf_gpio_cfg_sense_set(pin, NRF_GPIO_PIN_NOSENSE);
    }

    uint32_t handler_id = PIN_GET_HANDLER_ID(m_cb.pin_flags[pin]);

    if (handler_id != PIN_FLAG_NO_HANDLER)
    {
        m_cb.handlers[handler_id] = NULL;
        err = nrfx_flag32_free(&m_cb.allocated_evt_handlers, handler_id);
    }

    if (!(flags & NRFX_GPIOTE_SKIP_SETUP))
    {
        nrf_gpio_cfg_default(pin);
    }

    m_cb.pin_flags[pin] = PIN_FLAG_HANDLER(PIN_FLAG_UNUSED);

    return err;
}

/** @brief Configure pin as input. */
static void input_config(nrfx_gpiote_pin_t pin, uint32_t flags)
{
    if (!(flags & NRFX_GPIOTE_SKIP_SETUP))
    {
        nrf_gpio_pin_pull_t pull = (flags & NRFX_GPIOTE_PULL_UP) ?
            NRF_GPIO_PIN_PULLUP :
            ((flags & NRFX_GPIOTE_PULL_DOWN) ?
                NRF_GPIO_PIN_PULLDOWN : NRF_GPIO_PIN_NOPULL);

        nrf_gpio_cfg_input(pin, pull);
    }

    m_cb.pin_flags[pin] = PIN_FLAG_INPUT | PIN_FLAG_HANDLER(PIN_FLAG_NO_HANDLER);
}

static inline nrf_gpiote_polarity_t get_polarity(uint32_t flags)
{
    switch (flags & NRFX_GPIOTE_OUT_TASK_ACTION_MASK)
    {
        case NRFX_GPIOTE_OUT_TASK_SET:
            return NRF_GPIOTE_POLARITY_LOTOHI;
        case NRFX_GPIOTE_OUT_TASK_CLR:
            return NRF_GPIOTE_POLARITY_HITOLO;
        default:
            return NRF_GPIOTE_POLARITY_TOGGLE;
    }
}

static inline nrf_gpio_pin_drive_t get_drive(uint32_t flags)
{
    (void)flags;
    /*todo*/
    return NRF_GPIO_PIN_S0S1;
}

static void output_config(nrfx_gpiote_pin_t pin, uint32_t flags)
{
    nrf_gpio_pin_drive_t drive = get_drive(flags);

    if (flags & NRFX_GPIOTE_INIT_HIGH)
    {
        nrf_gpio_pin_set(pin);
    }
    else
    {
        nrf_gpio_pin_clear(pin);
    }

    nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_OUTPUT,
                 (flags & NRFX_GPIOTE_INPUT) ? NRF_GPIO_PIN_INPUT_CONNECT :
                                               NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL, drive, NRF_GPIO_PIN_NOSENSE);

    if (flags & NRFX_GPIOTE_USE_TASK)
    {
        uint32_t chan = (flags >> 13) & 0xf;
        nrf_gpiote_outinit_t outinit = (flags & NRFX_GPIOTE_INIT_HIGH) ?
                        NRF_GPIOTE_INITIAL_VALUE_HIGH :
                        NRF_GPIOTE_INITIAL_VALUE_LOW;

        nrf_gpiote_task_configure(NRF_GPIOTE, chan, pin,
                      get_polarity(flags), outinit);
        m_cb.pin_flags[pin] = PIN_FLAG_OUTPUT | PIN_FLAG_TE | PIN_FLAG_HANDLER(chan);
    }
    else
    {
        m_cb.pin_flags[pin] = PIN_FLAG_OUTPUT | PIN_FLAG_HANDLER(PIN_FLAG_NO_HANDLER);
    }
}

nrfx_err_t nrfx_gpiote_pin_config(nrfx_gpiote_pin_t pin, uint32_t flags)
{
    nrfx_err_t err = NRFX_SUCCESS;

    switch (flags & NRFX_GPIOTE_DIR_MSK)
    {
        case NRFX_GPIOTE_DISCONNECTED:
            err = pin_uninit(pin, flags);
            break;
        case NRFX_GPIOTE_INPUT:
            input_config(pin, flags);
            break;
        default:
            output_config(pin, flags);
            break;
    }

    return err;
}

void nrfx_gpiote_global_callback_set(nrfx_gpiote_evt_handler_t handler)
{
        m_cb.global_handler = handler;
}

static void pin_int_flags_set(nrfx_gpiote_pin_t pin, uint32_t int_flags)
{
    uint8_t flags = m_cb.pin_flags[pin];

    flags &= ~PIN_FLAG_ACTIVE_MASK;
    flags |= (int_flags & NRFX_GPIOTE_INT_LOW) ? PIN_FLAG_ACTIVE_LOW : 0;
    flags |= (int_flags & NRFX_GPIOTE_INT_HIGH) ? PIN_FLAG_ACTIVE_HIGH : 0;

    if (int_flags & NRFX_GPIOTE_INT_EDGE)
    {
        nrf_bitmask_bit_clear(pin, m_cb.level_pins);
    }
    else
    {
        nrf_bitmask_bit_set(pin, m_cb.level_pins);
    }

    if (PIN_GET_HANDLER_ID(flags) == PIN_FLAG_UNUSED)
    {
            flags &= ~PIN_HANDLER_MASK;
            flags |= PIN_FLAG_HANDLER(PIN_FLAG_NO_HANDLER);
    }

    if (int_flags & NRFX_GPIOTE_INT_USE_IN_EVT)
    {
            flags |= PIN_FLAG_TE;
    }
    else
    {
        flags &= ~PIN_FLAG_TE;
#if !defined(NRF_GPIO_LATCH_PRESENT)
        nrf_bitmask_bit_set(pin, (uint8_t *)m_cb.port_pins);
#endif
    }

    m_cb.pin_flags[pin] = flags;
}

/** @brief Set new handler, if handler was not previously set allocate it. */
static nrfx_err_t pin_handler_set(nrfx_gpiote_pin_t pin, nrfx_gpiote_evt_handler_t handler)
{
    nrfx_err_t err;
    uint8_t flags = m_cb.pin_flags[pin];
    uint8_t id;

    if (PIN_GET_HANDLER_ID(flags) == PIN_FLAG_NO_HANDLER)
    {
        /* No handler yet assigned, allocated new handler and assign. */
        err = nrfx_flag32_alloc(&m_cb.allocated_evt_handlers, &id);
        if (err != NRFX_SUCCESS)
        {
            return err;
        }

        flags &= ~PIN_HANDLER_MASK;
        flags |= PIN_FLAG_HANDLER(id);
        m_cb.pin_flags[pin] = flags;
    }
    else
    {
        /* Overwrite existing handler. */
        id = PIN_GET_HANDLER_ID(flags);
        err = NRFX_SUCCESS;
    }

    m_cb.handlers[id] = handler;

    return NRFX_SUCCESS;
}

static nrf_gpiote_polarity_t get_evt_polarity(nrfx_gpiote_pin_t pin)
{
    switch (m_cb.pin_flags[pin] & PIN_FLAG_ACTIVE_BOTH)
    {
        case PIN_FLAG_ACTIVE_BOTH:
            return NRF_GPIOTE_POLARITY_TOGGLE;
        case PIN_FLAG_ACTIVE_HIGH:
            return NRF_GPIOTE_POLARITY_LOTOHI;
        default:
            return NRF_GPIOTE_POLARITY_HITOLO;
    }
}

static void pin_evt_config(nrfx_gpiote_pin_t pin, uint8_t ch)
{
    nrf_gpiote_event_disable(NRF_GPIOTE, ch);
    nrf_gpiote_event_configure(NRF_GPIOTE, ch, pin, get_evt_polarity(pin));
}


static inline nrf_gpio_pin_sense_t get_sense(nrfx_gpiote_pin_t pin)
{
    uint8_t flags = m_cb.pin_flags[pin];

    switch (flags & PIN_FLAG_ACTIVE_MASK)
    {
        case PIN_FLAG_ACTIVE_BOTH:
            return nrf_gpio_pin_read(pin) ? NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH;
        case PIN_FLAG_ACTIVE_LOW:
            return NRF_GPIO_PIN_SENSE_LOW;
        default:
            return NRF_GPIO_PIN_SENSE_HIGH;
    }
}

static void sense_enable(nrfx_gpiote_pin_t pin)
{
    nrf_gpio_cfg_sense_dir(pin, get_sense(pin),
                           (m_cb.pin_flags[pin] & PIN_FLAG_DIR_MASK) == PIN_FLAG_OUTPUT);
}

nrfx_err_t nrfx_gpiote_pin_int_config(nrfx_gpiote_pin_t pin, uint32_t int_flags,
                                        nrfx_gpiote_evt_handler_t handler)
{
    nrfx_err_t err;

    if (int_flags & NRFX_GPIOTE_INT_HANDLER)
    {
        err = pin_handler_set(pin, handler);
        if (err != NRFX_SUCCESS)
        {
            nrfx_gpiote_pin_config(pin, NRFX_GPIOTE_DISCONNECTED);
            return err;
        }
    }

    if (int_flags & NRFX_GPIOTE_INT_CFG_PRESENT)
    {
        pin_int_flags_set(pin, int_flags);
        if (int_flags & NRFX_GPIOTE_INT_USE_IN_EVT)
        {
		uint8_t ch = (int_flags & NRFX_GPIOTE_INT_CH_PRESENT) ?
			NRFX_GPIOTE_INT_CHAN_GET(int_flags) : pin_ch_get(pin);

                pin_evt_config(pin, ch);
        }
    }

    if (int_flags & NRFX_GPIOTE_INT_DISABLE)
    {
        if (m_cb.pin_flags[pin] & PIN_FLAG_TE)
        {
            nrf_gpiote_int_disable(NRF_GPIOTE, NRFX_BIT(pin_ch_get(pin)));
        }
        else
        {
            nrf_gpio_cfg_sense_set(pin, NRF_GPIO_PIN_NOSENSE);
        }
    }

    /* Possible INT_DISABLE but EVT_ENABLE */
    if (int_flags & (NRFX_GPIOTE_INT_ENABLE | NRFX_GPIOTE_EVT_ENABLE))
    {
        if (m_cb.pin_flags[pin] & PIN_FLAG_TE)
        {
            uint32_t ch = pin_ch_get(pin);

            nrf_gpiote_event_enable(NRF_GPIOTE, ch);
            if (int_flags & NRFX_GPIOTE_INT_ENABLE)
            {
		nrf_gpiote_event_clear(NRF_GPIOTE, nrf_gpiote_in_event_get(ch));
                nrf_gpiote_int_enable(NRF_GPIOTE, NRFX_BIT(ch));
            }
        }
        else if (int_flags & NRFX_GPIOTE_INT_ENABLE)
        {
            sense_enable(pin);
        }
    }

    return NRFX_SUCCESS;
}

/* Returns gpiote TE channel associated with the pin */
static int8_t channel_port_get(uint32_t pin)
{
    return PIN_GET_TASK_ID(m_cb.pin_flags[pin]);
}

/* Return handler associated with given pin or null. */
static nrfx_gpiote_evt_handler_t channel_handler_get(nrfx_gpiote_pin_t pin)
{
    uint8_t handler_id = PIN_GET_HANDLER_ID(m_cb.pin_flags[pin]);

    NRFX_ASSERT(handler_id != PIN_FLAG_UNUSED);

    if (handler_id == PIN_FLAG_NO_HANDLER)
    {
       return NULL;
    }

    return m_cb.handlers[handler_id];
}

static nrf_gpiote_polarity_t pin_polarity_get(nrfx_gpiote_pin_t pin)
{
    switch (m_cb.pin_flags[pin] & PIN_FLAG_ACTIVE_MASK)
    {
        case PIN_FLAG_ACTIVE_LOW:
            return NRF_GPIOTE_POLARITY_HITOLO;
        case PIN_FLAG_ACTIVE_HIGH:
            return NRF_GPIOTE_POLARITY_LOTOHI;
        default:
            return NRF_GPIOTE_POLARITY_TOGGLE;
    }
}


nrfx_err_t nrfx_gpiote_init(uint8_t interrupt_priority)
{
    nrfx_err_t err_code;

    if (m_cb.state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = NRFX_ERROR_INVALID_STATE;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    for (uint32_t i = 0; i < MAX_PIN_NUMBER; i++)
    {
        if (nrf_gpio_pin_present_check(i))
        {
            m_cb.pin_flags[i] = PIN_FLAG_HANDLER(PIN_FLAG_UNUSED);
        }

    }

    memset(m_cb.skip_config, 0, sizeof(m_cb.skip_config));
    memset(m_cb.level_pins, 0, sizeof(m_cb.level_pins));

    if (interrupt_priority != 0xff)
    {
        NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(NRF_GPIOTE), interrupt_priority);
        NRFX_IRQ_ENABLE(nrfx_get_irq_number(NRF_GPIOTE));
    }
    nrf_gpiote_event_clear(NRF_GPIOTE, NRF_GPIOTE_EVENT_PORT);
    nrf_gpiote_int_enable(NRF_GPIOTE, GPIOTE_INTENSET_PORT_Msk);
    m_cb.state = NRFX_DRV_STATE_INITIALIZED;
    m_cb.allocated_channels_mask = NRFX_GPIOTE_APP_CHANNELS_MASK;
    m_cb.allocated_evt_handlers = NRFX_BIT_MASK(NRFX_GPIOTE_CONFIG_NUM_OF_EVT_HANDLERS);

    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}


bool nrfx_gpiote_is_init(void)
{
    return (m_cb.state != NRFX_DRV_STATE_UNINITIALIZED) ? true : false;
}


void nrfx_gpiote_uninit(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    uint32_t i;

    for (i = 0; i < MAX_PIN_NUMBER; i++)
    {
        if (nrf_gpio_pin_present_check(i) && pin_in_use(i))
        {
            nrfx_gpiote_pin_config(i, NRFX_GPIOTE_DISCONNECTED);
        }
    }
    m_cb.state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

static inline bool is_app_channel(uint8_t index)
{
    return NRFX_GPIOTE_APP_CHANNELS_MASK & (1UL << index);
}

nrfx_err_t nrfx_gpiote_channel_free(uint8_t channel)
{
    return nrfx_flag32_free(&m_cb.allocated_channels_mask, channel);
}

nrfx_err_t nrfx_gpiote_channel_alloc(uint8_t * p_channel)
{
    return nrfx_flag32_alloc(&m_cb.allocated_channels_mask, p_channel);
}

nrfx_err_t nrfx_gpiote_out_init(nrfx_gpiote_pin_t                pin,
                                nrfx_gpiote_out_config_t const * p_config)
{
    uint8_t ch;
    nrfx_err_t err;

    if (p_config->task_pin) {
        err = nrfx_gpiote_channel_alloc(&ch);
        if (err != NRFX_SUCCESS) {
            return err;
        }

    }

    err = nrfx_gpiote_out_prealloc_init(pin, p_config, ch);
    if (err == NRFX_ERROR_BUSY && p_config->task_pin) {
        nrfx_gpiote_channel_free(ch);
    }

    return err;
}

nrfx_err_t nrfx_gpiote_out_prealloc_init(nrfx_gpiote_pin_t                pin,
                                         nrfx_gpiote_out_config_t const * p_config,
                                         uint8_t                          channel)
{
    uint32_t flags;

    if (pin_in_use(pin)) {
        return NRFX_ERROR_BUSY;
    }

    flags = NRFX_GPIOTE_OUTPUT |
        ((p_config->init_state == NRF_GPIOTE_INITIAL_VALUE_LOW) ?
            NRFX_GPIOTE_INIT_LOW : NRFX_GPIOTE_INIT_HIGH);

    if (p_config->task_pin)
    {
        flags |= NRFX_GPIOTE_OUT_TASK(channel) |
            ((p_config->action == NRF_GPIOTE_POLARITY_LOTOHI) ?
            NRFX_GPIOTE_OUT_TASK_SET :
             ((p_config->action == NRF_GPIOTE_POLARITY_HITOLO) ?
              NRFX_GPIOTE_OUT_TASK_CLR : NRFX_GPIOTE_OUT_TASK_TOGGLE));
    }

    return nrfx_gpiote_pin_config(pin, flags);
}

void nrfx_gpiote_out_uninit(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));

    nrfx_gpiote_pin_config(pin, NRFX_GPIOTE_DISCONNECTED);
}


void nrfx_gpiote_out_set(nrfx_gpiote_pin_t pin)
{
    (void)pin_is_raw_output; /* Add to avoid compiler warnings when asserts disabled.*/
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_raw_output(pin));

    nrf_gpio_pin_set(pin);
}


void nrfx_gpiote_out_clear(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_raw_output(pin));

    nrf_gpio_pin_clear(pin);
}


void nrfx_gpiote_out_toggle(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_raw_output(pin));

    nrf_gpio_pin_toggle(pin);
}

void nrfx_gpiote_out_task_enable(nrfx_gpiote_pin_t pin)
{
    (void)pin_is_task_output; /* Add to avoid compiler warnings when asserts disabled.*/
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(pin));

    nrf_gpiote_task_enable(NRF_GPIOTE, (uint32_t)PIN_GET_TASK_ID(m_cb.pin_flags[pin]));
}


void nrfx_gpiote_out_task_disable(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(pin));

    nrf_gpiote_task_disable(NRF_GPIOTE, (uint32_t)channel_port_get(pin));
}


nrf_gpiote_task_t nrfx_gpiote_out_task_get(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(pin));

    return  nrf_gpiote_out_task_get((uint8_t)channel_port_get(pin));
}


uint32_t nrfx_gpiote_out_task_addr_get(nrfx_gpiote_pin_t pin)
{
    nrf_gpiote_task_t task = nrfx_gpiote_out_task_get(pin);
    return nrf_gpiote_task_address_get(NRF_GPIOTE, task);
}


#if defined(GPIOTE_FEATURE_SET_PRESENT)
nrf_gpiote_task_t nrfx_gpiote_set_task_get(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(pin));

    return nrf_gpiote_set_task_get((uint8_t)channel_port_get(pin));
}


uint32_t nrfx_gpiote_set_task_addr_get(nrfx_gpiote_pin_t pin)
{
    nrf_gpiote_task_t task = nrfx_gpiote_set_task_get(pin);
    return nrf_gpiote_task_address_get(NRF_GPIOTE, task);
}
#endif // defined(GPIOTE_FEATURE_SET_PRESENT)


#if defined(GPIOTE_FEATURE_CLR_PRESENT)
nrf_gpiote_task_t nrfx_gpiote_clr_task_get(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(pin));

    return nrf_gpiote_clr_task_get((uint8_t)channel_port_get(pin));
}


uint32_t nrfx_gpiote_clr_task_addr_get(nrfx_gpiote_pin_t pin)
{
    nrf_gpiote_task_t task = nrfx_gpiote_clr_task_get(pin);
    return nrf_gpiote_task_address_get(NRF_GPIOTE, task);
}
#endif // defined(GPIOTE_FEATURE_CLR_PRESENT)


void nrfx_gpiote_out_task_force(nrfx_gpiote_pin_t pin, uint8_t state)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(pin));

    nrf_gpiote_outinit_t init_val =
        state ? NRF_GPIOTE_INITIAL_VALUE_HIGH : NRF_GPIOTE_INITIAL_VALUE_LOW;
    nrf_gpiote_task_force(NRF_GPIOTE, (uint32_t)channel_port_get(pin), init_val);
}


void nrfx_gpiote_out_task_trigger(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(pin));

    nrf_gpiote_task_t task = nrf_gpiote_out_task_get((uint8_t)channel_port_get(pin));
    nrf_gpiote_task_trigger(NRF_GPIOTE, task);
}


#if defined(GPIOTE_FEATURE_SET_PRESENT)
void nrfx_gpiote_set_task_trigger(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));
    NRFX_ASSERT(pin_in_use_by_te(pin));

    nrf_gpiote_task_t task = nrf_gpiote_set_task_get((uint8_t)channel_port_get(pin));
    nrf_gpiote_task_trigger(NRF_GPIOTE, task);
}


#endif // defined(GPIOTE_FEATURE_SET_PRESENT)

#if  defined(GPIOTE_FEATURE_CLR_PRESENT)
void nrfx_gpiote_clr_task_trigger(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(pin));
    NRFX_ASSERT(pin_in_use_by_te(pin));

    nrf_gpiote_task_t task = nrf_gpiote_clr_task_get((uint8_t)channel_port_get(pin));
    nrf_gpiote_task_trigger(NRF_GPIOTE, task);
}


#endif // defined(GPIOTE_FEATURE_CLR_PRESENT)


nrfx_err_t nrfx_gpiote_in_init(nrfx_gpiote_pin_t               pin,
                               nrfx_gpiote_in_config_t const * p_config,
                               nrfx_gpiote_evt_handler_t       evt_handler)
{
    uint8_t ch;
    nrfx_err_t err;

    if (p_config->hi_accuracy)
    {
        err = nrfx_gpiote_channel_alloc(&ch);
        if (err != NRFX_SUCCESS)
        {
            return err;
        }
    }
    else
    {
        ch = 0;
    }

    err = nrfx_gpiote_in_prealloc_init(pin, p_config, ch, evt_handler);
    if (err == NRFX_ERROR_NO_MEM)
    {
        nrfx_gpiote_channel_free(ch);
    }

    return err;
}

nrfx_err_t nrfx_gpiote_in_prealloc_init(nrfx_gpiote_pin_t               pin,
                                        nrfx_gpiote_in_config_t const * p_config,
                                        uint8_t                         channel,
                                        nrfx_gpiote_evt_handler_t       evt_handler)
{
    nrfx_err_t err;
    uint32_t flags;

    if (p_config->is_watcher)
    {
           flags = NRFX_GPIOTE_OUTPUT | NRFX_GPIOTE_INPUT;
    }
    else
    {
        flags = NRFX_GPIOTE_INPUT |
            ((p_config->skip_gpio_setup ? NRFX_GPIOTE_SKIP_SETUP : 0) |
            (p_config->pull == NRF_GPIO_PIN_PULLUP) ? NRFX_GPIOTE_PULL_UP :
            ((p_config->pull == NRF_GPIO_PIN_PULLDOWN) ? NRFX_GPIOTE_PULL_DOWN : 0));
    }

    if (p_config->skip_gpio_setup)
    {
        nrf_bitmask_bit_set(pin, m_cb.skip_config);
        flags |= NRFX_GPIOTE_SKIP_SETUP;
    }

    err = nrfx_gpiote_pin_config(pin, flags);
    if (err != NRFX_SUCCESS)
    {
        return err;
    }

    flags = NRFX_GPIOTE_INT_CFG_PRESENT | NRFX_GPIOTE_INT_HANDLER | NRFX_GPIOTE_INT_EDGE |
        ((p_config->sense == NRF_GPIOTE_POLARITY_LOTOHI) ?
        NRFX_GPIOTE_INT_HIGH :
        ((p_config->sense == NRF_GPIOTE_POLARITY_HITOLO) ?
         NRFX_GPIOTE_INT_LOW : NRFX_GPIOTE_INT_LOW | NRFX_GPIOTE_INT_HIGH)) |
        (p_config->hi_accuracy ? NRFX_GPIOTE_INT_CHAN(channel) : 0);


    return nrfx_gpiote_pin_int_config(pin, flags, evt_handler);
}

void nrfx_gpiote_in_event_enable(nrfx_gpiote_pin_t pin, bool int_enable)
{
    uint32_t flags = NRFX_GPIOTE_EVT_ENABLE | (int_enable ? NRFX_GPIOTE_INT_ENABLE : 0);
    nrfx_err_t err = nrfx_gpiote_pin_int_config(pin, flags, NULL /* not used */);
    (void)err;
    NRFX_ASSERT(err == NRFX_SUCCESS);
}


void nrfx_gpiote_in_event_disable(nrfx_gpiote_pin_t pin)
{
    uint32_t flags = NRFX_GPIOTE_INT_DISABLE;
    nrfx_err_t err = nrfx_gpiote_pin_int_config(pin, flags, NULL /* not used */);
    (void)err;
    NRFX_ASSERT(err == NRFX_SUCCESS);
}


void nrfx_gpiote_in_uninit(nrfx_gpiote_pin_t pin)
{
    (void)pin_is_input; /* Added to avoid compiler warnings when asserts are off. */
    NRFX_ASSERT(pin_is_input(pin));
    uint32_t flags = NRFX_GPIOTE_DISCONNECTED;

    if (nrf_bitmask_bit_is_set(pin, m_cb.skip_config))
    {
        flags |= NRFX_GPIOTE_SKIP_SETUP;
    }

    nrfx_err_t err = nrfx_gpiote_pin_config(pin, flags);
    (void)err;
    NRFX_ASSERT(err == NRFX_SUCCESS);
}


bool nrfx_gpiote_in_is_set(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    return nrf_gpio_pin_read(pin) ? true : false;
}


nrf_gpiote_event_t nrfx_gpiote_in_event_get(nrfx_gpiote_pin_t pin)
{
    (void)pin_in_use_by_port; /* Added to avoid compiler warnings when asserts are off. */
    NRFX_ASSERT(nrf_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_input(pin));
    NRFX_ASSERT(pin_in_use_by_port(pin) || pin_in_use_by_te(pin));

    if (pin_in_use_by_te(pin))
    {
        return nrf_gpiote_in_event_get((uint8_t)channel_port_get(pin));
    }

    return NRF_GPIOTE_EVENT_PORT;
}


uint32_t nrfx_gpiote_in_event_addr_get(nrfx_gpiote_pin_t pin)
{
    nrf_gpiote_event_t event = nrfx_gpiote_in_event_get(pin);
    return nrf_gpiote_event_address_get(NRF_GPIOTE, event);
}

static void call_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t polarity)
{
    nrfx_gpiote_evt_handler_t handler = channel_handler_get(pin);

    if (handler)
    {
        handler(pin, polarity);
    }
    if (m_cb.global_handler)
    {
        m_cb.global_handler(pin, polarity);
    }
}

static void cond_call_sense_handler(nrfx_gpiote_pin_t pin,
                                    nrf_gpiote_polarity_t polarity,
                                    nrf_gpio_pin_sense_t sense)
{
    /* Invoke user handler only if the sensed pin level
     * matches its polarity configuration. */
    if (((polarity == NRF_GPIOTE_POLARITY_TOGGLE) ||
         (sense == NRF_GPIO_PIN_SENSE_HIGH && polarity == NRF_GPIOTE_POLARITY_LOTOHI) ||
         (sense == NRF_GPIO_PIN_SENSE_LOW && polarity == NRF_GPIOTE_POLARITY_HITOLO)))
    {
        call_handler(pin, polarity);
    }
}

#if defined(NRF_GPIO_LATCH_PRESENT)
static bool latch_pending_read_and_check(uint32_t * latch)
{
    nrf_gpio_latches_read_and_clear(0, GPIO_COUNT, latch);

    for (uint32_t port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
    {
        if (latch[port_idx])
        {
            /* If any of the latch bits is still set, it means another edge has been captured
             * before or during the interrupt processing. Therefore event-processing loop
             * should be executed again. */
            return true;
        }
    }
    return false;
}

static void port_event_handle(uint32_t * latch)
{
    do {
        for(uint32_t i = 0; i < GPIO_COUNT; i++)
        {
            while (latch[i])
            {
                uint32_t pin = 31 - __CLZ(latch[i]);

		/* Convert to absolute value. */
		pin += 32 * i;
                nrf_gpio_pin_sense_t sense;
                nrf_gpiote_polarity_t polarity;

                nrf_bitmask_bit_clear(pin, latch);
                sense = nrf_gpio_pin_sense_get(pin);
                polarity = pin_polarity_get(pin);

                if (!nrf_bitmask_bit_is_set(pin, m_cb.level_pins))
                {
                    /* Reconfigure sense to the opposite level, so the internal
                     * PINx.DETECT signal can be deasserted. Therefore PORT
                     * event generated again, unless some other PINx.DETECT
                     * signal is still active. */
                    nrf_gpio_pin_sense_t next_sense =
                        (sense == NRF_GPIO_PIN_SENSE_HIGH) ? NRF_GPIO_PIN_SENSE_LOW :
                                         NRF_GPIO_PIN_SENSE_HIGH;
                    nrf_gpio_cfg_sense_set(pin, next_sense);
                }

                /* Try to clear LATCH bit corresponding to currently processed pin.
                 * This may not succeed if the pin's state changed during the interrupt processing
                 * and now it matches the new sense configuration. In such case,
                 * the pin will be processed again in another iteration of the outer loop. */
                nrf_gpio_pin_latch_clear(pin);

                cond_call_sense_handler(pin, polarity, sense);
           }
       }
    } while(latch_pending_read_and_check(latch));
}

#else

static bool input_read_and_check(uint32_t * input, uint32_t * pins_to_check)
{
    bool process_inputs_again;
    uint32_t new_input[GPIO_COUNT];

    nrf_gpio_ports_read(0, GPIO_COUNT, new_input);

    process_inputs_again = false;
    for (uint32_t port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
    {
        /* Execute XOR to find out which inputs have changed. */
        uint32_t input_diff = input[port_idx] ^ new_input[port_idx];
        input[port_idx] = new_input[port_idx];
        if (input_diff)
        {
            /* If any differences among inputs were found, mark those pins
             * to be processed again. */
            pins_to_check[port_idx] = input_diff;
            process_inputs_again = true;
        }
        else
        {
            pins_to_check[port_idx] = 0;
        }
    }
    return process_inputs_again;
}

static void port_event_handle(uint32_t * input)
{
    uint32_t pins_to_check[GPIO_COUNT];

    for (uint32_t port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
    {
        pins_to_check[port_idx] = 0xFFFFFFFF;
    }

    do {
        for (uint32_t i = 0; i < GPIO_COUNT; i++)
        {
            uint32_t port_pin_map = m_cb.port_pins[i];
            uint8_t pin;
            while (port_pin_map)
            {
                pin = 31 - __CLZ(port_pin_map);
                port_pin_map &= ~NRFX_BIT(pin);
                /* Absolute */
                pin += 32 * i;
                if (nrf_bitmask_bit_is_set(pin, pins_to_check))
                {
                    nrf_gpiote_polarity_t polarity;
                    nrf_gpio_pin_sense_t sense;
                    bool pin_state;

                    sense = nrf_gpio_pin_sense_get(pin);
                    polarity = pin_polarity_get(pin);
                    pin_state = nrf_bitmask_bit_is_set(pin, input);

                    /* Process pin further only if its state matches its sense level. */
                    if ((pin_state && (sense == NRF_GPIO_PIN_SENSE_HIGH)) ||
                        (!pin_state && (sense == NRF_GPIO_PIN_SENSE_LOW)) )
                    {
                        /* Reconfigure sense to the opposite level, so the internal PINx.DETECT signal
                         * can be deasserted. Therefore PORT event can be generated again,
                         * unless some other PINx.DETECT signal is still active. */
                        NRFX_LOG_DEBUG("PORT event for pin: %d, polarity: %d.", pin, polarity);
                        if (!nrf_bitmask_bit_is_set(pin, m_cb.level_pins))
                        {
                            nrf_gpio_pin_sense_t next_sense =
                                (sense == NRF_GPIO_PIN_SENSE_HIGH) ? NRF_GPIO_PIN_SENSE_LOW :
                                                                     NRF_GPIO_PIN_SENSE_HIGH;
                            nrf_gpio_cfg_sense_set(pin, next_sense);
                        }

                        cond_call_sense_handler(pin, polarity, sense);
                    }
                }
            }
        }
    } while (input_read_and_check(input, pins_to_check));
}
#endif // defined(NRF_GPIO_LATCH_PRESENT)

static void gpiote_evt_handle(uint32_t mask)
{
    while (mask)
    {
        uint32_t ch = 31 - __CLZ(mask);
        mask &= ~NRFX_BIT(ch);
        nrfx_gpiote_pin_t pin = nrf_gpiote_event_pin_get(NRF_GPIOTE, ch);

        nrf_gpiote_polarity_t polarity = nrf_gpiote_event_polarity_get(NRF_GPIOTE, ch);
        call_handler(pin, polarity);
    }
}

void nrfx_gpiote_irq_handler(void)
{
    uint32_t status            = 0;
    uint32_t input[GPIO_COUNT] = {0};

    /* collect status of all GPIOTE pin events. Processing is done once all are collected and cleared.*/
    uint32_t            i;
    nrf_gpiote_event_t event = NRF_GPIOTE_EVENT_IN_0;
    uint32_t            mask  = (uint32_t)NRF_GPIOTE_INT_IN0_MASK;

    for (i = 0; i < GPIOTE_CH_NUM; i++)
    {
        if (nrf_gpiote_event_check(NRF_GPIOTE, event) &&
            nrf_gpiote_int_enable_check(NRF_GPIOTE, mask))
        {
            nrf_gpiote_event_clear(NRF_GPIOTE, event);
            status |= mask;
        }
        mask <<= 1;
        /* Incrementing to next event, utilizing the fact that events are grouped together
         * in ascending order. */
        event = (nrf_gpiote_event_t)((uint32_t)event + sizeof(uint32_t));
    }

    /* collect PORT status event, if event is set read pins state. Processing is postponed to the
     * end of interrupt. */
    if (nrf_gpiote_event_check(NRF_GPIOTE, NRF_GPIOTE_EVENT_PORT))
    {
        nrf_gpiote_event_clear(NRF_GPIOTE, NRF_GPIOTE_EVENT_PORT);
        status |= (uint32_t)NRF_GPIOTE_INT_PORT_MASK;
#if defined(NRF_GPIO_LATCH_PRESENT)
        nrf_gpio_latches_read_and_clear(0, GPIO_COUNT, input);
#else
        nrf_gpio_ports_read(0, GPIO_COUNT, input);
#endif
    }

    /* Process pin events. */
    gpiote_evt_handle(status & ~NRF_GPIOTE_INT_PORT_MASK);

    /* Process PORT event. */
    if (status & (uint32_t)NRF_GPIOTE_INT_PORT_MASK)
    {
        port_event_handle(input);
    }
}

#endif // NRFX_CHECK(NRFX_GPIOTE_ENABLED)
