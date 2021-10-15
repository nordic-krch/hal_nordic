/*$$$LICENCE_NORDIC_STANDARD<2015>$$$*/

#include <nrfx.h>

#if NRFX_CHECK(NRFX_UARTE_ENABLED)

#if !(NRFX_CHECK(NRFX_UARTE0_ENABLED) || \
      NRFX_CHECK(NRFX_UARTE1_ENABLED) || \
      NRFX_CHECK(NRFX_UARTE2_ENABLED) || \
      NRFX_CHECK(NRFX_UARTE3_ENABLED))
#error "No enabled UARTE instances. Check <nrfx_config.h>."
#endif

#include <nrfx_uarte.h>
#include "prs/nrfx_prs.h"
#include <hal/nrf_gpio.h>
#include <string.h>

#define NRFX_LOG_MODULE UARTE
#include <nrfx_log.h>

#define UARTEX_LENGTH_VALIDATE(peripheral, drv_inst_idx, len1, len2)     \
    (((drv_inst_idx) == NRFX_CONCAT_3(NRFX_, peripheral, _INST_IDX)) && \
     NRFX_EASYDMA_LENGTH_VALIDATE(peripheral, len1, len2))

#if NRFX_CHECK(NRFX_UARTE0_ENABLED)
#define UARTE0_LENGTH_VALIDATE(...)  UARTEX_LENGTH_VALIDATE(UARTE0, __VA_ARGS__)
#else
#define UARTE0_LENGTH_VALIDATE(...)  0
#endif

#if NRFX_CHECK(NRFX_UARTE1_ENABLED)
#define UARTE1_LENGTH_VALIDATE(...)  UARTEX_LENGTH_VALIDATE(UARTE1, __VA_ARGS__)
#else
#define UARTE1_LENGTH_VALIDATE(...)  0
#endif

#if NRFX_CHECK(NRFX_UARTE2_ENABLED)
#define UARTE2_LENGTH_VALIDATE(...)  UARTEX_LENGTH_VALIDATE(UARTE2, __VA_ARGS__)
#else
#define UARTE2_LENGTH_VALIDATE(...)  0
#endif

#if NRFX_CHECK(NRFX_UARTE3_ENABLED)
#define UARTE3_LENGTH_VALIDATE(...)  UARTEX_LENGTH_VALIDATE(UARTE3, __VA_ARGS__)
#else
#define UARTE3_LENGTH_VALIDATE(...)  0
#endif

#define UARTE_LENGTH_VALIDATE(drv_inst_idx, length)     \
    (UARTE0_LENGTH_VALIDATE(drv_inst_idx, length, 0) || \
     UARTE1_LENGTH_VALIDATE(drv_inst_idx, length, 0) || \
     UARTE2_LENGTH_VALIDATE(drv_inst_idx, length, 0) || \
     UARTE3_LENGTH_VALIDATE(drv_inst_idx, length, 0))

#define UARTE_HW_RX_FIFO_SIZE 5

#define UARTE_FLAG_RX_ENABLED NRFX_BIT(0)
#define UARTE_FLAG_TX_LOCKED NRFX_BIT(1)
#define UARTE_FLAG_RX_STOP_ON_END NRFX_BIT(2)
#define UARTE_FLAG_RX_CONT NRFX_BIT(3)

typedef struct
{
    uint8_t *                 p_buf;
    uint8_t *                 p_next_buf;
    uint8_t *                 p_active_buf;
    size_t                    len;
    size_t                    next_len;
    uint8_t                   flush_buf[UARTE_HW_RX_FIFO_SIZE];
    uint8_t                   flush_cnt;
    uint8_t                   offset;
} uarte_rx_data_t;

typedef struct
{
    uint8_t const * p_buf;
    uint8_t const * p_next_buf;
    size_t          len;
    int             amount;
    bool            stop_on_end;
    uint8_t         poll_out_byte;
    bool            locked;
} uarte_tx_data_t;

typedef struct
{
    void                     * p_context;
    nrfx_uarte_event_handler_t handler;
    uarte_rx_data_t            rx;
    uarte_tx_data_t            tx;
    nrfx_drv_state_t           state;
    nrfx_atomic_t              flags;
} uarte_control_block_t;

static uarte_control_block_t m_cb[NRFX_UARTE_ENABLED_COUNT];

static const uint32_t rx_int_mask = NRF_UARTE_INT_ERROR_MASK |
                                    NRF_UARTE_INT_ENDRX_MASK |
                                    NRF_UARTE_INT_RXTO_MASK |
                                    NRF_UARTE_INT_RXSTARTED_MASK;

static void apply_config(nrfx_uarte_t        const * p_instance,
                         nrfx_uarte_config_t const * p_config)
{
    const nrfx_uarte_psel_config_t *p_psel_config = p_config->p_psel_config;
    if (p_psel_config)
    {
	if (p_psel_config->gpio_config)
	{
            if (p_psel_config->tx_pin != NRF_UARTE_PSEL_DISCONNECTED)
            {
                nrf_gpio_pin_set(p_psel_config->tx_pin);
                nrf_gpio_cfg_output(p_psel_config->tx_pin);
            }
            if (p_psel_config->rx_pin != NRF_UARTE_PSEL_DISCONNECTED)
            {
                nrf_gpio_cfg_input(p_psel_config->rx_pin, NRF_GPIO_PIN_NOPULL);
            }

            if (p_config->hal_cfg.hwfc == NRF_UARTE_HWFC_ENABLED)
            {
                if (p_psel_config->cts_pin != NRF_UARTE_PSEL_DISCONNECTED)
                {
                    nrf_gpio_cfg_input(p_psel_config->cts_pin, NRF_GPIO_PIN_NOPULL);
                }
                if (p_psel_config->rts_pin != NRF_UARTE_PSEL_DISCONNECTED)
                {
                    nrf_gpio_pin_set(p_psel_config->rts_pin);
                    nrf_gpio_cfg_output(p_psel_config->rts_pin);
                }
            }
	}

        nrf_uarte_txrx_pins_set(p_instance->p_reg, p_psel_config->tx_pin, p_psel_config->rx_pin);
        nrf_uarte_hwfc_pins_set(p_instance->p_reg, p_psel_config->rts_pin, p_psel_config->cts_pin);
    }

    nrf_uarte_baudrate_set(p_instance->p_reg, p_config->baudrate);
    nrf_uarte_configure(p_instance->p_reg, &p_config->hal_cfg);
}

static void pins_to_default(nrfx_uarte_t const * p_instance)
{
    /* Reset pins to default states */
    uint32_t txd;
    uint32_t rxd;
    uint32_t rts;
    uint32_t cts;

    txd = nrf_uarte_tx_pin_get(p_instance->p_reg);
    rxd = nrf_uarte_rx_pin_get(p_instance->p_reg);
    rts = nrf_uarte_rts_pin_get(p_instance->p_reg);
    cts = nrf_uarte_cts_pin_get(p_instance->p_reg);
    nrf_uarte_txrx_pins_disconnect(p_instance->p_reg);
    nrf_uarte_hwfc_pins_disconnect(p_instance->p_reg);

    if (txd != NRF_UARTE_PSEL_DISCONNECTED)
    {
        nrf_gpio_cfg_default(txd);
    }
    if (rxd != NRF_UARTE_PSEL_DISCONNECTED)
    {
        nrf_gpio_cfg_default(rxd);
    }
    if (cts != NRF_UARTE_PSEL_DISCONNECTED)
    {
        nrf_gpio_cfg_default(cts);
    }
    if (rts != NRF_UARTE_PSEL_DISCONNECTED)
    {
        nrf_gpio_cfg_default(rts);
    }
}

static void apply_workaround_for_enable_anomaly(nrfx_uarte_t const * p_instance)
{
#if defined(NRF5340_XXAA_APPLICATION) || defined(NRF5340_XXAA_NETWORK) || defined(NRF9160_XXAA)
    // Apply workaround for anomalies:
    // - nRF9160 - anomaly 23
    // - nRF5340 - anomaly 44
    volatile uint32_t const * rxenable_reg =
        (volatile uint32_t *)(((uint32_t)p_instance->p_reg) + 0x564);
    volatile uint32_t const * txenable_reg =
        (volatile uint32_t *)(((uint32_t)p_instance->p_reg) + 0x568);

    if (*txenable_reg == 1)
    {
        nrf_uarte_task_trigger(p_instance->p_reg, NRF_UARTE_TASK_STOPTX);
    }

    if (*rxenable_reg == 1)
    {
        nrf_uarte_enable(p_instance->p_reg);
        nrf_uarte_task_trigger(p_instance->p_reg, NRF_UARTE_TASK_STOPRX);

        bool workaround_succeded;
        // The UARTE is able to receive up to four bytes after the STOPRX task has been triggered.
        // On lowest supported baud rate (1200 baud), with parity bit and two stop bits configured
        // (resulting in 12 bits per data byte sent), this may take up to 40 ms.
        NRFX_WAIT_FOR(*rxenable_reg == 0, 40000, 1, workaround_succeded);
        if (!workaround_succeded)
        {
            NRFX_LOG_ERROR("Failed to apply workaround for instance with base address: %p.",
                           (void *)p_instance->p_reg);
        }

        (void)nrf_uarte_errorsrc_get_and_clear(p_instance->p_reg);
        nrf_uarte_disable(p_instance->p_reg);
    }
#else
    (void)(p_instance);
#endif // defined(NRF5340_XXAA_APPLICATION) || defined(NRF5340_XXAA_NETWORK) || defined(NRF9160_XXAA)
}


/* Function returns true if new transfer can be started. Since TXSTOPPED
 * (and ENDTX) is cleared before triggering new transfer, TX is ready for new
 * transfer if any event is set.
 */
static bool is_tx_ready(NRF_UARTE_Type * p_uarte, bool stop_or_end)
{
    return nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_TXSTOPPED) ||
        (!stop_or_end ?
               nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDTX) : 0);
}

static void prepare_rx(NRF_UARTE_Type * p_uarte)
{

    /**
     * Stop any currently running RX operations. This can occur when a
     * bootloader sets up the UART hardware and does not clean it up
     * before jumping to the next application.
     */
    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXSTARTED))
    {
        nrf_uarte_enable(p_uarte);
        nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPRX);
        while (!nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXTO) &&
               !nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ERROR))
        {
            /* Busy wait for event to register */
        }
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED);
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXTO);
        nrf_uarte_disable(p_uarte);
    }
}

static void prepare_tx(NRF_UARTE_Type * p_uarte)
{
    uint8_t dummy;

    nrf_uarte_enable(p_uarte);

    /* Set TXSTOPPED event by requesting fake (zero-length) transfer.
     * Pointer to RAM variable (data->tx_buffer) is set because otherwise
     * such operation may result in HardFault or RAM corruption.
     */
    nrf_uarte_tx_buffer_set(p_uarte, &dummy, 0);
    nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTTX);

    /* switch off transmitter to save an energy */
    nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPTX);

    while (!is_tx_ready(p_uarte, false))
    {}

    nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDTX);

    nrf_uarte_disable(p_uarte);
}

nrfx_err_t nrfx_uarte_init(nrfx_uarte_t const *        p_instance,
                           nrfx_uarte_config_t const * p_config,
                           nrfx_uarte_event_handler_t  event_handler)
{
    NRFX_ASSERT(p_config);
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        return NRFX_ERROR_INVALID_STATE;
    }

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    static nrfx_irq_handler_t const irq_handlers[NRFX_UARTE_ENABLED_COUNT] = {
        #if NRFX_CHECK(NRFX_UARTE0_ENABLED)
        nrfx_uarte_0_irq_handler,
        #endif
        #if NRFX_CHECK(NRFX_UARTE1_ENABLED)
        nrfx_uarte_1_irq_handler,
        #endif
        #if NRFX_CHECK(NRFX_UARTE2_ENABLED)
        nrfx_uarte_2_irq_handler,
        #endif
        #if NRFX_CHECK(NRFX_UARTE3_ENABLED)
        nrfx_uarte_3_irq_handler,
        #endif
    };
    if (nrfx_prs_acquire(p_instance->p_reg,
            irq_handlers[p_instance->drv_inst_idx]) != NRFX_SUCCESS)
    {
        err_code = NRFX_ERROR_BUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif // NRFX_CHECK(NRFX_PRS_ENABLED)

    memset(p_cb, 0, sizeof(uarte_control_block_t));
    apply_config(p_instance, p_config);

    apply_workaround_for_enable_anomaly(p_instance);

    p_cb->handler   = event_handler;
    p_cb->p_context = p_config->p_context;
    p_cb->state     = NRFX_DRV_STATE_INITIALIZED;
    p_cb->tx.stop_on_end = p_config->tx_stop_on_end;

    nrf_uarte_event_clear(p_instance->p_reg, NRF_UARTE_EVENT_ENDRX);
    nrf_uarte_event_clear(p_instance->p_reg, NRF_UARTE_EVENT_ENDTX);
    nrf_uarte_event_clear(p_instance->p_reg, NRF_UARTE_EVENT_ERROR);
    nrf_uarte_event_clear(p_instance->p_reg, NRF_UARTE_EVENT_RXTO);
    nrf_uarte_event_clear(p_instance->p_reg, NRF_UARTE_EVENT_TXSTOPPED);
    NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number((void *)p_instance->p_reg),
                          p_config->interrupt_priority);
    NRFX_IRQ_ENABLE(nrfx_get_irq_number((void *)p_instance->p_reg));

    prepare_rx(p_instance->p_reg);
    prepare_tx(p_instance->p_reg);

    uint32_t tx_int_mask = p_config->tx_stop_on_end ? 0 : NRF_UARTE_INT_ENDTX_MASK;
    uint32_t int_mask = tx_int_mask | ((event_handler) ? rx_int_mask : 0);

    nrf_uarte_int_enable(p_instance->p_reg, int_mask);

    return err_code;
}

void nrfx_uarte_uninit(nrfx_uarte_t const * p_instance)
{
    nrfx_err_t err;
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;

    NRFX_IRQ_DISABLE(nrfx_get_irq_number((void *)p_uarte));

    err = nrfx_uarte_rx_abort(p_instance, true);
    (void)err;

    err = nrfx_uarte_tx_abort(p_instance, true, false);
    (void)err;

    pins_to_default(p_instance);

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    nrfx_prs_release(p_uarte);
#endif

    p_cb->state   = NRFX_DRV_STATE_UNINITIALIZED;
    p_cb->handler = NULL;
    NRFX_LOG_INFO("Instance uninitialized: %d.", p_instance->drv_inst_idx);
}

static void tx_start(NRF_UARTE_Type * p_uarte, const uint8_t *buf, size_t len, bool en_int)
{
    nrf_uarte_tx_buffer_set(p_uarte, buf, len);
    nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDTX);
    nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_TXSTOPPED);

    nrf_uarte_enable(p_uarte);

    if (en_int)
    {
        nrf_uarte_int_enable(p_uarte, NRF_UARTE_INT_TXSTOPPED_MASK);
    }

    nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTTX);
}

static bool is_rx_active(uarte_control_block_t * p_cb)
{
    return (p_cb->flags & UARTE_FLAG_RX_ENABLED) ? true : false;
}

/* Must be called with interrupts locked. */
static void disable_hw_from_tx(NRF_UARTE_Type *        p_uarte,
                                  uarte_control_block_t * p_cb)
{
    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_TXSTOPPED) && !is_rx_active(p_cb))
    {
        nrf_uarte_disable(p_uarte);
    }
}

/* Block until transfer is completed. Disable UARTE if RX is not active. */
static void block_on_tx(NRF_UARTE_Type *        p_uarte,
                        uarte_control_block_t * p_cb,
                        bool lock)
{
    while (!is_tx_ready(p_uarte, p_cb->tx.stop_on_end))
    {}

    uint32_t key = NRFX_IRQ_LOCK();

    if (lock)
    {
        NRFX_ATOMIC_FETCH_OR(&p_cb->flags, UARTE_FLAG_TX_LOCKED);
    }

    if (!p_cb->tx.stop_on_end)
    {
        if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDTX))
        {
            nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPTX);
            while (!is_tx_ready(p_uarte, false))
            {}
        }
        else
        {
            NRFX_IRQ_UNLOCK(key);
            return;
        }
    }
    else if (!nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_TXSTOPPED))
    {
        NRFX_IRQ_UNLOCK(key);
        return;
    }

    disable_hw_from_tx(p_uarte, p_cb);
    NRFX_IRQ_UNLOCK(key);
}


nrfx_err_t nrfx_uarte_tx(nrfx_uarte_t const * p_instance,
                         uint8_t const *      p_data,
                         size_t               length)
{
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;
    NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(p_data);
    NRFX_ASSERT(length > 0);
    NRFX_ASSERT(UARTE_LENGTH_VALIDATE(p_instance->drv_inst_idx, length));

    nrfx_err_t err_code;

    // EasyDMA requires that transfer buffers are placed in DataRAM,
    // signal error if the are not.
    if (!nrfx_is_in_ram(p_data))
    {
        err_code = NRFX_ERROR_INVALID_ADDR;
        NRFX_LOG_WARNING("Address in ram");
        return err_code;
    }

    NRFX_CRITICAL_SECTION_ENTER();
    if (p_cb->tx.len)
    {
        err_code = NRFX_ERROR_BUSY;
    }
    else
    {
        err_code = NRFX_SUCCESS;
        p_cb->tx.len = length;
        if (is_tx_ready(p_uarte, p_cb->tx.stop_on_end))
        {
            p_cb->tx.p_buf = p_data;
            p_cb->tx.amount = -1;
            tx_start(p_uarte, p_data, length, p_cb->handler != NULL);
        }
        else if (p_cb->handler)
        {
            p_cb->tx.p_next_buf = p_data;
            p_cb->tx.p_buf = NULL;
            nrf_uarte_int_enable(p_uarte, NRF_UARTE_INT_TXSTOPPED_MASK);
        }
    }
    NRFX_CRITICAL_SECTION_EXIT();

    NRFX_LOG_INFO("Transfer buf:%p len:%d.", p_data, length);
    NRFX_LOG_DEBUG("Tx data:");
    NRFX_LOG_HEXDUMP_DEBUG(p_data, length);

    if ((p_cb->handler == NULL) && (err_code == NRFX_SUCCESS))
    {
        if (p_cb->tx.p_buf)
        {

            block_on_tx(p_uarte, p_cb, false);
            // Check if transfer got aborted.
            if (length > nrf_uarte_tx_amount_get(p_uarte))
            {
                err_code = length == (size_t)p_cb->tx.amount ? NRFX_SUCCESS : NRFX_ERROR_FORBIDDEN;
            }
        }
        else
        {
            err_code = NRFX_ERROR_FORBIDDEN;
        }

        p_cb->tx.len = 0;
        p_cb->tx.p_buf = NULL;
    }

    return err_code;
}

/* Wait until the transmitter is in the idle state. When this function returns,
 * IRQ's are locked with the returned key (on success).
 *
 * @retval NRFX_SUCCESS Interrupts locked and transmitter ready for new transfer.
 * @retval NRFX_ERROR_TIMEOUT Transmitter was not ready, interrupts are not locked on return.
 */
static nrfx_err_t wait_tx_ready(NRF_UARTE_Type        * p_uarte,
                                uarte_control_block_t * p_cb,
                                uint32_t              * p_key,
                                uint32_t                timeout_us)
{
    NRFX_ASSERT(timeout_us < INT32_MAX);
    uint32_t key;
    int32_t t = timeout_us;
    uint32_t chunk_us = 100;
    uarte_tx_data_t * p_tx_data = &p_cb->tx;

    do {
        /* wait arbitrary time before back off. */
        bool res;

        NRFX_WAIT_FOR(is_tx_ready(p_uarte, p_tx_data->stop_on_end) ||
                     (p_cb->flags & UARTE_FLAG_TX_LOCKED), chunk_us, 1, res);

        if (res)
        {
            key = NRFX_IRQ_LOCK();
	    /* It may happen that another context disabled uart while pending
	     * for it to become ready. In that case return with an error since
	     * transmitting is not possible.
	     */
            if (p_cb->flags & UARTE_FLAG_TX_LOCKED)
	    {
		    NRFX_IRQ_UNLOCK(key);
		    return NRFX_ERROR_FORBIDDEN;
	    }

            if (is_tx_ready(p_uarte, p_tx_data->stop_on_end))
            {
                /* If there was active TX transfer and it was not
                 * handled (because we are in higher priority) then
                 * collect amount to pass it to TX_DONE event when
                 * event is eventually handled.
                 */
                if (p_tx_data->len && (p_tx_data->amount < 0))
                {
                    p_tx_data->amount = nrf_uarte_tx_amount_get(p_uarte);
                }
                break;
            }

            NRFX_IRQ_UNLOCK(key);
        }

        t -= chunk_us;
        /* todo sleep in thread context. */
    } while (!timeout_us || t > 0);


    if (t <= 0)
    {
        return NRFX_ERROR_TIMEOUT;
    }

    *p_key = key;

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_uarte_poll_out(nrfx_uarte_t const * p_instance, uint8_t byte, uint32_t timeout_us)
{
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;
    uint32_t key;
    nrfx_err_t err = wait_tx_ready(p_instance->p_reg, p_cb, &key, timeout_us);
    if (err != NRFX_SUCCESS)
    {
         return err;
    }

    p_cb->tx.poll_out_byte = byte;
    tx_start(p_uarte, &p_cb->tx.poll_out_byte, 1, p_cb->handler != NULL);

    NRFX_IRQ_UNLOCK(key);


    if (p_cb->handler == NULL)
    {
        block_on_tx(p_uarte, p_cb, false);
    }

    return NRFX_SUCCESS;
}

bool nrfx_uarte_tx_in_progress(nrfx_uarte_t const * p_instance)
{
    return (m_cb[p_instance->drv_inst_idx].tx.len != 0);
}

nrfx_err_t nrfx_uarte_tx_abort(nrfx_uarte_t const * p_instance, bool sync, bool lock)
{
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;

    if (sync)
    {
        uint32_t mask = nrf_uarte_int_enable_check(p_uarte, NRF_UARTE_INT_ENDTX_MASK);

        nrf_uarte_int_disable(p_uarte, NRF_UARTE_INT_TXSTOPPED_MASK | NRF_UARTE_INT_ENDTX_MASK);
        nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPTX);

        block_on_tx(p_uarte, p_cb, lock);
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDTX);
        nrf_uarte_int_enable(p_uarte, mask);
        p_cb->tx.len = 0;
    }
    else
    {
        if (p_cb->tx.len == 0)
        {
            return NRFX_ERROR_INVALID_STATE;
        }

        if (lock)
        {
            NRFX_ATOMIC_FETCH_OR(&p_cb->flags, UARTE_FLAG_TX_LOCKED);
        }

        nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPTX);
    }

    NRFX_LOG_INFO("TX transaction aborted.");

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_uarte_tx_unlock(nrfx_uarte_t const * p_instance)
{
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    uint32_t prev = NRFX_ATOMIC_FETCH_AND(&p_cb->flags, ~UARTE_FLAG_TX_LOCKED);

    return (prev & UARTE_FLAG_TX_LOCKED) ? NRFX_SUCCESS : NRFX_ERROR_INVALID_STATE;
}

static void user_handler(uarte_control_block_t * p_cb, nrfx_uarte_evt_type_t type)
{
    nrfx_uarte_event_t event = {
        .type = type
    };

    p_cb->handler(&event, p_cb->p_context);
}

static void user_handler_on_rx_disabled(uarte_control_block_t * p_cb, size_t flush_cnt)
{
    nrfx_uarte_event_t event = {
        .type = NRFX_UARTE_EVT_RX_DISABLED,
        .data = {
            .rx_disabled = {
                .flush_cnt = flush_cnt
            }
        }
    };

    p_cb->handler(&event, p_cb->p_context);
}

static void user_handler_on_error(NRF_UARTE_Type *        p_uarte,
                  uarte_control_block_t * p_cb)
{
    nrfx_uarte_event_t event = {
        .type = NRFX_UARTE_EVT_ERROR,
        .data = {
            .error = {
                .error_mask = nrf_uarte_errorsrc_get_and_clear(p_uarte)
            }
        }
    };

    p_cb->handler(&event, p_cb->p_context);
}

static void user_handler_on_rxtx_done(uarte_control_block_t * p_cb,
               nrfx_uarte_evt_type_t type,
               uint8_t *p_data, size_t len)
{
    nrfx_uarte_event_t event = {
        .type = type,
        .data = {
            .rxtx = {
                .p_data = p_data,
                .bytes = len
            }
        }
    };

    p_cb->handler(&event, p_cb->p_context);
}

static void release_rx(uarte_control_block_t * p_cb)
{
    NRFX_ATOMIC_FETCH_AND(&p_cb->flags,
                          ~(UARTE_FLAG_RX_ENABLED | UARTE_FLAG_RX_STOP_ON_END));
}

static bool is_tx_active(NRF_UARTE_Type * p_uarte)
{
    return !nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_TXSTOPPED) ||
        nrf_uarte_int_enable_check(p_uarte, NRF_UARTE_INT_TXSTOPPED_MASK);
}

static void disable_hw_from_rx(NRF_UARTE_Type * p_uarte)
{
    NRFX_CRITICAL_SECTION_ENTER();

    if (!is_tx_active(p_uarte))
    {
        nrf_uarte_disable(p_uarte);
    }

    NRFX_CRITICAL_SECTION_EXIT();
}

static void on_rx_disabled(NRF_UARTE_Type        * p_uarte,
                           uarte_control_block_t * p_cb,
			   size_t                  flush_cnt)
{
    nrf_uarte_shorts_set(p_uarte, 0);
    disable_hw_from_rx(p_uarte);

    p_cb->rx.p_active_buf = NULL;
    p_cb->rx.p_buf = NULL;
    p_cb->rx.p_next_buf = NULL;
    release_rx(p_cb);
    user_handler_on_rx_disabled(p_cb, flush_cnt);
}

/* Some data may be left in flush buffer. It need to be copied into rx buffer.
 * If flushed data exceeds input buffer rx enabled is terminated. */
static bool rx_flushed_handler(NRF_UARTE_Type * p_uarte, uarte_control_block_t * p_cb)
{
    if (p_cb->rx.flush_cnt > 0)
    {
        if (p_cb->rx.flush_cnt > p_cb->rx.len)
        {
	    uint8_t * p_buf = p_cb->rx.p_buf;
	    size_t len = p_cb->rx.len;

	    p_cb->rx.p_buf = NULL;
	    p_cb->rx.len = 0;
            memcpy(p_buf, p_cb->rx.flush_buf, len);
            p_cb->rx.flush_cnt -= len;
            memmove(p_cb->rx.flush_buf, &p_cb->rx.flush_buf[len], p_cb->rx.flush_cnt);
            user_handler_on_rxtx_done(p_cb, NRFX_UARTE_EVT_RX_DONE, p_buf, len);
	    if (p_cb->flags & UARTE_FLAG_RX_STOP_ON_END)
	    {
                on_rx_disabled(p_uarte, p_cb, 0);
	    }

            return false;
        }
        else
        {
            memcpy(p_cb->rx.p_buf, p_cb->rx.flush_buf, p_cb->rx.flush_cnt);
            p_cb->rx.offset = p_cb->rx.flush_cnt;
            p_cb->rx.flush_cnt = 0;
        }
    }

    return true;
}

nrfx_err_t nrfx_uarte_rx_enable(nrfx_uarte_t const * p_instance,
                                bool stop_on_end, bool cont)
{
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;
    uint32_t prev_flags;

    prev_flags = NRFX_ATOMIC_FETCH_OR(&p_cb->flags, UARTE_FLAG_RX_ENABLED);
    if (prev_flags & UARTE_FLAG_RX_ENABLED)
    {
        return NRFX_ERROR_BUSY;
    }

    nrfx_uarte_rx_int_disable(p_instance);
    nrf_uarte_enable(p_uarte);
    nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);
    nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED);
    nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXTO);

    uint32_t flags = (cont ? UARTE_FLAG_RX_CONT : 0) |
                     (stop_on_end ? UARTE_FLAG_RX_STOP_ON_END : 0);

    NRFX_ATOMIC_FETCH_OR(&p_cb->flags, flags);

    if (p_cb->rx.p_buf == NULL && p_cb->handler)
    {
        user_handler(p_cb, NRFX_UARTE_EVT_RX_BUF_REQUEST);
    }

    // Expecting to get buffer set as a response to the request.
    if (p_cb->rx.p_buf == NULL)
    {
        release_rx(p_cb);
        return NRFX_ERROR_NO_MEM;
    }

    /* Check if instance is still enabled. It might get disabled at some point. */
    if (p_cb->flags & UARTE_FLAG_RX_ENABLED)
    {
        if (!nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXSTARTED))
        {
            /* Manually trigger RX if it was not yet started. */
            nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTRX);
        }

        if (p_cb->handler) {
            nrfx_uarte_rx_int_enable(p_instance);
        }
    }

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_uarte_rx_buffer_set(nrfx_uarte_t const * p_instance,
                            uint8_t * p_data, size_t length)
{
    NRFX_ASSERT(m_cb[p_instance->drv_inst_idx].state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(UARTE_LENGTH_VALIDATE(p_instance->drv_inst_idx, length));
    NRFX_ASSERT(p_data);
    NRFX_ASSERT(length > 0);
    NRFX_ASSERT(UARTE_LENGTH_VALIDATE(p_instance->drv_inst_idx, length));

    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (!nrfx_is_in_ram(p_data))
    {
        return NRFX_ERROR_INVALID_ADDR;
    }

    if (p_cb->handler)
    {
        nrfx_uarte_rx_int_disable(p_instance);
    }

    if (p_cb->rx.p_buf == NULL ||
        (!p_cb->handler && nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDRX)))
    {
        p_cb->rx.p_buf = p_data;
        p_cb->rx.len = length;
        if (p_cb->rx.p_buf)
        {
            nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);
        }
        if (rx_flushed_handler(p_uarte, p_cb))
        {
            nrf_uarte_rx_buffer_set(p_uarte,
                                    &p_cb->rx.p_buf[p_cb->rx.offset],
                                    p_cb->rx.len - p_cb->rx.offset);
            if (p_cb->flags & UARTE_FLAG_RX_ENABLED)
            {
                nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTRX);
            }
        }
    }
    else if (p_cb->rx.p_next_buf == NULL)
    {
        p_cb->rx.p_next_buf = p_data;
        p_cb->rx.next_len = length;

        nrf_uarte_rx_buffer_set(p_uarte, p_data, length);
        if (p_cb->flags & UARTE_FLAG_RX_CONT)
        {
            nrf_uarte_shorts_set(p_uarte, NRF_UARTE_SHORT_ENDRX_STARTRX);
        }
    }
    else
    {
        err_code = NRFX_ERROR_BUSY;
    }

    if (p_cb->handler)
    {
        nrfx_uarte_rx_int_enable(p_instance);
    }

    return err_code;
}

#define FLUSHRX_WORKAROUND 1
static void rx_flush(NRF_UARTE_Type * p_uarte, uarte_control_block_t * p_cb)
{
    /* Flushing RX fifo requires buffer bigger than 4 bytes to empty fifo*/
    uint32_t prev_rx_amount = nrf_uarte_rx_amount_get(p_uarte);

    if (FLUSHRX_WORKAROUND)
    {
        /* There is a HW bug which results in rx amount value not being updated
         * when fifo was empty. It is then hard to determine if fifo contained
         * number of bytes equal to the last transfer or was empty. We try to
         * determine that by watermarking flush buffer to check if it was overwritten.
         * However, if fifo contained amount of bytes equal to last transfer and
         * bytes are equal to watermarking it will be dropped. */
        memset(p_cb->rx.flush_buf, 0xAA, sizeof(p_cb->rx.flush_buf));
    }

    nrf_uarte_rx_buffer_set(p_uarte, m_cb->rx.flush_buf, sizeof(m_cb->rx.flush_buf));
    /* Final part of handling RXTO event is in ENDRX interrupt
     * handler. ENDRX is generated as a result of FLUSHRX task.
     */
    nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);
    nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_FLUSHRX);
    while (!nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDRX))
    {
        /* empty */
    }
    nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED);
    nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);

    /* Previous flush_cnt non-zero value indicates that we are interested in flushed data. */
    uint32_t rx_amount = nrf_uarte_rx_amount_get(p_uarte);

    if (rx_amount > UARTE_HW_RX_FIFO_SIZE)
    {
	    p_cb->rx.flush_cnt = 0;
    }
    else
    {
    	p_cb->rx.flush_cnt = m_cb->rx.flush_cnt ? rx_amount : 0;
    }

    if (FLUSHRX_WORKAROUND)
    {
        if (p_cb->rx.flush_cnt == prev_rx_amount)
        {
            for (size_t i = 0; i < sizeof(p_cb->rx.flush_buf); i++)
            {
                if (p_cb->rx.flush_buf[i] != 0xAA)
                {
                    return;
                }
            }
            p_cb->rx.flush_cnt = 0;
        }
    }
}

static void wait_for_rx_completion(NRF_UARTE_Type *        p_uarte,
                                   uarte_control_block_t * p_cb,
                                   bool wait_for_rxto)
{
    nrf_uarte_shorts_disable(p_uarte, NRF_UARTE_SHORT_ENDRX_STARTRX);
    while(nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDRX) == false)
    {}

    nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);
    if (wait_for_rxto || p_cb->flags & UARTE_FLAG_RX_STOP_ON_END)
    {
        while(nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXTO) == false)
        {}
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXTO);

        rx_flush(p_uarte, p_cb);
        disable_hw_from_rx(p_uarte);
    }

    p_cb->rx.p_buf = NULL;
    p_cb->rx.p_next_buf = NULL;
    p_cb->rx.p_active_buf = NULL;
    release_rx(p_cb);
}

nrfx_err_t rx_abort(NRF_UARTE_Type *        p_uarte,
                    uarte_control_block_t * p_cb, bool sync)
{
    if (!(p_cb->flags & UARTE_FLAG_RX_ENABLED))
    {
        return NRFX_ERROR_INVALID_STATE;
    }

    nrf_uarte_shorts_disable(p_uarte, NRF_UARTE_SHORT_ENDRX_STARTRX);
    NRFX_ATOMIC_FETCH_OR(&p_cb->flags, UARTE_FLAG_RX_STOP_ON_END);

    p_cb->rx.flush_cnt = 0;
    if (sync)
    {
        nrf_uarte_int_disable(p_uarte, rx_int_mask);
        nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPRX);
        wait_for_rx_completion(p_uarte, p_cb, true);
    }
    else
    {
        nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPRX);
    }

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_uarte_rx_abort(nrfx_uarte_t const * p_instance, bool sync)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(m_cb[p_instance->drv_inst_idx].state == NRFX_DRV_STATE_INITIALIZED);

    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;

    return rx_abort(p_uarte, p_cb, sync);
}

nrfx_err_t nrfx_uarte_get_rx(nrfx_uarte_t const * p_instance,
                             uint8_t **           pp_data,
                             size_t *             p_length)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(m_cb[p_instance->drv_inst_idx].state == NRFX_DRV_STATE_INITIALIZED);

    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRF_UARTE_Type * p_uarte = p_instance->p_reg;

    if (p_cb->handler)
    {
        return NRFX_ERROR_NOT_SUPPORTED;
    }

    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDRX))
    {
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);
        *pp_data = p_cb->rx.p_buf;
        *p_length = p_cb->rx.len;
        p_cb->rx.p_buf = NULL;
        return NRFX_SUCCESS;
    }

    return NRFX_ERROR_BUSY;
}

nrfx_err_t nrfx_uarte_rx(nrfx_uarte_t const * p_instance,
                         uint8_t *            p_data,
                         size_t               length,
                         bool                 stop_on_end)
{
    nrfx_err_t err_code = nrfx_uarte_rx_buffer_set(p_instance, p_data, length);
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    if (err_code == NRFX_SUCCESS)
    {
        err_code = nrfx_uarte_rx_enable(p_instance, stop_on_end, true);
        if (err_code == NRFX_ERROR_BUSY)
        {
            err_code = NRFX_SUCCESS;
        }
    }

    if (p_cb->handler == NULL)
    {
        wait_for_rx_completion(p_instance->p_reg, p_cb, false);
        if (length > nrf_uarte_rx_amount_get(p_instance->p_reg))
        {
            err_code = NRFX_ERROR_FORBIDDEN;
        }
    }

    return err_code;
}

bool nrfx_uarte_rx_ready(nrfx_uarte_t const * p_instance)
{
    return nrf_uarte_event_check(p_instance->p_reg, NRF_UARTE_EVENT_ENDRX);
}

void nrfx_uarte_rx_int_disable(nrfx_uarte_t const * p_instance)
{
    nrf_uarte_int_disable(p_instance->p_reg, rx_int_mask);
}

void nrfx_uarte_rx_int_enable(nrfx_uarte_t const * p_instance)
{
    nrf_uarte_int_enable(p_instance->p_reg, rx_int_mask);
}

uint32_t nrfx_uarte_errorsrc_get(nrfx_uarte_t const * p_instance)
{
    nrf_uarte_event_clear(p_instance->p_reg, NRF_UARTE_EVENT_ERROR);
    return nrf_uarte_errorsrc_get_and_clear(p_instance->p_reg);
}

static void rxstarted_irq_handler(uarte_control_block_t * p_cb)
{
    if (p_cb->rx.p_active_buf)
    {
        p_cb->rx.p_active_buf = p_cb->rx.p_next_buf;
    }
    else
    {
        p_cb->rx.p_active_buf = p_cb->rx.p_buf;
    }
    user_handler(p_cb, NRFX_UARTE_EVT_RX_BUF_REQUEST);
}

static void rxto_irq_handler(NRF_UARTE_Type *        p_uarte,
                              uarte_control_block_t * p_cb)
{
    if (p_cb->rx.p_buf)
    {
        user_handler_on_rxtx_done(p_cb, NRFX_UARTE_EVT_RX_DONE, p_cb->rx.p_buf, 0);
        p_cb->rx.p_buf = NULL;
    }

    rx_flush(p_uarte, p_cb);

    on_rx_disabled(p_uarte, p_cb, p_cb->rx.flush_cnt);
}

static void endrx_irq_handler(NRF_UARTE_Type *        p_uarte,
                              uarte_control_block_t * p_cb)
{
    int rx_amount = nrf_uarte_rx_amount_get(p_uarte);
    size_t exp_len = p_cb->rx.len - p_cb->rx.offset;
    bool cont = nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXSTARTED) &&
                (p_cb->flags & UARTE_FLAG_RX_CONT);
    bool aborted = rx_amount < (int)exp_len;

    if (aborted)
    {
        if (cont)
        {
            /* Second buffer aborted when first is not yet handled. */
            nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED);
            user_handler_on_rxtx_done(p_cb, NRFX_UARTE_EVT_RX_DONE,
                                      p_cb->rx.p_buf, p_cb->rx.len);
            p_cb->rx.p_buf = p_cb->rx.p_next_buf;
            p_cb->rx.len = p_cb->rx.next_len;
            p_cb->rx.p_next_buf = NULL;
        }
    }

    user_handler_on_rxtx_done(p_cb, NRFX_UARTE_EVT_RX_DONE,
                              p_cb->rx.p_buf, rx_amount + p_cb->rx.offset);
    p_cb->rx.offset = 0;

    NRFX_CRITICAL_SECTION_ENTER();
    p_cb->rx.p_buf = p_cb->rx.p_next_buf;
    p_cb->rx.len = p_cb->rx.next_len;
    p_cb->rx.p_next_buf = NULL;
    p_cb->rx.next_len = 0;

    nrf_uarte_shorts_set(p_uarte, 0);
    if (p_cb->rx.p_buf == NULL && p_cb->flags & UARTE_FLAG_RX_STOP_ON_END)
    {
        nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPRX);
    }

    p_cb->rx.flush_cnt = sizeof(p_cb->rx.flush_buf);

    NRFX_CRITICAL_SECTION_EXIT();

    /* If next buffer was set but RXSTARTED is not set it may indicate that new
     * buffer was set late (e.g. in the context of the RX_DONE event handler).
     * In that case, it is still possible to continue by manually triggering
     * STARTRX. It must occur before RXTO happens.
     */
    if (p_cb->rx.p_buf && !cont && !aborted)
    {
        nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STARTRX);
        if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXTO))
        {
            nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXTO);
            nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED);
            nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPRX);
            user_handler(p_cb, NRFX_UARTE_EVT_RX_BUF_TOO_LATE);
        }
    }
}

static void pending_tx_handler(NRF_UARTE_Type *  p_uarte,
                               uarte_tx_data_t * p_tx)
{
    /* If there is a pending tx request, it means that uart_tx()
     * was called when there was ongoing uart_poll_out. Handling
     * TXSTOPPED interrupt means that uart_poll_out has completed.
     */
    NRFX_ASSERT(p_tx->p_next_buf);

    NRFX_CRITICAL_SECTION_ENTER();

    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_TXSTOPPED))
    {
        p_tx->p_buf = p_tx->p_next_buf;
        p_tx->p_next_buf = NULL;
        p_tx->amount = -1;
        tx_start(p_uarte, p_tx->p_buf, p_tx->len, true);
    }

    NRFX_CRITICAL_SECTION_EXIT();

}

static void txstopped_irq_handler(NRF_UARTE_Type *        p_uarte,
                                  uarte_control_block_t * p_cb)
{
    nrf_uarte_int_disable(p_uarte, NRF_UARTE_INT_TXSTOPPED_MASK);

    NRFX_CRITICAL_SECTION_ENTER();
    disable_hw_from_tx(p_uarte, p_cb);
    NRFX_CRITICAL_SECTION_EXIT();

    // If no length set, it means that it was poll_out completion.
    if (p_cb->tx.len == 0)
    {
        return;
    }

    // if p_buf is null it indicates that tx setup interrupted poll out and
    // tx buffer is pending.
    if (p_cb->tx.p_buf == NULL)
    {
        pending_tx_handler(p_uarte, &p_cb->tx);
        return;
    }

    size_t amount;
    NRFX_CRITICAL_SECTION_ENTER();
    amount = p_cb->tx.amount >= 0 ? (size_t)p_cb->tx.amount : nrf_uarte_tx_amount_get(p_uarte);
    NRFX_CRITICAL_SECTION_EXIT();

    nrfx_uarte_evt_type_t type = (amount == p_cb->tx.len) ?
            NRFX_UARTE_EVT_TX_DONE : NRFX_UARTE_EVT_TX_ABORTED;

    p_cb->tx.len = 0;
    user_handler_on_rxtx_done(p_cb, type, (uint8_t *)p_cb->tx.p_buf, amount);
}

static void error_irq_handler(NRF_UARTE_Type *        p_uarte,
                              uarte_control_block_t * p_cb)
{
    user_handler_on_error(p_uarte, p_cb);
    rx_abort(p_uarte, p_cb, false);
}

static void endtx_irq_handler(NRF_UARTE_Type * p_uarte)
{
    // Locking since poll_out can interrupt at anytime. In that case we don't
    // want to stop ongoing poll_out.
    NRFX_CRITICAL_SECTION_ENTER();
    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDTX))
    {
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDTX);
        nrf_uarte_task_trigger(p_uarte, NRF_UARTE_TASK_STOPTX);
    }
    NRFX_CRITICAL_SECTION_EXIT();
}

static void uarte_irq_handler(NRF_UARTE_Type *        p_uarte,
                              uarte_control_block_t * p_cb)
{
    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ERROR) &&
        nrf_uarte_int_enable_check(p_uarte, NRF_UARTE_INT_ERROR_MASK))
    {
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ERROR);
        error_irq_handler(p_uarte, p_cb);
    }

    /* ENDRX must be handled before RXSTARTED. If RXSTARTED occurred after check
     * for RXSTARTED in isr (which may happen when UARTE interrupt got preempted),
     * events are not cleared and isr will be called again. RXSTARTED will be handled first.
     */
    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDRX) &&
        nrf_uarte_int_enable_check(p_uarte, NRF_UARTE_INT_ENDRX_MASK))
    {
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_ENDRX);

        endrx_irq_handler(p_uarte, p_cb);
    }

    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXSTARTED) &&
        !nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDRX) &&
        nrf_uarte_int_enable_check(p_uarte, NRF_UARTE_INT_RXSTARTED_MASK))
    {
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXSTARTED);
        rxstarted_irq_handler(p_cb);
    }

    /* RXTO must be handled after ENDRX which should notify the buffer.
     * Skip if ENDRX is set when RXTO is set. It means that
     * ENDRX occurred after check for ENDRX in isr which may happen when
     * UARTE interrupt got preempted. Events are not cleared
     * and isr will be called again. ENDRX will be handled first.
     */
    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_RXTO) &&
        !nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDRX) &&
        nrf_uarte_int_enable_check(p_uarte, NRF_UARTE_INT_RXTO_MASK))
    {
        nrf_uarte_event_clear(p_uarte, NRF_UARTE_EVENT_RXTO);
        rxto_irq_handler(p_uarte, p_cb);
    }

    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_ENDTX)
    && nrf_uarte_int_enable_check(p_uarte, NRF_UARTE_INT_ENDTX_MASK))
    {
        endtx_irq_handler(p_uarte);
    }

    if (nrf_uarte_event_check(p_uarte, NRF_UARTE_EVENT_TXSTOPPED)
    && nrf_uarte_int_enable_check(p_uarte, NRF_UARTE_INT_TXSTOPPED_MASK))
    {
        txstopped_irq_handler(p_uarte, p_cb);
    }
}

#if NRFX_CHECK(NRFX_UARTE0_ENABLED)
void nrfx_uarte_0_irq_handler(void)
{
    uarte_irq_handler(NRF_UARTE0, &m_cb[NRFX_UARTE0_INST_IDX]);
}
#endif

#if NRFX_CHECK(NRFX_UARTE1_ENABLED)
void nrfx_uarte_1_irq_handler(void)
{
    uarte_irq_handler(NRF_UARTE1, &m_cb[NRFX_UARTE1_INST_IDX]);
}
#endif

#if NRFX_CHECK(NRFX_UARTE2_ENABLED)
void nrfx_uarte_2_irq_handler(void)
{
    uarte_irq_handler(NRF_UARTE2, &m_cb[NRFX_UARTE2_INST_IDX]);
}
#endif

#if NRFX_CHECK(NRFX_UARTE3_ENABLED)
void nrfx_uarte_3_irq_handler(void)
{
    uarte_irq_handler(NRF_UARTE3, &m_cb[NRFX_UARTE3_INST_IDX]);
}
#endif

#endif // NRFX_CHECK(NRFX_UARTE_ENABLED)
