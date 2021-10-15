/*$$$LICENCE_NORDIC_STANDARD<2015>$$$*/

#ifndef NRFX_UARTE_H__
#define NRFX_UARTE_H__

#include <nrfx.h>
#include <hal/nrf_uarte.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_uarte UARTE driver
 * @{
 * @ingroup nrf_uarte
 * @brief   UARTE peripheral driver.
 */

/** @brief Structure for the UARTE driver instance. */
typedef struct
{
    NRF_UARTE_Type * p_reg;        ///< Pointer to a structure with UARTE registers.
    uint8_t          drv_inst_idx; ///< Index of the driver instance. For internal use only.
} nrfx_uarte_t;

#ifndef __NRFX_DOXYGEN__
enum {
#if NRFX_CHECK(NRFX_UARTE0_ENABLED)
    NRFX_UARTE0_INST_IDX,
#endif
#if NRFX_CHECK(NRFX_UARTE1_ENABLED)
    NRFX_UARTE1_INST_IDX,
#endif
#if NRFX_CHECK(NRFX_UARTE2_ENABLED)
    NRFX_UARTE2_INST_IDX,
#endif
#if NRFX_CHECK(NRFX_UARTE3_ENABLED)
    NRFX_UARTE3_INST_IDX,
#endif
    NRFX_UARTE_ENABLED_COUNT
};
#endif

/** @brief Macro for creating a UARTE driver instance. */
#define NRFX_UARTE_INSTANCE(id)                               \
{                                                             \
    .p_reg        = NRFX_CONCAT_2(NRF_UARTE, id),             \
    .drv_inst_idx = NRFX_CONCAT_3(NRFX_UARTE, id, _INST_IDX), \
}

/** @brief Types of UARTE driver events. */
typedef enum
{
    NRFX_UARTE_EVT_TX_DONE, ///< Requested TX transfer completed.
    NRFX_UARTE_EVT_RX_DONE, ///< Requested RX transfer completed.
    NRFX_UARTE_EVT_ERROR,   ///< Error reported by UART peripheral.

    NRFX_UARTE_EVT_TX_ABORTED, ///< TX transfer aborted.
    NRFX_UARTE_EVT_RX_BUF_REQUEST, ///< Request for a RX buffer.
    NRFX_UARTE_EVT_RX_DISABLED,   ///< Receiver is disabled.
    NRFX_UARTE_EVT_RX_BUF_TOO_LATE,   ///< RX buffer request handled too late.
} nrfx_uarte_evt_type_t;

/** @brief Structure for the UARTE pin configuration. */
typedef struct
{
    uint32_t tx_pin;     ///< TXD pin number.
    uint32_t rx_pin;     ///< RXD pin number.
    uint32_t cts_pin;    ///< CTS pin number.
    uint32_t rts_pin;    ///< RTS pin number.
    bool     gpio_config;///< Flag indicating if gpio should be configured
} nrfx_uarte_psel_config_t;

/** @brief Structure for the UARTE configuration. */
typedef struct
{
    nrfx_uarte_psel_config_t * p_psel_config;      ///< Pin configuration. If NULL driver skips configuration.
    void *                     p_context;          ///< Context passed to interrupt handler.
    nrf_uarte_baudrate_t       baudrate;           ///< Baud rate.
    uint8_t                    interrupt_priority; ///< Interrupt priority.
    nrf_uarte_config_t         hal_cfg;            ///< Parity, flow control and stop bits settings.
    bool                       tx_stop_on_end;     ///< Flag indicating if the STOPTX task is triggered on the ENDTX event
} nrfx_uarte_config_t;

#if defined(UARTE_CONFIG_STOP_Msk) || defined(__NRFX_DOXYGEN__)
    /** @brief UARTE additional stop bits configuration. */
    #define NRFX_UARTE_DEFAULT_EXTENDED_STOP_CONFIG   \
        .stop = (nrf_uarte_stop_t)NRF_UARTE_STOP_ONE,
#else
    #define NRFX_UARTE_DEFAULT_EXTENDED_STOP_CONFIG
#endif

#if defined(UARTE_CONFIG_PARITYTYPE_Msk) || defined(__NRFX_DOXYGEN__)
    /** @brief UARTE additional parity type configuration. */
    #define NRFX_UARTE_DEFAULT_EXTENDED_PARITYTYPE_CONFIG   \
        .paritytype = NRF_UARTE_PARITYTYPE_EVEN,
#else
    #define NRFX_UARTE_DEFAULT_EXTENDED_PARITYTYPE_CONFIG
#endif


/**
 * @brief UARTE driver default configuration.
 *
 * This configuration sets up UARTE with the following options:
 * - hardware flow control disabled
 * - no parity bit
 * - one stop bit
 * - baudrate: 115200
 *
 * @param[in] _pin_tx TX pin.
 * @param[in] _pin_rx RX pin.
 */
#define NRFX_UARTE_DEFAULT_CONFIG(_pin_tx, _pin_rx)                                 \
{                                                                                   \
    .pseltxd            = _pin_tx,                                                  \
    .pselrxd            = _pin_rx,                                                  \
    .pselcts            = NRF_UARTE_PSEL_DISCONNECTED,                              \
    .pselrts            = NRF_UARTE_PSEL_DISCONNECTED,                              \
    .p_context          = NULL,                                                     \
    .baudrate           = NRF_UARTE_BAUDRATE_115200,                                \
    .interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY,                   \
    .hal_cfg            = {                                                         \
        .hwfc           = NRF_UARTE_HWFC_DISABLED,                                  \
        .parity         = NRF_UARTE_PARITY_EXCLUDED,                                \
        NRFX_UARTE_DEFAULT_EXTENDED_STOP_CONFIG                                     \
        NRFX_UARTE_DEFAULT_EXTENDED_PARITYTYPE_CONFIG                               \
    }                                                                               \
}


/** @brief Structure for the UARTE transfer completion event. */
typedef struct
{
    uint8_t * p_data; ///< Pointer to memory used for transfer.
    size_t    bytes;  ///< Number of bytes transfered.
} nrfx_uarte_xfer_evt_t;

/** @brief Structure for the UARTE RX disable event. */
typedef struct
{
    size_t    flush_cnt;  ///< Number of bytes flushed from RX FIFO.
                          /**< They will be copied to the next provided buffer. */
} nrfx_uarte_rx_disabled_evt_t;

/** @brief Structure for UARTE error event. */
typedef struct
{
    nrfx_uarte_xfer_evt_t rxtx;       ///< Transfer details, including number of bytes transferred.
    uint32_t              error_mask; ///< Mask of error flags that generated the event.
} nrfx_uarte_error_evt_t;

/** @brief Structure for UARTE event. */
typedef struct
{
    nrfx_uarte_evt_type_t type; ///< Event type.
    union
    {
        nrfx_uarte_xfer_evt_t  rxtx;              ///< Data provided for transfer completion events.
        nrfx_uarte_error_evt_t error;             ///< Data provided for error event.
        nrfx_uarte_rx_disabled_evt_t rx_disabled; ///< Data provided for error event.
    } data;                                       ///< Union to store event data.
} nrfx_uarte_event_t;

/**
 * @brief UARTE interrupt event handler.
 *
 * @param[in] p_event   Pointer to event structure. Event is allocated on the stack so it is available
 *                      only within the context of the event handler.
 * @param[in] p_context Context passed to the interrupt handler, set on initialization.
 */
typedef void (*nrfx_uarte_event_handler_t)(nrfx_uarte_event_t const * p_event,
                                           void *                     p_context);

/**
 * @brief Function for initializing the UARTE driver.
 *
 * This function configures and enables UARTE. After this function GPIO pins are controlled by UARTE.
 *
 * @param[in] p_instance    Pointer to the driver instance structure.
 * @param[in] p_config      Pointer to the structure with the initial configuration.
 * @param[in] event_handler Event handler provided by the user. If not provided driver works in
 *                          blocking mode.
 *
 * @retval NRFX_SUCCESS             Initialization was successful.
 * @retval NRFX_ERROR_INVALID_STATE Driver is already initialized.
 * @retval NRFX_ERROR_BUSY          Some other peripheral with the same
 *                                  instance ID is already in use. This is
 *                                  possible only if @ref nrfx_prs module
 *                                  is enabled.
 */
nrfx_err_t nrfx_uarte_init(nrfx_uarte_t const *        p_instance,
                           nrfx_uarte_config_t const * p_config,
                           nrfx_uarte_event_handler_t  event_handler);

/**
 * @brief Function for uninitializing the UARTE driver.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_uarte_uninit(nrfx_uarte_t const * p_instance);

/**
 * @brief Function for getting the address of the specified UARTE task.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] task       Task.
 *
 * @return Task address.
 */
NRFX_STATIC_INLINE uint32_t nrfx_uarte_task_address_get(nrfx_uarte_t const * p_instance,
                                                        nrf_uarte_task_t     task);

/**
 * @brief Function for getting the address of the specified UARTE event.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] event      Event.
 *
 * @return Event address.
 */
NRFX_STATIC_INLINE uint32_t nrfx_uarte_event_address_get(nrfx_uarte_t const * p_instance,
                                                         nrf_uarte_event_t    event);

/**
 * @brief Function for sending data over UARTE.
 *
 * If an event handler is provided in nrfx_uarte_init() call, this function
 * returns immediately and the handler is called when the transfer is done.
 * Otherwise, the transfer is performed in blocking mode, that is this function
 * returns when the transfer is finished. Blocking mode is not using interrupt
 * so there is no context switching inside the function.
 *
 * @note Peripherals using EasyDMA (including UARTE) require the transfer buffers
 *       to be placed in the Data RAM region. If this condition is not met,
 *       this function will fail with the error code NRFX_ERROR_INVALID_ADDR.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] p_data     Pointer to data.
 * @param[in] length     Number of bytes to send. Maximum possible length is
 *                       dependent on the used SoC (see the MAXCNT register
 *                       description in the Product Specification). The driver
 *                       checks it with assertion.
 *
 * @retval NRFX_SUCCESS            Initialization was successful.
 * @retval NRFX_ERROR_BUSY         Driver is already transferring.
 * @retval NRFX_ERROR_FORBIDDEN    The transfer was aborted from a different context
 *                                 (blocking mode only).
 * @retval NRFX_ERROR_INVALID_ADDR p_data does not point to RAM buffer.
 */
nrfx_err_t nrfx_uarte_tx(nrfx_uarte_t const * p_instance,
                         uint8_t const *      p_data,
                         size_t               length);

/**
 * @brief Function for sending one byte.
 *
 * If an event handler is provided in nrfx_uarte_init() call, this function
 * returns immediately and the handler is called when the transfer is done.
 * Otherwise, the transfer is performed in blocking mode, that is this function
 * returns when the transfer is finished. Blocking mode is not using interrupt
 * so there is no context switching inside the function.
 *
 * @note Peripherals using EasyDMA (including UARTE) require the transfer buffers
 *       to be placed in the Data RAM region. If this condition is not met,
 *       this function will fail with the error code NRFX_ERROR_INVALID_ADDR.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] byte       Byte.
 * @param[in] timeout_us Timeout occurs when byte is not sent for given time. When
 *                       set to 0 timeout is disabled and function blocks until byte
 *                       transfer is started or completed in blocking mode.
 *
 * @retval NRFX_SUCCESS            Initialization was successful.
 * @retval NRFX_ERROR_TIMEOUT      Timeout.
 * @retval NRFX_ERROR_FORBIDDEN    UARTE was disabled during poll_out.
 */
nrfx_err_t nrfx_uarte_poll_out(nrfx_uarte_t const * p_instance,
                               uint8_t byte,
                               uint32_t timeout_us);

/**
 * @brief Function for checking if UARTE is currently transmitting.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @retval true  The UARTE is transmitting.
 * @retval false The UARTE is not transmitting.
 */
bool nrfx_uarte_tx_in_progress(nrfx_uarte_t const * p_instance);

/**
 * @brief Function for aborting any ongoing transmission.
 * @note When abortion is not synchronous @ref NRFX_UARTE_EVT_TX_ABORTED event will
 *       be generated in non-blocking mode. It will contain number of bytes sent
 *       until the abort was called. The event handler will be called from the UARTE
 *       interrupt context.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] sync       If true operation is synchronous. Transmitter is stopped upon
 *                       function return and no event is generated.
 * @param[in] lock       Prevent using TX until @ref nrfx_uarte_tx_unlock is called.
 *
 * @retval NRFX_SUCCESS Successfully initiated abort or when transmitter synchronously stopped.
 * @retval NRFX_ERROR_INVALID_STATE Attempt to asychrnously abort when no transfer is active.
 */
nrfx_err_t nrfx_uarte_tx_abort(nrfx_uarte_t const * p_instance, bool sync, bool lock);

/**
 * @brief Function for unlocking transmission.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @reval NRFX_SUCCESS Successful unlocking.
 * @retval NRFX_ERROR_INVALID_STATE Transmission was not locked.
 */
nrfx_err_t nrfx_uarte_tx_unlock(nrfx_uarte_t const * p_instance);

/**
 * @brief Enable receiver.
 *
 * From that context event handler will be called with @ref NRFX_UARTE_EVENT_RX_BUF_REQUEST event.
 * User my respnd and provide a buffer using @ref nrfx_uarte_rx_buffer_set. Error is returned if
 * buffer is not provided. After that receiver is started and another @ref NRFX_UARTE_EVT_RX_BUF_REQUEST
 * is generated. If new buffer is not provided then receiver is disabled when buffer is filled.
 * If new buffer is provided then receiver will seamlessly switch to a new buffer (using short).
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] stop_on_end When true RX is disabled when new buffers are not provided.
 * @param[in] cont        When true ENDRX event is shortend with STARTRX task. Flag
 *                        should not be used with short buffers when there is a risk
 *                        that new buffer is not provided on time. If option is set
 *                        and new buffer is not provided on time, receiver starts to
 *                        overwrite current buffer. If false, new transfer will be
 *                        triggered from ENDRX interrupt handler (if new buffer was
 *                        provided).
 *
 * @retval NRFX_SUCCESS      Receiver successfully enabled.
 * @retval NRFX_ERROR_BUSY   When receiver is already enabled.
 * @retval NRFX_ERROR_NO_MEM When buffer was not provided.
 */
nrfx_err_t nrfx_uarte_rx_enable(nrfx_uarte_t const * p_instance,
				bool stop_on_end, bool cont);

/**
 * @brief Function for providing reception buffer.
 *
 * Function shall be called as a response to @ref NRFX_UARTE_EVT_RX_BUF_REQUEST event.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @retval NRFX_SUCCESS             Buffer successfully set.
 * @retval NRFX_ERROR_INVALID_STATE Buffer provided without pending request.
 * @retval NRFX_ERROR_TIMEOUT       Buffer provided too late. Receiver is being disabled.
 */
nrfx_err_t nrfx_uarte_rx_buffer_set(nrfx_uarte_t const * p_instance,
		                    uint8_t * p_data, size_t length);

/**
 * @brief Function for receiving data over UARTE.
 *
 * If an event handler is provided in the nrfx_uarte_init() call, this function
 * returns immediately and the handler is called when the transfer is done.
 * Otherwise, the transfer is performed in blocking mode, that is this function
 * returns when the transfer is finished. Blocking mode is not using interrupt so
 * there is no context switching inside the function.
 * The receive buffer pointer is double-buffered in non-blocking mode. The secondary
 * buffer can be set immediately after starting the transfer and will be filled
 * when the primary buffer is full. The double-buffering feature allows
 * receiving data continuously.
 *
 * @note Peripherals using EasyDMA (including UARTE) require the transfer buffers
 *       to be placed in the Data RAM region. If this condition is not met,
 *       this function fails with the error code NRFX_ERROR_INVALID_ADDR.
 *
 * @warning When the double-buffering feature is used and the UARTE interrupt
 *          is processed with a delay (for example, due to a higher priority interrupt)
 *          long enough for both buffers to get filled completely,
 *          the event handler will be invoked only once, to notify that
 *          the first buffer has been filled. This is because from hardware perspective it
 *          is impossible to deduce in such case if the second buffer was also filled completely or not.
 *          To prevent this from happening, keep the UARTE interrupt latency low
 *          or use large enough reception buffers.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] p_data     Pointer to data.
 * @param[in] length     Number of bytes to receive. Maximum possible length is
 *                       dependent on the used SoC (see the MAXCNT register
 *                       description in the Product Specification). The driver
 *                       checks it with assertion.
 *
 * @retval NRFX_SUCCESS            Initialization is successful.
 * @retval NRFX_ERROR_BUSY         The driver is already receiving
 *                                 (and the secondary buffer has already been set
 *                                 in non-blocking mode).
 * @retval NRFX_ERROR_FORBIDDEN    The transfer is aborted from a different context
 *                                 (blocking mode only).
 * @retval NRFX_ERROR_INTERNAL     The UARTE peripheral reports an error.
 * @retval NRFX_ERROR_INVALID_ADDR p_data does not point to RAM buffer.
 */
nrfx_err_t nrfx_uarte_rx(nrfx_uarte_t const * p_instance,
                         uint8_t *            p_data,
                         size_t               length,
			 bool                 stop_on_end);

/**
 * @brief Function for getting completed RX buffer.
 *
 * Function is intended to be used in blocking mode only. It returns address and
 * length of the buffer from the receiver but only if ENDRX event is set. Function
 * can be used for polling receiver in blocking mode. Contrary to @ref nrfx_uarte_rx
 * which blocks until byte is received.
 *
 * @param[in]  p_instance Pointer to the driver instance structure.
 * @param[out] pp_data    Location where buffer address if written.
 * @param[out] p_length   Location where buffer length is written.
 *
 * @retval NRFX_SUCCESS         If reception was completed. @p pp_data and @p p_data
 *                              content is valid.
 * @retval NRFX_ERROR_FORBIDDEN If driver is configured in non-blocking mode and feature
 *                              is not supported.
 * @retval NRFX_ERROR_BUSY      If reception is not completed.
 */
nrfx_err_t nrfx_uarte_get_rx(nrfx_uarte_t const * p_instance,
                             uint8_t **           pp_data,
                             size_t *             p_length);

/**
 * @brief Function for disabling receiver interrupts.
 *
 * Function disables interrupts from ENDRX, RXSTARTED and RXTO events. This prevents
 * generation of @ref NRFX_UARTE_EVT_RX_DONE, @ref NRFX_UARTE_EVT_RX_BUF_REQUEST,
 * @ref NRFX_UARTE_EVT_RX_DISABLED and @ref NRFX_UARTE_EVT_RX_BUF_TOO_LATE.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_uarte_rx_int_disable(nrfx_uarte_t const * p_instance);

/**
 * @brief Function for enabling receiver interrupts.
 *
 * Function enables interrupts disabled by @ref nrfx_uarte_rx_int_disable.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_uarte_rx_int_enable(nrfx_uarte_t const * p_instance);

/**
 * @brief Function for testing the receiver state in blocking mode.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @retval true  The receiver has at least one byte of data to get.
 * @retval false The receiver is empty.
 */
bool nrfx_uarte_rx_ready(nrfx_uarte_t const * p_instance);

/**
 * @brief Function for aborting any ongoing reception.
 * @note @ref NRFX_UARTE_EVT_RX_DONE event will be generated in non-blocking mode.
 *       It will contain number of bytes received until the abort was called. The event
 *       handler will be called from the UARTE interrupt context.
 *
 * @warning When the double-buffering feature is used and the UARTE interrupt
 *          is processed with a delay (for example, due to a higher priority
 *          interrupt) long enough for the first buffer to be filled completely,
 *          the event handler will be supplied with the pointer to the first
 *          buffer and the number of bytes received in the second buffer.
 *          This is because from hardware perspective it is impossible to deduce
 *          the reception of which buffer has been aborted.
 *          To prevent this from happening, keep the UARTE interrupt latency low
 *          or use large enough reception buffers.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] sync       If true receiver is disabled synchronously.
 *
 *
 * @retval NRFX_SUCCESS             Successfully initiate disabling or disabled (synchronous mode).
 * @retval NRFX_ERROR_INVALID_STATE Receiver was not enabled.
 */
nrfx_err_t nrfx_uarte_rx_abort(nrfx_uarte_t const * p_instance, bool sync);

/**
 * @brief Function for reading error source mask. Mask contains values from @ref nrf_uarte_error_mask_t.
 * @note Function must be used in the blocking mode only. In case of non-blocking mode, an error event is
 *       generated. Function clears error sources after reading.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @return Mask of reported errors.
 */
uint32_t nrfx_uarte_errorsrc_get(nrfx_uarte_t const * p_instance);


#ifndef NRFX_DECLARE_ONLY
NRFX_STATIC_INLINE uint32_t nrfx_uarte_task_address_get(nrfx_uarte_t const * p_instance,
                                                        nrf_uarte_task_t     task)
{
    return nrf_uarte_task_address_get(p_instance->p_reg, task);
}

NRFX_STATIC_INLINE uint32_t nrfx_uarte_event_address_get(nrfx_uarte_t const * p_instance,
                                                         nrf_uarte_event_t    event)
{
    return nrf_uarte_event_address_get(p_instance->p_reg, event);
}
#endif // NRFX_DECLARE_ONLY

/** @} */


void nrfx_uarte_0_irq_handler(void);
void nrfx_uarte_1_irq_handler(void);
void nrfx_uarte_2_irq_handler(void);
void nrfx_uarte_3_irq_handler(void);


#ifdef __cplusplus
}
#endif

#endif // NRFX_UARTE_H__
