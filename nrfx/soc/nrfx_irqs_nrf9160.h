/*$$$LICENCE_NORDIC_STANDARD<2018>$$$*/

#ifndef NRFX_IRQS_NRF9160_H__
#define NRFX_IRQS_NRF9160_H__

#ifdef __cplusplus
extern "C" {
#endif


// SPU_IRQn

// CLOCK_POWER_IRQn
#define nrfx_power_clock_irq_handler    CLOCK_POWER_IRQHandler

// UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn
#if NRFX_CHECK(NRFX_PRS_ENABLED) && NRFX_CHECK(NRFX_PRS_BOX_0_ENABLED)
#define nrfx_prs_box_0_irq_handler  UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQHandler
#else
#define nrfx_spim_0_irq_handler     UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQHandler
#define nrfx_spis_0_irq_handler     UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQHandler
#define nrfx_twim_0_irq_handler     UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQHandler
#define nrfx_twis_0_irq_handler     UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQHandler
#define nrfx_uarte_0_irq_handler    UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQHandler
#endif

// UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn
#if NRFX_CHECK(NRFX_PRS_ENABLED) && NRFX_CHECK(NRFX_PRS_BOX_1_ENABLED)
#define nrfx_prs_box_1_irq_handler  UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQHandler
#else
#define nrfx_spim_1_irq_handler     UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQHandler
#define nrfx_spis_1_irq_handler     UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQHandler
#define nrfx_twim_1_irq_handler     UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQHandler
#define nrfx_twis_1_irq_handler     UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQHandler
#define nrfx_uarte_1_irq_handler    UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQHandler
#endif

// UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn
#if NRFX_CHECK(NRFX_PRS_ENABLED) && NRFX_CHECK(NRFX_PRS_BOX_2_ENABLED)
#define nrfx_prs_box_2_irq_handler  UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQHandler
#else
#define nrfx_spim_2_irq_handler     UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQHandler
#define nrfx_spis_2_irq_handler     UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQHandler
#define nrfx_twim_2_irq_handler     UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQHandler
#define nrfx_twis_2_irq_handler     UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQHandler
#define nrfx_uarte_2_irq_handler    UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQHandler
#endif

// UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn
#if NRFX_CHECK(NRFX_PRS_ENABLED) && NRFX_CHECK(NRFX_PRS_BOX_3_ENABLED)
#define nrfx_prs_box_3_irq_handler  UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQHandler
#else
#define nrfx_spim_3_irq_handler     UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQHandler
#define nrfx_spis_3_irq_handler     UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQHandler
#define nrfx_twim_3_irq_handler     UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQHandler
#define nrfx_twis_3_irq_handler     UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQHandler
#define nrfx_uarte_3_irq_handler    UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQHandler
#endif

// GPIOTE0_IRQn
#define nrfx_gpiote_irq_handler     GPIOTE_IRQHandler

// SAADC_IRQn
#define nrfx_saadc_irq_handler      SAADC_IRQHandler

// TIMER0_IRQn
#define nrfx_timer_0_irq_handler    TIMER0_IRQHandler

// TIMER1_IRQn
#define nrfx_timer_1_irq_handler    TIMER1_IRQHandler

// TIMER2_IRQn
#define nrfx_timer_2_irq_handler    TIMER2_IRQHandler

// RTC0_IRQn
#define nrfx_rtc_0_irq_handler      RTC0_IRQHandler

// RTC1_IRQn
#define nrfx_rtc_1_irq_handler      RTC1_IRQHandler

// WDT_IRQn
#define nrfx_wdt_irq_handler        WDT_IRQHandler

// EGU0_IRQn
#define nrfx_swi_0_irq_handler      EGU0_IRQHandler

// EGU1_IRQn
#define nrfx_swi_1_irq_handler      EGU1_IRQHandler

// EGU2_IRQn
#define nrfx_swi_2_irq_handler      EGU2_IRQHandler

// EGU3_IRQn
#define nrfx_swi_3_irq_handler      EGU3_IRQHandler

// EGU4_IRQn
#define nrfx_swi_4_irq_handler      EGU4_IRQHandler

// EGU5_IRQn
#define nrfx_swi_5_irq_handler      EGU5_IRQHandler

// PWM0_IRQn
#define nrfx_pwm_0_irq_handler      PWM0_IRQHandler

// PWM1_IRQn
#define nrfx_pwm_1_irq_handler      PWM1_IRQHandler

// PWM2_IRQn
#define nrfx_pwm_2_irq_handler      PWM2_IRQHandler

// PWM3_IRQn
#define nrfx_pwm_3_irq_handler      PWM3_IRQHandler

// PDM_IRQn
#define nrfx_pdm_irq_handler        PDM_IRQHandler

// I2S_IRQn
#define nrfx_i2s_irq_handler        I2S_IRQHandler

// FPU_IRQn

// GPIOTE1_IRQn

// KMU_IRQn

// CRYPTOCELL_IRQn

#ifdef __cplusplus
}
#endif

#endif // NRFX_IRQS_NRF9160_H__
