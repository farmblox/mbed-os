/* mbed Microcontroller Library
 * Copyright (c) 2018, STMicroelectronics
 * SPDX-License-Identifier: Apache-2.0
 *
 * STM32WL-specific deep sleep implementation.
 * Overrides the __WEAK hal_deepsleep() in sleep.c to apply
 * ES0506 errata 2.2.11 workaround: wait for LDORDY before
 * entering STOP2 to prevent power supply deadlock on wakeup.
 */

#if DEVICE_SLEEP

#include "sleep_api.h"
#include "us_ticker_api.h"
#include "us_ticker_data.h"
#include "mbed_critical.h"
#include "mbed_error.h"

extern void save_timer_ctx(void);
extern void restore_timer_ctx(void);
extern void SetSysClock(void);
extern int serial_is_tx_ongoing(void);
extern int mbed_sdk_inited;

/*  Wait loop - assuming tick is 1 us */
static void wait_loop(uint32_t timeout)
{
    uint32_t t1, t2, elapsed = 0;
    t1 = us_ticker_read();
    do {
        t2 = us_ticker_read();
        elapsed = (t2 > t1) ? (t2 - t1) : ((uint64_t)t2 + 0xFFFFFFFF - t1 + 1);
    } while (elapsed < timeout);
}

static void ForcePeriphOutofDeepSleep(void)
{
    uint32_t pFLatency = 0;
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK) {
        error("ForcePeriphOutofDeepSleep clock issue\r\n");
    }
}

static void ForceOscOutofDeepSleep(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11; /* 48MHz */
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        error("ForceOscOutofDeepSleep clock issue\r\n");
    }
}

void hal_deepsleep(void)
{
    if (serial_is_tx_ongoing()) {
        return;
    }

    core_util_critical_section_enter();

    save_timer_ctx();

    int pwrClockEnabled = __HAL_RCC_PWR_IS_CLK_ENABLED();
    int lowPowerModeEnabled = PWR->CR1 & PWR_CR1_LPR;

    if (!pwrClockEnabled) {
        __HAL_RCC_PWR_CLK_ENABLE();
    }
    if (lowPowerModeEnabled) {
        HAL_PWREx_DisableLowPowerRunMode();
    }

    /* ES0506 2.2.11: Potential deadlock on wakeup from STOP2.
     * Wait for LDO ready before entering low-power mode to ensure
     * the power supply management block restarts properly on wakeup. */
    while (!(PWR->SR2 & PWR_SR2_LDORDY_Msk)) {}

    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    if (lowPowerModeEnabled) {
        HAL_PWREx_EnableLowPowerRunMode();
    }
    if (!pwrClockEnabled) {
        __HAL_RCC_PWR_CLK_DISABLE();
    }

    mbed_sdk_inited = 0;

    ForceOscOutofDeepSleep();
    ForcePeriphOutofDeepSleep();
    SetSysClock();

    wait_loop(500);

    restore_timer_ctx();

    mbed_sdk_inited = 1;

    core_util_critical_section_exit();
}

#endif
