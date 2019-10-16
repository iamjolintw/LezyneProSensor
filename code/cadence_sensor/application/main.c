/*
 *	main.c
 *
 *  Created on: July 07, 2019
 *	Author: Lezyne
 */

#include <stdbool.h>
#include "ble_core.h"
#include "sensor_accelerometer.h"

#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

#define DEBUG_LOG_TIMESTAMP_ENABLE
#ifdef DEBUG_LOG_TIMESTAMP_ENABLE
	#if (NRF_LOG_USES_TIMESTAMP == 0)
		#error "Please enable NRF_LOG_USES_TIMESTAMP in sdk_config.h"
	#endif
#endif

/**@brief Function for using rtc counter for LOG timestamp usage.
 */
#ifdef DEBUG_LOG_TIMESTAMP_ENABLE
static uint32_t get_rtc_counter(void)
{
    return NRF_RTC1->COUNTER;
}
#endif

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
#ifndef DEBUG_LOG_TIMESTAMP_ENABLE
    ret_code_t err_code = NRF_LOG_INIT(NULL);
#else
	ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter);
#endif
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
	if(accel_task_check_enable(SENSOR_TASK_INT_FIFO))
	{
		acc_read_fifodata();
		accel_task_disable_mask(SENSOR_TASK_INT_FIFO);
	}
	if(accel_task_check_enable(SENSOR_TASK_INT_DUMP))
	{
		acc_read_fifodata_datadump();
		accel_task_disable_mask(SENSOR_TASK_INT_DUMP);
	}
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for initialize gpiote module
 */
static void gpio_model_init(void)
{
	// disable interrupt of GPIO to avoid the case of wakeup from sleep by GPIO event and trigger the interrupt before done the initialize procedure.
	NVIC_DisableIRQ(GPIOTE_IRQn);

	// Initialize.
    ret_code_t err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Log Initialize.
    log_init();
    NRF_LOG_INFO("ble initialize.");

    // BLE Initialize.
	ble_init();

    // GPIO Initialize.
	gpio_model_init();

	// Accelerometer Initialize.
	accel_init();

	// BLE advertising.
	ble_advertising_entry();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}

/**
 * @}
 */
