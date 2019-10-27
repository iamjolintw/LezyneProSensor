/*
 *	main.c
 *
 *  Created on: June 30, 2019
 *	Author: Lezyne
 */

#include <stdbool.h>
#include "ble_core.h"
#include "sensor_accelerometer.h"
#include "analog.h"
#include "app_timer.h"
#include "sys_conf.h"

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

APP_TIMER_DEF(m_led_control_timer_id);

/**@brief Handler for scan timer event
 * @param p_context     Pointer to context
 */
static void led_control_timeout_handler(void * p_context)
{
	// turn off the GLED
	nrf_drv_gpiote_out_set(ACC_LEDG_PIN);	// low enable
}

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
	// Disable interrupt of GPIO to avoid the case of wakeup from sleep by GPIO event and trigger the interrupt before done the initialize procedure.
	NVIC_DisableIRQ(GPIOTE_IRQn);

	// Initialize.
    ret_code_t err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initialize LEDs (low enable)
 */
static void led_init(void)
{
	ret_code_t err_code;
	/* LED init */
	nrf_drv_gpiote_out_config_t out_configG = GPIOTE_CONFIG_OUT_SIMPLE(false);	// turn on (low enable)
	err_code = nrf_drv_gpiote_out_init(ACC_LEDG_PIN, &out_configG);
	nrf_drv_gpiote_out_config_t out_configR = GPIOTE_CONFIG_OUT_SIMPLE(true);	// turn off (low enable)
	err_code += nrf_drv_gpiote_out_init(ACC_LEDR_PIN, &out_configR);
	APP_ERROR_CHECK(err_code);

    /* Create timer to turn off the G-LED */
	err_code = app_timer_create(&m_led_control_timer_id,
								APP_TIMER_MODE_SINGLE_SHOT,
    							led_control_timeout_handler);
    APP_ERROR_CHECK(err_code);

    /* Kicking the timer */
    uint32_t timer_ticks = APP_TIMER_TICKS(GLED_TURN_OFF_TIMER_TICKS);
    err_code = app_timer_start(m_led_control_timer_id, timer_ticks, NULL);
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

    // LED Initialize (must behind ble_init(requires timer init) & gpio_model_init(requires GPIO init)).
    led_init();

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
