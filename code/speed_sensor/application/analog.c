/*
 * analog.c
 *
 *  Created on: Oct 03, 2019
 *  Author: Terry
 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "analog.h"
#include "ble_core.h"
#include "nrf_drv_saadc.h"
#include "sys_conf.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"


#define SAMPLES_IN_BUFFER 			(1)
#define SAMPLES_TRIGGER_TIMER		(1000)	// unit:ms
#define SAMPLES_TRIGGER_FAST_TIMER	(5)		// unit:ms
#define BAT_AVERAGE_COUNTER			(100)	// the average number of battery value

static const nrf_drv_timer_t 	m_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t     	m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     	m_ppi_channel;
static uint32_t 				batt_adc_sum = PERCENT_OVER_90;
static uint32_t					old_batt_adc_sum = PERCENT_OVER_90;
static bool						b_done_first_time = false;

// functions
static void bat_percent_lookup(uint32_t battery_level);
static void set_timer(uint32_t sample_timer);

/**
 * @brief Function for set timer.
 */
static void set_timer(uint32_t sample_timer)
{
	ret_code_t err_code;

	if (nrf_drv_timer_is_enabled(&m_timer))
		nrf_drv_timer_disable(&m_timer);
	/* setup m_timer for compare event every sample_timer */
	uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, sample_timer);
	nrf_drv_timer_extended_compare(&m_timer,
			 	 	 	 	 	 	 NRF_TIMER_CC_CHANNEL1,
									 ticks,
									 NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK,
									 false);
	nrf_drv_timer_enable(&m_timer);
	uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
			 	 	 	 	 	 	 NRF_TIMER_CC_CHANNEL1);
	uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();
    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for SAADC converter.
 */
static void bat_percent_lookup(uint32_t battery_level)
{
    uint8_t new_batt;

    if(battery_level > PERCENT_OVER_90)
    {
        new_batt = 100;
    }
    else if(battery_level > PERCENT_OVER_80)
    {
        new_batt = 90;
    }
    else if(battery_level > PERCENT_OVER_70)
    {
        new_batt = 80;
    }
    else if(battery_level > PERCENT_OVER_60)
    {
        new_batt = 70;
    }
    else if(battery_level > PERCENT_OVER_50)
    {
        new_batt = 60;
    }
    else if(battery_level > PERCENT_OVER_40)
    {
        new_batt = 50;
    }
    else if(battery_level > PERCENT_OVER_30)
    {
        new_batt = 40;
    }
    else if(battery_level > PERCENT_OVER_20)
    {
        new_batt = 30;
    }
    else if(battery_level > PERCENT_OVER_10)
    {
        new_batt = 20;
    }
    else if(battery_level > PERCENT_0)
    {
        new_batt = 10;
    }
    else if(battery_level <= PERCENT_0)
    {
        new_batt = 0;
    }

    NRF_LOG_INFO("battery_level = %d, battery = %d", battery_level, new_batt);
    battery_level_update(new_batt);
}

/**
 * @brief Function for saadc handler.
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	static uint8_t get_batt_count;

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t 	err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        for (uint8_t i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
        	// get data from buffer
#ifndef CSCS_MOCK_ENABLE
            batt_adc_sum += p_event->data.done.p_buffer[i];
#else
            static uint16_t fake_data = (PERCENT_OVER_90 + 50);
            fake_data --;
            if (fake_data < (PERCENT_0 - 50))
            	fake_data = (PERCENT_OVER_90 + 50);
            batt_adc_sum += fake_data;
#endif
            get_batt_count++;

            // take average and report to the air
            if (get_batt_count >= BAT_AVERAGE_COUNTER)
            {
            	// reset parameters
            	get_batt_count = 0;

            	// take average
            	batt_adc_sum = batt_adc_sum / (BAT_AVERAGE_COUNTER + 1);

            	// report to the air
                if (old_batt_adc_sum > batt_adc_sum)
                {
                	// update batt_adc_sum
                	old_batt_adc_sum = batt_adc_sum;
                	// report to the air
                	bat_percent_lookup(batt_adc_sum);
                }

            	// set flag
            	if (!b_done_first_time) {
            		set_timer(SAMPLES_TRIGGER_TIMER);
            		b_done_first_time = true;
            	}
            }
        }
    }
}

/**
 * @brief Function for timer handler.
 */
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
	// Do Nothing
}

/**
 * @brief Function for saadc event enable.
 */
void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for saadc event initialize.
 */
void saadc_sampling_event_init(void)
{
    ret_code_t err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every SAMPLES_TRIGGER_FAST_TIMER */
    set_timer(SAMPLES_TRIGGER_FAST_TIMER);
}

/**
 * @brief Function for saadc initialize.
 */
void saadc_init(void)
{
    ret_code_t err_code;
    static nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

    saadc_config.low_power_mode = true;
    saadc_config.resolution = SAADC_RESOLUTION_VAL_10bit;
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;

    channel_config.gain = SAADC_CH_CONFIG_GAIN_Gain1_6;
    channel_config.acq_time = SAADC_CH_CONFIG_TACQ_15us;

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for the disable battery related components.
 */
void bat_sleep(void)
{
	/* stop timer first */
	nrf_drv_saadc_abort();
	nrf_drv_timer_disable(&m_timer);
	nrf_drv_timer_uninit(&m_timer);

	/* uninitialize SAADC */
    nrf_drv_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power
    NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
    NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set

    /* uninitialize PPI */
    nrf_drv_ppi_uninit();
}

/**
 * @brief Function for the entry of battery functions.
 */
void bat_init(void)
{
	NRF_LOG_INFO("bat_init.");

	/* reset parameters */
	batt_adc_sum = PERCENT_OVER_90;

	/* SAADC & PPI initialize */
    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();

    NRF_LOG_INFO("bat_init done.");
}
