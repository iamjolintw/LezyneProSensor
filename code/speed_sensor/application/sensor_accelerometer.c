/*
 * 	sensor_accelerometer.c
 *
 *  Created on: June 30, 2019
 *  Author: Lezyne
 *  		History: 2019/07/01	 first version Angle algorithm
 *  				 2019/07/04  2nd version Angle algorithm, simplified, with forward and backwards judgment
 *  				 2019/07/31  3nd version zero crossing algorithm with two different average threshold for high and low speed
 *
 */

#ifndef SENSOR_ACCELEROMETER_C_
#define SENSOR_ACCELEROMETER_C_

#include <math.h>
#include <stdbool.h>

#include "sensor_accelerometer.h"
#include "ble_core.h"
#include "nrf_drv_twi.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "sensorsim.h"

#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* I2C configuration */
static const nrf_drv_twi_t acce_m_twi = NRF_DRV_TWI_INSTANCE(MMA8652_TWI_INSTANCE_ID);

/* Configuration */
//#define SENSOR_DEBUG_OUTPUT			// test purpose: switch for log message
//#define ACCELEROMETER_SELF_ACTIVATE	// test purpose: self active
//#define ACCELEROMETER_SELF_TIMEOUT	// test purpose: self timeout
//#define ACCELEROMETER_DUMP_FIFO   	// test purpose: dump the G-force of x,y,z through OTA

/* Data rate 10ms = 100Hz, the highest bicycle speed = 75 Km/h = 20.83 m/s,
   max RPM = 20.83 / 2m (circumference) = 10.41 Hz, but needs at least 4 samples for calculation = 41.64 Hz ~= 50Hz */
#define ACCEL_DATARATE  						DATA_RATE_10MS //DATA_RATE_20MS
#define ACCEL_DATARATE_MS						10//20			/* ACCEL_DATARATE_MS shall match to ACCEL_DATARATE */
/* ASLP_RATE_160MS = 6.25 hz */
#define ACCEL_SLEEPP_DATARATE					ASLP_RATE_160MS
/* Threshold = 0x08; The step count is 0.063g/ count,  (0.5g/0.063g = 7.9) */
#define ACCEL_FF_MT_THS_VALUE  					(THS3_MASK)
/* Set the debounce counter to eliminate false readings */
#define ACCEL_FF_MT_DEBOUNCE_COUNT 				2
/* Set compensate time */
#define ACCEL_PROCESS_COMPENSATE_TIME			3		// unit:ms
/* FIFO Event Sample Count Watermark, the value shall be less than 64 */
#define DEF_WATERMARK_VAL 						25
/* Minimum moving angle (degree per 0.25s (25*10ms)), degree 23 equal 2.1 Km/h (23x(1000/ACCEL_DATARATE_MS/DEF_WATERMARK_VAL)x2100*3600/360/1000) */
#define ACCEL_MOVE_ANGLE_MIN					23
/* Moving counter: 6 movements in 25*10ms = 1.5s time window */
#define ACCEL_MOVE_COUNT_MIN					6
/* Non-moving counter: (10 - 6)*25*10ms = 1.0s time window */
#define ACCEL_MOVE_COUNT_MAX 					10
/* Switching the threshold by different speed, this define for WHEEL_CIRCUMFERENCE_MM = 2100 mm */
#define ACCEL_MOVE_SPEED_HIGH_MH				(900*WHEEL_CIRCUMFERENCE_MM/100) //18900 = 18.9 Km/h  (degree*WHEEL_CIRCUMFERENCE_MM*3600/360/1000)
#define ACCEL_MOVE_SPEED_MED_MH					(575*WHEEL_CIRCUMFERENCE_MM/100) //13986 = 12.0 Km/h  (degree*WHEEL_CIRCUMFERENCE_MM*3600/360/1000)
#define ACCEL_MOVE_SPEED_LOW_MH					(100*WHEEL_CIRCUMFERENCE_MM/100) //2100  =  2.1 Km/h  (degree*WHEEL_CIRCUMFERENCE_MM*3600/360/1000)

#ifdef ACCELEROMETER_SELF_ACTIVATE
/* Set the minimum duration time to SLEEP mode, ASLP_COUNT Step = 160ms, 160ms * 10 = 1.6s */
#define ACCEL_ENTER_SLEEP_COUNTER				10
#endif

APP_TIMER_DEF(m_csc_meas_timer_id);                                                 /**< CSC measurement timer. */

/* 12bit xyz fifo data */
static uint8_t accel_buff[6*DEF_WATERMARK_VAL] = {0};
/* Pending task indicator */
static t_accel_task_pending task_pending_singal = {0};
/* Protector for critical global variable */
static volatile bool rw_lock_protect_flag = false;
/* acc angle variable */
static int16_t last_angle_residue = DEF_INVALID_LAST_ANGLE;

/* Time stamp: range: 0 ~ 256 seconds. Needs to take care of wrap-around */
static uint16_t ui16_total_time = 0;
/* Stationary counter: to indicate sensor is moving or not */
static uint8_t 	ui8_movecnt = 0;
/* Last lap indicator: range: 0 ~ 8 million km */
static uint32_t ui32_last_lap = 0;
/* Flag for switching the high speed algorithm or low speed algorithm */
static bool 	acc_speed_high_flag = false;
/* lap indicator: range: 0 ~ 8 million km */
static uint32_t ui32_total_step = 0;
/* State variable for zero crossing judgment */
static step_detect_t  step_state = eSTEP_RESET;
/* Sample number for zero crossing judgment: range: 0 ~ 11930 hours */
static uint32_t ui32_step_sameple_counter = 0;
/* Sample number for detect a lap: range: 0 ~ 11930 hours */
static uint32_t ui32_step_detect_number = 0;
/* Last average value of zero crossing judgment */
static float last_average_weighting = 0 ;

/* test purpose: FIFO dump */
#ifdef ACCELEROMETER_DUMP_FIFO
static int16_t  x_sample2[DEF_WATERMARK_VAL] = {0} ;
static int16_t  y_sample2[DEF_WATERMARK_VAL] = {0} ;
static int16_t  z_sample2[DEF_WATERMARK_VAL] = {0} ;
#endif

/* FUNCTIONS */
static void 		application_timers_start(void);
static void 		application_timers_stop(void);
static void 		acc_step_update_angle(uint16_t *angle_array, float *mag_array);
static void 		acc_step_mag_update(float mag_update_value);
static ret_code_t 	accel_write_reg(uint8_t reg_addr, uint8_t reg_data);
static ret_code_t 	accel_read_reg(uint8_t reg_addr, uint8_t *reg_data);
static ret_code_t 	accel_burst_read_reg(uint8_t addr, uint8_t * pdata, size_t size);
static void 		accel_i2c_gpio_enable (void);
static void 		accel_config_fifo_int(bool enable);
static void 		accel_config_motion_int(bool enable);
static void 		accel_wake_up(void);
static void 		accel_standby(void);
static float 		lowPassExponential(float input, float average, float factor);
static ret_code_t 	rw_lock_set(bool config);
static bool 		rw_lock_get(void);
static void 		accel_display_reg(void);

#ifdef ACCELEROMETER_SELF_TIMEOUT
static void 		accel_self_csc_meas_timeout_handler(void * p_context);
#endif

/**@brief handler for interrupt of input pin1, for Motion Detection event
 */
void mma8652_int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint8_t ui8_int_source = 0, ui8_status = 0, ui8_sysmod = 0, ff_mt_src = 0;

	/* read to clear interrupt */
	accel_read_reg(MMA8652_INT_SOURCE,		&ui8_int_source);
	accel_read_reg(MMA8652_STATUS_00,		&ui8_status);
	accel_read_reg(MMA8652_SYSMOD, 			&ui8_sysmod);
	accel_read_reg(MMA8652_FF_MT_SRC,		&ff_mt_src);

	/* Debug: motion detection interrupt */
	if (ui8_int_source & SRC_FF_MT_MASK)
	{
#ifdef SENSOR_DEBUG_OUTPUT
		NRF_LOG_INFO("MMA8652_int1 ST:%2x, INT: %2x, SYSMOD:%2x, FF_MT_SRC: %x ", ui8_status, ui8_int_source, ui8_sysmod, ff_mt_src);
#endif
	}
}

/**@brief handler for interrupt of input pin2 for FIFO interrupt and Auto-WAKE/SLEEP event
 */
void mma8652_int2_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint8_t ui8_int_source = 0, ui8_status = 0, ui8_sysmod = 0;
	ret_code_t result;

	/* read to clear interrupt */
	accel_read_reg(MMA8652_INT_SOURCE,		&ui8_int_source);
	accel_read_reg(MMA8652_STATUS_00,		&ui8_status);
	accel_read_reg(MMA8652_SYSMOD, 			&ui8_sysmod);

	/* FIFO Interrupt */
    if (ui8_int_source & SRC_FIFO_MASK)
	{
#ifdef SENSOR_DEBUG_OUTPUT
    	//NRF_LOG_INFO("mma8652_int2 ST:%2x, INT: %2x, SYSMOD:%2x ",ui8_status, ui8_int_source, ui8_sysmod);
#endif
    	/* watermark is not full, wait until watermark is full */
    	if((ui8_status & (F_WMRK_FLAG_MASK)) != (F_WMRK_FLAG_MASK))
    	{
    		NRF_LOG_INFO("Warning!!! FIFO watermark is not full, do not need to read buffer.");
    		return;
    	}
    	/* rise SENSOR_TASK_INT_FIFO to process the data of buffer in idle_state_handle() later */
#ifdef ACCELEROMETER_DUMP_FIFO
    	result = accel_task_enable_mask(SENSOR_TASK_INT_DUMP);
#else
    	result = accel_task_enable_mask(SENSOR_TASK_INT_FIFO);
#endif
    	if(result == NRF_SUCCESS)
    	{
    		/* read 12bit fifo data */
   			accel_burst_read_reg(MMA8652_OUT_X_MSB, accel_buff, 6*DEF_WATERMARK_VAL);
       	}
	}

#ifdef ACCELEROMETER_SELF_ACTIVATE
	/* Auto-WAKE/SLEEP interrupt */
	if(ui8_int_source & SRC_ASLP_MASK)
	{
		accel_set_active();
	}
#endif
}

#ifdef ACCELEROMETER_SELF_TIMEOUT
/**@brief Function accel_csc_meas_timeout_handler.
 *  self test timeout handler, should remove this function after official release
 */
static void accel_self_csc_meas_timeout_handler(void * p_context)
{
    uint8_t ctrl_reg[2] = {0};
    accel_read_reg(MMA8652_SYSMOD, 			&ctrl_reg[0]);
    accel_read_reg(MMA8652_CTRL_REG4,		&ctrl_reg[1]);
#ifdef SENSOR_DEBUG_OUTPUT
    uint32_t pin1_status = nrf_gpio_pin_read(MMA8652_INT1_PIN);
    uint32_t pin2_status = nrf_gpio_pin_read(MMA8652_INT2_PIN);
   	NRF_LOG_INFO("self timeout_handler: sysmode: %2x, STATUS: %2x, gpio1: %d, gpio2: %d", ctrl_reg[0], ctrl_reg[1], pin1_status, pin2_status);
#endif
   	ble_cscs_meas_t accel_measuremen;
	accel_csc_measurement(&accel_measuremen);
}
#endif

/**@brief Function for the Timer initialization.
 */
static void accel_timers_init(void)
{
    /* Create cycle speed and condence timer */
	ret_code_t err_code = app_timer_create(&m_csc_meas_timer_id,
                                APP_TIMER_MODE_REPEATED,
#ifdef ACCELEROMETER_SELF_TIMEOUT
								accel_self_csc_meas_timeout_handler);	// selftest: print log
#else
    							accel_csc_meas_timeout_handler);		// timeout handler in ble_core.c
#endif
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    /* Start application timers */
    uint32_t csc_meas_timer_ticks = APP_TIMER_TICKS(SPEED_AND_CADENCE_MEAS_INTERVAL);
    NRF_LOG_INFO("csc_meas_timer_ticks = %d.",csc_meas_timer_ticks);
    ret_code_t err_code = app_timer_start(m_csc_meas_timer_id, csc_meas_timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for stopping application timers.
 */
static void application_timers_stop(void)
{
    ret_code_t err_code = app_timer_stop(m_csc_meas_timer_id);
    APP_ERROR_CHECK(err_code);
}

/**	@brief Brief : I2C bus write
*	@param reg_addr : register
*	@param reg_data : data
*  	@Return Status of the I2C write
*/
static ret_code_t accel_write_reg(uint8_t reg_addr, uint8_t reg_data)
{
	uint8_t data[2];
	data[0] = reg_addr;
	data[1] = reg_data;
	ret_code_t err_code = nrf_drv_twi_tx(&acce_m_twi, MMA8652_ADDRESS, data, 2, false);
    nrf_delay_ms(2);
	if (err_code)
		NRF_LOG_INFO(0, "accel_write_reg error\r\n");
	APP_ERROR_CHECK(err_code);
	return err_code;
}

/**	@brief Brief : I2C bus read
*	@param reg_addr : register
*	@param reg_data : This data read from the sensor
*  	@Return Status of the I2C read
*/
static ret_code_t accel_read_reg(uint8_t reg_addr, uint8_t *reg_data)
{
	ret_code_t err_code;
	uint8_t addr8 = (uint8_t)reg_addr;
	/* write register address */
	err_code = nrf_drv_twi_tx(&acce_m_twi, MMA8652_ADDRESS, &addr8, 1, true);
	nrf_delay_ms(2);
	if (err_code)
		NRF_LOG_INFO("accel_I2C_read (W) error(0x%X).",err_code);
	/* read data */
	err_code = nrf_drv_twi_rx(&acce_m_twi, MMA8652_ADDRESS, reg_data, 1);
	if (err_code)
		NRF_LOG_INFO("accel_I2C_read (R) error(0x%X) -[reg_addr:%x].",err_code, reg_addr);
	APP_ERROR_CHECK(err_code);
	return err_code;
}

/** @brief Read continuous data from sensor.
*   @param reg_addr : Start address to read.
*   @param reg_data : Pointer to the buffer to fill with data.
*   @param size : Byte count of data to read.
*   @return NRF_SUCCESS or reason of error.
*/
static ret_code_t accel_burst_read_reg(uint8_t reg_addr, uint8_t * reg_data, size_t size)
{
	ret_code_t err_code;
	uint8_t addr8 = (uint8_t)reg_addr;
	/* write register address */
	err_code = nrf_drv_twi_tx(&acce_m_twi, MMA8652_ADDRESS, &addr8, 1, true);
	if (err_code)
		NRF_LOG_INFO("accel_burst_read_reg (W) error(0x%X).",err_code);
	/* read data */
	err_code = nrf_drv_twi_rx(&acce_m_twi, MMA8652_ADDRESS, reg_data, size);
	if (err_code)
		NRF_LOG_INFO("accel_burst_read_reg (R) error(0x%X).",err_code);
	APP_ERROR_CHECK(err_code);
	return err_code;
}

/**	@brief accel_task_enable_mask
*	@param enable : enable which type of t_accel_task_pending
*  	@Return Status of enable success or not
*/
ret_code_t accel_task_enable_mask(t_accel_task_pending enable)
{
	if ((task_pending_singal & enable) == enable)
	{
		NRF_LOG_INFO("Warning!! the flag is already enabled: 0x%x", enable);
		return NRF_ERROR_INVALID_STATE;
	}
	else
	{
		task_pending_singal = (task_pending_singal | enable);
	}
	return NRF_SUCCESS;
}

/**	@brief accel_task_disable_mask
*	@param disable : enable which type of t_accel_task_pending
*/
void accel_task_disable_mask(t_accel_task_pending disable)
{
	if ((task_pending_singal & disable) == disable)
	{
		task_pending_singal = (task_pending_singal&(!disable));
	}
	else
	{
		NRF_LOG_INFO("Warning!! the flag is not enabled: 0x%x",disable);
	}
}

/**	@brief : accel_task_check_enable
*	@param check : check the type of t_accel_task_pending enable or not
*  	@Return Status of ture or false
*/
bool accel_task_check_enable(t_accel_task_pending check)
{
	return (task_pending_singal&check);
}

/**	@brief : rw_lock_set
*	@param config : true or false of rw_lock_protect_flag
*  	@Return Status of enable success or not
*/
static ret_code_t rw_lock_set(bool config)
{
	if (rw_lock_protect_flag == config)
	{
		NRF_LOG_INFO("Warning!! Invalid state, flag: %x, config: %x", rw_lock_protect_flag, config);
		return NRF_ERROR_INVALID_STATE;
	}
	else
	{
		rw_lock_protect_flag = config;
	}
	return NRF_SUCCESS;
}

/**	@brief : rw_lock_get
*  	@Return rw_lock_protect_flag true or false
*/
static bool rw_lock_get(void)
{
	return rw_lock_protect_flag;
}

/**@brief Function for the sensor accelerometer configuration.
 */
static void accel_configuration(void)
{
	NRF_LOG_INFO("accel_configuration.");

	uint8_t who_n_i = 0;
	uint8_t reset_status = 0;

	/* read WHO_AND_I first */
	accel_read_reg(MMA8652_WHO_AM_I, &who_n_i);
	if(who_n_i != MMA8652_WHO_AM_I_OUT)
	{
		NRF_LOG_ERROR(" Device ID not match!! :MMA8652 0x4A: 0x%x", who_n_i);
		return;
	}

    /* RESET sensor, all registers are reset to default */
    accel_write_reg(MMA8652_CTRL_REG2, RST_MASK);
    do {
    	nrf_delay_ms(5);
    	accel_read_reg(MMA8652_CTRL_REG2, &reset_status);
    } while (reset_status & RST_MASK);

	/* Set to standby mode */
    accel_standby();

    /* clear interrupt in case there were something vestigial */
    uint8_t ui8_temp = 0;
	accel_read_reg(MMA8652_INT_SOURCE,		&ui8_temp);
	accel_read_reg(MMA8652_STATUS_00,		&ui8_temp);
	accel_read_reg(MMA8652_SYSMOD, 			&ui8_temp);
	accel_read_reg(MMA8652_FF_MT_SRC,		&ui8_temp);

    /* Set F_SETUP, disable FIFO first, watermark = 25 * DATA_RATE_20MS = 500ms) */
	accel_write_reg(MMA8652_F_SETUP, DEF_WATERMARK_VAL);
#ifdef ACCELEROMETER_SELF_ACTIVATE
	/* Set REG2, enable Auto-SLEEP and High Resolution */
    accel_write_reg(MMA8652_CTRL_REG2, MOD_HIGH_RES | SLPE_MASK);
    /* Set REG3, Configure Wake from Freefall/Motion interrupt, and the INT pins for Push-Pull */
    accel_write_reg(MMA8652_CTRL_REG3, WAKE_FF_MT_MASK | IPOL_MASK);
    /* Set REG4, enable Auto-SLEEP/WAKE Interrupt and the FIFO Interrupt, (enable Motion detection interrupt in initialization) */
    accel_write_reg(MMA8652_CTRL_REG4, INT_EN_ASLP_MASK | INT_EN_FF_MT_MASK);
	/* set ASLP_COUNT, setup sleep counter ACCEL_ENTER_SLEEP_COUNTER */
	accel_write_reg(MMA8652_ASLP_COUNT, ACCEL_ENTER_SLEEP_COUNTER);
#else
    /* Set REG2, disable Auto-SLEEP, and High Resolution */
    accel_write_reg(MMA8652_CTRL_REG2, MOD_HIGH_RES); // without auto wake up
    /* Set REG3, the INT pins for Push-Pull */
    accel_write_reg(MMA8652_CTRL_REG3, IPOL_MASK);
    /* Set REG4, enable FIFO Interrupt, (do not enable Motion detection interrupt in initialization) */
    accel_write_reg(MMA8652_CTRL_REG4, INT_EN_FF_MT_MASK);
#endif
    /* Set REG5, Motion detection Interrupt Enable mapped to INT1, and FIFO Interrupt (and Auto-SLEEP) mapped to INT2(default) */
    accel_write_reg(MMA8652_CTRL_REG5,  INT_EN_FF_MT_MASK); //Set the interrupt to route to INT1
    /* Set REG1, Set to 10ms data rate period (100Hz), 160ms sleep datarate, non-Fast read mode (12bit). */
    accel_write_reg(MMA8652_CTRL_REG1, ACCEL_DATARATE | ACCEL_SLEEPP_DATARATE);

	/* Motion configuration and status registers */
	/* Set MMA8652_FF_MT_CFG, setup motion event after the debounce counter time is reached, ELE = 0, OAE = 1, Event flag enable on X and Z*/
	accel_write_reg(MMA8652_FF_MT_CFG, OAE_MASK | ZEFE_MASK);
	/* Set MMA8652_FF_MT_THS, setup THS = ACCEL_FF_MT_THS_VALUE */
	accel_write_reg(MMA8652_FF_MT_THS, ACCEL_FF_MT_THS_VALUE | DBCNTM_MASK);
	/* Set MMA8652_FF_MT_COUNT, setup debounce counter ACCEL_FF_MT_DEBOUNCE_COUNT */
	accel_write_reg(MMA8652_FF_MT_COUNT, ACCEL_FF_MT_DEBOUNCE_COUNT);

	/* Turn off HPF for Data Out and set 8g Mode, SENSITIVITY_8G */
	accel_write_reg(MMA8652_XYZ_DATA_CFG, ACCEL_FULL_SCALE_REG);
	accel_write_reg(MMA8652_HP_FILTER_CUTOFF, PULSE_LPF_EN_MASK | PULSE_HPF_BYP_MASK);
    /* Set back to active and wake up */
	accel_display_reg();
	accel_wake_up();
}

/**@brief Function for debugging.
 */
static void accel_display_reg(void)
{
#ifdef SENSOR_DEBUG_OUTPUT
    uint8_t ctrl_reg[13];
    accel_read_reg(MMA8652_F_SETUP,			&ctrl_reg[0]);
    accel_read_reg(MMA8652_CTRL_REG1,		&ctrl_reg[1]);
    accel_read_reg(MMA8652_CTRL_REG2,		&ctrl_reg[2]);
    accel_read_reg(MMA8652_CTRL_REG3,		&ctrl_reg[3]);
    accel_read_reg(MMA8652_CTRL_REG4,		&ctrl_reg[4]);
    accel_read_reg(MMA8652_CTRL_REG5,		&ctrl_reg[5]);
    accel_read_reg(MMA8652_INT_SOURCE,		&ctrl_reg[6]);
	accel_read_reg(MMA8652_FF_MT_CFG,		&ctrl_reg[7]);
	accel_read_reg(MMA8652_FF_MT_THS,		&ctrl_reg[8]);
	accel_read_reg(MMA8652_FF_MT_COUNT,		&ctrl_reg[9]);
	accel_read_reg(MMA8652_ASLP_COUNT,		&ctrl_reg[10]);
	accel_read_reg(MMA8652_XYZ_DATA_CFG,	&ctrl_reg[11]);
	accel_read_reg(MMA8652_FF_MT_SRC,		&ctrl_reg[12]);

	NRF_LOG_INFO("F_SETUP:%x, R1:%x, R2:%x, R3:%x ", 	ctrl_reg[0], 	ctrl_reg[1], 	ctrl_reg[2], 	ctrl_reg[3]);
	NRF_LOG_INFO("R4:%x, R5:%x, INT:%x. ", 				ctrl_reg[4], 	ctrl_reg[5], 	ctrl_reg[6]);
	NRF_LOG_INFO("ASLP:%x, XYZ:%x, FT_SRC:%x.",  		ctrl_reg[10], 	ctrl_reg[11], 	ctrl_reg[12]);
	NRF_LOG_INFO("MT CFG:%x, MT THS:%x, MT COUNT:%x.", 	ctrl_reg[7], 	ctrl_reg[8], 	ctrl_reg[9]);
#endif
}

/**@brief Function for enable of disable fifo interrupt.
 *	bool enable: disable:0, enable:1		
 */
static void accel_config_fifo_int(bool enable)
{
	uint8_t ctrl_reg4 = 0;
	accel_read_reg(MMA8652_CTRL_REG4, &ctrl_reg4);
	nrf_delay_ms(10);
	if(enable)
	{
		accel_write_reg(MMA8652_F_SETUP, 	(F_MODE_FILL| DEF_WATERMARK_VAL));
		accel_write_reg(MMA8652_CTRL_REG4, 	(ctrl_reg4 	| INT_EN_FIFO_MASK));
    }
    else
    {
    	accel_write_reg(MMA8652_F_SETUP, 0x00);
    	accel_write_reg(MMA8652_CTRL_REG4, 	(ctrl_reg4 & ~INT_EN_FIFO_MASK));
    }
}

/**@brief Function for enable or disable motion detection interrupt.
 *	bool enable: disable:0, enable:1		
 */
static void accel_config_motion_int(bool enable)
{
	uint8_t ctrl_reg4 = 0;
	accel_read_reg(MMA8652_CTRL_REG4, &ctrl_reg4);
	if(enable)
	{
	    /* Set REG4, enable Auto-SLEEP/WAKE Interrupt and the FIFO Interrupt */
#ifndef ACCELEROMETER_SELF_ACTIVATE
		accel_write_reg(MMA8652_CTRL_REG4, (ctrl_reg4 | INT_EN_FF_MT_MASK));
#else
		accel_write_reg(MMA8652_CTRL_REG4, (ctrl_reg4 | INT_EN_FF_MT_MASK |INT_EN_ASLP_MASK));
#endif
	}
	else
	{
		/* Set REG4, enable Auto-SLEEP/WAKE Interrupt and the FIFO Interrupt */
#ifndef ACCELEROMETER_SELF_ACTIVATE
	    accel_write_reg(MMA8652_CTRL_REG4, (ctrl_reg4 & ~INT_EN_FF_MT_MASK));
#else
	    accel_write_reg(MMA8652_CTRL_REG4, (ctrl_reg4 & ~ (INT_EN_FF_MT_MASK |INT_EN_ASLP_MASK)));
#endif
	}
}

/**@brief Initialize I2C (TWI).
 */
static ret_code_t accel_i2c_gpio_init (void)
{
	/* I2C initialize */
    const nrf_drv_twi_config_t twi_config = {
       .scl                = MMA8652_I2C_SCL_PIN,
       .sda                = MMA8652_I2C_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false,
       .hold_bus_uninit    = false
    };
    ret_code_t err_code = nrf_drv_twi_init(&acce_m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&acce_m_twi);
	return err_code;
}

/**@brief start gpio event I2C.
 */
static void accel_i2c_gpio_enable (void)
{
    /* GPIO Initialize */
#if 0
    nrf_drv_gpiote_in_config_t in_config1 = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
	err_code = nrf_drv_gpiote_in_init(MMA8652_INT1_PIN, &in_config1, mma8652_int1_handler);
#else
    nrf_drv_gpiote_in_config_t in_config1 = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    ret_code_t err_code = nrf_drv_gpiote_in_init(MMA8652_INT1_PIN, &in_config1, mma8652_int1_handler);
#endif
	APP_ERROR_CHECK(err_code);
	nrf_drv_gpiote_in_config_t in_config2 = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    err_code = nrf_drv_gpiote_in_init(MMA8652_INT2_PIN, &in_config2, mma8652_int2_handler);
    APP_ERROR_CHECK(err_code);

	nrf_drv_gpiote_in_event_enable(MMA8652_INT2_PIN, true);
	nrf_drv_gpiote_in_event_enable(MMA8652_INT1_PIN, true);
	NVIC_EnableIRQ(GPIOTE_IRQn);
}

/**@brief Function for set the sensor accelerometer to enable Motion detection and disable FIFO.
 */
void accel_set_deactive(void)
{
#ifndef ACCELEROMETER_SELF_ACTIVATE
	NRF_LOG_INFO("accel_set_deactive: disable FIFO, enable motion detection");
	/* Set to standby mode */
    accel_standby();
	accel_config_fifo_int(false);
	accel_config_motion_int(true);
	acc_step_reset_angle();
#ifndef ACCELEROMETER_SELF_TIMEOUT
	application_timers_stop();
#endif
	/* Set back to active mode */
	accel_wake_up();
	accel_display_reg();
#endif
}

/**@brief Function for set the sensor accelerometer to disable Motion detection and enable FIFO.
 */
void accel_set_active(void)
{
	NRF_LOG_INFO("accel_set_active: enable FIFO, disable motion detection");
	/* Set to standby mode */
    accel_standby();
	accel_config_fifo_int(true);
	accel_config_motion_int(false);
#ifndef ACCELEROMETER_SELF_TIMEOUT
	application_timers_start();
#endif
	/* Set back to active */
	accel_wake_up();
}

/**@brief Function for set the sensor accelerometer to ACTIVATE mode.
 *	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE			
 */
void accel_wake_up(void)
{
	/* Read REG1 byte first; then enable ACTIVE bit for going to activate mode */
	uint8_t ctrl_reg1;
	accel_read_reg(MMA8652_CTRL_REG1, 	& ctrl_reg1);
	accel_write_reg(MMA8652_CTRL_REG1, 	ctrl_reg1 | ACTIVE_MASK);
	nrf_delay_ms(50);
}

/**@brief Function for set the sensor accelerometer to STANDBY mode.
 * 	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE
 */
void accel_standby(void)
{
    /* Read REG1 byte first; then disable ACTIVE bit for going to standby mode */
	uint8_t ctrl_reg1;
	accel_read_reg(MMA8652_CTRL_REG1, 	&ctrl_reg1);
	accel_write_reg(MMA8652_CTRL_REG1, 	ctrl_reg1 & ~ACTIVE_MASK);
	nrf_delay_ms(50);
}

/**@brief Function lowPassExponential fifo accelerometer data.
 *	ensure factor belongs to  [0,1]
 */
static float lowPassExponential(float input, float average, float factor)
{
    return (average + factor * (input - average));
}

/**@brief Function for dump x,y,z g x 1000 value to ota.
 */
void acc_read_fifodata_datadump(void)
{
#ifdef ACCELEROMETER_DUMP_FIFO
	ble_cscs_meas_t cscs_measurement;
	uint8_t i = 0;

	for(i = 0; i < DEF_WATERMARK_VAL; i++)
	{
		static float last_ay1 = 0, last_az1 = 0 , last_ax1 = 0;

		x_sample2[i] = ((int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)] << 8) | (uint16_t)accel_buff[(i*6)+1]));
		y_sample2[i] = ((int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+2]<< 8)| (uint16_t)accel_buff[(i*6)+3]));
		z_sample2[i] = ((int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+4]<< 8)| (uint16_t)accel_buff[(i*6)+5]));

		last_ax1 = ((float)x_sample2[i])/(float)(ACCEL_SENSITIVITY_CONFIG*16);
		last_ay1 = ((float)y_sample2[i])/(float)(ACCEL_SENSITIVITY_CONFIG*16);
		last_az1 = ((float)z_sample2[i])/(float)(ACCEL_SENSITIVITY_CONFIG*16);

		int16_t in16_ax = (int16_t)(last_ax1*1000);
		int16_t in16_ay = (int16_t)(last_ay1*1000);
		int16_t in16_az = (int16_t)(last_az1*1000);

		//NRF_LOG_INFO( "ID: %5d: X: %d, Z: %d, mag: %d", ui32_step_sameple_counter, in16_ax, in16_az, mag_xyz);
		cscs_measurement.is_wheel_rev_data_present = true;
		cscs_measurement.is_crank_rev_data_present = true;
		cscs_measurement.cumulative_wheel_revs = ui32_step_sameple_counter;
		cscs_measurement.last_wheel_event_time = (uint32_t)in16_ax;
		cscs_measurement.cumulative_crank_revs = (uint16_t)in16_ay;
		cscs_measurement.last_crank_event_time	 =(uint16_t)in16_az;
		ui32_step_sameple_counter ++;

		uint8_t retry_count = 0;
		while ((accel_csc_meas_timeout_handler2(cscs_measurement) != NRF_SUCCESS))
		{
			nrf_delay_ms(4);
			retry_count++;
			if(retry_count>=3)
				break;
		}
	}
#endif
}

/**@brief Function for processing x,y,z data.
 */
void acc_read_fifodata(void)
{
	#define LOW_PASS_FACTOR 	0.3 // ensure factor belongs to  [0,1]

	static float last_ay = 0, last_az = 0;
	float f_average_ang = 0, ay = 0, az = 0;
	//static float last_ax = 0;
	//float ax = 0;
	float mag_accel_sample[DEF_WATERMARK_VAL] = {0};
	uint16_t angle_sample[DEF_WATERMARK_VAL] = {0};

	for(uint8_t i = 0; i < DEF_WATERMARK_VAL; i++)
	{
		int16_t accel_xyz[3];
		accel_xyz[0] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)] << 8) | (uint16_t)accel_buff[(i*6)+1]);
		accel_xyz[1] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+2] << 8) | (uint16_t)accel_buff[(i*6)+3]);
		accel_xyz[2] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+4] << 8) | (uint16_t)accel_buff[(i*6)+5]);
		//ax = ((float)accel_xyz[0])/(float)(ACCEL_SENSITIVITY_CONFIG*16);
		ay = ((float)accel_xyz[1])/(float)(ACCEL_SENSITIVITY_CONFIG*16);
		az = ((float)accel_xyz[2])/(float)(ACCEL_SENSITIVITY_CONFIG*16);
		/* low pass filter */
		//ax = lowPassExponential(ax, last_ax, LOW_PASS_FACTOR);
		//last_ax = ax;
		ay = lowPassExponential(ay, last_ay, LOW_PASS_FACTOR);
		last_ay = ay;
		az = lowPassExponential(az, last_az, LOW_PASS_FACTOR);
		last_az = az;
		/* LezyneSPD use z and y axis for calculating angle value */
		f_average_ang = (float)(atan2((double)az,(double)ay)*180/PI)+180.0;
		angle_sample[i] = (uint16_t)(f_average_ang + 0.5);		// round 0.4 down, round 0.5 up
		/* LezyneSPD use y axis to check zero-crossing condition */
		mag_accel_sample[i] = ay;
	}
	/* processing angle and mag data */
	acc_step_update_angle(angle_sample, mag_accel_sample);
}

/**@brief Function acc_step_update_angle process accelerometer angular data to lap and total angle.
 */
static void acc_step_update_angle(uint16_t *angle_array, float *mag_array)
{
	uint8_t i = 0;
	uint16_t last_angle = angle_array[DEF_WATERMARK_VAL-1];
	int16_t current_angle = 0, temp_angle_diff = 0;

	rw_lock_set(true);	// protect the global variable
	/* ticks time */
	static uint32_t last_ticks = 0;
	uint32_t ticks_current = app_timer_cnt_get();
	uint32_t diff_ticks = (ticks_current > last_ticks)? (ticks_current - last_ticks) : (ticks_current + 0xFFFFFF - last_ticks);
	uint32_t diff_time = (ROUNDED_DIV((diff_ticks * 1000), APP_TIMER_CLOCK_FREQ))+ ACCEL_PROCESS_COMPENSATE_TIME;

	for(i = 0; i < DEF_WATERMARK_VAL ; i++)
	{
		/* 25 accelerometer samples to check zero-crossing condition */
		acc_step_mag_update(mag_array[i]);

		/* 25 angular samples processing */
		if(i == 0) // compare first sample with last_angle_residue
		{
			temp_angle_diff = (last_angle_residue == DEF_INVALID_LAST_ANGLE)? 0 : (int16_t)(angle_array[i] - last_angle_residue);
		}
		else	// compare n with n-1
		{
			temp_angle_diff = (int16_t)(angle_array[i] - angle_array[i-1]);
		}

		if (temp_angle_diff > DEF_ANGLE_180_DEGREE)	// backward revolve and across the 360 degree, i.e. n = 350,  n-1 = 10
		{
			temp_angle_diff = temp_angle_diff - DEF_ANGLE_360_DEGREE ;
		}
		else if (temp_angle_diff < (DEF_ANGLE_180_DEGREE * -1)) // forward revolve and across the 360 degree, i.e. n = 10, n-1 = 350
		{
			temp_angle_diff = temp_angle_diff + DEF_ANGLE_360_DEGREE;
		}
		else if((temp_angle_diff > DEF_MAX_ANGLE_WINDOW) || (temp_angle_diff < (DEF_MAX_ANGLE_WINDOW * -1)))
		{
			NRF_LOG_INFO("Warning!!! i:(%d) the difference (%d) > 75, [%d]-[%d] ",i,temp_angle_diff, angle_array[i] , (i==0)?last_angle_residue:angle_array[i-1] );
		}

		current_angle += temp_angle_diff;
	}

	current_angle = abs(current_angle);
	last_angle_residue = last_angle;		// update last_angle_residue for current last_angle

	/* event time is 1024-based. maximum value shall less that  64,000 (0x10000 / 0x800 * 2000) */
	ui16_total_time = (ui16_total_time + diff_time) % (DEF_TOTAL_TIME_STAMP_MAXIMUM);
	last_ticks = ticks_current;
	rw_lock_set(false);	//release protection

	/* movement detection */
	if(current_angle < ACCEL_MOVE_ANGLE_MIN) 	// speed < 2.0kmh
	{
		if( ui8_movecnt == 0)	// no moving
		{
			return;
		}
		else if( ui8_movecnt < ACCEL_MOVE_COUNT_MIN) 	// moving -> stop, counter less minimum stop requirement.
		{
			ui8_movecnt = 0;
			return;
		}
		else	// gradually stop or speed over 20Kmh, angle close to 0
		{
			if(acc_speed_high_flag)
			{
				/* do nothing */
			}
			else
			{
				ui8_movecnt--;
			}
		}
	}
	else										// speed >= 2.0kmh
	{
		if(ui8_movecnt == ACCEL_MOVE_COUNT_MAX)
		{
			/* do nothing */
		}
		else if(ui8_movecnt < ACCEL_MOVE_COUNT_MIN )
		{
			ui8_movecnt++;
			return; // do not need to update total angle and lap
		}
		else 									//(ui32_movecnt >= ACCEL_MOVE_COUNT_MINX)
		{
			ui8_movecnt = ACCEL_MOVE_COUNT_MAX;
		}
	} /*end of movement detection*/

}

/**@brief Function acc_step_reset_angle reset to default value.
 */
void acc_step_reset_angle(void)
{
    last_angle_residue = DEF_INVALID_LAST_ANGLE;
    ui16_total_time = 0;
    step_state = eSTEP_RESET;
    ui32_step_sameple_counter = 0;
}

/**@brief Function process accelerometer data to check zero-crossing condition.
 */
static void acc_step_mag_update(float mag_update_value)
{
	/* ============= Step 0: Initialize ============= */
	#define MAX_FILTER_WINDOW      			0.15f
	#define AVERAGE_ALPHA_FACTOR			0.05f

	static float peakmax = 0, valleymax = 0;
	float step_temp_min = 0, step_temp_max = 0;

	if(ui32_step_sameple_counter == 0) // initiate default value
	{
		last_average_weighting =  mag_update_value;
		ui32_step_sameple_counter ++;
		return;
	}
	//#low pass filter
	last_average_weighting = lowPassExponential(mag_update_value, last_average_weighting, AVERAGE_ALPHA_FACTOR);
	step_temp_min = last_average_weighting - MAX_FILTER_WINDOW;
	step_temp_max = last_average_weighting + MAX_FILTER_WINDOW;
#ifdef SENSOR_DEBUG_OUTPUT
	NRF_LOG_INFO("mag_update_value: %d, average_weighting: %d, min: %d, max: %d", (int16_t)(mag_update_value*1000),(int16_t)(average_weighting*1000),(int16_t)(step_temp_min*1000),(int16_t)(step_temp_max*1000));
#endif

	/* ============= Step 2: State machine for zero crossing ============= */
	switch(step_state)
	{
		case eSTEP_RESET:
		{
			if((ui8_movecnt > ACCEL_MOVE_COUNT_MIN)) // start moving
			{
				step_state = eSTEP_START_PEAK;
				valleymax = step_temp_min;
				peakmax = step_temp_max;
			}
		}
		break;

		case eSTEP_START_PEAK:
		{
			if((mag_update_value > step_temp_max) && (mag_update_value < peakmax))
			{
				valleymax = mag_update_value;
				step_state = eSTEP_PEAK_DETECT;
			}
			else if(mag_update_value > peakmax)
			{
				peakmax = mag_update_value;
			}
		}
		break;

		case eSTEP_PEAK_DETECT:
		{
			if(mag_update_value < step_temp_min)
			{
				step_state = eSTEP_STRAT_VALLEY;
				valleymax = mag_update_value;
			}
			else if (mag_update_value >= peakmax)
			{
				peakmax = mag_update_value;
			}
		}
		break;

		case eSTEP_STRAT_VALLEY:
		{
			if((mag_update_value > valleymax))
			{
				step_state = eSTEP_STEP_DETECT;
			}
			else
			{
				valleymax = mag_update_value;
			}
		}
		break;

		case eSTEP_STEP_DETECT:
		{
			if((mag_update_value >step_temp_max))
			{
				ui32_total_step++;
				ui32_step_detect_number = ui32_step_sameple_counter;
				valleymax = step_temp_min;
				peakmax = step_temp_max;
				step_state = eSTEP_START_PEAK;
			}
			else if(mag_update_value < valleymax)
			{
				valleymax = mag_update_value;
			}
		}
		break;

		default:
			break;
	}
#ifdef SENSOR_DEBUG_OUTPUT
	NRF_LOG_INFO("step_state: %d, peakmax: %d, valleymax: %d", step_state,(int16_t)(peakmax*1000),(int16_t)(valleymax*1000));
#endif
	ui32_step_sameple_counter ++;
}

/**@brief Function for Simple accelerometer offset calibration.
 */  
void accel_calibration(void)
{  
	char X_offset, Y_offset, Z_offset;
	uint8_t accel_data[6] = {0};
	uint16_t Xout_12_bit,Yout_12_bit,Zout_12_bit;

	/* Standby Mode */
	accel_standby();
  
	/* Read register 0x01~0x06(raw x,y,z) */
	accel_burst_read_reg(OUT_X_MSB_REG, accel_data, 6);

	Xout_12_bit = ((short) (accel_data[0]<<8 | accel_data[1])) >> 4; // Compute 12-bit X-axis acceleration output value
	Yout_12_bit = ((short) (accel_data[2]<<8 | accel_data[3])) >> 4; // Compute 12-bit Y-axis acceleration output value
	Zout_12_bit = ((short) (accel_data[4]<<8 | accel_data[5])) >> 4; // Compute 12-bit Z-axis acceleration output value

	X_offset = Xout_12_bit / 2 * (-1); // Compute X-axis offset correction value
	Y_offset = Yout_12_bit / 2 * (-1); // Compute Y-axis offset correction value
	Z_offset = (Zout_12_bit - ACCEL_SENSITIVITY_CONFIG) / 2 * (-1); // Compute Z-axis offset correction value

	accel_write_reg(OFF_X_REG, X_offset);
	nrf_delay_ms(2);
	accel_write_reg( OFF_Y_REG, Y_offset);
	nrf_delay_ms(2);
	accel_write_reg(OFF_Z_REG, Z_offset);
	nrf_delay_ms(2);

	accel_wake_up(); // Active mode again
}

/**@brief Function for populating simulated cycling speed measurements.
 */
ret_code_t accel_csc_measurement(ble_cscs_meas_t * p_measurement)
{
	#define ACCEL_EVENT_TIME_FACTOR 						1.024f

    static uint16_t ui16_last_total_time = 0, ui16_last_event_time = 0;	// total time is 1000-based, event time is 1024-based time
    static uint32_t ui32_last_step_sameple = 0, ui32_last_step_detect = 0;

    uint16_t event_time_inc = 0, total_time_diff = 0, ui16_wheel_event_time = 0;			// event time is 1024-based time
   	uint16_t current_lap = 0, average_speed_kmh = 0;
    uint16_t current_sample = 0;

    if(rw_lock_get())	// if rw_lock is ture means the critical global variable is updating.
    {
    	NRF_LOG_INFO("Warning!!! Read and Write collision, skip report");
    	return NRF_ERROR_INVALID_STATE;
    }
    else if((ui8_movecnt > 0) && !acc_speed_high_flag) // if still moving
    {
    	/* if the lap remains unchanged and speed low than 2.1km, do not send CSC report */
    	if((ui32_last_lap == ui32_total_step))
    	{
#ifdef SENSOR_DEBUG_OUTPUT
    		NRF_LOG_INFO("accel_csc_measurement(): lap without change, skip report");

			ble_cscs_meas_t cscs_measurement;
			cscs_measurement.is_wheel_rev_data_present = true;
			cscs_measurement.is_crank_rev_data_present = true;
			cscs_measurement.cumulative_wheel_revs = 0xAAAAAAAA;
			cscs_measurement.last_wheel_event_time = 0xAAAA;
			cscs_measurement.cumulative_crank_revs = (uint16_t)(0XAAAA);
			cscs_measurement.last_crank_event_time = (uint16_t)(ui8_movecnt);
			accel_csc_meas_timeout_handler2(cscs_measurement);
#endif
    		return NRF_ERROR_INVALID_STATE;
    	}
    }

	total_time_diff = (ui16_total_time>ui16_last_total_time)? ui16_total_time - ui16_last_total_time : ui16_total_time + DEF_TOTAL_TIME_STAMP_MAXIMUM - ui16_last_total_time;

	/* step based time event calculation*/
	current_sample = (uint16_t)(ui32_step_detect_number - ui32_last_step_detect);
	event_time_inc = (uint16_t)((float)(current_sample *((float)total_time_diff/(float)(ui32_step_sameple_counter - ui32_last_step_sameple)))*ACCEL_EVENT_TIME_FACTOR);
	ui16_wheel_event_time = (event_time_inc != 0)? ui16_last_event_time + event_time_inc : (ui16_total_time * ACCEL_EVENT_TIME_FACTOR);

	/* speed calculation */
	current_lap = ui32_total_step - ui32_last_lap;
	/* average speed = current lap * circumference (mm) * 36000(s) / ((wheel event - last event time)/1024*1000) */
	average_speed_kmh = (float)(current_lap * DEF_ANGLE_360_DEGREE * ANGLE_SPEED_TO_METER_PER_HOUR *ACCEL_EVENT_TIME_FACTOR/ (event_time_inc));

	/* speed check */
	if(average_speed_kmh > ACCEL_MOVE_SPEED_HIGH_MH)
	{
		if(!acc_speed_high_flag)
		{
			acc_speed_high_flag = true;
		}
	}
	else if(average_speed_kmh > ACCEL_MOVE_SPEED_MED_MH)
	{
		// Do nothing
	}
	else
	{
		if(acc_speed_high_flag)
		{
			acc_speed_high_flag = false;
		}
	}

	p_measurement->is_wheel_rev_data_present = true;
	p_measurement->is_crank_rev_data_present = false;
	p_measurement->cumulative_wheel_revs = ui32_total_step;
	p_measurement->last_wheel_event_time = (uint16_t)ui16_wheel_event_time;
#ifdef SENSOR_DEBUG_OUTPUT
	/* for test log */
	p_measurement->is_crank_rev_data_present = true;
	p_measurement->cumulative_crank_revs 	 = (uint16_t)(current_sample);
	p_measurement->last_crank_event_time	 = (uint16_t)(average_speed_kmh/100);
#endif
	/* last lap depend on angle mode or step mode */
	ui32_last_lap = ui32_total_step;
	ui32_last_step_sameple = ui32_step_sameple_counter;
	ui32_last_step_detect = ui32_step_detect_number;
	ui16_last_event_time = ui16_wheel_event_time;
	ui16_last_total_time = ui16_total_time;
#ifdef SENSOR_DEBUG_OUTPUT
	NRF_LOG_INFO("current_sample: %d, event_time: %d, total_time_diff: %d, Speed: %d kmh", current_sample, ui16_wheel_event_time , total_time_diff, average_speed_kmh);
	NRF_LOG_INFO("sameple_counter: %d, step_detect_num: %d, event_time_inc: %d", ui32_step_sameple_counter, ui32_step_detect_number, event_time_inc );
#endif
    return NRF_SUCCESS;
}

/**@brief Function initialize function.
 */
void accel_init(void)
{
	NRF_LOG_INFO("accel_init.");
#ifndef CSCS_MOCK_ENABLE
	/* hardware initialize - I2C & GPIO */
	accel_i2c_gpio_init();

	/* configuration */
	accel_configuration();

#ifndef CSCS_MOCK_ENABLE
	/* enable gpio */
	accel_i2c_gpio_enable();
#endif

#endif
	/* timer init */
	accel_timers_init();

#if (defined(ACCELEROMETER_SELF_TIMEOUT) && !defined(ACCELEROMETER_DUMP_FIFO))|| defined(CSCS_MOCK_ENABLE)
	application_timers_start();
#endif

	/* reset acc step angle parameters */
	acc_step_reset_angle();
}
#endif /* SENSOR_ACCELEROMETER_C_ */
