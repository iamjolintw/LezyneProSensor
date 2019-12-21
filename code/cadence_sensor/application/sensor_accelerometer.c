/*
 * 	sensor_accelerometer.c
 *
 *  Created on: June 30, 2019
 *  Author: Lezyne
 *  		History: 2019/07/01	 first version Angle algorithm
 *  				 2019/07/04  2nd version Angle algorithm, simplified, with forward and backwards judgment
 *  				 2019/07/31  3nd version zero crossing algorithm with two different average threshold for high and low speed
 *					 2019/08/07  Cycling Cadence version
 *                   2019/09/25  porting 4nd version zero crossing algorithm from Speed sensor
 *                   2019/11/16  New Cadence algorithm
 *					 2019/12/10  add selftest function
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
static const nrf_drv_twi_t acce_m_twi = NRF_DRV_TWI_INSTANCE(ACC_TWI_INSTANCE_ID);

/* Configuration */
//#define SENSOR_ALWAYS_REPORT_DEBUG		// test purpose: always report measurement even speed is zero
//#define SENSOR_DEBUG_OUTPUT				// test purpose: switch for log message

/* Data rate 10ms = 100Hz, the highest bicycle speed = 75 Km/h = 20.83 m/s,
   max RPM = 20.83 / 2m (circumference) = 10.41 Hz, but needs at least 4 samples for calculation = 41.64 Hz ~= 50Hz */
#ifndef ACCELEROMETER_DUMP_FIFO
#ifndef ACC_ST16G_ENABLE
#define ACCEL_DATARATE 							DATA_RATE_10MS
#define ACCEL_DATARATE_MS 						10			/* ACCEL_DATARATE_MS shall match to ACCEL_DATARATE */
#else
#define ACCEL_LIS2DE12_DATARATE 				LIS2DE12_REG1_100HZ
#define ACCEL_DATARATE_MS 						10			/* ACCEL_DATARATE_MS shall match to ACCEL_DATARATE */
#define ACCEL_LIS2DE12_LMP_DATARATE 			LIS2DE12_REG1_10HZ
#endif
#else
#define ACCEL_DATARATE 							DATA_RATE_10MS
#define ACCEL_DATARATE_MS 						10			/* ACCEL_DATARATE_MS shall match to ACCEL_DATARATE */
#define ACCEL_LIS2DE12_DATARATE 				LIS2DE12_REG1_100HZ
#define ACCEL_LIS2DE12_LMP_DATARATE 			LIS2DE12_REG1_10HZ
#endif


#ifndef ACC_ST16G_ENABLE
/* ASLP_RATE_160MS = 6.25 hz */
#define ACCEL_SLEEPP_DATARATE					ASLP_RATE_160MS
/* Threshold = 0x18; The step count is 0.063g/ count,  (1.25g/0.063g ~= 20 =0x14) */
#define ACCEL_FF_MT_THS_VALUE  					(THS4_MASK | THS2_MASK)
/* Set the debounce counter to eliminate false readings */
#define ACCEL_FF_MT_DEBOUNCE_COUNT 				10
#else
/* Threshold = 0x9; The step count is 0.127g/ count,  (1.05g/0.127g ~= 8) */
#define ACCEL_FF_MT_THS_VALUE  					20
/* Set the debounce counter to eliminate false readings */
#define ACCEL_FF_MT_DEBOUNCE_COUNT 				0
#endif
/* Set compensate time */
#define ACCEL_PROCESS_COMPENSATE_TIME			0		// unit:ms
/* FIFO Event Sample Count Watermark, the value shall be less than 64 */
#define DEF_WATERMARK_VAL 						25
/* Minimum moving angle (degree per 0.25s (25*10ms)), degree 20 equal 13 RPM (30x(1000/ACCEL_DATARATE_MS/DEF_WATERMARK_VAL)/360*60) */
#define ACCEL_MOVE_ANGLE_MIN					20
/* Maximum moving RPM */
#define ACCEL_MOVE_RPM_MAX						214
#define	ACCEL_MOVE_RPM_MIN						15
#define	ACCEL_WEIGHT_RPM_DIFF					15
#ifndef ACC_ST16G_ENABLE
/* Moving counter: 6 movements in 25*10ms = 1.5s time window */
#define ACCEL_MOVE_COUNT_MIN					6
/* Non-moving counter: (10 - 6)*25*10ms = 1.0s time window */
#define ACCEL_MOVE_COUNT_MAX 					10
/* Threshold of square of accelerometer value */
#define ACCEL_MOVE_SPEED_MID_G_VALUE 			1.2f	//accelerometer value >1.09 or <-1.09
#define ACCEL_MOVE_SPEED_HIGH_G_VALUE 			4.0f	//accelerometer value >2or <-2
#else
/* Moving counter: 3 movements in 25*10ms = 0.75s time window */
#define ACCEL_MOVE_COUNT_MIN 					2
/* Non-moving counter: (10 - 2)*25*10ms = 2s time window */
#define ACCEL_MOVE_COUNT_MAX 					10
/* Threshold of square of accelerometer value */
//#define ACCEL_MOVE_SPEED_MID_G_VALUE 			2.25f	//accelerometer value >1.5 or <-1.5
//#define ACCEL_MOVE_SPEED_HIGH_G_VALUE 			10.0f	//accelerometer value >3.x or <-3.x
#endif

/* If last_sample_counter smaller than it, should not check current sample_counter (200 RPM) */
//#define ACCE_LAST_SAMPLE_COUNTER_MIN 			30
/* If last_sample_counter larger than it, should not check current sample_counter (40 RPM) */
//#define ACCE_LAST_SAMPLE_COUNTER_MAX 			100

/* Speed calculation interval (milliseconds) */
#define ACCEL_SPEED_AND_CADENCE_CAL_INTERVAL 	SPEED_AND_CADENCE_MEAS_INTERVAL

APP_TIMER_DEF(m_csc_meas_timer_id);                                                 /**< CSC measurement timer. */

/* 12bit xyz fifo data */
static uint8_t accel_buff[6*DEF_WATERMARK_VAL] = {0};
/* Pending task indicator */
static t_accel_task_pending task_pending_singal = {0};
/* Protector for critical global variable */
static volatile bool rw_lock_protect_flag = false;
/* acc angle variable */
static int16_t 	last_angle_residue = DEF_INVALID_LAST_ANGLE;
/* Time stamp: range: 0 ~ 256 seconds. Needs to take care of wrap-around */
static uint16_t ui16_total_time = 0;
/* Stationary counter: to indicate sensor is moving or not */
static uint8_t 	ui8_movecnt = 0;
/* Flag for switching the high speed algorithm or low speed algorithm */
static t_accel_speed_flag 	acc_speed_high_flag = ACCE_SPEED_LOW;
/* Flag for force report cadence */
static bool 	acc_meas_report_flag = false;
/* lap indicator: range: 65535 laps will overflow. it is around 9.1 hours with the 120 RPM speed */
static uint16_t ui16_total_step = 0;
/* Sample number for zero crossing judgment: range: 0 ~ 11930 hours */
static uint32_t ui32_step_sample_counter = 0;
/* Sample number for detect a lap: range: 0 ~ 11930 hours */
static uint32_t ui32_step_detect_number = 0;
/* Last average value of zero crossing judgment */
static float last_average_weighting = 0 ;
/* The average of one shot rpm */
static uint16_t  oneshot_rpm_avg = 0;
static float 	EMA_low = 0, EMA_high = 0;


/* test purpose: FIFO dump */
#ifdef ACCELEROMETER_DUMP_FIFO
//static int16_t  x_sample2[DEF_WATERMARK_VAL] = {0} ;
static int16_t  y_sample2[DEF_WATERMARK_VAL] = {0} ;
//static int16_t  z_sample2[DEF_WATERMARK_VAL] = {0} ;
#endif

/* FUNCTIONS */
#ifndef ACCELEROMETER_DUMP_FIFO
static void 		application_timers_start(void);
static void 		application_timers_stop(void);
#endif
static void 		acc_step_update_angle(uint16_t *angle_array, float *mag_array);
static void 		acc_step_mag_update(float mag_update_value);
static void 		accel_display_reg(void);
static ret_code_t 	accel_write_reg(uint8_t reg_addr, uint8_t reg_data);
static ret_code_t 	accel_read_reg(uint8_t reg_addr, uint8_t *reg_data);
static ret_code_t 	accel_burst_read_reg(uint8_t addr, uint8_t * pdata, size_t size);
static ret_code_t 	accel_i2c_init(void);
static void 		accel_gpio_init(void);
static void 		accel_config_fifo_int(bool enable);
static void 		accel_config_motion_int(bool enable);
static void 		accel_wake_up(void);
static void 		accel_standby(void);
static float 		lowPassExponential(float input, float average, float factor, float limited_factor);
static float 		movingAvg(float input);
static float 		rpmAvg(uint16_t input);
static float 		bandpassEMA(float input);
static ret_code_t 	rw_lock_set(bool config);
static void 		accel_display_reg(void);
static void 		speed_flag_set(uint16_t current_speed);
static bool		 	mid_speed_flag_get(void);
#ifdef ACC_ST16G_ENABLE
static void 		accel_lis2de12_reset_reg(void);
static ret_code_t 	lis2dh12_wait_for_data(int timeout_ms);
static ret_code_t 	accel_dump_reg_info(uint8_t * ctrl_reg , uint8_t size);
#endif

/**@brief handler for interrupt of input pin1, for Motion Detection event
 */
void int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
#ifndef ACC_ST16G_ENABLE

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
#else
	uint8_t fifo_src =  (LIS2DE12_FIFO_SRC_REG_CNT_MSK | LIS2DE12_FIFO_SRC_REG_WTM);
	//accel_read_reg(LIS2DE12_FIFO_SRC_REG, 	&fifo_src);
#ifdef SENSOR_DEBUG_OUTPUT
	uint8_t int_src = 0, ui8_int_ref = 0;
	accel_read_reg(LIS2DE12_INT_REFERENCE, 	&ui8_int_ref);
	accel_read_reg(LIS2DE12_INT1_SRC_REG, 	&int_src);
	accel_read_reg(LIS2DE12_FIFO_SRC_REG, 	&fifo_src);
	NRF_LOG_INFO("LIS2DE12_int1 INT1_SRC: %x, int_ref: %x, fifo_src: %x ", int_src, ui8_int_ref, fifo_src);
#endif
	/* FIFO Interrupt */
	if ((fifo_src & LIS2DE12_FIFO_SRC_REG_WTM))
	{
		ret_code_t result;
		/* watermark is not full, wait until watermark is full */
		//if((fifo_src & (LIS2DE12_FIFO_SRC_REG_CNT_MSK)) < (DEF_WATERMARK_VAL))
		//{
		//	NRF_LOG_INFO("Warning!!! FIFO watermark is not full, do not need to read buffer.");
		//	return;
		//}
		/* rise SENSOR_TASK_INT_FIFO to process the data of buffer in idle_state_handle() later */
#ifdef ACCELEROMETER_DUMP_FIFO
		result = accel_task_enable_mask(SENSOR_TASK_INT_DUMP);
#else
		result = accel_task_enable_mask(SENSOR_TASK_INT_FIFO);
#endif
		if(result == NRF_SUCCESS)
		{
			/* read 12bit fifo data */
			result = accel_burst_read_reg(LIS2DE12_OUT_FIFO, accel_buff, 6*DEF_WATERMARK_VAL);
		}
		else
		{
			/* error handling for no time to process, skip new data */
			uint8_t new_accel_buff[6*DEF_WATERMARK_VAL+12] = {0};
			result = accel_burst_read_reg(LIS2DE12_OUT_FIFO, new_accel_buff, 6*DEF_WATERMARK_VAL);
			accel_read_reg(LIS2DE12_FIFO_SRC_REG, &fifo_src);
		}
	}
#endif
}

/**@brief handler for interrupt of input pin2 for FIFO interrupt and Auto-WAKE/SLEEP event
 */
void int2_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
#ifndef ACC_ST16G_ENABLE
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

#else
#ifdef SENSOR_DEBUG_OUTPUT
	uint8_t int_src = 0, fifo_src = 0, ui8_int_ref = 0;
	accel_read_reg(LIS2DE12_INT_REFERENCE, 	&ui8_int_ref);
	accel_read_reg(LIS2DE12_INT1_SRC_REG, 	&int_src);
	accel_read_reg(LIS2DE12_FIFO_SRC_REG, 	&fifo_src);
	NRF_LOG_INFO("LIS2DE12_int2 INT2_SRC: %x, FIFO_SRC: %x, REF: %x", int_src, fifo_src, ui8_int_ref);

    uint32_t pin1_status = nrf_gpio_pin_read(ACC_INT1_PIN);
    uint32_t pin2_status = nrf_gpio_pin_read(ACC_INT2_PIN);
   	NRF_LOG_INFO(" gpio1: %d, gpio2: %d", pin1_status, pin2_status);
#endif
#endif
}

/**@brief Function for the Timer initialization.
 */
static void accel_timers_init(void)
{
    /* Create cycle speed and Cadence timer */
	ret_code_t err_code = app_timer_create(&m_csc_meas_timer_id,
                                APP_TIMER_MODE_REPEATED,
    							accel_csc_meas_timeout_handler);		// timeout handler in ble_core.c
    APP_ERROR_CHECK(err_code);
}

#ifndef ACCELEROMETER_DUMP_FIFO
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
#endif //ifndef ACCELEROMETER_DUMP_FIFO

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
	ret_code_t err_code = nrf_drv_twi_tx(&acce_m_twi, ACC_I2C_ADDRESS, data, 2, false);

    nrf_delay_ms(1);
	if (err_code)
		NRF_LOG_INFO("accel_write_reg error\r\n");
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
	err_code = nrf_drv_twi_tx(&acce_m_twi, ACC_I2C_ADDRESS, &addr8, 1, true);

	nrf_delay_ms(2);	// LIS2DH12 shall delay 2 ms after I2C write to avoid exception
	if (err_code)
	{
		NRF_LOG_INFO("accel_I2C_read (W) error(0x%X). retry",err_code);
		//retry
		err_code = nrf_drv_twi_tx(&acce_m_twi, ACC_I2C_ADDRESS, &addr8, 1, true);
		nrf_delay_ms(2); // LIS2DH12 shall delay 2 ms after I2C write to avoid exception
	}
	/* read data */
	err_code = nrf_drv_twi_rx(&acce_m_twi, ACC_I2C_ADDRESS, reg_data, 1);
	if (err_code)
	{
		NRF_LOG_INFO("accel_I2C_read (R) error(0x%X) -[reg_addr:%x].",err_code, reg_addr);
		//retry
		err_code = nrf_drv_twi_rx(&acce_m_twi, ACC_I2C_ADDRESS, reg_data, 1);
	}
	if(reg_addr != LIS2DE12_WHO_AM_I)
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
#ifndef ACC_ST16G_ENABLE
	uint8_t addr8 = (uint8_t)reg_addr;
#else
	uint8_t addr8 = (uint8_t)(reg_addr | 0x80);
#endif
	/* write register address */
	err_code = nrf_drv_twi_tx(&acce_m_twi, ACC_I2C_ADDRESS, &addr8, 1, true);
	nrf_delay_ms(3); // LIS2DH12 shall delay 2 ms after I2C write to avoid exception
	if (err_code)
	{
		NRF_LOG_INFO("accel_burst_read_reg (W) error(0x%X).retry",err_code);
		err_code = nrf_drv_twi_tx(&acce_m_twi, ACC_I2C_ADDRESS, &addr8, 1, true);
		nrf_delay_ms(2); // LIS2DH12 shall delay 2 ms after I2C write to avoid exception
	}
	/* read data */
	err_code = nrf_drv_twi_rx(&acce_m_twi, ACC_I2C_ADDRESS, reg_data, size);
	if (err_code)
	{
		NRF_LOG_INFO("accel_burst_read_reg (R) error(0x%X). retry",err_code);
		nrf_delay_ms(2); // LIS2DH12 shall delay 2 ms after I2C write to avoid exception
		err_code = nrf_drv_twi_rx(&acce_m_twi, ACC_I2C_ADDRESS, reg_data, size);
	}
	//APP_ERROR_CHECK(err_code);		// do not cause fatal error within burst read 
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
#define RW_LOCK_GET()	rw_lock_protect_flag
//static bool rw_lock_get(void)
//{
//	return rw_lock_protect_flag;
//}

/**	@brief : speed_flag_set
*	@param : value current_speed_sample turn into speed in meter per hour
*/
static void speed_flag_set(uint16_t current_rpm)
{
	/* Threshold of square of accelerometer value */
	#define ACCEL_MOVE_RPM_HIGH			200 // RPM  (3.5 lap per second *60)
	#define ACCEL_MOVE_RPM_MEDHIGH		110 // RPM  (2.5 lap per second *60)
	#define ACCEL_MOVE_RPM_MED			20 	// RPM  (1.5 lap per second *60)

	// last_average_weighting is the average line help to check unexpected cases
	if((current_rpm > ACCEL_MOVE_RPM_HIGH))
	{
		acc_speed_high_flag = ACCE_SPEED_HIGH;
	}
	else if(current_rpm > ACCEL_MOVE_RPM_MEDHIGH)
	{
		acc_speed_high_flag = ACCE_SPEED_MIDHIGH;
	}
	else if(current_rpm > ACCEL_MOVE_RPM_MED)
	{
		acc_speed_high_flag = ACCE_SPEED_MID;
	}
	else
	{
    	acc_speed_high_flag = ACCE_SPEED_LOW;
   	}
}

/**	@brief : mid_speed_flag_get
*  	@Return acc_speed_high_flag is mid speed or not
*/
static bool mid_speed_flag_get(void)
{
	if(acc_speed_high_flag == ACCE_SPEED_LOW)
	{
		return false;
	}
	else
	{
		return true;
	}
}

/**@brief Function for the sensor accelerometer configuration.
 */
static void accel_configuration(void)
{
	NRF_LOG_INFO("accel_configuration.");

	uint8_t who_n_i = 0;

	/* read WHO_AND_I first */
#ifndef ACC_ST16G_ENABLE
	accel_read_reg(MMA8652_WHO_AM_I, &who_n_i);
	if(who_n_i != MMA8652_WHO_AM_I_OUT)
	{
		NRF_LOG_ERROR(" Device ID not match!! :MMA8652 0x4A: 0x%x", who_n_i);
		return;
	}

    /* RESET sensor, all registers are reset to default */
    accel_write_reg(MMA8652_CTRL_REG2, RST_MASK);
	uint8_t reset_status = 0;
    do {
    	nrf_delay_ms(5);
    	accel_read_reg(MMA8652_CTRL_REG2, &reset_status);
    } while (reset_status & RST_MASK);

	/* Set to standby mode */
    accel_standby();
    uint8_t ui8_temp = 0;
    /* clear interrupt in case there were something vestigial */
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
	accel_write_reg(MMA8652_FF_MT_CFG, OAE_MASK | XEFE_MASK);
	/* Set MMA8652_FF_MT_THS, setup THS = ACCEL_FF_MT_THS_VALUE */
	accel_write_reg(MMA8652_FF_MT_THS, ACCEL_FF_MT_THS_VALUE | DBCNTM_MASK);
	/* Set MMA8652_FF_MT_COUNT, setup debounce counter ACCEL_FF_MT_DEBOUNCE_COUNT */
	accel_write_reg(MMA8652_FF_MT_COUNT, ACCEL_FF_MT_DEBOUNCE_COUNT);

	/* Turn off HPF for Data Out and set 8g Mode, SENSITIVITY_8G */
	accel_write_reg(MMA8652_XYZ_DATA_CFG, ACCEL_FULL_SCALE_REG);
	accel_write_reg(MMA8652_HP_FILTER_CUTOFF, PULSE_LPF_EN_MASK | PULSE_HPF_BYP_MASK);
#else
	/* read WHO_AND_I first */
	accel_read_reg(LIS2DE12_WHO_AM_I, &who_n_i);
	if(who_n_i != LIS2DE12_WHO_AM_I_OUT)
	{
		NRF_LOG_ERROR(" Device ID not match!! : who_n_i: 0x%x", who_n_i);
		nrf_drv_gpiote_out_clear(ACC_LEDR_PIN);
		return;
	}
	else
	{
		NRF_LOG_INFO("Sensor type: LIS2DE12.");
	}

	/* Reset chip */
	nrf_delay_ms(2);
	accel_lis2de12_reset_reg();
	nrf_delay_ms(50);

	/* Enable YZ axes and 10hrz data, enable the sensor */
	accel_write_reg(LIS2DE12_REG1, ACCEL_LIS2DE12_LMP_DATARATE | LIS2DE12_REG1_LOW_POWER | LIS2DE12_REG1_ALL_AXES);
	/* FIFO bypass, on INT2 */
	accel_write_reg(LIS2DE12_FIFO_CTRL_REG, DEF_WATERMARK_VAL);
	/* Enable the FIFO Watermark Interrupt and Set it to INT1 */
	accel_write_reg(LIS2DE12_REG3, LIS2DE12_REG3_FIFO_WTR_INT1);
	/*  BDU and 16G Scale */
	accel_write_reg(LIS2DE12_REG4, LIS2DE12_REG4_BDU_READ | ACCEL_FULL_SCALE_REG | LIS2DE12_REG4_HR_MSK);
	/* Use the FIFO */
	accel_write_reg(LIS2DE12_REG5, LIS2DE12_REG5_USE_FIFO);
	/* Motion detection, using XY */
	accel_write_reg(LIS2DE12_INT1_CFG_REG, LIS2DE12_INT1_CFG_YHIE_MSK | LIS2DE12_INT1_CFG_XHIE_MSK);
	accel_write_reg(LIS2DE12_INT1_THRESHHOLD_REG, ACCEL_FF_MT_THS_VALUE);
	accel_write_reg(LIS2DE12_INT1_DURATION_REG, ACCEL_FF_MT_DEBOUNCE_COUNT);
#endif
    /* Set back to active and wake up */
	accel_wake_up();
	accel_display_reg();

	if(accel_lis2de12_selftest() != NRF_SUCCESS)
    {
		nrf_drv_gpiote_out_clear(ACC_LEDR_PIN);
		NRF_LOG_INFO("Self Test Failed!!!");
    }
	else
	{
		/* Turn off the Red light (low enable) once finishing the communication with accelerometer */
		nrf_drv_gpiote_out_set(ACC_LEDR_PIN);
	}
}

#ifdef ACC_ST16G_ENABLE
/**@brief Function accel_lis2de12_reset_reg for reset lis2de12 register value to default.
 */
static void accel_lis2de12_reset_reg(void)
{

    // Set bypass to clear the fifo
	accel_write_reg(LIS2DE12_FIFO_CTRL_REG, 		0x00 );
	nrf_delay_ms(50);
    // Reboot the chip
	accel_write_reg(LIS2DE12_REG5, 					LIS2DE12_REG5_REBOOT );
	nrf_delay_ms(200);

	accel_write_reg(LIS2DE12_REG0, 					0x10 );
	accel_write_reg(LIS2DE12_REG1, 					0x07 );
	accel_write_reg(LIS2DE12_REG2, 					0x00 );
	accel_write_reg(LIS2DE12_REG3, 					0x00 );
	accel_write_reg(LIS2DE12_REG4, 					0x00 );
	accel_write_reg(LIS2DE12_REG5, 					0x00 );
	accel_write_reg(LIS2DE12_REG6, 					0x00 );
	accel_write_reg(LIS2DE12_INT_REFERENCE, 		0x00 );
	accel_write_reg(LIS2DE12_INT1_CFG_REG, 			0x00 );
	accel_write_reg(LIS2DE12_INT1_THRESHHOLD_REG, 	0x00 );
	accel_write_reg(LIS2DE12_INT1_DURATION_REG, 	0x00 );
	accel_write_reg(LIS2DE12_INT2_CFG_REG,		 	0x00 );
	accel_write_reg(LIS2DE12_INT2_THRESHHOLD_REG, 	0x00 );
	accel_write_reg(LIS2DE12_INT2_DURATION_REG, 	0x00 );
	accel_write_reg(LIS2DE12_CLICK_CFG_REG, 		0x00 );
	accel_write_reg(LIS2DE12_CLICK_THS_REG, 		0x00 );
	accel_write_reg(LIS2DE12_TIME_LIMIT_REG, 		0x00 );
	accel_write_reg(LIS2DE12_TIME_LATENCY_REG, 		0x00 );
	accel_write_reg(LIS2DE12_TIME_WINDOW_REG, 		0x00 );
	accel_write_reg(LIS2DE12_ACT_THS_REG, 			0x00 );
	accel_write_reg(LIS2DE12_ACT_DUR_REG, 			0x00 );
}

static ret_code_t lis2dh12_wait_for_data(int timeout_ms)
{
	ret_code_t rc = NRF_SUCCESS;
    uint8_t status;
    uint32_t time_limit = app_timer_cnt_get() + (timeout_ms * APP_TIMER_CLOCK_FREQ);

    while (1) {
        rc = accel_read_reg(LIS2DE12_STATUS, &status);
        if (rc != NRF_SUCCESS || (status & LIS2DE12_STATUS_ZYXDA) != 0) {
            break;
        }
        if (app_timer_cnt_get()  > time_limit) {
            rc = NRF_ERROR_INVALID_STATE;
            break;
        }
    	nrf_delay_ms(1);
    }
    return rc;
}
#endif

/**@brief Function for accel_lis2de12_selftest, only call it when in initiating state
 */
ret_code_t accel_lis2de12_selftest(void)
{
#ifdef ACC_ST16G_ENABLE
	uint8_t ctrl_reg[20];
	// LeyzneProCAD initial register value
	const uint8_t speed_verified_words[20] = {0x10,0x2F,0x00,0x04,0xA8,0x40,0x00,0x19,0x0A,0x14,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	ret_code_t check_flag = NRF_SUCCESS;

	check_flag = accel_dump_reg_info((uint8_t *) ctrl_reg, 20);
	if( check_flag == NRF_SUCCESS)
	{
		//check
		//10-2F-00-04-A8-40-00-19-0A-14-00-00-00-00-00-00-00-00-00
		for(uint8_t i = 0; i<20; i++)
		{
			if(ctrl_reg[i] != speed_verified_words[i])
			{
				check_flag = NRF_ERROR_INVALID_STATE;
				NRF_LOG_INFO("INCONSISTENT:%d,%x",i,ctrl_reg[i] );
			}
		}
		if(check_flag != NRF_SUCCESS)
			return check_flag;
	}
	// check G value
	uint8_t check_data[6] = {0};
	int16_t int16_x = 0, int16_y = 0, int16_z = 0;
	int16_t mx = 0, my = 0, mz = 0; //mg
	#define ACCEL_G_VALUE_CITERIA_MAX 	1500
	#define ACCEL_G_VALUE_CITERIA_MIN 	-1500

	//skip first one shot fifo output data
	check_flag = lis2dh12_wait_for_data(10);
	check_flag += accel_burst_read_reg(LIS2DE12_OUT_FIFO, check_data, 6);
	// one shot fifo data
	check_flag += lis2dh12_wait_for_data(10);
	if(check_flag == NRF_SUCCESS)
	{
		check_flag = accel_burst_read_reg(LIS2DE12_OUT_FIFO, check_data, 6);
		int16_x= (int16_t)((uint16_t)((uint16_t)check_data[1] << 8) | (uint16_t)check_data[0]);
		int16_y = (int16_t)((uint16_t)((uint16_t)check_data[3] << 8) | (uint16_t)check_data[2]);
		int16_z = (int16_t)((uint16_t)((uint16_t)check_data[5] << 8) | (uint16_t)check_data[4]);

		mx = ((float)int16_x*ACCEL_SENSITIVITY_MG_CONFIG)/(float)(16);
		my = ((float)int16_y*ACCEL_SENSITIVITY_MG_CONFIG)/(float)(16);
		mz = ((float)int16_z*ACCEL_SENSITIVITY_MG_CONFIG)/(float)(16);

		NRF_LOG_INFO("X:%d Y:%d Z:%d", mx, my, mz);
		if((mx > ACCEL_G_VALUE_CITERIA_MAX) || ( mx < ACCEL_G_VALUE_CITERIA_MIN)
			|| (my > ACCEL_G_VALUE_CITERIA_MAX) || ( my < ACCEL_G_VALUE_CITERIA_MIN)
			|| (mz > ACCEL_G_VALUE_CITERIA_MAX) || ( mz < ACCEL_G_VALUE_CITERIA_MIN))
		{
			NRF_LOG_INFO("XYZDATA exceed normal range!!");
			check_flag = NRF_ERROR_INVALID_STATE;
		}
	}
	return check_flag;
#else
	return NRF_SUCCESS;
#endif
}

/**@brief Function for debugging.
 */
static void accel_display_reg(void)
{
#ifdef SENSOR_DEBUG_OUTPUT
    uint8_t ctrl_reg[13];
#ifndef ACC_ST16G_ENABLE
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
#else
	accel_read_reg(LIS2DE12_FIFO_CTRL_REG, 	&ctrl_reg[0]);
	accel_read_reg(LIS2DE12_REG1, 			&ctrl_reg[1]);
	accel_read_reg(LIS2DE12_REG2,			&ctrl_reg[2]);
	accel_read_reg(LIS2DE12_REG3, 			&ctrl_reg[3]);
	accel_read_reg(LIS2DE12_REG4,			&ctrl_reg[4]);
	accel_read_reg(LIS2DE12_REG5,			&ctrl_reg[5]);
	accel_read_reg(LIS2DE12_REG6,			&ctrl_reg[6]);

	NRF_LOG_INFO("FIFO_CTRL:%x, R1:%x, R2:%x, R3:%x ", 	ctrl_reg[0], 	ctrl_reg[1], 	ctrl_reg[2], 	ctrl_reg[3]);
	NRF_LOG_INFO("R4:%x, R5:%x, R6:%x. ", 				ctrl_reg[4], 	ctrl_reg[5], 	ctrl_reg[6]);
#endif
#endif
}

/**@brief Function for enable of disable fifo interrupt.
 *	bool enable: disable:0, enable:1		
 */
static void accel_config_fifo_int(bool enable)
{
#ifndef ACC_ST16G_ENABLE
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
#else
	uint8_t fifo_ctrl_reg = 0;
	accel_read_reg(LIS2DE12_FIFO_CTRL_REG, &fifo_ctrl_reg);
	if(enable)
	{
		accel_write_reg(LIS2DE12_FIFO_CTRL_REG, fifo_ctrl_reg | LIS2DE12_FIFO_STREAM);
	}
	else
	{
		accel_write_reg(LIS2DE12_FIFO_CTRL_REG, fifo_ctrl_reg & ~LIS2DE12_FIFO_FM_MSK);
	}
#endif
}

/**@brief Function for enable or disable motion detection interrupt.
 *	bool enable: disable:0, enable:1		
 */
static void accel_config_motion_int(bool enable)
{
#ifndef ACC_ST16G_ENABLE
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
#else
	if(enable)
	{
		accel_write_reg(LIS2DE12_REG6, LIS2DE12_REG6_INT1_FUNC_INT2);
		accel_write_reg(LIS2DE12_REG4, LIS2DE12_REG4_BDU_READ | ACCEL_FULL_SCALE_REG);
		accel_write_reg(LIS2DE12_REG1, ACCEL_LIS2DE12_LMP_DATARATE | LIS2DE12_REG1_LOW_POWER | LIS2DE12_REG1_ALL_AXES);
	}
	else
	{
		accel_write_reg(LIS2DE12_REG6, 0x00);
		accel_write_reg(LIS2DE12_REG4, LIS2DE12_REG4_BDU_READ | ACCEL_FULL_SCALE_REG | LIS2DE12_REG4_HR_MSK);
		accel_write_reg(LIS2DE12_REG1, ACCEL_LIS2DE12_DATARATE | LIS2DE12_REG1_ALL_AXES);
	}
#endif	
}

/**@brief Initialize I2C (TWI).
 */
static ret_code_t accel_i2c_init(void)
{
    const nrf_drv_twi_config_t twi_config = {
       .scl                = ACC_I2C_SCL_PIN,
       .sda                = ACC_I2C_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false,
       .hold_bus_uninit    = false
    };
    ret_code_t err_code = nrf_drv_twi_init(&acce_m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&acce_m_twi);
	return err_code;
}

#ifdef ACC_ST16G_ENABLE
/**@brief Initialize pin switch for choosing I2C slave address.
 */
static ret_code_t accel_i2c_opt(void)
{
	nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true);
	ret_code_t err_code = nrf_drv_gpiote_out_init(ACC_I2C_OPTION, &out_config);
	APP_ERROR_CHECK(err_code);
	return err_code;
}
#endif

/**@brief start gpio.
 */
static void accel_gpio_init(void)
{
#ifndef ACC_ST16G_ENABLE
    nrf_drv_gpiote_in_config_t in_config1 = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    ret_code_t err_code = nrf_drv_gpiote_in_init(ACC_INT1_PIN, &in_config1, int1_handler);
	APP_ERROR_CHECK(err_code);

	nrf_drv_gpiote_in_config_t in_config2 = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    err_code = nrf_drv_gpiote_in_init(ACC_INT2_PIN, &in_config2, int2_handler);
    APP_ERROR_CHECK(err_code);
#else
	nrf_drv_gpiote_in_config_t in_config1 = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
	in_config1.pull = NRF_GPIO_PIN_PULLDOWN;
	ret_code_t err_code = nrf_drv_gpiote_in_init(ACC_INT1_PIN, &in_config1, int1_handler);
	APP_ERROR_CHECK(err_code);

	nrf_drv_gpiote_in_config_t in_config2 = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
	in_config2.pull = NRF_GPIO_PIN_PULLDOWN;
    err_code = nrf_drv_gpiote_in_init(ACC_INT2_PIN, &in_config2, int2_handler);
    APP_ERROR_CHECK(err_code);
#endif
	nrf_drv_gpiote_in_event_enable(ACC_INT2_PIN, true);
	nrf_drv_gpiote_in_event_enable(ACC_INT1_PIN, true);
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
#ifndef ACCELEROMETER_DUMP_FIFO
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
#ifndef ACCELEROMETER_DUMP_FIFO
	application_timers_start();
#endif
	/* Set back to active */
	accel_wake_up();
	accel_display_reg();
}

/**@brief Function for set the sensor accelerometer to ACTIVATE mode.
 *	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE			
 */
void accel_wake_up(void)
{
#ifndef ACC_ST16G_ENABLE
	/* Read REG1 byte first; then enable ACTIVE bit for going to activate mode */
	uint8_t ctrl_reg1;
	accel_read_reg(MMA8652_CTRL_REG1, 	&ctrl_reg1);
	accel_write_reg(MMA8652_CTRL_REG1, 	ctrl_reg1 | ACTIVE_MASK);
	nrf_delay_ms(50);
#else
	uint8_t ui8_temp;
	//accel_write_reg(LIS2DE12_REG1, ACCEL_LIS2DE12_DATARATE | LIS2DE12_REG1_ALL_AXES); // do not config data rate here. set it in accel_config_motion_int.
	nrf_delay_ms(50);
	accel_read_reg(LIS2DE12_INT_REFERENCE, &ui8_temp);
#endif
}

/**@brief Function for set the sensor accelerometer to STANDBY mode.
 * 	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE
 */
void accel_standby(void)
{
    /* Read REG1 byte first; then disable ACTIVE bit for going to standby mode */
	uint8_t ctrl_reg1;
#ifndef ACC_ST16G_ENABLE
	accel_read_reg(MMA8652_CTRL_REG1, 	&ctrl_reg1);
	accel_write_reg(MMA8652_CTRL_REG1, 	ctrl_reg1 & ~ACTIVE_MASK);
#else
	accel_read_reg(LIS2DE12_REG1, 	&ctrl_reg1);
	accel_write_reg(LIS2DE12_REG1, 	ctrl_reg1 & ~LIS2DE12_REG1_ACTIVE_MSK);
#endif	
	nrf_delay_ms(50);
}

/**@brief Function lowPassExponential fifo accelerometer data.
 *	ensure factor belongs to  [0,1], limited_factor is for trimming off unexpected distance
 */
static float lowPassExponential(float input, float average, float factor, float limited_factor)
{
	float low_pass_diff = (input - average);
	if(low_pass_diff < 0) // to make it go down easy, go up with penalty
	{
		if((average - input) > limited_factor)
			low_pass_diff = limited_factor * (-1);
		return (average + factor * (low_pass_diff));
	}
	else
	{
		if((input - average) > limited_factor)
			low_pass_diff = limited_factor;
		return (average + factor  * (low_pass_diff));
	}
}

/**@brief Function movingAvg fifo accelerometer data.
 *
 */
static float movingAvg(float input)
{

	#define ACCE_MOVING_AVG_WINDOWS_HIGH 		14		// define the number of samples for HIGH speed state for moving average
	#define ACCE_MOVING_AVG_WINDOWS_MID 		17		// define the number of samples for LOW and MID speed state for moving average
	#define ACCE_MOVING_AVG_WINDOWS_LOW 		19		// define the number of samples for LOW and MID speed state for moving average
	#define ACCE_MOVING_AVG_WINDOWS_MAXIMUM 	ACCE_MOVING_AVG_WINDOWS_LOW


	static float 	arr_numbers[ACCE_MOVING_AVG_WINDOWS_MAXIMUM] ={0};
	static uint16_t last_pos = 0;
	static float 	total_sum_high 		= 0, total_sum_mid	= 0, total_sum_low	= 0;

	/* Subtract the oldest number from the prev sum, add the new number */
	total_sum_low 		= total_sum_low + input - arr_numbers[last_pos];
	total_sum_mid 		= total_sum_mid + input - arr_numbers[(last_pos + ACCE_MOVING_AVG_WINDOWS_MAXIMUM - ACCE_MOVING_AVG_WINDOWS_MID) % ACCE_MOVING_AVG_WINDOWS_MAXIMUM];
	total_sum_high  	= total_sum_high + input - arr_numbers[(last_pos + ACCE_MOVING_AVG_WINDOWS_MAXIMUM - ACCE_MOVING_AVG_WINDOWS_HIGH) % ACCE_MOVING_AVG_WINDOWS_MAXIMUM];
	/* Assign the input to the position in the array */
	arr_numbers[last_pos] = input;
	last_pos = (last_pos + 1) % ACCE_MOVING_AVG_WINDOWS_MAXIMUM;
	/* return the average */
	if((acc_speed_high_flag == ACCE_SPEED_HIGH) || (acc_speed_high_flag == ACCE_SPEED_MIDHIGH))
		return (float)(total_sum_high / ACCE_MOVING_AVG_WINDOWS_HIGH);
	else if(acc_speed_high_flag == ACCE_SPEED_MID)
		return (float)(total_sum_mid / ACCE_MOVING_AVG_WINDOWS_MID);
	else
		return (float)(total_sum_low / ACCE_MOVING_AVG_WINDOWS_LOW);
}

/**@brief Function rpmAvg for get average speed.
 *
 */
static float rpmAvg(uint16_t input)
{
	#define ACCE_RPM_AVG_WINDOWS 	3
	static uint16_t 	rpm_arr_numbers[ACCE_RPM_AVG_WINDOWS] ={0};
	static uint16_t 	rpm_last_pos = 0;
	static uint32_t 	rpm_total_sum = 0;
	static uint16_t		last_input_temp = 0;
	float 			average = 0;

	/* Subtract the oldest number from the prev sum, add the new number */
	rpm_total_sum = rpm_total_sum + input - rpm_arr_numbers[rpm_last_pos];
	/* Assign the input to the position in the array */
	rpm_arr_numbers[rpm_last_pos] = input;
	rpm_last_pos = (rpm_last_pos + 1) % ACCE_RPM_AVG_WINDOWS;

	/* return the average */
	if(rpm_total_sum == input) // if rpm_arr_numbers is empty, report fist input value immediately.
		average = rpm_total_sum;
	else if(rpm_total_sum == (input + last_input_temp)) // if rpm_arr_numbers only exists one data, report average of 2 data.
		average = rpm_total_sum / 2;
	else
		average = (rpm_total_sum / ACCE_RPM_AVG_WINDOWS);
	last_input_temp = input;
	return average;
}
/**@brief Function bandpassEMA for band pass filter.
 *
 */
static float bandpassEMA(float input)
{
	#define ACCE_EMA_A_LOW		0.02f
	#define ACCE_EMA_A_HIGH		1.40f
	//EMA_low and EMA_high declare as global variable

	EMA_low  = (ACCE_EMA_A_LOW * input) + ((1 - ACCE_EMA_A_LOW) * EMA_low);
	EMA_high = (ACCE_EMA_A_HIGH * input) + ((1 - ACCE_EMA_A_HIGH) * EMA_high);

    return (EMA_high - EMA_low);
}
/**@brief Function for dump x,y,z g x 1000 value to ota.
 */
void acc_read_fifodata_datadump(void)
{
#ifdef ACCELEROMETER_DUMP_FIFO
	//ble_cscs_meas_t cscs_measurement;
	uint8_t i = 0;
	int16_t bufZ[DEF_WATERMARK_VAL] = {0};
	//float ay1 = 0, az1 = 0 , ax1 = 0;

	for(i = 0; i < DEF_WATERMARK_VAL; i++)
	{
#ifndef ACC_ST16G_ENABLE
		int16_t in16_ax = 0, in16_ay = 0, in16_az = 0;
		x_sample2[i] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)] << 8) | (uint16_t)accel_buff[(i*6)+1]);
		y_sample2[i] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+2] << 8) | (uint16_t)accel_buff[(i*6)+3]);
		z_sample2[i] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+4] << 8) | (uint16_t)accel_buff[(i*6)+5]);
		ax1 = ((float)x_sample2[i])/(float)(ACCEL_SENSITIVITY_CONFIG*16);
		ay1 = ((float)y_sample2[i])/(float)(ACCEL_SENSITIVITY_CONFIG*16);
		az1 = ((float)z_sample2[i])/(float)(ACCEL_SENSITIVITY_CONFIG*16);

		in16_ax = (int16_t)(ax1*1000);
		in16_ay = (int16_t)(ay1*1000);
		in16_az = (int16_t)(az1*1000);

		//NRF_LOG_INFO( "ID: %5d: X: %d, Y: %d, Z: %d", ui32_step_sample_counter, in16_ax, in16_ay, in16_az);
		cscs_measurement.is_wheel_rev_data_present = true;
		cscs_measurement.is_crank_rev_data_present = true;
		cscs_measurement.cumulative_wheel_revs = ui32_step_sample_counter;
		cscs_measurement.last_wheel_event_time = (uint16_t)in16_ax;
		cscs_measurement.cumulative_crank_revs = (uint16_t)in16_ay;
		cscs_measurement.last_crank_event_time	 =(uint16_t)in16_az;
		ui32_step_sample_counter++;

		uint8_t retry_count = 0;
		while ((accel_csc_meas_timeout_handler2(cscs_measurement) != NRF_SUCCESS))
		{
			nrf_delay_ms(2);
			retry_count++;
			if(retry_count>=3)
				break;
		}
#else
		//x_sample2[i] = ((int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+1] << 8) | (uint16_t)accel_buff[(i*6)]));
		y_sample2[i] = ((int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+3]<< 8)| (uint16_t)accel_buff[(i*6)+2]));
		//z_sample2[i] = ((int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+5]<< 8)| (uint16_t)accel_buff[(i*6)+4]));

		//int16_t in16_ax = (float)(x_sample2[i]*ACCEL_SENSITIVITY_MG_CONFIG)/(float)(16);
		//int16_t in16_ay = (float)(y_sample2[i]*ACCEL_SENSITIVITY_MG_CONFIG)/(float)(16);
		//int16_t in16_az = (float)(z_sample2[i]*ACCEL_SENSITIVITY_MG_CONFIG)/(float)(16);
        bufZ[i] = (y_sample2[i]*ACCEL_SENSITIVITY_MG_CONFIG/16);
#endif
	}
#ifdef ACC_ST16G_ENABLE
	accel_nus_data_push(bufZ);
#endif
#endif
}

/**@brief Function for processing x,y,z data.
 */
void acc_read_fifodata(void)
{
	float f_average_ang = 0, ay = 0, ax = 0;
	//float ax = 0;
	float mag_accel_sample[DEF_WATERMARK_VAL] = {0};
	uint16_t angle_sample[DEF_WATERMARK_VAL] = {0};

	for(uint8_t i = 0; i < DEF_WATERMARK_VAL; i++)
	{
		int16_t accel_xyz[3];
#ifndef ACC_ST16G_ENABLE
		accel_xyz[0] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)] << 8) | (uint16_t)accel_buff[(i*6)+1]);
		accel_xyz[1] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+2] << 8) | (uint16_t)accel_buff[(i*6)+3]);
		accel_xyz[2] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+4] << 8) | (uint16_t)accel_buff[(i*6)+5]);

		ax = ((float)accel_xyz[0])/(float)(ACCEL_SENSITIVITY_CONFIG*16);
		ay = ((float)accel_xyz[1])/(float)(ACCEL_SENSITIVITY_CONFIG*16);
		//az = ((float)accel_xyz[2])/(float)(ACCEL_SENSITIVITY_CONFIG*16);
		/* LezyneCAD use x and y axis for calculating angle value */
		f_average_ang = (float)(atan2((double)ax,(double)ay)*180/PI)+180.0;

		angle_sample[i] = (uint16_t)(f_average_ang + 0.5);		// round 0.4 down, round 0.5 up
		/* LezyneCAD  use y axis to check zero-crossing condition */
		mag_accel_sample[i] = ay;
#else
		accel_xyz[0] = ((int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+1]<< 8) | (uint16_t)accel_buff[(i*6)]));
		accel_xyz[1] = ((int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+3]<< 8) | (uint16_t)accel_buff[(i*6)+2]));
		//accel_xyz[2] = ((int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+5]<< 8) | (uint16_t)accel_buff[(i*6)+4]));

		ax = (float)(accel_xyz[0] * ACCEL_SENSITIVITY_MG_CONFIG)/(float)(16 * 1000);
		ay = (float)(accel_xyz[1] * ACCEL_SENSITIVITY_MG_CONFIG)/(float)(16 * 1000);
		//az = (float)(accel_xyz[2] * ACCEL_SENSITIVITY_MG_CONFIG)/(float)(16 * 1000);
		/* LezyneCAD-ST16G use x and y axis for calculating angle value */
		f_average_ang = (float)(atan2((double)ax,(double)ay)*180/PI)+180.0;
		angle_sample[i] = (uint16_t)(f_average_ang + 0.5);		// round 0.4 down, round 0.5 up
		/* LezyneCAD-ST16G use x and y axis to check zero-crossing condition */
		mag_accel_sample[i] = ay;
#endif
	}
	/* processing angle and mag data */
	acc_step_update_angle(angle_sample, mag_accel_sample);
}

/**@brief Function acc_step_update_angle process accelerometer angular data to lap and total angle.
 */
static void acc_step_update_angle(uint16_t *angle_array, float *mag_array)
{
	uint8_t 	i = 0;
	int16_t 	current_angle = 0, temp_angle_diff = 0;

	/* ticks time */
	static uint32_t last_ticks = 0;
	uint32_t ticks_current = app_timer_cnt_get();
	rw_lock_set(true);	// protect the global variable
	uint32_t diff_ticks = (ticks_current > last_ticks)? (ticks_current - last_ticks) : (ticks_current + 0xFFFFFF - last_ticks);
	uint32_t diff_time = (ROUNDED_DIV((diff_ticks * 1000), APP_TIMER_CLOCK_FREQ))+ ACCEL_PROCESS_COMPENSATE_TIME;

	for(i = 0 ; i < DEF_WATERMARK_VAL ; i++)
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
			temp_angle_diff = temp_angle_diff - DEF_ANGLE_360_DEGREE;
		}
		else if (temp_angle_diff < (DEF_ANGLE_180_DEGREE * -1)) // forward revolve and across the 360 degree, i.e. n = 10, n-1 = 350
		{
			temp_angle_diff = temp_angle_diff + DEF_ANGLE_360_DEGREE;
		}
		//else if((temp_angle_diff > DEF_MAX_ANGLE_WINDOW) || (temp_angle_diff < (DEF_MAX_ANGLE_WINDOW * -1)))
		//{
		//	NRF_LOG_INFO("Warning!!! i:(%d) the difference (%d) > 75, [%d]-[%d] ", i, temp_angle_diff, angle_array[i], (i==0)?last_angle_residue:angle_array[i-1]);
		//}

		current_angle += temp_angle_diff;
	}
	/* event time is 1024-based. maximum value shall less than 64,000 (0x10000 / 0x800 * 2000) */
	ui16_total_time = (ui16_total_time + diff_time) % (DEF_TOTAL_TIME_STAMP_MAXIMUM);
	last_ticks = ticks_current;
	rw_lock_set(false);	//release protection

	current_angle = abs(current_angle);
	last_angle_residue = angle_array[DEF_WATERMARK_VAL-1];		// update last_angle_residue for current last_angle

	/* movement detection */
	if(current_angle < ACCEL_MOVE_ANGLE_MIN) 	// speed < 20 RPM
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
			if(mid_speed_flag_get())
			{
				/* do nothing */
			}
			else
			{
				ui8_movecnt--;
			}
		}
	}
	else										// speed >= 20 RPM
	{
		if(ui8_movecnt == ACCEL_MOVE_COUNT_MAX)
		{
			/* do nothing */
		}
		else if(ui8_movecnt < ACCEL_MOVE_COUNT_MIN )
		{
			ui8_movecnt++;
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
    ui32_step_sample_counter = 0;
}

/**@brief Function process accelerometer data to check zero-crossing condition.
 */
static void acc_step_mag_update(float mag_update_value)
{
	/* ============= Step 0: Initialize ============= */
#ifndef ACC_ST16G_ENABLE
	/* low speed configuration: 0~16km */
	#define LOW_PASS_FACTOR_LOW_SPEED 			0.15f // ensure factor belongs to  [0,1]
	#define AVERAGE_FACTOR_LOW_SPEED			0.01f // ensure factor belongs to  [0,1]
	#define LIMITED_FACTOR_LOW_SPEED			0.15f

	/* mid speed configuration: 16km ~ 30km */
	#define LOW_PASS_FACTOR_MID_SPEED 			0.23f // ensure factor belongs to  [0,1]
	#define AVERAGE_FACTOR_MID_SPEED			0.03f // ensure factor belongs to  [0,1]
	#define LIMITED_FACTOR_MID_SPEED			0.23f

	/* mid-high speed configuration: 30 ~ 40 */
	#define LOW_PASS_FACTOR_MIDHIGH_SPEED 		0.40f // ensure factor belongs to  [0,1]
	#define AVERAGE_FACTOR_MIDHIGH_SPEED		0.08f // ensure factor belongs to  [0,1]
	#define LIMITED_FACTOR_MIDHIGH_SPEED		0.40f

	/* high speed configuration: over 40 */
	#define LOW_PASS_FACTOR_HIGH_SPEED 			0.80f // ensure factor belongs to  [0,1]
	#define AVERAGE_FACTOR_HIGH_SPEED			0.20f // ensure factor belongs to  [0,1]
	#define LIMITED_FACTOR_HIGH_SPEED			2.00f

	/* window only need to define high and low */
	#define MAX_FILTER_WINDOW_HIGH_SPEED   		0.05f
	#define MAX_FILTER_WINDOW_LOW_SPEED	   		0.15f
	static float 	peakmax = 0, valleymax = 0;
	static float 	last_mag_update_value = 0;
	float 			step_temp_min = 0, step_temp_max = 0;
	float 			filter_window = 0, low_pass_factor = 0, average_factor = 0, limited_factor = 0;

	if(ui32_step_sample_counter == 0) // initiate default value
	{
		last_average_weighting =  mag_update_value;
		last_mag_update_value = mag_update_value;
		ui32_step_sample_counter ++;
		return;
	}

	if(acc_speed_high_flag == ACCE_SPEED_HIGH)
	{
		filter_window 	= MAX_FILTER_WINDOW_HIGH_SPEED;
		low_pass_factor = LOW_PASS_FACTOR_HIGH_SPEED;
		average_factor 	= AVERAGE_FACTOR_HIGH_SPEED;
		limited_factor  = LIMITED_FACTOR_HIGH_SPEED;
	}
	else if(acc_speed_high_flag == ACCE_SPEED_MIDHIGH)
	{
		filter_window 	= MAX_FILTER_WINDOW_HIGH_SPEED;
		low_pass_factor = LOW_PASS_FACTOR_MIDHIGH_SPEED;
		average_factor 	= AVERAGE_FACTOR_MIDHIGH_SPEED;
		limited_factor  = LIMITED_FACTOR_MIDHIGH_SPEED;
	}
	else if(acc_speed_high_flag == ACCE_SPEED_MID)
	{
		filter_window 	= MAX_FILTER_WINDOW_HIGH_SPEED;
		low_pass_factor = LOW_PASS_FACTOR_MID_SPEED;
		average_factor 	= AVERAGE_FACTOR_MID_SPEED;
		limited_factor  = LIMITED_FACTOR_MID_SPEED;
	}
	else
	{
		filter_window 	= MAX_FILTER_WINDOW_HIGH_SPEED;
		low_pass_factor = LOW_PASS_FACTOR_LOW_SPEED;
		average_factor 	= AVERAGE_FACTOR_LOW_SPEED;
		limited_factor  = LIMITED_FACTOR_LOW_SPEED;
	}

	/* moving average */
	mag_update_value = movingAvg(mag_update_value);
	/* low pass filter */
	//mag_update_value = lowPassExponential(mag_update_value, last_mag_update_value, low_pass_factor, limited_factor);
	//last_mag_update_value = mag_update_value;

	/* calculate average, max and min line */
	last_average_weighting = lowPassExponential(mag_update_value, last_average_weighting, average_factor, MAX_LIMITED_FACTOR_HIGH_SPEED);
	step_temp_min = last_average_weighting - filter_window;
	step_temp_max = last_average_weighting + filter_window;
#else
	/* low speed configuration: 0~16km */
	#define LOW_PASS_FACTOR_LOW_SPEED 			0.30f // ensure factor belongs to  [0,1]
	#define LIMITED_FACTOR_LOW_SPEED			0.20f

	/* mid speed configuration: 16km ~ 30km */
	#define LOW_PASS_FACTOR_MID_SPEED 			0.35f // ensure factor belongs to  [0,1]
	#define LIMITED_FACTOR_MID_SPEED			0.25f

	/* mid-high speed configuration: 30 ~ 45 */
	#define LOW_PASS_FACTOR_MIDHIGH_SPEED 		0.40f // ensure factor belongs to  [0,1]
	#define LIMITED_FACTOR_MIDHIGH_SPEED		0.40f

	/* high speed configuration: over 45 */
	#define LOW_PASS_FACTOR_HIGH_SPEED 			0.66f // ensure factor belongs to  [0,1]
	#define LIMITED_FACTOR_HIGH_SPEED			0.66f
	
	#define AVERAGE_FACTOR_LOW_SPEED			0.05f
	#define MAX_LIMITED_FACTOR_HIGH_SPEED		2.00f
	
	/* window */
	#define MAX_FILTER_WINDOW_LOW_SPEED   		0.55f
	#define MAX_FILTER_WINDOW_MID_SPEED   		0.42f
	#define MAX_FILTER_WINDOW_MIDHIGH_SPEED   	0.25f
	#define MAX_FILTER_WINDOW_HIGH_SPEED   		0.18f

	#define ACCE_MAG_VALUE_FILTER_MAXIMUM		2.5f
	#define ACCE_MAG_VALUE_FILTER_MINIMUM		-2.5f

	#define MAX_SAMPLE_NUMBER_RESET				400

	#define ACCE_RPM_DIFF_MINI 					24		// highest RPM, 24 samples is 250 RPM
	static step_detect_t  step_state = eSTEP_RESET; 		// State variable for zero crossing judgment
	static float 	peakmax = 0, valleymax = 0;
	static float 	last_mag_update_value = 0;
	float 			step_temp_min = 0, step_temp_max = 0;
	float 			filter_window = 0, low_pass_factor = 0, limited_factor = 0;

	if(ui32_step_sample_counter == 0) // initiate default value
	{
		last_mag_update_value = 0;
		step_state = eSTEP_RESET;
		ui32_step_sample_counter ++;
		return;
	}

	if(acc_speed_high_flag == ACCE_SPEED_HIGH)
	{
		low_pass_factor = LOW_PASS_FACTOR_HIGH_SPEED;
		limited_factor  = LIMITED_FACTOR_HIGH_SPEED;
		filter_window 	= MAX_FILTER_WINDOW_HIGH_SPEED;
	}
	else if(acc_speed_high_flag == ACCE_SPEED_MIDHIGH)
	{
		low_pass_factor = LOW_PASS_FACTOR_MIDHIGH_SPEED;
		limited_factor  = LIMITED_FACTOR_MIDHIGH_SPEED;
		filter_window 	= MAX_FILTER_WINDOW_MIDHIGH_SPEED;
	}
	else if(acc_speed_high_flag == ACCE_SPEED_MID)
	{
		low_pass_factor = LOW_PASS_FACTOR_MID_SPEED;
		limited_factor  = LIMITED_FACTOR_MID_SPEED;
		filter_window 	= MAX_FILTER_WINDOW_MID_SPEED;
	}
	else
	{
		low_pass_factor = LOW_PASS_FACTOR_LOW_SPEED;
		limited_factor  = LIMITED_FACTOR_LOW_SPEED;
		filter_window 	= MAX_FILTER_WINDOW_LOW_SPEED;
	}

	if((mag_update_value > ACCE_MAG_VALUE_FILTER_MAXIMUM + EMA_low))
		mag_update_value = ACCE_MAG_VALUE_FILTER_MAXIMUM + EMA_low;
	else if (mag_update_value < ACCE_MAG_VALUE_FILTER_MINIMUM + EMA_low)
		mag_update_value = ACCE_MAG_VALUE_FILTER_MINIMUM + EMA_low;
	/* moving average */
	mag_update_value = movingAvg(mag_update_value);
	/* low pass filter */
	mag_update_value = lowPassExponential(mag_update_value, last_mag_update_value, low_pass_factor, limited_factor);
	last_mag_update_value = mag_update_value;
	/* band filter */
    mag_update_value = bandpassEMA(mag_update_value);

	/* calculate average, max and min line */
    if((acc_speed_high_flag == ACCE_SPEED_LOW) || (acc_speed_high_flag == ACCE_SPEED_MID))
    {
    	last_average_weighting = 0;
    }
    else
    {
    	last_average_weighting = lowPassExponential(mag_update_value, last_average_weighting, AVERAGE_FACTOR_LOW_SPEED, MAX_LIMITED_FACTOR_HIGH_SPEED);
    }
    step_temp_min = last_average_weighting - filter_window;
	step_temp_max = last_average_weighting + filter_window;
#endif

#ifdef SENSOR_DEBUG_OUTPUT
	//NRF_LOG_INFO("mag_update_value: %d, average_weighting: %d, min: %d, max: %d",(int16_t)(mag_update_value*1000),(int16_t)(last_average_weighting*1000),(int16_t)(step_temp_min*1000),(int16_t)(step_temp_max*1000));
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
				step_state = eSTEP_START_VALLEY;
				valleymax = mag_update_value;
			}
			else if (mag_update_value >= peakmax)
			{
				peakmax = mag_update_value;
			}
		}
		break;

		case eSTEP_START_VALLEY:
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
				if(ui8_movecnt > ACCEL_MOVE_COUNT_MIN) // check is moving or not
				{
					uint32_t sample_diff = ui32_step_sample_counter - ui32_step_detect_number;
					uint16_t oneshot_rpm_current = (6000 / sample_diff);
					if(sample_diff < ACCE_RPM_DIFF_MINI)
					{
						/* current sample number should not less than ACCE_RPM_DIFF_MINI. It means RPM more than 214 */

					}
					else
					{
						ui16_total_step++;
						if(sample_diff > MAX_SAMPLE_NUMBER_RESET)	// only update lap when sample_diff small than 400 (4s)
						{
							acc_meas_report_flag = true;
						}
						ui32_step_detect_number = ui32_step_sample_counter;
						oneshot_rpm_avg = (uint16_t)(((float)oneshot_rpm_avg * 0.2) + ((float)oneshot_rpm_current * 0.8) + 0.5);
					}
				}
				valleymax = mag_update_value;
				peakmax = mag_update_value;
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
	//NRF_LOG_INFO("step_state: %d, peakmax: %d, valleymax: %d",step_state,(int16_t)(peakmax*1000),(int16_t)(valleymax*1000));
#endif
	ui32_step_sample_counter ++;
}

/**@brief Function for Simple accelerometer offset calibration.
 */  
void accel_calibration(void)
{  
	char X_offset, Y_offset, Z_offset;
	uint8_t accel_data[6] = {0};
	uint16_t Xout_12_bit, Yout_12_bit, Zout_12_bit;

	/* Standby Mode */
	accel_standby();
  
	/* Read register 0x01~0x06(raw x,y,z) */
	accel_burst_read_reg(OUT_X_MSB_REG, accel_data, 6);

	Xout_12_bit = ((short) (accel_data[0]<<8 | accel_data[1])) >> 4; // Compute 12-bit X-axis acceleration output value
	Yout_12_bit = ((short) (accel_data[2]<<8 | accel_data[3])) >> 4; // Compute 12-bit Y-axis acceleration output value
	Zout_12_bit = ((short) (accel_data[4]<<8 | accel_data[5])) >> 4; // Compute 12-bit Z-axis acceleration output value

	X_offset = Xout_12_bit / 2 * (-1); // Compute X-axis offset correction value
	Y_offset = Yout_12_bit / 2 * (-1); // Compute Y-axis offset correction value
#ifndef ACC_ST16G_ENABLE
	Z_offset = (Zout_12_bit - ACCEL_SENSITIVITY_CONFIG) / 2 * (-1); // Compute Z-axis offset correction value
#else
	Z_offset = (Zout_12_bit - 128) / 2 * (-1); // for LIS2DH12 16G setting
#endif
	accel_write_reg(OFF_X_REG, X_offset);
	nrf_delay_ms(2);
	accel_write_reg(OFF_Y_REG, Y_offset);
	nrf_delay_ms(2);
	accel_write_reg(OFF_Z_REG, Z_offset);
	nrf_delay_ms(2);

	accel_wake_up(); // Active mode again
}

/**@brief Function for populating simulated cycling cadence measurements.
 */
ret_code_t accel_csc_measurement(ble_cscs_meas_t * p_measurement)
{
	#define ACCEL_EVENT_TIME_FACTOR 			1.024f
	#define ACCEL_CSC_NOMOVE_REPORT_COUNT		4				//report csc measurement when lap is not changing for 3 seconds.

	static uint16_t ui16_last_lap = 0;							// Last lap indicator: 9.1 hours to overflow with the speed of 120 RPM
    static uint16_t ui16_last_total_time 		= 0, ui16_last_event_time = 0;		// total time is 1000-based, event time is 1024-based time
    static uint16_t uin16_not_moving_counter 	= 0;								// overflow after 18 hours, no need to check
    static uint16_t ui16_last_report_lap		= 0;
    uint16_t 	 	total_time_diff 			= 0, ui16_wheel_event_time = 0;	// event time is 1024-based time
    bool			report_last_flag			= false;

    if(RW_LOCK_GET())	// if rw_lock is ture means the critical global variable is updating.
    {
    	NRF_LOG_INFO("Warning!!! Read and Write collision, skip report");
    	return NRF_ERROR_INVALID_STATE;
    }
    /* Moving flag check: total laps is change or not. */
    if(ui16_last_lap == ui16_total_step)
    {
		uin16_not_moving_counter ++;
    }
    else if(oneshot_rpm_avg <= ACCEL_MOVE_RPM_MIN)		// If oneshot average is lower than 15 RPM, considering it as non-moving.
    {
    	oneshot_rpm_avg = 0;
    	uin16_not_moving_counter ++;
    }
    else
    {
    	uin16_not_moving_counter =	0;
    }

    if(uin16_not_moving_counter >= ACCEL_CSC_NOMOVE_REPORT_COUNT)
    {
    	oneshot_rpm_avg = 0;
    }

	/* recalculate average rpm by moving average */
	float average_rpm = rpmAvg(oneshot_rpm_avg);

	/* some special average of rpm modification	*/
	if(((uin16_not_moving_counter >0) && (uin16_not_moving_counter <ACCEL_CSC_NOMOVE_REPORT_COUNT))||(uin16_not_moving_counter >= ACCEL_CSC_NOMOVE_REPORT_COUNT +3))
		report_last_flag =  true;
	else if(uin16_not_moving_counter >= ACCEL_CSC_NOMOVE_REPORT_COUNT)
		average_rpm = 0;
	else if(average_rpm > ACCEL_MOVE_RPM_MAX)
		average_rpm = ACCEL_MOVE_RPM_MAX;
	else if(abs(oneshot_rpm_avg - average_rpm) > ACCEL_WEIGHT_RPM_DIFF)		// if RPM is falling/rising more than 15 RPM, weight twice oneshot_rpm_avg
		average_rpm = rpmAvg(oneshot_rpm_avg);

    /* set new speed flag */
    speed_flag_set((uint16_t)average_rpm);
    /* time difference between last report measurement*/
	total_time_diff = (ui16_total_time>ui16_last_total_time)? ui16_total_time - ui16_last_total_time : ui16_total_time + (DEF_TOTAL_TIME_STAMP_MAXIMUM - ui16_last_total_time);

	if (acc_meas_report_flag) // the beginning of the first lap, report it immediately
	{
		acc_meas_report_flag = false;
		//ui16_last_report_lap++;
		ui16_wheel_event_time = ui16_last_event_time + (total_time_diff * ACCEL_EVENT_TIME_FACTOR);
		ui16_last_total_time = ui16_total_time;
		//uin16_not_moving_counter = 0;
	}
	else if(report_last_flag)
	{
		if((uin16_not_moving_counter <= 1) // report first unchanged event
	#ifdef	SENSOR_ALWAYS_REPORT_DEBUG
			|| (uin16_not_moving_counter > ACCEL_CSC_NOMOVE_REPORT_COUNT)
	#endif
		)
		{
			ui16_wheel_event_time = ui16_last_event_time;
		}
		else
		{
			return NRF_ERROR_INVALID_STATE;
		}
	}
	else
	{
		if(average_rpm > ACCEL_MOVE_RPM_MIN)
		{
			float ms_per_lap = 0;
			uint16_t current_lap = 0;
			ms_per_lap = (60 * 1000) / average_rpm;	// ms_per_lap is calculate each lap takes how many milliseconds during average_rpm
			if(total_time_diff < ms_per_lap)  // if time difference less than a lap, set it a lap
			{
				current_lap = 1;
			}
			else
			{
				/* rpm = 60 * 1000 * laps_per_ms */
				current_lap =  total_time_diff / ms_per_lap;
			}
			ui16_last_report_lap += current_lap;
			/* (lap / rpm)  = duration (min) */
			ui16_wheel_event_time = ui16_last_event_time + (uint16_t)(((float)current_lap / average_rpm) * 60 * 1000 * ACCEL_EVENT_TIME_FACTOR);
			ui16_last_total_time = ui16_total_time;
		}
		else 	//average_rpm == 0, update ui16_wheel_event_time without changing the laps
		{
			ui16_wheel_event_time = ui16_last_event_time + (total_time_diff * ACCEL_EVENT_TIME_FACTOR);
			ui16_last_total_time = ui16_total_time;
		}
	}

	p_measurement->is_crank_rev_data_present = true;
	p_measurement->cumulative_crank_revs 	 = ui16_last_report_lap;
	p_measurement->last_crank_event_time	 = ui16_wheel_event_time;

#ifdef SENSOR_DEBUG_OUTPUT
	/* for test log */
	p_measurement->is_wheel_rev_data_present = true;
	p_measurement->cumulative_wheel_revs = (uint16_t)(report_last_flag + (acc_meas_report_flag * 16));
	p_measurement->last_wheel_event_time = (uint16_t)(average_rpm);
#endif
	/* last lap depend on angle mode or step mode */
	ui16_last_lap = ui16_total_step;
	ui16_last_event_time = ui16_wheel_event_time;
    return NRF_SUCCESS;
}

/**@brief Function for accel_dump_reg_info
 */
ret_code_t accel_dump_reg_info(uint8_t * ctrl_reg , uint8_t size)
{
    //uint8_t ctrl_reg[22];
    if (size < 20)
    {
    	return NRF_ERROR_INVALID_STATE;
    }

	accel_read_reg(LIS2DE12_REG0, 					&ctrl_reg[0] );
	accel_read_reg(LIS2DE12_REG1, 					&ctrl_reg[1] );
	accel_read_reg(LIS2DE12_REG2, 					&ctrl_reg[2] );
	accel_read_reg(LIS2DE12_REG3, 					&ctrl_reg[3] );
	accel_read_reg(LIS2DE12_REG4, 					&ctrl_reg[4] );
	accel_read_reg(LIS2DE12_REG5, 					&ctrl_reg[5] );
	accel_read_reg(LIS2DE12_REG6, 					&ctrl_reg[6] );
	accel_read_reg(LIS2DE12_FIFO_CTRL_REG, 			&ctrl_reg[7] );
	accel_read_reg(LIS2DE12_INT1_CFG_REG, 			&ctrl_reg[8] );
	accel_read_reg(LIS2DE12_INT1_THRESHHOLD_REG, 	&ctrl_reg[9] );
	accel_read_reg(LIS2DE12_INT1_DURATION_REG, 		&ctrl_reg[10] );
	accel_read_reg(LIS2DE12_INT2_CFG_REG,		 	&ctrl_reg[11] );
	accel_read_reg(LIS2DE12_INT2_THRESHHOLD_REG, 	&ctrl_reg[12] );
	accel_read_reg(LIS2DE12_INT2_DURATION_REG, 		&ctrl_reg[13] );
	accel_read_reg(LIS2DE12_CLICK_CFG_REG, 			&ctrl_reg[14] );
	accel_read_reg(LIS2DE12_CLICK_THS_REG, 			&ctrl_reg[15] );
	accel_read_reg(LIS2DE12_TIME_LIMIT_REG, 		&ctrl_reg[16] );
	accel_read_reg(LIS2DE12_TIME_LATENCY_REG, 		&ctrl_reg[17] );
	accel_read_reg(LIS2DE12_TIME_WINDOW_REG, 		&ctrl_reg[18] );
	accel_read_reg(LIS2DE12_ACT_THS_REG, 			&ctrl_reg[19] );
	//NRF_LOG_INFO("R:%x %x %x %x %x %x" ,ctrl_reg[0], ctrl_reg[1], ctrl_reg[2], ctrl_reg[3], ctrl_reg[4], ctrl_reg[5]);
	//NRF_LOG_INFO("R:%x %x %x %x %x %x" ,ctrl_reg[6], ctrl_reg[7], ctrl_reg[8], ctrl_reg[9], ctrl_reg[10], ctrl_reg[11]);
	//NRF_LOG_INFO("R:%x %x %x %x %x %x" ,ctrl_reg[12], ctrl_reg[13], ctrl_reg[14], ctrl_reg[15], ctrl_reg[16], ctrl_reg[17]);
	//NRF_LOG_INFO("R:%x %x" ,ctrl_reg[18], ctrl_reg[19]);
	return NRF_SUCCESS;
}

/**@brief Function initialize function.
 */
void accel_init(void)
{
	NRF_LOG_INFO("accel_init.");

#ifndef CSCS_MOCK_ENABLE
#ifdef ACC_ST16G_ENABLE
	/* hardware initialize - decide I2C address */
	accel_i2c_opt();
#endif

	/* hardware initialize - I2C */
	accel_i2c_init();

	/* configuration */
	accel_configuration();

	/* harware initialize - gpio */
	accel_gpio_init();
#endif

	/* timer init */
	accel_timers_init();

#ifdef CSCS_MOCK_ENABLE
	application_timers_start();
#endif

	/* reset acc step angle parameters */
	acc_step_reset_angle();
}
#endif /* SENSOR_ACCELEROMETER_C_ */
