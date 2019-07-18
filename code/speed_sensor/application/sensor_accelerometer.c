/*
 * 	sensor_accelerometer.c
 *
 *  Created on: June 30, 2019
 *  Author: Lezyne
 *  		History: 2019/07/01	 first version Angle algorithm
 *  				 2019/07/04  2nd version Angle algorithm, simplified, with forward and backwards judgement
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
#define SENSOR_DEBUG_OUTPUT
//#define ACCELEROMETER_SELF_ACTIVATE
//#define ACCELEROMETER_SELF_TIMEOUT


/* Data rate 20ms = 50Hz, the highest bicycle speed =75 Km/h = 20.83 m/s, 
   max RPM = 20.83 / 2m (circomference) = 10.41 Hz but need a least 4 samples to calculate = 41.64 Hz ~= 50Hz  */ 
#define ACCEL_DATARATE  						DATA_RATE_20MS
#define ACCEL_DATARATE_MS						20			/* ACCEL_DATARATE_MS shall match to ACCEL_DATARATE */
/* ASLP_RATE_160MS = 6.25 hz */
#define ACCEL_SLEEPP_DATARATE					ASLP_RATE_160MS
/* Threshold =0x08; The step count is 0.063g/ count,  (0.5g/0.063g = 7.9); */
#define ACCEL_FF_MT_THS_VALUE  					(THS3_MASK)
/* Set the debounce counter to eliminate false readings for 20ms data rate, with requirement of 100 ms, 100/20 = 5 counts */
#define ACCEL_FF_MT_DEBOUNCE_COUNT 				2
/* set the minimum duration time to SLEEP mode, ASLP_COUNT Step = 320ms, 320ms * 15 = 4.8s */    							
#define ACCEL_ENTER_SLEEP_COUNTER				10

/* set compensate time */
#define ACCEL_ANGLE_PROCESS_COMPENSATE_TIME				3	// ms

/* FIFO Event Sample Count Watermark, the value shall be less than 64 */
#define DEF_WATERMARK_VAL 						25   //25

/* the define of minimum moving angle (degree per 0.5s (25*20ms)), degree 23 equal 1.0 Kmh (23x2x2100*3600/360/1000) */
#define ACCEL_MOVE_ANGLE_MIN					23
#define ACCEL_MOVE_COUNT_MIN					5		// moving counter: 5 movements in a 4*25*20ms = 2.5s time window
#define ACCEL_MOVE_COUNT_MAX 					15		// non-moving counter: (15 - 5)* 25*20ms = 5s time window


APP_TIMER_DEF(m_csc_meas_timer_id);                                                 /**< CSC measurement timer. */

/* 12bit xyz fifo data */
static uint8_t accel_buff[6*DEF_WATERMARK_VAL];
/* pending task indicator */
static t_accel_task_pending task_pending_singal ={0};
/* protector for critical global variable*/
static volatile bool rw_lock_protect_flag = false;
/* acc angle variable */
static int16_t 	last_angle_residue	= DEF_INVALID_LAST_ANGLE;
static uint32_t ui32_total_lap=0;			// this variable allows 8 million km in maximum before overflow.
static uint32_t ui32_total_angle= 0;		// this variable allows 23,860 km in maximum before overflow.
static uint16_t ui16_total_time= 0;			// this variable allows 256 second before overflow. need to take care warps around case.

static bool  	clockwise_flag = true;
static uint8_t 	ui8_movecnt =0;

/* FUNCTIONS */
static void 	application_timers_start(void);
static void 	application_timers_stop(void);
static void 	acc_step_update_angle(uint16_t *angle_array);

static void 		accel_self_csc_meas_timeout_handler(void * p_context);
static ret_code_t 	accel_write_reg(uint8_t reg_addr, uint8_t reg_data);
static ret_code_t 	accel_read_reg(uint8_t reg_addr, uint8_t *reg_data);
static ret_code_t 	accel_burst_read_reg(uint8_t addr, uint8_t * pdata, size_t size);
static void 		accel_i2c_gpio_enable (void);
static void 		accel_config_fifo_int(bool enable);
static void 		accel_config_motion_int(bool enable);
static void 		accel_weak_up(void);
static void 		accel_standby(void);
static float 		lowPassExponential(float input, float average);
static ret_code_t 	rw_lock_set(bool config);
static bool 		rw_lock_get(void);

/**@brief handler for interrupt of input pin1, for Motion Detection and Auto-WAKE/SLEEP event
 */
void mma8652_int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

	uint8_t ui8_int_source = 0;
	uint8_t ui8_status = 0, ui8_sysmod = 0, ff_mt_src = 0;
    uint32_t pin1_status=0;
    pin1_status = nrf_gpio_pin_read(MMA8652_INT1_PIN);

	/* Check System mode first*/
	accel_read_reg(MMA8652_INT_SOURCE,		&ui8_int_source);
	accel_read_reg(MMA8652_STATUS_00,		&ui8_status);
	accel_read_reg(MMA8652_SYSMOD, 			&ui8_sysmod);
	accel_read_reg(MMA8652_FF_MT_SRC,		&ff_mt_src);

	// motion detection interrupt
	if (ui8_int_source & SRC_FF_MT_MASK)
	{

		NRF_LOG_INFO("MMA8652_int1_handler STATUS:%2x, INT: %2x, SYSMOD:%2x, gpio1: %d, FF_MT_SRC: %x ", ui8_status, ui8_int_source, ui8_sysmod, pin1_status, ff_mt_src);
	}
}
/**@brief handler for interrupt of input pin2 for FIFO interrupt
 */
void mma8652_int2_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint8_t ui8_int_source = 0;
	uint8_t ui8_status = 0, ui8_sysmod = 0;
	ret_code_t result;
	accel_read_reg(MMA8652_INT_SOURCE,		&ui8_int_source);
	accel_read_reg(MMA8652_STATUS_00,		&ui8_status);
	accel_read_reg(MMA8652_SYSMOD, 			&ui8_sysmod);

	//NRF_LOG_INFO("mma8652_int2_handler.");
	// FIFO Interrupt enabled
    if (ui8_int_source & SRC_FIFO_MASK)
	{
    	//NRF_LOG_INFO("mma8652_int2_handler STATUS:%2x, INT: %2x, SYSMOD:%2x ",ui8_status, ui8_int_source, ui8_sysmod);

    	// watermark not full, wait until watermark is full
    	if((ui8_status & (F_WMRK_FLAG_MASK)) != (F_WMRK_FLAG_MASK))
    	{
    		//NRF_LOG_INFO("Warning!!! FIFO watermark not full, do not need to read buffer.");
    		return;
    	}
    	//enable SENSOR_TASK_INT_FIFO sub-task mask, and it will process buffer date in idle_state_handle() in main.c
    	result = accel_task_enable_mask(SENSOR_TASK_INT_FIFO);
    	if(result == NRF_SUCCESS)
    	{
    		// read 12bit fifo data
   			accel_burst_read_reg(MMA8652_OUT_X_MSB, accel_buff, 6*DEF_WATERMARK_VAL);
       	}
	}
    // without FIFO enable, Data ready interrupt
	if (ui8_int_source & SRC_DRDY_MASK)
	{
		NRF_LOG_INFO("mma8652_int2_handler. Data Ready Handle");
	    accel_task_enable_mask(SENSOR_TASK_INT_DRDY);
	}
#ifdef ACCELEROMETER_SELF_ACTIVATE
	// Auto-WAKE/SLEEP interrupt
	if(ui8_int_source & SRC_ASLP_MASK)
	{
		accel_set_active();
#if 0
		if ((ui8_sysmod & SYSMOD_WAKE) == SYSMOD_WAKE)
		{
		    accel_set_active();
			NRF_LOG_INFO("Auto-WAKE/SLEEP detection: SYSMOD:1, enable FIFO, disable motion detection")
		}
		else if((ui8_sysmod & SYSMOD_SLEEP) == SYSMOD_SLEEP)
		{
		    accel_set_deactive();
		    NRF_LOG_INFO("Auto-WAKE/SLEEP detection: SYSMOD:2, diable FIFO, enable motion detection")
		}
#endif
	}
#endif
}


#ifdef ACCELEROMETER_SELF_TIMEOUT
/**@brief Function accel_csc_meas_timeout_handler.
 *  self test timeout handler, should remove this function after official release
 */
static void accel_self_csc_meas_timeout_handler(void * p_context)
{
    //NRF_LOG_INFO("accel_csc_meas_timeout_handler");
	static uint16_t sleep_counter_test =0;
    uint8_t ctrl_reg[4];

    accel_read_reg(MMA8652_SYSMOD, 			&ctrl_reg[2]);
    accel_read_reg(MMA8652_INT_SOURCE,		&ctrl_reg[0]);
    accel_read_reg(MMA8652_STATUS_00,		&ctrl_reg[1]);
    accel_read_reg(MMA8652_CTRL_REG4,		&ctrl_reg[3]);

   	NRF_LOG_INFO("self timeout_handler: sysmode: %2x, INT: %2x, STATUS: %2x, sleep_c: %d", ctrl_reg[2], ctrl_reg[0],ctrl_reg[1], sleep_counter_test);

   	{
   		ble_cscs_meas_t accel_measuremen;
		accel_csc_measurement(&accel_measuremen);
   	}

   	if(sleep_counter_test > 26)
   	{
   		accel_set_deactive();
   		sleep_counter_test = 0;
   	}
   	sleep_counter_test ++;
}
#endif

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void accel_timers_init(void)
{
	ret_code_t err_code;

    // Create cycle speed and condence timer.
    err_code = app_timer_create(&m_csc_meas_timer_id,
                                APP_TIMER_MODE_REPEATED,
#ifdef ACCELEROMETER_SELF_TIMEOUT
								accel_self_csc_meas_timeout_handler);  // selftest print log
#else
    							accel_csc_meas_timeout_handler);		// timeout handler in ble_core.c
#endif
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;
    uint32_t csc_meas_timer_ticks;

    // Start application timers.
    csc_meas_timer_ticks = APP_TIMER_TICKS(SPEED_AND_CADENCE_MEAS_INTERVAL);
    NRF_LOG_INFO("csc_meas_timer_ticks = %d.",csc_meas_timer_ticks);

    err_code = app_timer_start(m_csc_meas_timer_id, csc_meas_timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for stopping application timers.
 */
static void application_timers_stop(void)
{
    ret_code_t err_code;
    err_code = app_timer_stop(m_csc_meas_timer_id);
    APP_ERROR_CHECK(err_code);
}

/**	@brief Brief : I2C bus write
*	@param reg_addr : Address of the first register
*	@param reg_data : TIt is a value hold in the array
*  	@Return Status of the I2C write
*/
static ret_code_t accel_write_reg(uint8_t reg_addr, uint8_t reg_data)
{
#if 0
	ret_code_t err_code = nrf_drv_twi_tx(&acce_m_twi, reg_addr, &reg_data, 1, false);
	if (err_code)
		NRF_LOG_INFO("accel_I2C_write error(0x%X).",err_code);
	APP_ERROR_CHECK(err_code);
	return err_code;
#else
	uint8_t data[2];
	data[0] = reg_addr;
	data[1] = reg_data;
	ret_code_t err_code = nrf_drv_twi_tx(&acce_m_twi, MMA8652_ADDRESS, data, 2, false);
    nrf_delay_ms(2);
	if (err_code)
		NRF_LOG_INFO(0, "accel_write_reg error\r\n");
	APP_ERROR_CHECK(err_code);
	return err_code;
#endif
}


/**	@brief Brief : I2C bus read
*	@param reg_addr : Address of the first register
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

/**
 * @brief Read continuous data from sensor.
 *
 * @param     addr  Start address to read.
 * @param[in] pdata Pointer to the buffer to fill with data.
 * @param     size  Byte count of data to read.
 *
 * @return NRF_SUCCESS or reason of error.
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

/**	@brief Brief : accel_task_enable_mask
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
		task_pending_singal = (task_pending_singal | enable );
	}
	return NRF_SUCCESS;
}

/**	@brief Brief : accel_task_disable_mask
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
/**	@brief Brief : accel_task_check_enable
*	@param check : check the type of t_accel_task_pending enable or not
*  	@Return Status of ture or false
*/
bool accel_task_check_enable(t_accel_task_pending check)
{
	return (task_pending_singal&check);
}
/**	@brief Brief : rw_lock_set
*	@param config : config true or false of rw_lock_protect_flag
*  	@Return Status of enable success or not
*/
static ret_code_t rw_lock_set(bool config)
{
	if (rw_lock_protect_flag == config)
	{
		NRF_LOG_INFO("Warning!! Ivalid state, flag: %x, config: %x", rw_lock_protect_flag, config);
		return NRF_ERROR_INVALID_STATE;
	}
	else
	{
		rw_lock_protect_flag = config;
	}
	return NRF_SUCCESS;
}
/**	@brief Brief : rw_lock_get
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
	if(who_n_i != MMA8652_WHO_AM_I_OUT )
	{
		NRF_LOG_ERROR(" Device ID not match!! :MMA8652 0x4A: 0x%x", who_n_i );
		return;
	}

    /* RESET sensor, all registers are reset to default */
    accel_write_reg(MMA8652_CTRL_REG2, RST_MASK);
    do {
    	nrf_delay_ms(5);
    	accel_read_reg(MMA8652_CTRL_REG2, &reset_status);
    } while (reset_status & RST_MASK);

	/* Set to standby mode*/
    accel_standby();

    /* clear interrupt in case there were something vestigial */
    uint8_t ui8_temp = 0;
	accel_read_reg(MMA8652_INT_SOURCE,		&ui8_temp);
	accel_read_reg(MMA8652_STATUS_00,		&ui8_temp);
	accel_read_reg(MMA8652_SYSMOD, 			&ui8_temp);
	accel_read_reg(MMA8652_FF_MT_SRC,		&ui8_temp);


#if 0 // for test
    accel_write_reg(MMA8652_CTRL_REG1, ASLP_RATE_1_56HZ|ACCEL_DATARATE); //CRTL_REG1: 50 Hz mode DR = 010, Active = 0,DATA_RATE_160MS,DATA_RATE_640MS,ACCEL_DATARATE
      //accel_write_reg(MMA8652_F_SETUP, 0x90|DEF_WATERMARK_VAL); //FIFO Set to Fill Mode:0x90
      //accel_write_reg(MMA8652_F_SETUP, F_MODE_FILL|DEF_WATERMARK_VAL);
      accel_write_reg(MMA8652_F_SETUP, DEF_WATERMARK_VAL);
    accel_write_reg(MMA8652_CTRL_REG2, MOD_HIGH_RES|SLPE_MASK); // enable SLPM
    accel_write_reg(MMA8652_CTRL_REG3, WAKE_FF_MT_MASK); //
    //accel_write_reg(MMA8652_CTRL_REG4, 0x41); //Enable the interrupt Pin for the FIFO
    accel_write_reg(MMA8652_CTRL_REG4, INT_EN_ASLP_MASK | INT_EN_FIFO_MASK | INT_EN_FF_MT_MASK); // ASLP & FF-MT should enable at the same time
    accel_write_reg(MMA8652_CTRL_REG5, INT_EN_ASLP_MASK | INT_EN_FF_MT_MASK); //Set the interrupt to route to INT1
    accel_write_reg(MMA8652_XYZ_DATA_CFG, FULL_SCALE_2G); //HPF_OUT set, 4g mode

    /* Motion configuration and status registers */
	/* set MMA8652_FF_MT_CFG, setup motion event after the debounce counter time is reached, ELE = 0, OAE = 1*/
	accel_write_reg(MMA8652_FF_MT_CFG, 0x58 );
	/* set MMA8652_FF_MT_THS, setup THS = ACCEL_FF_MT_THS_VALUE */
	accel_write_reg(MMA8652_FF_MT_THS, THS3_MASK );
	/* set MMA8652_FF_MT_COUNT, setup debounce counter ACCEL_FF_MT_DEBOUNCE_COUNT */
	accel_write_reg(MMA8652_FF_MT_COUNT, ACCEL_FF_MT_DEBOUNCE_COUNT );

	/* set ASLP_COUNT, setup sleep counter ACCEL_ENTER_SLEEP_COUNTER */
	//accel_write_reg(MMA8652_ASLP_COUNT, 0x40 );
	accel_write_reg(MMA8652_ASLP_COUNT, 0x0A );
#else
    /* Set F_SETUP, disable FIFO first , watermark = 25 * DATA_RATE_20MS = 500ms) */
    accel_write_reg(MMA8652_F_SETUP, DEF_WATERMARK_VAL);
	//accel_write_reg(MMA8652_F_SETUP, F_MODE_FILL|DEF_WATERMARK_VAL);
#ifdef ACCELEROMETER_SELF_ACTIVATE
	/* Set REG2, enable Auto-SLEEP and High Resolution */
    accel_write_reg(MMA8652_CTRL_REG2, MOD_HIGH_RES|SLPE_MASK);
    /* Set REG3, Configure Wake from Freefall/Motion interrupt, and the INT pins for Push-Pull */
    accel_write_reg(MMA8652_CTRL_REG3, WAKE_FF_MT_MASK);
    /* Set REG4, enable Auto-SLEEP/WAKE Interrupt and the FIFO Interrupt, (enable Motion detection interrupt in initialization) */
    accel_write_reg(MMA8652_CTRL_REG4, INT_EN_ASLP_MASK | INT_EN_FIFO_MASK | INT_EN_FF_MT_MASK);
#else
    /* Set REG2, disable Auto-SLEEP, and High Resolution */
    accel_write_reg(MMA8652_CTRL_REG2, MOD_HIGH_RES); // without auto wake up
    /* Set REG3, Configure Wake from Freefall/Motion interrupt, and the INT pins for Push-Pull */
    accel_write_reg(MMA8652_CTRL_REG3, IPOL_MASK|PP_OD_MASK);
    /* Set REG4, enable FIFO Interrupt, (do not enable Motion detection interrupt in initialization) */
    accel_write_reg(MMA8652_CTRL_REG4,  INT_EN_FIFO_MASK); // without auto wake up
#endif

    /* Set REG5, Motion detection Interrupt Enable mapped to INT1, and FIFO Interrupt (and Auto-SLEEP) mapped to INT2(default) */
    accel_write_reg(MMA8652_CTRL_REG5,  INT_EN_FF_MT_MASK); //Set the interrupt to route to INT1
    //accel_write_reg(MMA8652_CTRL_REG5,  INT_EN_ASLP_MASK | INT_EN_FF_MT_MASK);

    /* Set REG1, Set to 20ms data rate period (50Hz), 160ms sleep datarate, non-Fast read mode (12bit). */
    accel_write_reg(MMA8652_CTRL_REG1, ACCEL_DATARATE | ACCEL_SLEEPP_DATARATE );

	/* Motion configuration and status registers */
	/* set MMA8652_FF_MT_CFG, setup motion event after the debounce counter time is reached, ELE = 0, OAE = 1, Event flag enable on X and Z*/
	//accel_write_reg(MMA8652_FF_MT_CFG, OAE_MASK|ZEFE_MASK|XEFE_MASK );
	accel_write_reg(MMA8652_FF_MT_CFG, OAE_MASK|ZEFE_MASK );
	/* set MMA8652_FF_MT_THS, setup THS = ACCEL_FF_MT_THS_VALUE */
	accel_write_reg(MMA8652_FF_MT_THS, ACCEL_FF_MT_THS_VALUE | DBCNTM_MASK );
	/* set MMA8652_FF_MT_COUNT, setup debounce counter ACCEL_FF_MT_DEBOUNCE_COUNT */
	accel_write_reg(MMA8652_FF_MT_COUNT, ACCEL_FF_MT_DEBOUNCE_COUNT );

	/* set ASLP_COUNT, setup sleep counter ACCEL_ENTER_SLEEP_COUNTER */
	accel_write_reg(MMA8652_ASLP_COUNT, ACCEL_ENTER_SLEEP_COUNTER );

	/* Turn off HPF for Data Out and set 2g Mode */
	accel_write_reg(MMA8652_XYZ_DATA_CFG, FULL_SCALE_2G );

#endif
    // Set back to active
	accel_weak_up();

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


	//NRF_LOG_INFO("F_SETUP:%x, R1:%x, R2:%x, R3:%x ",
    //        ctrl_reg[0], ctrl_reg[1], ctrl_reg[2], ctrl_reg[3]);

	//NRF_LOG_INFO("R4:%x, ASLP:%x, XYZ:%x, FT_SRC:%x.",ctrl_reg[10], ctrl_reg[11], ctrl_reg[12], ctrl_reg[4]);

	//NRF_LOG_INFO("R5:%x, INT:%x. ",
    //        ctrl_reg[5], ctrl_reg[6]);

	//NRF_LOG_INFO("MT:%x %x %x.",
	//             ctrl_reg[7], ctrl_reg[8], ctrl_reg[9]);

	#endif
	NRF_LOG_INFO("accel_configuration end.");
}

/**@brief Function for enable of disable fifo interrupt.
 *	bool enable: disable:0, enable:1		
 */
static void accel_config_fifo_int(bool enable)
{
    nrf_delay_ms(10);
	if(enable)
	{
		accel_write_reg(MMA8652_F_SETUP, F_MODE_CIRCULAR|DEF_WATERMARK_VAL);
    }
    else
    {
    	accel_write_reg(MMA8652_F_SETUP, 0x00);
    }
}

/**@brief Function for enable or disable motion detection interrupt.
 *	bool enable: disable:0, enable:1		
 */
static void accel_config_motion_int(bool enable)
{
	uint8_t ctrl_reg4 = 0;

	if(enable)
	{
		accel_read_reg(MMA8652_CTRL_REG4, &ctrl_reg4);
	    /* Set REG4, enable Auto-SLEEP/WAKE Interrupt and the FIFO Interrupt */
	    accel_write_reg(MMA8652_CTRL_REG4, (ctrl_reg4 | INT_EN_FF_MT_MASK));
	}
	else
	{
	  	accel_read_reg(MMA8652_CTRL_REG4, &ctrl_reg4);
		/* Set REG4, enable Auto-SLEEP/WAKE Interrupt and the FIFO Interrupt */
	    accel_write_reg(MMA8652_CTRL_REG4, (ctrl_reg4 & ~INT_EN_FF_MT_MASK));
	}
}

/**@brief Initialize I2C (TWI).
 */
static ret_code_t accel_i2c_gpio_init (void)
{
	ret_code_t err_code;

	/* I2C initialize */
    const nrf_drv_twi_config_t twi_config = {
       .scl                = MMA8652_I2C_SCL_PIN,
       .sda                = MMA8652_I2C_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false,
       .hold_bus_uninit    = false
    };
    err_code = nrf_drv_twi_init(&acce_m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&acce_m_twi);

    /* GPIO Initialize */
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);	
    err_code = nrf_drv_gpiote_in_init(MMA8652_INT2_PIN, &in_config, mma8652_int2_handler);

    APP_ERROR_CHECK(err_code);

    /* GPIO Initialize as system wakeup pin */
	err_code = nrf_drv_gpiote_in_init(MMA8652_INT1_PIN, &in_config, mma8652_int1_handler);
	APP_ERROR_CHECK(err_code);

	return err_code;
}
/**@brief start gpio event I2C.
 */
static void accel_i2c_gpio_enable (void)
{
	nrf_drv_gpiote_in_event_enable(MMA8652_INT1_PIN, true);
	NVIC_EnableIRQ(GPIOTE_IRQn);
}

/**@brief Function for set the sensor accelerometer to ACTIVATE mode.
 *	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE			
 */
void accel_set_deactive(void)
{
	/* Set to standby mode */
    accel_standby();

    /* enable/disable interrupt */
	accel_config_fifo_int(false);
	accel_config_motion_int(true);
	NRF_LOG_INFO("accel_set_deactive: disable FIFO, enable motion detection");
	acc_step_reset_angle();
#ifndef ACCELEROMETER_SELF_ACTIVATE
	application_timers_stop();
#endif
	/* Set back to active mode */
	accel_weak_up();
}


/**@brief Function for set the sensor accelerometer to ACTIVATE mode.
 *	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE
 */
void accel_set_active(void)
{
	/* Set to standby mode*/
    accel_standby();

	accel_config_fifo_int(true);
	accel_config_motion_int(false);
#ifndef ACCELEROMETER_SELF_ACTIVATE
	application_timers_start();
#endif
	NRF_LOG_INFO("accel_set_active: enable FIFO, disable motion detection");
	// Set back to active
	accel_weak_up();

}
/**@brief Function for set the sensor accelerometer to ACTIVATE mode.
 *	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE
 */

/**@brief Function for set the sensor accelerometer to ACTIVATE mode.
 *	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE			
 */
void accel_weak_up(void)
{
	/* Read REG1 byte first; then disable ACTIVE bit for going to standby mode*/
	uint8_t ctrl_reg1;
	accel_read_reg(MMA8652_CTRL_REG1, & ctrl_reg1);
	accel_write_reg(MMA8652_CTRL_REG1, ctrl_reg1 | ACTIVE_MASK);
	nrf_delay_ms(50);
}

/**@brief Function for set the sensor accelerometer to STANDBY mode.
 * 	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE
 */
void accel_standby(void)
{
    /* Read REG1 byte first; then disable ACTIVE bit for going to standby mode*/		
	uint8_t ctrl_reg1;
	accel_read_reg(MMA8652_CTRL_REG1, &ctrl_reg1);
	accel_write_reg(MMA8652_CTRL_REG1, ctrl_reg1 & ~ACTIVE_MASK);
	nrf_delay_ms(50);
}

/**@brief Function lowPassExponential fifo accelerometer data.
 *	read numbers of DEF_WATERMARK_VAL from accel_buff[], and turn into DEF_WATERMARK_VAL angular samples
 */
static float lowPassExponential(float input, float average)
{
	#define LOW_PASS_FACTOR 0.7 // ensure factor belongs to  [0,1]
    return input*LOW_PASS_FACTOR + (1-LOW_PASS_FACTOR)*average;
}

/**@brief Function for burst read sensor fifo accelerometer data.
 *	read numbers of DEF_WATERMARK_VAL from accel_buff[], and turn into DEF_WATERMARK_VAL angular samples
 */
void acc_read_fifodata(void)
{
	float f_average_ang= 0, /*ax=0,*/ ay = 0, az = 0;
	static double last_ay = 0, last_az = 0;
	uint8_t i= 0;
	uint16_t angle_sample[DEF_WATERMARK_VAL] = {0};
	for(i = 0; i < DEF_WATERMARK_VAL; i++)
	{
		int16_t accel_xyz[3];
		accel_xyz[0] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)] << 8) | (uint16_t)accel_buff[(i*6)+1]);
		accel_xyz[1] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+2] << 8) | (uint16_t)accel_buff[(i*6)+3]);
		accel_xyz[2] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+4] << 8) | (uint16_t)accel_buff[(i*6)+5]);

		//ax = ((float)accel_xyz[0])/(float)(SENSITIVITY_2G*16);
		ay = ((float)accel_xyz[1])/(float)(SENSITIVITY_2G*16);
		az = ((float)accel_xyz[2])/(float)(SENSITIVITY_2G*16);
		//f_average_ang = (float)(atan2((double)az,(double)ay)*180/PI)+180.0;
		//NRF_LOG_INFO( "f_average_ang0: " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(f_average_ang));
		//NRF_LOG_INFO( "X : " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(ax));
		//NRF_LOG_INFO( "Y : " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(ay));
		//NRF_LOG_INFO( "Z : " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(az));

		//low pass filter
		//ax = lowPassExponential(ax, last_ax);
		//last_ax = ax;
		ay = lowPassExponential(ay, last_ay);
		last_ay = ay;
		az = lowPassExponential(az, last_az);
		last_az = az;
		//NRF_LOG_INFO( "f_average_ang1: " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(f_average_ang));
		//NRF_LOG_INFO( "X2: " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(ax));
		//NRF_LOG_INFO( "Y2: " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(ay));
		//NRF_LOG_INFO( "Z2: " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(az));
		//mag_accel =/rt(ax*ax + ay*ay + az*az);
		//acc_step_update(mag_accel);

		// LezyneSPD use z and x axis
		f_average_ang = (float)(atan2((double)az,(double)ay)*180/PI)+180.0;
		//NRF_LOG_INFO( "f_average_ang: " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(f_average_ang));
		angle_sample[i] = (uint16_t)(f_average_ang + 0.5);
	}
	acc_step_update_angle(angle_sample);

}
/**@brief Function acc_step_update_angle process accelerometer angular data to lap and total angle.
 *
 */
static void acc_step_update_angle(uint16_t *angle_array)
{
	uint8_t i = 0;
	uint16_t last_angle = angle_array[DEF_WATERMARK_VAL-1];
	int16_t current_angle = 0, temp_angle_diff = 0;
	bool cur_is_clockwise = true;
	/* ticks time */
	static uint32_t last_ticks = 0;
	uint32_t ticks_current = app_timer_cnt_get();
	uint32_t diff_ticks = (ticks_current > last_ticks)? (ticks_current - last_ticks) : (ticks_current + 0xFFFFFF - last_ticks);
	//uint32_t system_time = ROUNDED_DIV((ticks_current * (APP_TIMER_PRESCALER  + 1) * 1000), APP_TIMER_CLOCK_FREQ);
	uint32_t diff_time = (ROUNDED_DIV((diff_ticks * 1000), APP_TIMER_CLOCK_FREQ))+ ACCEL_ANGLE_PROCESS_COMPENSATE_TIME;

	// 25 sample angles processing
	for(i = 0; i < DEF_WATERMARK_VAL ; i++) // compare angle_array[n] ,
	{
		if(i == 0) // compare first sample with last_angle_residue
		{
			temp_angle_diff = (last_angle_residue == DEF_INVALID_LAST_ANGLE)? 0 : (int16_t)(angle_array[i] - last_angle_residue);
		}
		else  	// compare n with n-1
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
#ifdef SENSOR_DEBUG_OUTPUT
		if(i == 0)
		{
			//NRF_LOG_INFO("i:%d,[%d]-[%d] =[%d]   ",i,angle_array[i], last_angle_residue,temp_angle_diff );
		}
		else
		{
			//NRF_LOG_INFO("i:%d,[%d]-[%d] =[%d]   ",i,angle_array[i], angle_array[i-1],temp_angle_diff );
		}
#endif
	}

	cur_is_clockwise = (current_angle >= 0)? true : false;
	current_angle = abs(current_angle);
	last_angle_residue = last_angle;		// update last_angle_residue for current last_angle

	rw_lock_set(true);	// protect the global varible
	// event time is 1024-based. maximum value shall less that  64,000 (0x10000 / 0x800 * 2000)
	ui16_total_time = (ui16_total_time + diff_time) % (DEF_TOTAL_TIME_STAMP_MAXIMUM);

	last_ticks =  ticks_current;
	// movement detection
	if(current_angle < ACCEL_MOVE_ANGLE_MIN) 	// speed < 1.0kmh
	{
		if( ui8_movecnt == 0)
		{
			//NRF_LOG_INFO("movecnt =0");
			rw_lock_set(false);	//release protection
			return;
		}
		else if( ui8_movecnt < ACCEL_MOVE_COUNT_MIN)
		{
			ui8_movecnt = 0;
			//NRF_LOG_INFO("movecnt < min");
			rw_lock_set(false);	//release protection
			return;
		}
		else
		{
			//NRF_LOG_INFO("movecnt -- ");
			ui8_movecnt--;
			clockwise_flag = cur_is_clockwise;
		}
	}
	else										// speed >= 1.0kmh
	{
		if(ui8_movecnt == ACCEL_MOVE_COUNT_MAX)
		{
			//NRF_LOG_INFO("movecnt = MAX");
		}
		else if(ui8_movecnt < ACCEL_MOVE_COUNT_MIN )
		{
			if(cur_is_clockwise == clockwise_flag)
			{
				ui8_movecnt++;
				//NRF_LOG_INFO("movecnt ++");
			}
			else
			{
				clockwise_flag = cur_is_clockwise;
				ui8_movecnt = 0;
				//NRF_LOG_INFO("movecnt ++ invert clockwise");
			}
			rw_lock_set(false);	//release protection
			return; // do not need to update total angle and lap
		}
		else 									//(ui32_movecnt >= ACCEL_MOVE_COUNT_MINX)
		{
			//NRF_LOG_INFO("movecnt > min");
			ui8_movecnt = ACCEL_MOVE_COUNT_MAX;
		}
	} /*end of movement detection*/

	ui32_total_angle += current_angle;
	ui32_total_lap = (ui32_total_angle/DEF_ANGLE_360_DEGREE);

	rw_lock_set(false);	//release protection

#ifdef SENSOR_DEBUG_OUTPUT
	//NRF_LOG_INFO("update_angle_test(), Total A:%d, Total L:%d, Cur_A: %d. ", ui32_total_angle,ui32_total_lap, current_angle )
#endif
}

/**@brief Function acc_step_reset_angle reset to default value.
 *
 */
void acc_step_reset_angle(void)
{
    last_angle_residue = DEF_INVALID_LAST_ANGLE ;
    ui32_total_lap = 0;
    ui32_total_angle = 0;
    ui16_total_time =0;
}


/**@brief Function for Simple accelerometer offset calibration.
 */  
void accel_calibration (void)  
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
	Z_offset = (Zout_12_bit - SENSITIVITY_2G) / 2 * (-1); // Compute Z-axis offset correction value

	accel_write_reg(OFF_X_REG, X_offset);
	nrf_delay_ms(2);
	accel_write_reg( OFF_Y_REG, Y_offset);
	nrf_delay_ms(2);
	accel_write_reg(OFF_Z_REG, Z_offset);
	nrf_delay_ms(2);

	accel_weak_up(); // Active mode again
}

/**@brief Function for populating simulated cycling speed measurements.
 */
ret_code_t accel_csc_measurement(ble_cscs_meas_t * p_measurement)
{
    static uint16_t ui16_last_total_time = 0, ui16_last_event_time = 0;	// total time is 1000-based, event time is 1024-based time
    static uint32_t ui32_last_total_angle = 0, ui32_last_lap = 0;
    uint16_t event_time_inc =0, total_time_diff = 0, last_wheel_event_time = 0;			// event time is 1024-based time
    uint16_t current_angle = 0, average_speed_kmh = 0,  current_lap = 0;
    //uint8_t ctrl_reg[4];

    if(rw_lock_get())	// if rw_lock is ture means the critical global variable is updating.
    {
    	NRF_LOG_INFO("Warning!!! Read and Write collision, skip report");
    	//for test
    	p_measurement->is_wheel_rev_data_present = true;
    	p_measurement->is_crank_rev_data_present = false;
    	p_measurement->cumulative_wheel_revs = 0xFFFFFFFF;
    	p_measurement->last_wheel_event_time = 0xFFFF;  //need to change
    	return NRF_ERROR_INVALID_STATE;
    }
    else if( (ui8_movecnt > ACCEL_MOVE_COUNT_MIN) && (ui32_last_lap == ui32_total_lap))
    {
    	NRF_LOG_INFO("Warning!!! Lap no changed, skip report");
    	return NRF_ERROR_INVALID_STATE;
    }

#if 0
    {
		uint16_t average_speed_kmh2 =0, current_angle2 =0;
		current_angle2 =  ui32_total_angle - ui32_last_total_angle;
		current_lap = (current_angle2/DEF_ANGLE_360_DEGREE);
		event_time_inc = ui16_total_time - ui16_last_total_time;
		average_speed_kmh2 =  (float) (current_angle2 * ANGLE_SPEED_TO_METER_PER_HOUR / event_time_inc );

		last_wheel_event_time = ui16_last_total_time + (float)(current_lap * DEF_ANGLE_360_DEGREE * event_time_inc * 1.024 / current_angle2 );
		//NRF_LOG_INFO("last_wheel_event_time1: %d",last_wheel_event_time);
		//NRF_LOG_INFO("Report: Speed2: %d kmh, ui32_total_angle: %d, last angle: %d ", average_speed_kmh2, ui32_total_angle,ui32_last_total_angle );
	}
#endif

    total_time_diff = (ui16_total_time>ui16_last_total_time)? ui16_total_time - ui16_last_total_time : ui16_total_time + DEF_TOTAL_TIME_STAMP_MAXIMUM - ui16_last_total_time;
    current_angle = ui32_total_angle - (ui32_last_lap * DEF_ANGLE_360_DEGREE );
    current_lap = (current_angle/DEF_ANGLE_360_DEGREE);
    event_time_inc = (float)(current_lap * 1.024 * DEF_ANGLE_360_DEGREE * total_time_diff / (ui32_total_angle - ui32_last_total_angle) );
    if(event_time_inc !=0)
    {
    	last_wheel_event_time =  ui16_last_event_time + event_time_inc;			// do not take care warps around case.
    }
    else
    {
    	last_wheel_event_time = (ui16_total_time * 1024 / 1000 );					// no move case, need update latest event time (changing to 1024-based)
    }
    //NRF_LOG_INFO("last_wheel_event_time2: %d, ui16_last_total_time: %d, ui16_total_time: %d, event_time_inc: %d ", last_wheel_event_time, ui16_last_total_time, ui16_total_time, event_time_inc);


    average_speed_kmh = (float) (current_lap * DEF_ANGLE_360_DEGREE * ANGLE_SPEED_TO_METER_PER_HOUR * 1.024 / event_time_inc);

    //NRF_LOG_INFO("Report: Speed1: %d kmh, ui32_total_angle: %d, last angle: %d ", average_speed_kmh, ui32_total_angle,ui32_last_total_angle );
    //NRF_LOG_INFO("average_speed_kmh: %d, current_lap:%d, event time: %d, last time:%d",average_speed_kmh ,current_lap, last_wheel_event_time, ui16_last_event_time  );
    //NRF_LOG_INFO("ui32_last_lap: %d,  ui32_total_lap: %d",ui32_last_lap,ui32_total_lap);

    p_measurement->is_wheel_rev_data_present = true;
    p_measurement->is_crank_rev_data_present = false;
    p_measurement->cumulative_wheel_revs = ui32_total_lap;
    p_measurement->last_wheel_event_time = last_wheel_event_time;  //need to change
//for test log
    p_measurement->is_crank_rev_data_present = true;
    p_measurement->cumulative_crank_revs 	 = (uint16_t)(average_speed_kmh);
    p_measurement->last_crank_event_time	 = (uint16_t)(ui16_total_time);

    ui16_last_event_time = last_wheel_event_time;
    ui16_last_total_time = ui16_total_time;
    ui32_last_total_angle = ui32_total_angle;
    ui32_last_lap = ui32_total_lap;
    //NRF_LOG_INFO("measurement Report: total_lap: %d, last_time: %d, Speed: %d kmh", p_measurement->cumulative_wheel_revs, last_wheel_event_time,average_speed_kmh);
    return NRF_SUCCESS;
}

#if 0
/**@brief Function for populating simulated cycling speed and cadence measurements.
 */
static void accel_csc_measurement(ble_cscs_meas_t * p_measurement)
{
    static uint16_t cumulative_crank_revs = 0;
    static uint16_t event_time            = 0;
    static uint16_t wheel_revolution_mm   = 0;
    static uint16_t crank_rev_degrees     = 0;

    uint16_t mm_per_sec;
    uint16_t degrees_per_sec;
    uint16_t event_time_inc;

    // Per specification event time is in 1/1024th's of a second.
    event_time_inc = (1024 * SPEED_AND_CADENCE_MEAS_INTERVAL) / 1000;

    // Calculate simulated wheel revolution values.
    p_measurement->is_wheel_rev_data_present = true;

    mm_per_sec = KPH_TO_MM_PER_SEC * sensorsim_measure(&m_speed_kph_sim_state,
                                                       &m_speed_kph_sim_cfg);

    wheel_revolution_mm     += mm_per_sec * SPEED_AND_CADENCE_MEAS_INTERVAL / 1000;
    m_cumulative_wheel_revs += wheel_revolution_mm / WHEEL_CIRCUMFERENCE_MM;
    wheel_revolution_mm     %= WHEEL_CIRCUMFERENCE_MM;

    p_measurement->cumulative_wheel_revs = m_cumulative_wheel_revs;
    p_measurement->last_wheel_event_time =
        event_time + (event_time_inc * (mm_per_sec - wheel_revolution_mm) / mm_per_sec);

    // Calculate simulated cadence values.
    p_measurement->is_crank_rev_data_present = true;

    degrees_per_sec = RPM_TO_DEGREES_PER_SEC * sensorsim_measure(&m_crank_rpm_sim_state,
                                                                 &m_crank_rpm_sim_cfg);

    crank_rev_degrees     += degrees_per_sec * SPEED_AND_CADENCE_MEAS_INTERVAL / 1000;
    cumulative_crank_revs += crank_rev_degrees / DEGREES_PER_REVOLUTION;
    crank_rev_degrees     %= DEGREES_PER_REVOLUTION;

    p_measurement->cumulative_crank_revs = cumulative_crank_revs;
    p_measurement->last_crank_event_time =
        event_time + (event_time_inc * (degrees_per_sec - crank_rev_degrees) / degrees_per_sec);

    event_time += event_time_inc;
}


// type
struct ble_cscs_s
{
    ble_cscs_evt_handler_t       evt_handler;                           /**< Event handler to be called for handling events in the Cycling Speed and Cadence Service. */
    uint16_t                     service_handle;                        /**< Handle of Cycling Speed and Cadence Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     meas_handles;                          /**< Handles related to the Cycling Speed and Cadence Measurement characteristic. */
    ble_gatts_char_handles_t     feature_handles;                       /**< Handles related to the Cycling Speed and Cadence feature characteristic. */
    ble_gatts_char_handles_t     sensor_loc_handles;                    /**< Handles related to the Cycling Speed and Cadence Sensor Location characteristic. */
    uint16_t                     conn_handle;                           /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint16_t                     feature;                               /**< Bit mask of features available on sensor. */
    ble_sc_ctrlpt_t              ctrl_pt;                               /**< data for speed and cadence control point */
};
typedef struct ble_cscs_meas_s
{
    bool        is_wheel_rev_data_present;                              /**< True if Wheel Revolution Data is present in the measurement. */
    bool        is_crank_rev_data_present;                              /**< True if Crank Revolution Data is present in the measurement. */
    uint32_t    cumulative_wheel_revs;                                  /**< Cumulative Wheel Revolutions. */
    uint16_t    last_wheel_event_time;                                  /**< Last Wheel Event Time. */
    uint16_t    cumulative_crank_revs;                                  /**< Cumulative Crank Revolutions. */
    uint16_t    last_crank_event_time;                                  /**< Last Crank Event Time. */
} ble_cscs_meas_t;
#endif


/**@brief Function initialize function.
 */
void accel_init(void)
{
	NRF_LOG_INFO("accel_init.");

	/* hardware initialize - I2C & GPIO */
	accel_i2c_gpio_init();

	/* configuration */
	accel_configuration();

	/* timer init */
	accel_timers_init();

#ifdef ACCELEROMETER_SELF_ACTIVATE
	application_timers_start();
#endif

	/* enable gpio */
	accel_i2c_gpio_enable();
	
	/* reset acc step angle parameters */
	acc_step_reset_angle();

}
#endif /* SENSOR_ACCELEROMETER_C_ */
