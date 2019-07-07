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

/* Data rate 20ms = 50Hz, the highest bicycle speed =75 Km/h = 20.83 m/s, 
   max RPM = 20.83 / 2m (circomference) = 10.41 Hz but need a least 4 samples to calculate = 41.64 Hz ~= 50Hz  */ 
#define ACCEL_DATARATE  						DATA_RATE_80MS  //DATA_RATE_20MS
/* ASLP_RATE_160MS = 6.25 hz */
#define ACCEL_SLEEPP_DATARATE					ASLP_RATE_160MS
/* Threshold =0x08; The step count is 0.063g/ count,  (0.5g/0.063g = 7.9); */
#define ACCEL_FF_MT_THS_VALUE  					(THS3_MASK)
/* Set the debounce counter to eliminate false readings for 20ms data rate, with requirement of 100 ms, 100/20 = 5 counts */
#define ACCEL_FF_MT_DEBOUNCE_COUNT 				2
/* set the minimum duration time to SLEEP mode, ASLP_COUNT Step = 320ms, 320ms * 15 = 4.8s */    							
#define ACCEL_ENTER_SLEEP_COUNTER				10

/* FIFO Event Sample Count Watermark, the value shall be less than 64 */
#define  DEF_WATERMARK_VAL 						25   //25

#define  SENSOR_8BIT_DATA_OUTPUT_SUPPORT

APP_TIMER_DEF(m_csc_meas_timer_id);                                                 /**< CSC measurement timer. */

/* 12bit xyz fifo data */
static uint8_t accel_buff[6*DEF_WATERMARK_VAL];
/* acc_step_update_angle variable */
static t_accel_task_pending task_pending_singal ={0};
static int16_t last_angle_residue	= DEF_INVALID_LAST_ANGLE;
static uint32_t ui32_total_lap=0;
static uint32_t total_angle= 0;


/* FUNCTIONS */
static void 	application_timers_start(void);
static void 	application_timers_stop(void);
static void 	acc_step_update_angle(uint16_t *angle_array);

static void 	accel_self_csc_meas_timeout_handler(void * p_context);
static ret_code_t accel_write_reg(uint8_t reg_addr, uint8_t reg_data);
static ret_code_t accel_read_reg(uint8_t reg_addr, uint8_t *reg_data);
static ret_code_t accel_burst_read_reg(uint8_t addr, uint8_t * pdata, size_t size);
static void 	accel_i2c_gpio_enable (void);
static void 	accel_config_fifo_int(bool enable);
static void 	accel_config_motion_int(bool enable);
static void 	accel_weak_up(void);
static void 	accel_standby(void);

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
	NRF_LOG_INFO("mma8652_int2_handler STATUS:%2x, INT: %2x, SYSMOD:%2x",ui8_status, ui8_int_source, ui8_sysmod);
	//NRF_LOG_INFO("mma8652_int2_handler.");

	// FIFO Interrupt enabled
    if (ui8_int_source & SRC_FIFO_MASK)
	{
    	NRF_LOG_INFO("mma8652_int2_handler. FIFO Handle");

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
}

/**@brief handler for interrupt of input pin1, for Motion Detection and Auto-WAKE/SLEEP event
 */
void mma8652_int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint8_t ui8_int_source = 0;
	uint8_t ui8_status = 0, ui8_sysmod = 0, ff_mt_src = 0;
	//ret_code_t result;

	/* Check System mode first*/
	accel_read_reg(MMA8652_INT_SOURCE,		&ui8_int_source);
	accel_read_reg(MMA8652_STATUS_00,		&ui8_status);
	accel_read_reg(MMA8652_SYSMOD, 			&ui8_sysmod);
	NRF_LOG_INFO("mma8652_int1_handler STATUS:%2x, INT: %2x, SYSMOD:%2x", ui8_status, ui8_int_source, ui8_sysmod);

	// motion detection interrupt
	if (ui8_int_source & SRC_FF_MT_MASK)
	{
		NRF_LOG_INFO("FF MT Detection INT:");
		accel_read_reg(MMA8652_FF_MT_SRC,		&ff_mt_src);
		NRF_LOG_INFO("MMA8652_FF_MT_SRC: %x ",ff_mt_src );

	}
	// Auto-WAKE/SLEEP interrupt
	if(ui8_int_source & SRC_ASLP_MASK)
	{
		NRF_LOG_INFO("Auto-WAKE/SLEEP detection: SYSMOD: %x",ui8_sysmod);
		if ((ui8_sysmod & SYSMOD_WAKE) == SYSMOD_WAKE)
		{
			/* sleep -> wake up*/
			/* Set to standby mode*/
		    accel_standby();

			accel_config_fifo_int(true);
			accel_config_motion_int(false);
			NRF_LOG_INFO("enable FIFO, disable motion detection")
			/* after wake up, start timer to send cscs information to ble*/
			//application_timers_stop();
			// Set back to active
			accel_weak_up();
		}
		else if((ui8_sysmod & SYSMOD_SLEEP) == SYSMOD_SLEEP)
		{
			/* wake up -> sleep,*/
			/* Set to standby mode*/
		    accel_standby();
			accel_config_fifo_int(false);
			accel_config_motion_int(true);
			NRF_LOG_INFO("disable FIFO, enable motion detection")
			/* after sleep, stop timer to send cscs information to ble*/
			//application_timers_stop();
			// Set back to active
			accel_weak_up();
		}
	}
}

/**@brief Function accel_csc_meas_timeout_handler.
 *  self test timeout handler, should remove this function after official release
 */
static void accel_self_csc_meas_timeout_handler(void * p_context)
{
    //uint32_t        err_code = NRF_SUCCESS;
    //ble_cscs_meas_t cscs_measurement;

    //NRF_LOG_INFO("accel_csc_meas_timeout_handler");
#if 1   // only for test!!!
    uint8_t ctrl_reg[4];
    accel_read_reg(MMA8652_SYSMOD, 			&ctrl_reg[2]);
    accel_read_reg(MMA8652_INT_SOURCE,		&ctrl_reg[0]);
    accel_read_reg(MMA8652_STATUS_00,		&ctrl_reg[1]);
    accel_read_reg(MMA8652_CTRL_REG4,		&ctrl_reg[3]);
   	NRF_LOG_INFO("timeout_handler: sysmode: %2x, INT: %2x, STATUS: %2x, REG4: %2x.", ctrl_reg[2], ctrl_reg[0],ctrl_reg[1],ctrl_reg[3]);
   	{
   		ble_cscs_meas_t accel_measuremen;
		accel_csc_measurement(&accel_measuremen);
   	}


#else
    if (ble_connection_status())
    {
		UNUSED_PARAMETER(p_context);

		accel_csc_measurement(&cscs_measurement);

		err_code = ble_cscs_measurement_send(&m_cscs, &cscs_measurement);
		if ((err_code != NRF_SUCCESS) &&
			(err_code != NRF_ERROR_INVALID_STATE) &&
			(err_code != NRF_ERROR_RESOURCES) &&
			(err_code != NRF_ERROR_BUSY) &&
			(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
		   )
		{
			APP_ERROR_HANDLER(err_code);
		}
    }
#endif
}

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
								//accel_csc_meas_timeout_handler);		// timeout handler in ble_core.c
								accel_self_csc_meas_timeout_handler);  // selftest print log
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

/**@brief Function for the sensor accelerometer configuration.
 */
static void accel_configuration(void)
{
	NRF_LOG_INFO("accel_configuration.");

	uint8_t who_n_i;
	/* read WHO_AND_I first */
	accel_read_reg(MMA8652_WHO_AM_I, &who_n_i);
	if(who_n_i != MMA8652_WHO_AM_I_OUT )
	{
		NRF_LOG_ERROR(" Device ID not match!! :MMA8652 0x4A: 0x%x", who_n_i );
		return;
	}

    /* RESET sensor, all registers are reset to default */
    accel_write_reg(MMA8652_CTRL_REG2, RST_MASK);
    nrf_delay_ms(100);

	/* Set to standby mode*/
    accel_standby();


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
    
	/* Set REG2, enable Auto-SLEEP enable and High Resolution */
    accel_write_reg(MMA8652_CTRL_REG2, MOD_HIGH_RES|SLPE_MASK);

    /* Set REG4, enable Auto-SLEEP/WAKE Interrupt and the FIFO Interrupt, (do not enable Motion detection interrupt in initialization) */
    accel_write_reg(MMA8652_CTRL_REG4, INT_EN_ASLP_MASK | INT_EN_FIFO_MASK | INT_EN_FF_MT_MASK);

    /* Set REG5, Auto-SLEEP/WAKE and Motion detection Interrupt Enable mapped to INT1, and FIFO Interrupt mapped to INT2(default) */
    accel_write_reg(MMA8652_CTRL_REG5, INT_EN_ASLP_MASK | INT_EN_FF_MT_MASK); //Set the interrupt to route to INT1

    /* Set REG3, Configure Wake from Freefall/Motion interrupt, and the INT pins for Push-Pull */
    accel_write_reg(MMA8652_CTRL_REG3, WAKE_FF_MT_MASK);

    /* Set REG1, Set to 20ms data rate period (50Hz), 160ms sleep datarate, non-Fast read mode (12bit). */
    accel_write_reg(MMA8652_CTRL_REG1, ACCEL_DATARATE | ACCEL_SLEEPP_DATARATE );

	/* Motion configuration and status registers */
	/* set MMA8652_FF_MT_CFG, setup motion event after the debounce counter time is reached, ELE = 0, OAE = 1, Event flag enable on X and Z*/
	//accel_write_reg(MMA8652_FF_MT_CFG, OAE_MASK|ZEFE_MASK|XEFE_MASK );
	accel_write_reg(MMA8652_FF_MT_CFG, OAE_MASK|ZEFE_MASK );

	/* set MMA8652_FF_MT_THS, setup THS = ACCEL_FF_MT_THS_VALUE */
	accel_write_reg(MMA8652_FF_MT_THS, ACCEL_FF_MT_THS_VALUE );
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
    uint8_t ctrl_reg[12];

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

	NRF_LOG_INFO("F_SETUP:%x, R1:%x, R2:%x, R3:%x, R4:%x.",
            ctrl_reg[0], ctrl_reg[1], ctrl_reg[2], ctrl_reg[3], ctrl_reg[4]);
	NRF_LOG_INFO("R5:%x, INT:%x, MT:%x %x %x.",
            ctrl_reg[5], ctrl_reg[6], ctrl_reg[7], ctrl_reg[8], ctrl_reg[9]);
	NRF_LOG_INFO("ASLP:%x, XYZ:%x.",ctrl_reg[10], ctrl_reg[11]);
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
		accel_write_reg(MMA8652_F_SETUP, F_MODE_FILL|DEF_WATERMARK_VAL);
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
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
    //err_code = nrf_drv_gpiote_in_init(MMA8652_INT1_PIN, &in_config, mma8652_int1_handler);
    err_code = nrf_drv_gpiote_in_init(MMA8652_INT2_PIN, &in_config, mma8652_int2_handler);

    APP_ERROR_CHECK(err_code);

    /* GPIO Initialize as system wakeup pin */
    nrf_drv_gpiote_in_config_t in_config_1 = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    in_config_1.pull = NRF_GPIO_PIN_PULLUP;
	err_code = nrf_drv_gpiote_in_init(MMA8652_INT1_PIN, &in_config_1, mma8652_int1_handler);
	APP_ERROR_CHECK(err_code);

	return err_code;
}
/**@brief start gpio event I2C.
 */
static void accel_i2c_gpio_enable (void)
{
	nrf_drv_gpiote_in_event_enable(MMA8652_INT2_PIN, true);
	nrf_drv_gpiote_in_event_enable(MMA8652_INT1_PIN, true);
	NVIC_EnableIRQ(GPIOTE_IRQn);
}

/**@brief Function for set the sensor accelerometer to ACTIVATE mode.
 *	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE			
 */
void accel_set_deactive(void)
{
	/* Set to standby mode*/
    accel_standby();

	accel_config_fifo_int(false);
	accel_config_motion_int(true);
	NRF_LOG_INFO("accel_set_deactive: disable FIFO, enable motion detection")
	/* stop timer to send cscs information to ble*/
	application_timers_stop();
	acc_step_reset_angle();
	// Set back to active
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
	NRF_LOG_INFO("accel_set_active: enable FIFO, disable motion detection")
	application_timers_start();
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

/**@brief Function for burst read sensor fifo accelerometer data.
 *	read numbers of DEF_WATERMARK_VAL from accel_buff[], and turn into DEF_WATERMARK_VAL angular samples
 */
void acc_read_fifodata(void)
{
	float f_average_ang= 0, ax=0,ay=0, az= 0;
	uint8_t i= 0;
	uint16_t angle_sample[DEF_WATERMARK_VAL] = {0};
	for(i = 0; i < DEF_WATERMARK_VAL; i++)
	{
		int16_t accel_xyz[3];
		accel_xyz[0] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)] << 8) | (uint16_t)accel_buff[(i*6)+1]);
		accel_xyz[1] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+2] << 8) | (uint16_t)accel_buff[(i*6)+3]);
		accel_xyz[2] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+4] << 8) | (uint16_t)accel_buff[(i*6)+5]);

		ax = ((float)accel_xyz[0])/(float)(SENSITIVITY_2G*16);
		//ay = ((float)accel_xyz[1])/(float)(SENSITIVITY_2G*16);
		az = ((float)accel_xyz[2])/(float)(SENSITIVITY_2G*16);
		//NRF_LOG_INFO( "X: " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(ax));
		//NRF_LOG_INFO( "Y: " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(ay));
		//NRF_LOG_INFO( "Z: " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(az));
		//mag_accel =/rt(ax*ax + ay*ay + az*az);
		//acc_step_update(mag_accel);

		// LezyneSPD use z and x axis
		f_average_ang = (float)(atan2((double)az,(double)ax)*180/PI)+180.0;
		//NRF_LOG_INFO( "f_average_ang: " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT(f_average_ang));
		angle_sample[i]=f_average_ang;
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

	for(i = 0; i < DEF_WATERMARK_VAL ; i++) // compare angle_array[n] ,
	{
		if(i == 0) // compare first sample with last_angle_residue
		{
			if(last_angle_residue == DEF_INVALID_LAST_ANGLE)
			{
				temp_angle_diff = 0;
				//last_angle_residue = angle_array[0];
			}
			else
			{
				temp_angle_diff = (int16_t)(angle_array[i] - last_angle_residue);
			}
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
			NRF_LOG_INFO("Warning!!! i:(%d) the difference (%d) > 75, [%d]-[%d] ",angle_array[i] , (i==0)?last_angle_residue:angle_array[i-1] );
		}

		current_angle += temp_angle_diff;
		if(i == 0)
		{
			//NRF_LOG_INFO("i:%d,[%d]-[%d] =[%d]   ",i,angle_array[i], last_angle_residue,temp_angle_diff );
		}
		else
		{
			//NRF_LOG_INFO("i:%d,[%d]-[%d] =[%d]   ",i,angle_array[i], angle_array[i-1],temp_angle_diff );
		}
	}

	last_angle_residue = last_angle;		// update last_angle_residue for current last_angle

	total_angle += abs(current_angle);
	ui32_total_lap = (total_angle/DEF_ANGLE_360_DEGREE);

	//NRF_LOG_INFO("update_angle_test(), Total A:%d, Total L:%d, Cur_A: %d. ", total_angle,ui32_total_lap, current_angle )
	NRF_LOG_INFO("update_angle_test(),n:[%3d]-0:[%3d] = :%3d, Total: %5d",last_angle,angle_array[0],current_angle, total_angle);
}

/**@brief Function acc_step_reset_angle reset to default value.
 *
 */
void acc_step_reset_angle(void)
{
    last_angle_residue = DEF_INVALID_LAST_ANGLE ;
    ui32_total_lap = 0;
    total_angle = 0;
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

/**@brief Function for populating simulated cycling speed and cadence measurements.
 */
void accel_csc_measurement(ble_cscs_meas_t * p_measurement)
{
#if 0
    bool        is_wheel_rev_data_present;                              /**< True if Wheel Revolution Data is present in the measurement. */
    bool        is_crank_rev_data_present;                              /**< True if Crank Revolution Data is present in the measurement. */
    uint32_t    cumulative_wheel_revs;                                  /**< Cumulative Wheel Revolutions. */
    uint16_t    last_wheel_event_time;                                  /**< Last Wheel Event Time. */
    uint16_t    cumulative_crank_revs;                                  /**< Cumulative Crank Revolutions. */
    uint16_t    last_crank_event_time;                                  /**< Last Crank Event Time. */
#endif
    uint8_t ctrl_reg[4];
    accel_read_reg(MMA8652_SYSMOD, 			&ctrl_reg[2]);
    //accel_read_reg(MMA8652_INT_SOURCE,		&ctrl_reg[0]);
    //accel_read_reg(MMA8652_STATUS_00,		&ctrl_reg[1]);
    //accel_read_reg(MMA8652_CTRL_REG4,		&ctrl_reg[3]);

    p_measurement->is_wheel_rev_data_present = true;
    p_measurement->is_crank_rev_data_present = false;
    p_measurement->cumulative_wheel_revs = ui32_total_lap;
    p_measurement->last_wheel_event_time = (uint16_t) total_angle;  //need to change

    NRF_LOG_INFO("measurement Report: sysmode: %2x, cum_wheel_revs: %d, last_time: %d", ctrl_reg[2], p_measurement->cumulative_wheel_revs,p_measurement->last_wheel_event_time);

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

	application_timers_start(); // for test propose!!!

	/* enable gpio */
	accel_i2c_gpio_enable();
	
	/* reset acc step angle parameters*/
	acc_step_reset_angle();

}
#endif /* SENSOR_ACCELEROMETER_C_ */
