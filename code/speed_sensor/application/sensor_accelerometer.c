/*
 * 	sensor_accelerometer.c
 *
 *  Created on: June 30, 2019
 *  Author: Lezyne
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
#include "ble_cscs.h"
#include "sensorsim.h"

#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Configuration */
#define SENSOR_DEBUG_OUTPUT

APP_TIMER_DEF(m_csc_meas_timer_id);

/* I2C configuration */
static const nrf_drv_twi_t acce_m_twi = NRF_DRV_TWI_INSTANCE(MMA8652_TWI_INSTANCE_ID);

/* defination of ratio of the circumference of a circle to its diameter*/
#define PI 3.14159265

/* Data rate 20ms = 50Hz, the highest bicycle speed =75 Km/h = 20.83 m/s, 
   max RPM = 20.83 / 2m (circomference) = 10.41 Hz but need a least 4 samples to calculate = 41.64 Hz ~= 50Hz  */ 
#define ACCEL_DATARATE  						DATA_RATE_20MS
/* ASLP_RATE_160MS = 6.25 hz */
#define ACCEL_SLEEPP_DATARATE					ASLP_RATE_160MS
/* Threshold =0x30; The step count is 0.063g/ count,  3g/0.063g = 47.6; */
#define ACCEL_FF_MT_THS_VALUE  					(THS5_MASK | THS4_MASK)
/* Set the debounce counter to eliminate false readings for 20ms data rate, with requirement of 200 ms, 200/20 = 10 counts */  
#define ACCEL_FF_MT_DEBOUNCE_COUNT 				10
/* set the minimum duration time to SLEEP mode, ASLP_COUNT Step = 320ms, 320ms * 15 = 4.8s */    							
#define ACCEL_ENTER_SLEEP_COUNTER				15
/* Speed and cadence measurement interval (milliseconds). */
#define SPEED_AND_CADENCE_MEAS_INTERVAL 		1000
/* FIFO Event Sample Count Watermark, the value shall be less than 64 */
#define  DEF_WATERMARK_VAL 25

#define  SENSOR_8BIT_DATA_OUTPUT_SUPPORT
                                               /**< CSC measurement timer. */
#ifdef SENSOR_8BIT_DATA_OUTPUT_SUPPORT
	/* 8bit xyz fifo data */
	static uint8_t accel_buff[3*DEF_WATERMARK_VAL];
#else
	/* 12bit xyz fifo data */
	static uint8_t accel_buff[6*DEF_WATERMARK_VAL];
#endif

/* acc_step_update_angle variable */
static t_angle_step angle_step ={0};

/* FUNCTIONS */
static void acc_step_update_angle(float x, float y);
static void acc_step_reset_angle(void);
static void application_timers_start(void);
static void application_timers_stop(void);
static void acc_read_fifodata(void);
static ret_code_t accel_write_reg(uint8_t reg_addr, uint8_t reg_data);
static ret_code_t accel_read_reg(uint8_t reg_addr, uint8_t *reg_data);
static ret_code_t accel_burst_read_reg(uint8_t addr, uint8_t * pdata, size_t size);

/**@brief handler for interrupt of input pin
 */
void mma8652_int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	NRF_LOG_INFO("mma8652_int1_handler. FIFO Handle");
	acc_read_fifodata();
}

/**@brief handler for interrupt of input pin
 */
void mma8652_int2_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint8_t sysmode = 0;
	NRF_LOG_INFO("mma8652_int2_handler. Motion Detect Handle");

	/* Check System mode first*/
	accel_read_reg(MMA8652_SYSMOD, &sysmode);
	if (sysmode==SYSMOD_WAKE)
	{
		/* after wake up, start timer to send cscs information to ble*/
		//application_timers_start();
	}
	else if(sysmode==SYSMOD_SLEEP)
	{
		/* after wake up, start timer to send cscs information to ble*/
		//application_timers_stop();
	}
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
                                accel_csc_meas_timeout_handler);
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
	if (err_code)
		NRF_LOG_INFO("accel_I2C_read (W) error(0x%X).",err_code);
	/* read data */
	err_code = nrf_drv_twi_rx(&acce_m_twi, MMA8652_ADDRESS, reg_data, 1);
	if (err_code)
		NRF_LOG_INFO("accel_I2C_read (R) error(0x%X).",err_code);
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
static ret_code_t accel_burst_read_reg(uint8_t addr, uint8_t * pdata, size_t size)
{
	ret_code_t err_code = nrf_drv_twi_rx(&acce_m_twi, addr, pdata, size);
	if (err_code)
		NRF_LOG_INFO("accel_I2C_burst_read error(0x%X).",err_code);
	APP_ERROR_CHECK(err_code);
	return err_code;
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
    nrf_delay_ms(50);

    /* Set F_SETUP, FIFO set to Trigger mode, watermark = 25 * DATA_RATE_20MS = 500ms) */
    accel_write_reg(MMA8652_F_SETUP, F_MODE_TRIGGER | DEF_WATERMARK_VAL);
    nrf_delay_ms(2);
    
	/* Set REG2, enable Auto-SLEEP enable */
	accel_write_reg(MMA8652_CTRL_REG2, SLPE_MASK);
	nrf_delay_ms(2);
		
    /* Set REG4, enable Auto-SLEEP/WAKE Interrupt and the FIFO Interrupt, Motion detection interrupt */
    accel_write_reg(MMA8652_CTRL_REG4, INT_EN_ASLP_MASK | INT_EN_FIFO_MASK | INT_EN_FF_MT_MASK);
    nrf_delay_ms(2);

    /* Set REG5, Auto-SLEEP/WAKE and Motion detection Interrupt Enable mapped to INT2(default), and FIFO Interrupt mapped to INT1 */
    accel_write_reg(MMA8652_CTRL_REG5, INT_EN_FIFO_MASK);
    nrf_delay_ms(2);
		
    /* Set REG3, Configure Wake from Freefall/Motion interrupt, and the INT pins for Push-Pull and Active High */
    accel_write_reg(MMA8652_CTRL_REG3, WAKE_FF_MT_MASK | PP_OD_MASK);

	#ifdef SENSOR_8BIT_DATA_OUTPUT_SUPPORT
    /* Set REG1, Set to 20ms data rate period (50Hz), 160ms sleep datarate, Fast read mode (8bit). */
    accel_write_reg(MMA8652_CTRL_REG1,  ACCEL_DATARATE | ACCEL_SLEEPP_DATARATE | FREAD_MASK);
    #else
    /* Set REG1, Set to 20ms data rate period (50Hz), 160ms sleep datarate, non-Fast read mode (12bit). */
    accel_write_reg(MMA8652_CTRL_REG1, ACCEL_DATARATE | ACCEL_SLEEPP_DATARATE );
    #endif
    nrf_delay_ms(2);

	/* Motion configuration and status registers */
	/* set MMA8652_FF_MT_CFG, setup motion event after the debounce counter time is reached, ELE = 0, OAE = 1*/
	accel_write_reg(MMA8652_FF_MT_CFG, OAE_MASK );
	nrf_delay_ms(2);
	/* set MMA8652_FF_MT_THS, setup THS = ACCEL_FF_MT_THS_VALUE */
	accel_write_reg(MMA8652_FF_MT_THS, ACCEL_FF_MT_THS_VALUE );
	nrf_delay_ms(2);
	/* set MMA8652_FF_MT_COUNT, setup debounce counter ACCEL_FF_MT_DEBOUNCE_COUNT */
	accel_write_reg(MMA8652_FF_MT_COUNT, ACCEL_FF_MT_DEBOUNCE_COUNT );
	nrf_delay_ms(2);

	/* set ASLP_COUNT, setup sleep counter ACCEL_ENTER_SLEEP_COUNTER */
	accel_write_reg(MMA8652_ASLP_COUNT, ACCEL_ENTER_SLEEP_COUNTER );
	nrf_delay_ms(2);

	/* Turn on HPF for Data Out and set 2g Mode */
	accel_write_reg(MMA8652_XYZ_DATA_CFG, HPF_OUT_MASK );
	nrf_delay_ms(2);

    // Set back to active
    accel_set_active();
    nrf_delay_ms(2);

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
	nrf_delay_ms(2);
	NRF_LOG_INFO("R5:%x, INT:%x, MT:%x %x %x.",
            ctrl_reg[5], ctrl_reg[6], ctrl_reg[7], ctrl_reg[8], ctrl_reg[9]);
	nrf_delay_ms(2);
	NRF_LOG_INFO("ASLP:%x, XYZ:%x.",ctrl_reg[10], ctrl_reg[11]);
	#endif
	NRF_LOG_INFO("accel_configuration end.");
}

#if 0
need to enable/disable fifo interrupt?
/**@brief Function for enable of disable fifo interrupt.
 *	bool enable: disable:0, enable:1		
 */
static void accel_config_fifo_int(bool enable)
{
	if(enable)
	{
		uint8_t ctrl_reg4;
		accel_read_reg(MMA8652_CTRL_REG4, &ctrl_reg4);
		/* Set REG4, enable Auto-SLEEP/WAKE Interrupt and the FIFO Interrupt */
		accel_write_reg(MMA8652_CTRL_REG4, INT_EN_FIFO_MASK);
		nrf_delay_ms(2);
    }
    else
    {
    	uint8_t ctrl_reg4;
    	accel_read_reg(MMA8652_CTRL_REG4, &ctrl_reg4);
		/* Set REG4, Disable the FIFO Interrupt */
	    accel_write_reg(MMA8652_CTRL_REG4, ctrl_reg4 & ~INT_EN_FIFO_MASK);
	    nrf_delay_ms(2);
    }
}

/**@brief Function for enable or disable motion detection interrupt.
 *	bool enable: disable:0, enable:1		
 */
static void accel_config_motion_int(bool enable)
{
	if(enable)
	{
		/* Set REG2, enable Auto-SLEEP enable */
		accel_write_reg(MMA8652_CTRL_REG2, SLPE_MASK);
		nrf_delay_ms(2);
			
	    /* Set REG4, enable Auto-SLEEP/WAKE Interrupt and the FIFO Interrupt */
	    accel_write_reg(MMA8652_CTRL_REG4, INT_EN_ASLP_MASK | INT_EN_FIFO_MASK);
	    nrf_delay_ms(2);
	}
	else
	{
		uint8_t ctrl_reg2,ctrl_reg4;
	  	accel_read_reg(MMA8652_CTRL_REG2, &ctrl_reg2);
	  	accel_read_reg(MMA8652_CTRL_REG4, &ctrl_reg4);
	  	/* Set REG2, disable Auto-SLEEP enable */ 
		accel_write_reg(MMA8652_CTRL_REG2, ctrl_reg2 & ~SLPE_MASK);
		nrf_delay_ms(2);
		/* Set REG4, enable Auto-SLEEP/WAKE Interrupt and the FIFO Interrupt */
	    accel_write_reg(MMA8652_CTRL_REG4, ctrl_reg4 & ~INT_EN_ASLP_MASK);
	    nrf_delay_ms(2);
	}
}
#endif

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
	nrf_drv_gpiote_in_event_enable(MMA8652_INT1_PIN, true);
	NVIC_EnableIRQ(GPIOTE_IRQn);

	return err_code;
}


/**@brief Function for set the sensor accelerometer to ACTIVATE mode.
 *	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE			
 */
void accel_set_active(void)
{
	/* Read REG1 byte first; then disable ACTIVE bit for going to standby mode*/
	uint8_t ctrl_reg1;
	accel_read_reg(MMA8652_CTRL_REG1, &ctrl_reg1);
	accel_write_reg(MMA8652_CTRL_REG1, ctrl_reg1 | ACTIVE_MASK);
}

/**@brief Function for set the sensor accelerometer to ACTIVATE mode.
 *	operation mode: OFF, STANDBY, ACTIVATE-SLEEP, ACTIVATE-WAKE			
 */
void accel_weak_up(void)
{
	// TO DO: need it???
	/* Read REG1 byte first; then disable ACTIVE bit for going to standby mode*/
	uint8_t ctrl_reg1;
	accel_read_reg(MMA8652_CTRL_REG1, &ctrl_reg1);
	accel_write_reg(MMA8652_CTRL_REG1, ctrl_reg1 | ACTIVE_MASK);
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
}

/**@brief Function for burst read sensor fifo accelerometer data.
 *
 */
static void acc_read_fifodata(void)
{
    int i;
    static uint8_t intflag;
    uint8_t sysmode = 0;

    /* Check System mode first*/
    accel_read_reg(MMA8652_SYSMOD, &sysmode);
    if (sysmode==SYSMOD_WAKE)
    {

    }
    else if(sysmode==SYSMOD_SLEEP)
    {

    }
    else // sysmode == SYSMOD_STANDBY
    {

    }

    accel_read_reg(MMA8652_INT_SOURCE, &intflag);
	if(intflag & SRC_FIFO_MASK)
	{
		static float ax,ay; //,az;
		//static float mag_accel;
		uint8_t ui8_f_setup=0;
		#ifdef SENSOR_8BIT_DATA_OUTPUT_SUPPORT
			/* Fast read mode (8bit). */
			accel_burst_read_reg(MMA8652_OUT_X_MSB, accel_buff, 3*DEF_WATERMARK_VAL);
			accel_read_reg(MMA8652_F_SETUP, &ui8_f_setup);
			for(i = 0; i < DEF_WATERMARK_VAL; i++)
			{
				int16_t accel_xyz[3];

				accel_xyz[0] = ((uint16_t)accel_buff[(i*3)]);
				accel_xyz[1] = ((uint16_t)accel_buff[(i*3)+1]);
				accel_xyz[2] = ((uint16_t)accel_buff[(i*3)+2]);
				ax = ((float)accel_xyz[0])/(float)(SENSITIVITY_8BIT);
				ay = ((float)accel_xyz[1])/(float)(SENSITIVITY_8BIT);
				//az = ((float)accel_xyz[2])/(float)(SENSITIVITY_8BIT);
				//mag_accel = sqrt(ax*ax + ay*ay + az*az);
				//acc_step_update(mag_accel);
				acc_step_update_angle(ax,ay);
			}
		#else
			/* 12bit */
			accel_read_regs(MMA8652_OUT_X_MSB, accel_buff, 6*DEF_WATERMARK_VAL);
			accel_read_reg(MMA8652_F_SETUP);
			for(i = 0; i < DEF_WATERMARK_VAL; i++)
			{
				int16_t accel_xyz[3];

				accel_xyz[0] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)] << 8) | (uint16_t)accel_buff[(i*6)+1]);
				accel_xyz[1] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+2] << 8) | (uint16_t)accel_buff[(i*6)+3]);
				accel_xyz[2] = (int16_t)((uint16_t)((uint16_t)accel_buff[(i*6)+4] << 8) | (uint16_t)accel_buff[(i*6)+5]);
				ax = ((float)accel_xyz[0])/(float)(SENSITIVITY_2G*16);
				ay = ((float)accel_xyz[1])/(float)(SENSITIVITY_2G*16);
				az = ((float)accel_xyz[2])/(float)(SENSITIVITY_2G*16);
				mag_accel = sqrt(ax*ax + ay*ay + az*az);
				//acc_step_update(mag_accel);
				acc_step_update_angle(ax,ay);
			}
		#endif

	}
}
static void acc_step_update_angle(float x, float y)
{
	static float f_last_trs =0;
	int i=0;
	float f_temp_x =0;
	float f_temp_y =0;
	angle_step.f_raw_x[angle_step.ui32_rawidx] = x;
	angle_step.f_raw_y[angle_step.ui32_rawidx] = y;
	angle_step.ui32_rawidx++;
	angle_step.ui32_rawidx %= DEF_SAMPLE_TO_AVG;

	if(angle_step.ui32_sample < DEF_SAMPLE_TO_AVG)
	{
		angle_step.f_tempmin = 1.5;
		angle_step.f_tempmax = 0;
		angle_step.ui32_sample++;
		return;
	}
	for (i=0;i<DEF_SAMPLE_TO_AVG;i++ )
	{
		f_temp_x += angle_step.f_raw_x[i];
		f_temp_y += angle_step.f_raw_x[i];
	}
	//f_temp_x /= DEF_SAMPLE_TO_AVG;
	//f_temp_y /= DEF_SAMPLE_TO_AVG;
	angle_step.f_average_ang = ((float)atan(f_temp_y/f_temp_x) * 180.0 / PI) + 180;

	#ifdef SENSOR_DEBUG_OUTPUT
	NRF_LOG_INFO("Average Angle =(0x%X).", angle_step.f_average_ang);
	#endif
	// check clockwise

	// check anticlockwise
	angle_step.f_trs = angle_step.f_average_ang;
	f_last_trs = angle_step.f_trs;

	angle_step.ui32_sample++;
}

static void acc_step_reset_angle(void)
{
    angle_step.ui32_sample = 0;
    ///angle_step.startpeak = false;
    //angle_step.startvalley = false;
    //angle_step.step_state = eSTEP_RESET;
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

	accel_set_active(); // Active mode again
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

/**@brief Function for populating simulated cycling speed and cadence measurements.
 */
void accel_csc_measurement(ble_cscs_meas_t * p_measurement)
{
}

/**@brief Function initialize function.
 */
void accel_init(void)
{
	NRF_LOG_INFO("accel_init.");

	/* hardware initialize - I2C & GPIO */
	accel_i2c_gpio_init();

	/* configuration */
	//accel_configuration();

	/* timer init */
	accel_timers_init();

	/* timer start */
	application_timers_start();

	NRF_LOG_INFO("accel_init End.");
}
#endif /* SENSOR_ACCELEROMETER_C_ */
