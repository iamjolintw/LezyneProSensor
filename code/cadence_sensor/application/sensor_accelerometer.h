/*
 * 	sensor_accelerometer.h
 *
 *  Created on: June 30, 2019
 *  Author: Lezyne
 */

#ifndef SENSOR_ACCELEROMETER_H_
#define SENSOR_ACCELEROMETER_H_

#include <stdint.h>
#include <stdbool.h>
#include "mma8652_regs.h"
#include "sys_conf.h"
#include "nrf_error.h"
#include "nrf_log.h"
#include "ble_cscs.h"

/* DEFINITION */
#define MMA8652_ADDRESS 			0x1D

/* PARAMETERS */
enum acceSENSITIVITY {
	SENSITIVITY_2G = 1024,
	SENSITIVITY_4G = 512,
	SENSITIVITY_8G = 256,
	SENSITIVITY_8BIT =64
};

enum accelFSR {
	AFS_2g = 0,
	AFS_4g,
	AFS_8g
};

enum accelODR {
	AODR_800HZ = 0, // 200 Hz
	AODR_400HZ,
	AODR_200HZ,
	AODR_100HZ,
	AODR_50HZ,
	AODR_12_5HZ, // 12.5 Hz, etc.
	AODR_6_25HZ,
	AODR_1_56HZ
};

typedef enum{
	eSTEP_RESET,
	eSTEP_START_PEAK,
	eSTEP_PEAK_DETECT,
	eSTEP_STRAT_VALLEY,
	eSTEP_VALLEY_DETECT,
	eSTEP_STEP_DETECT,

}step_detect_t;

/* pending Task event */
typedef enum {
	SENSOR_TASK_INT_FIFO = 0x01,
	SENSOR_TASK_INT_DRDY = 0x02,
	SENSOR_TASK_INT_MTDT = 0x03,
	SENSOR_TASK_INT_DUMP = 0x04,
	SENSOR_TASK_MAX = 0xFF
}t_accel_task_pending;

#define DEF_MAX_ANGLE_WINDOW    85  // 75 +10 degree equal 85Km/hr in 50 hz ODR
#define DEF_ANGLE_90_DEGREE    90  //
#define DEF_ANGLE_180_DEGREE    180  //
#define DEF_ANGLE_180_DEGREE    180  //
#define DEF_ANGLE_270_DEGREE    270  //
#define DEF_ANGLE_360_DEGREE    360  //
/* defination a invalid angle for first time calculate lap*/
#define DEF_INVALID_LAST_ANGLE 	-999	//
/* defination of ratio of the circumference of a circle to its diameter*/
#define PI 3.14159265

#define ANGLE_SPEED_TO_METER_PER_HOUR ((WHEEL_CIRCUMFERENCE_MM * 3600)/(DEF_ANGLE_360_DEGREE))
#define DEF_TOTAL_TIME_STAMP_MAXIMUM 	64000	//// total time is 1000-based. event time is 1024-based. total maximum value shall less that  64,000 (0x10000 / 0x800 * 2000)

/* accelerometer sensitivity setting, default is 8G*/
//#define ACCELEROMETER_SENSITIVITY_2G_SUPPORT
//#define ACCELEROMETER_SENSITIVITY_4G_SUPPORT
#define ACCELEROMETER_SENSITIVITY_8G_SUPPORT

#ifdef ACCELEROMETER_SENSITIVITY_2G_SUPPORT
	#define ACCEL_SENSITIVITY_CONFIG 			SENSITIVITY_2G
	#define ACCEL_FULL_SCALE_REG				FULL_SCALE_2G
#elif ACCELEROMETER_SENSITIVITY_4G_SUPPORT
#define ACCEL_SENSITIVITY_CONFIG 				SENSITIVITY_4G
#define ACCEL_FULL_SCALE_REG					FULL_SCALE_4G
#else
#define ACCEL_SENSITIVITY_CONFIG 				SENSITIVITY_8G
#define ACCEL_FULL_SCALE_REG					FULL_SCALE_8G
#endif

/* FUNCTIONS */
void 		accel_init(void);
void 		accel_set_active(void);
void 		accel_set_deactive(void);
void 		accel_calibration (void);
void 		accel_task_disable_mask(t_accel_task_pending disable);
bool 		accel_task_check_enable(t_accel_task_pending check);
ret_code_t 	accel_task_enable_mask(t_accel_task_pending enable);
void 		acc_read_fifodata(void);
void 		acc_read_fifodata_datadump(void);
void 		acc_step_reset_angle(void);
ret_code_t 	accel_csc_measurement(ble_cscs_meas_t * p_measurement);

#endif /* SENSOR_ACCELEROMETER_H_ */
