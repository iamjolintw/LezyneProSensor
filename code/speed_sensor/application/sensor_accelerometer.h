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
	eSTEP_VALLEY_DETECT,
	eSTEP_STEP_DETECT,

}step_detect_t;

#if 0
typedef struct {
	uint32_t 		sample;
    float 			raw[5];
    uint32_t 		rawidx;
    float 			afilt;
    float 			min, tempmin;
    float 			max, tempmax;
    float 			trs, trsp, trsn;
    bool 			startpeak, startvalley, checkstep;
    float 			old, peakmax, peakidx, peakidxprev, valleymax, valleyidx;
    step_detect_t 	step_state;
    uint32_t 		step_wait_count;
    uint32_t 		temp_step;
} ;
#endif

#define DEF_STEP_WAIT_TIMEOUT   200 //200
#define DEF_RESET_VAL_COUNT     50
#define DEF_SAMPLE_TO_AVG       1
#define DEF_SAMPLE_1ST       	0
#define DEF_SAMPLE_2ND       	1
#define DEF_SAMPLE_3RD       	2
#define DEF_SAMPLE_4TH       	3
#define DEF_SAMPLE_5TH       	4

#define DEF_MAX_ANGLE_WINDOW    75.0f  // 75 degree equal 75Km/hr in 50 hz ODR


typedef struct {
	uint32_t 		ui32_sample;
    float 			f_raw_x[DEF_SAMPLE_TO_AVG];
    float 			f_raw_y[DEF_SAMPLE_TO_AVG];
    uint32_t 		ui32_rawidx;
    float 			f_average_ang;
    float 			f_min, f_tempmin;
    float 			f_max, f_tempmax;
    float 			f_trs, f_trsp, f_trsn;
} t_angle_step;

extern uint32_t steps;

/* FUNCTIONS */
void 	accel_init(void);
void 	accel_set_active(void);
void 	accel_weak_up(void);
void 	accel_standby(void);
void 	accel_calibration (void);
void 	accel_csc_measurement(ble_cscs_meas_t * p_measurement);

#endif /* SENSOR_ACCELEROMETER_H_ */
