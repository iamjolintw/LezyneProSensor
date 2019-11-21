/*
 * 	sys_conf.h
 *
 *  Created on: June 30, 2019
 *  Author: Lezyne
 */

#ifndef SYS_CONF_H
#define SYS_CONF_H

#ifndef CONFIG_NFCT_PINS_AS_GPIOS
	#define CONFIG_NFCT_PINS_AS_GPIOS
#endif

//#define ACCELEROMETER_DUMP_FIFO   	// test purpose: dump the G-force of x,y,z through OTA
//#define CSCS_MOCK_ENABLE 				// enable cscs simulator
#define ACC_ST16G_ENABLE				// if defined, this is new hardware with ST 16G accelerometer, or, this is old hardware with NXP 8G accelerometer

/*--------------------------------------------------------------------------------*/
/********************************* BLE - DEFINITION *******************************/
/*--------------------------------------------------------------------------------*/
#define DEVICE_NAME_WITH_SERIAL_NO													/**< switch for enable/disable combination serial number with device name. */
#define DEVICE_NAME                     "LezyneCAD"	                            	/**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "LEZYNE"                       				/**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM              			"ProCAD-1"									/**< Model Name. Will be passed to Device Information Service. */
#define HW_REV_NUM						"0.1"										/**< Hardware Version. Will be passed to Device Information Service. */
#define FW_REV_NUM						"0.09.2"										/**< Firmware Version. Will be passed to Device Information Service. */

/* According to iOS "Accessory-Design-Guideines.pdf" Version:
 * 23.6 Connection Parameters - also refer to Bluetooth 4.0 specification.
 * Slave Latency <= 30
 * 2 seconds <= connSupervisionTimeout <= 6 seconds
 * Interval Min modulo 15ms == 0
 * Interval Min >= 15 ms
 * One of the following: (1) (Interval Min + 15 ms) <= Interval Max (2) Interval Min == Interval Max == 15 ms
 * Interval Max * (Slave Latency + 1) <= 2 seconds
 * Intreval Max * (Slave Latency + 1) * 3 < connSupervisionTimeout
 * Note that if an accessory requests Interval Min == Interval Max == 15ms, some devices will scale the interval to 30ms to balance power and performance constraints.
 *
 * Note: 20191111 - broaden the connection interval for GPS computer (800 ~ 1050).
 */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

//#define MIN_CONNECTION_INTERVAL        	15											/**< Minimum acceptable connection interval (15 millisecond). */
//#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(MIN_CONNECTION_INTERVAL,		UNIT_1_25_MS)        /**< Minimum acceptable connection interval. */
//#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(MIN_CONNECTION_INTERVAL+15,	UNIT_1_25_MS)        /**< Maximum acceptable connection interval. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(210,	UNIT_1_25_MS)        	/**< Minimum acceptable connection interval. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1050,	UNIT_1_25_MS)        	/**< Maximum acceptable connection interval. */

#define APP_ADV_INTERVAL         		300                                 		/**< The advertising interval (in units of 0.625 ms. This value corresponds to 100 ms). */
#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

/* Speed and cadence measurement interval (milliseconds). */
#define SPEED_AND_CADENCE_MEAS_INTERVAL 800

#define WHEEL_CIRCUMFERENCE_MM          2100                                        /**< Simulated wheel circumference in millimeters. */

#define SPEED_SENSOR_LOCATION			BLE_SENSOR_LOCATION_RIGHT_CRANK
																					/**< the location of sensor. options:
																						BLE_SENSOR_LOCATION_FRONT_WHEEL,
																						BLE_SENSOR_LOCATION_LEFT_CRANK,
																						BLE_SENSOR_LOCATION_RIGHT_CRANK,
																						BLE_SENSOR_LOCATION_LEFT_PEDAL,
																						BLE_SENSOR_LOCATION_RIGHT_PEDAL,
																						BLE_SENSOR_LOCATION_FRONT_HUB,
																						BLE_SENSOR_LOCATION_REAR_DROPOUT,
																						BLE_SENSOR_LOCATION_CHAINSTAY,
																						BLE_SENSOR_LOCATION_REAR_WHEEL,
																						BLE_SENSOR_LOCATION_REAR_HUB. */

#define BLE_TX_POWER					(0)											/**< TX power. Default: 0 dB. Value: 4,0,-4,-8,-12,-16,-20,-40. */
#define GLED_TURN_OFF_TIMER_TICKS		(5000)										/**< Timer for turning off the R-LED (milliseconds). */

/*--------------------------------------------------------------------------------*/
/***************************** PERIPHERAL - DEFINITION ****************************/
/*--------------------------------------------------------------------------------*/
#define ACC_TWI_INSTANCE_ID			1
#ifndef ACC_ST16G_ENABLE
#define ACC_I2C_SCL_PIN				(7)
#define ACC_I2C_SDA_PIN				(9)
#define ACC_INT1_PIN				(8)
#else
#define ACC_I2C_OPTION				(6)
#define ACC_I2C_SDA_PIN				(7)
#define ACC_I2C_SCL_PIN				(8)
#define ACC_INT1_PIN				(9)
#endif
#define ACC_INT2_PIN				(10)

#define ACC_LEDG_PIN				(16)
#define ACC_LEDR_PIN				(17)

#endif // SYS_CONF_H
