/*
 * 	sys_conf.h
 *
 *  Created on: June 30, 2019
 *  Author: Lezyne
 */

#ifndef SYS_CONF_H
#define SYS_CONF_H

//#define CSCS_MOCK_ENABLE // enable cscs simulator

/*--------------------------------------------------------------------------------*/
/********************************* BLE - DEFINITION *******************************/
/*--------------------------------------------------------------------------------*/
#define DEVICE_NAME_WITH_SERIAL_NO													/**< switch for enable/disable combination serial number with device name. */
#define DEVICE_NAME                     "LezyneSPD"	                            	/**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "LEZYNE"                       				/**< Manufacturer. Will be passed to Device Information Service. */

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
 */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define MIN_CONNECTION_INTERVAL        	15											/**< Minimum acceptable connection interval (15 millisecond). */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(MIN_CONNECTION_INTERVAL,		UNIT_1_25_MS)        /**< Minimum acceptable connection interval. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(MIN_CONNECTION_INTERVAL+15,	UNIT_1_25_MS)        /**< Maximum acceptable connection interval. */

//#define APP_ADV_INTERVAL              40                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_INTERVAL                (100/0.625)                                 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 100 ms). */
#define APP_ADV_DURATION                6000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define SPEED_SENSOR_LOCATION			BLE_SENSOR_LOCATION_FRONT_WHEEL				/**< the location of sensor. options:
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

/*--------------------------------------------------------------------------------*/
/***************************** PERIPHERAL - DEFINITION ****************************/
/*--------------------------------------------------------------------------------*/
#define MMA8652_TWI_INSTANCE_ID			1
#define MMA8652_I2C_SCL_PIN				(7)
#define MMA8652_I2C_SDA_PIN				(9)
#define MMA8652_INT1_PIN				(8)
#define MMA8652_INT2_PIN				(10)

#endif // SYS_CONF_H
