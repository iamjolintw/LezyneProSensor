/*
 * 	ble_core.h
 *
 *  Created on: June 30, 2019
 *  Author: Lezyne
 */

#ifndef BLE_CORE_H
#define BLE_CORE_H

#include <stdbool.h>
#include "ble_cscs.h"

BLE_CSCS_DEF(m_cscs);                                                               /**< Cycling speed and cadence service instance. */

/* FUNCTION */
void ble_init(void);
void ble_advertising_entry(void);
void csc_meas_handler(ble_cscs_meas_t cscs_measurement);
bool ble_connection_status(void);

#endif // BLE_CORE_H
