/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_csc_main main.c
 * @{
 * @ingroup ble_sdk_app_csc
 * @brief Cycling Speed and Cadence Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Cycling Speed and Cadence
 * Service.
 * It also includes the sample code for Battery and Device Information services.
 * This application uses the @ref srvlib_conn_params module.
 *
 * This application implements supports for both Wheel revolution Data and Crank Revolution Data.
 * In addition, this application also has support for all 'Speed and Cadence Control Point'.
 */
#include <stdint.h>
#include <string.h>

#include "sys_conf.h"
#include "ble_core.h"
#include "sensor_accelerometer.h"

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_cscs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"

// buttonless DFU service
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "nrf_bootloader_info.h"
#include "ble_dfu.h"
#include "fds.h"
#include "nrf_power.h"

// log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define LINK_TOTAL                      NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + \
                                        NRF_SDH_BLE_CENTRAL_LINK_COUNT

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)                       /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define KPH_TO_MM_PER_SEC               278                                         /**< Constant to convert kilometers per hour into millimeters per second. */

#define MIN_SPEED_KPH                   10                                          /**< Minimum speed in kilometers per hour for use in the simulated measurement function. */
#define MAX_SPEED_KPH                   40                                          /**< Maximum speed in kilometers per hour for use in the simulated measurement function. */
#define SPEED_KPH_INCREMENT             1                                           /**< Value by which speed is incremented/decremented for each call to the simulated measurement function. */

#define DEGREES_PER_REVOLUTION          360                                         /**< Constant used in simulation for calculating crank speed. */
#define RPM_TO_DEGREES_PER_SEC          6                                           /**< Constant to convert revolutions per minute into degrees per second. */

#define MIN_CRANK_RPM                   20                                          /**< Minimum cadence in RPM for use in the simulated measurement function. */
#define MAX_CRANK_RPM                   110                                         /**< Maximum cadence in RPM for use in the simulated measurement function. */
#define CRANK_RPM_INCREMENT             3                                           /**< Value by which cadence is incremented/decremented in the simulated measurement function. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

BLE_CSCS_DEF(m_cscs);                                                               /**< Cycling speed and cadence service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);                             	/**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t          m_conn_handle = BLE_CONN_HANDLE_INVALID;                   /**< Handle of the current connection. */
static bool 			start_flag = false;
static bool				complete_flag = true;										/**< Indicator for transmit complete. */
static ble_sensor_location_t supported_locations[] =                                /**< Supported location for the sensor location. */
{
    BLE_SENSOR_LOCATION_FRONT_WHEEL,
    BLE_SENSOR_LOCATION_LEFT_CRANK,
    BLE_SENSOR_LOCATION_RIGHT_CRANK,
    BLE_SENSOR_LOCATION_LEFT_PEDAL,
    BLE_SENSOR_LOCATION_RIGHT_PEDAL,
    BLE_SENSOR_LOCATION_FRONT_HUB,
    BLE_SENSOR_LOCATION_REAR_DROPOUT,
    BLE_SENSOR_LOCATION_CHAINSTAY,
    BLE_SENSOR_LOCATION_REAR_WHEEL,
    BLE_SENSOR_LOCATION_REAR_HUB
};

static ble_uuid_t m_adv_uuids[] =                                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_CYCLING_SPEED_AND_CADENCE,  BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

static void advertising_start(void);

#ifdef CSCS_MOCK_ENABLE
#include "sensorsim.h"

static sensorsim_cfg_t   m_speed_kph_sim_cfg;                                       /**< Speed simulator configuration. */
static sensorsim_state_t m_speed_kph_sim_state;                                     /**< Speed simulator state. */
static sensorsim_cfg_t   m_crank_rpm_sim_cfg;                                       /**< Crank simulator configuration. */
static sensorsim_state_t m_crank_rpm_sim_state;                                     /**< Crank simulator state. */
static uint32_t m_cumulative_wheel_revs;                                            /**< Cumulative wheel revolutions. */
static bool     m_auto_calibration_in_progress;                                     /**< Set when an autocalibration is in progress. */

#define WHEEL_CIRCUMFERENCE_MM          2100                                        /**< Simulated wheel circumference in millimeters. */
#define KPH_TO_MM_PER_SEC               278                                         /**< Constant to convert kilometers per hour into millimeters per second. */

#define MIN_SPEED_KPH                   10                                          /**< Minimum speed in kilometers per hour for use in the simulated measurement function. */
#define MAX_SPEED_KPH                   40                                          /**< Maximum speed in kilometers per hour for use in the simulated measurement function. */
#define SPEED_KPH_INCREMENT             1                                           /**< Value by which speed is incremented/decremented for each call to the simulated measurement function. */

#define DEGREES_PER_REVOLUTION          360                                         /**< Constant used in simulation for calculating crank speed. */
#define RPM_TO_DEGREES_PER_SEC          6                                           /**< Constant to convert revolutions per minute into degrees per second. */

#define MIN_CRANK_RPM                   20                                          /**< Minimum cadence in RPM for use in the simulated measurement function. */
#define MAX_CRANK_RPM                   110                                         /**< Maximum cadence in RPM for use in the simulated measurement function. */
#define CRANK_RPM_INCREMENT             3                                           /**< Value by which cadence is incremented/decremented in the simulated measurement function. */

#endif


/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}

static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

/**@brief Function for handling Speed and Cadence Control point events
 *
 * @details Function for handling Speed and Cadence Control point events.
 *          This function parses the event and in case the "set cumulative value" event is received,
 *          sets the wheel cumulative value to the received value.
 */
ble_scpt_response_t sc_ctrlpt_event_handler(ble_sc_ctrlpt_t     * p_sc_ctrlpt,
                                            ble_sc_ctrlpt_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_SC_CTRLPT_EVT_SET_CUMUL_VALUE:
#ifdef CSCS_MOCK_ENABLE
        	m_cumulative_wheel_revs = p_evt->params.cumulative_value;
#endif
            break;

        case BLE_SC_CTRLPT_EVT_START_CALIBRATION:
#ifdef CSCS_MOCK_ENABLE
        	m_auto_calibration_in_progress = true;
#endif
            break;

        default:
            // No implementation needed.
            break;
    }
    return (BLE_SCPT_SUCCESS);
}

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start();
            break;

        default:
            break;
    }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

#ifdef DEVICE_NAME_WITH_SERIAL_NO
    /* The maximum length of BLE device name is 170 characters, however, the advertisement packet can only contain user payload of up to 29 bytes only. */
	uint16_t uLower = (uint16_t)(NRF_FICR->DEVICEID[0]);
    char strDeviceName[20] = {0};
	sprintf(&strDeviceName[0],"%s-%02X%02X",&DEVICE_NAME[0],(uint8_t)(uLower & 0xFF),(uint8_t)(uLower >> 8));
	err_code = sd_ble_gap_device_name_set(&sec_mode,
                                      (const uint8_t *) strDeviceName,
                                      strlen(strDeviceName));
	APP_ERROR_CHECK(err_code);
#else
	err_code = sd_ble_gap_device_name_set(&sec_mode,
										  (const uint8_t *)DEVICE_NAME,
										  strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);
#endif

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Cycling Speed and Cadence, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t              		err_code;
    ble_cscs_init_t       		cscs_init;
    ble_dis_init_t        		dis_init;
    ble_sensor_location_t 		sensor_location;
    nrf_ble_qwr_init_t    		qwr_init = {0};
    ble_dfu_buttonless_init_t 	dfus_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;
    for (uint32_t i = 0; i < LINK_TOTAL; i++)
    {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }

    // Initialize DFU service.
    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Cycling Speed and Cadence Service.
    memset(&cscs_init, 0, sizeof(cscs_init));

    cscs_init.evt_handler = NULL;
    cscs_init.feature     = BLE_CSCS_FEATURE_WHEEL_REV_BIT | BLE_CSCS_FEATURE_CRANK_REV_BIT |
                            BLE_CSCS_FEATURE_MULTIPLE_SENSORS_BIT;

    // Here the sec level for the Cycling Speed and Cadence Service can be changed/increased.
    cscs_init.csc_meas_cccd_wr_sec  = SEC_OPEN;
    cscs_init.csc_feature_rd_sec    = SEC_OPEN;
    cscs_init.csc_location_rd_sec   = SEC_OPEN;
    cscs_init.sc_ctrlpt_cccd_wr_sec = SEC_OPEN;
    cscs_init.sc_ctrlpt_wr_sec      = SEC_OPEN;

    cscs_init.ctrplt_supported_functions = BLE_SRV_SC_CTRLPT_CUM_VAL_OP_SUPPORTED
                                           | BLE_SRV_SC_CTRLPT_SENSOR_LOCATIONS_OP_SUPPORTED
                                           | BLE_SRV_SC_CTRLPT_START_CALIB_OP_SUPPORTED;
    cscs_init.ctrlpt_evt_handler            = sc_ctrlpt_event_handler;
    cscs_init.list_supported_locations      = supported_locations;
    cscs_init.size_list_supported_locations = sizeof(supported_locations) /
                                              sizeof(ble_sensor_location_t);

    sensor_location           = SPEED_SENSOR_LOCATION;                 // initializes the sensor location to add the sensor location characteristic.
    cscs_init.sensor_location = &sensor_location;

    err_code = ble_cscs_init(&m_cscs, &cscs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // init connection state
    ble_conn_state_init();
}

/**@brief Function for handling the Connection Parameter events.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail configuration parameter, but instead we use the
 *                event handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        if(err_code)
        	NRF_LOG_ERROR("sd_ble_gap_disconnect error");
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t connection_params_init;

    memset(&connection_params_init, 0, sizeof(connection_params_init));

    connection_params_init.p_conn_params                  = NULL;
    connection_params_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    connection_params_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    connection_params_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    connection_params_init.start_on_notify_cccd_handle    = m_cscs.meas_handles.cccd_handle;
    connection_params_init.disconnect_on_fail             = true;
    connection_params_init.evt_handler                    = on_conn_params_evt;
    connection_params_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&connection_params_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
	// deactivate accelerometer
#ifndef CSCS_MOCK_ENABLE
	accel_set_deactive();
#endif

    ret_code_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("\n\n>>>>>>>>> sleep_mode_enter go! <<<<<<<<<<\n\n");
	nrf_delay_ms(1000);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
        {
            uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.
            if (periph_link_cnt)
            {
            	NRF_LOG_INFO("Idle - re-advertising");
            	advertising_start();
            }
            else
            {
#if 0
            	NRF_LOG_INFO("Idle - re-advertising");
            	advertising_start();
#else
            	sleep_mode_enter();
#endif
            }
        }
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Connected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void on_connected(const ble_gap_evt_t * const p_gap_evt)
{
    ret_code_t  err_code;
    uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    NRF_LOG_INFO("Connection with link 0x%x established.", p_gap_evt->conn_handle);

    // Assign connection handle to available instance of QWR module.
    for (uint8_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
    }

    if (periph_link_cnt != NRF_SDH_BLE_PERIPHERAL_LINK_COUNT)
    {
        // Continue advertising. More connections can be established because the maximum link count has not been reached.
        advertising_start();
    }
}


/**@brief Function for handling the Disconnected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void on_disconnected(ble_gap_evt_t const * const p_gap_evt)
{
    uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
                 p_gap_evt->conn_handle,
                 p_gap_evt->params.disconnected.reason);

    if (periph_link_cnt == (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - 1))
    {
        // Advertising is not running when all connections are taken, and must therefore be started.
        advertising_start();
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connected(&p_ble_evt->evt.gap_evt);
        	err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        	APP_ERROR_CHECK(err_code);
#ifndef CSCS_MOCK_ENABLE
            /* activate accelerometer */
            accel_set_active();
#endif
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(&p_ble_evt->evt.gap_evt);
#ifndef CSCS_MOCK_ENABLE
            /* deactivate accelerometer */
            accel_set_deactive();
#endif
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;


        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gap_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_WRITE:
        	start_flag = true;
        	break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        	complete_flag = true;
        	break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             	err_code;
    ble_advertising_init_t 	init;
    ble_gap_addr_t			m_mac_addr;
    uint8_t const 			lezyne_mac_address[6] = {0xB4, 0x37, 0xD1, 0x08, 0x00, 0x00};

    // set MAC address
    err_code = sd_ble_gap_addr_get(&m_mac_addr);
    APP_ERROR_CHECK(err_code);

	for (uint8_t x=0 ; x<4 ; x++)
		m_mac_addr.addr[5-x] = lezyne_mac_address[x];

	uint16_t uLower = (uint16_t)(NRF_FICR->DEVICEID[0]);
	m_mac_addr.addr[1] 	= (uint8_t)(uLower & 0xFF);
	m_mac_addr.addr[0]	= (uint8_t)(uLower >> 8);

    m_mac_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;

    err_code = sd_ble_gap_addr_set(&m_mac_addr);
    APP_ERROR_CHECK(err_code);

    // Build and set advertising data.
    memset(&init, 0, sizeof(init));

    // Set manufacturing data
    ble_advdata_manuf_data_t			manuf_data;
    uint8_t data[]						= MANUFACTURER_VERSION;
    manuf_data.company_identifier		= 0x00;
    manuf_data.data.p_data				= data;
    manuf_data.data.size				= 2;//sizeof(data);
    init.advdata.p_manuf_specific_data	= &manuf_data;

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = false;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    advertising_config_get(&init.config);

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function to return the status of connection.
 */
bool ble_connection_status(void)
{
    for (uint8_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle != BLE_CONN_HANDLE_INVALID)
        {
        	return true;
        }
    }
	return false;
}

/**@brief Function for BLE main entry.
 */
void ble_init(void)
{
    ret_code_t err_code;

    // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);

    timers_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
#ifdef CSCS_MOCK_ENABLE
	sensor_simulator_init();
#endif
    conn_params_init();
    peer_manager_init();

    // Set TX power.
    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN,0,BLE_TX_POWER);
    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,0,BLE_TX_POWER);
}

/**@brief Function for BLE main entry.
 */
void ble_advertising_entry(void)
{
	// Start execution.
	NRF_LOG_INFO("ble advertising.");
	advertising_start();
}


#ifdef CSCS_MOCK_ENABLE
/**@brief Function for populating simulated cycling speed and cadence measurements.
 */
void csc_sim_measurement(ble_cscs_meas_t * p_measurement)
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

/**@brief Function for initializing the sensor simulators.
 */
void sensor_simulator_init(void)
{
    m_speed_kph_sim_cfg.min          = MIN_SPEED_KPH;
    m_speed_kph_sim_cfg.max          = MAX_SPEED_KPH;
    m_speed_kph_sim_cfg.incr         = SPEED_KPH_INCREMENT;
    m_speed_kph_sim_cfg.start_at_max = false;

    sensorsim_init(&m_speed_kph_sim_state, &m_speed_kph_sim_cfg);

    m_crank_rpm_sim_cfg.min          = MIN_CRANK_RPM;
    m_crank_rpm_sim_cfg.max          = MAX_CRANK_RPM;
    m_crank_rpm_sim_cfg.incr         = CRANK_RPM_INCREMENT;
    m_crank_rpm_sim_cfg.start_at_max = false;

    sensorsim_init(&m_crank_rpm_sim_state, &m_crank_rpm_sim_cfg);

    m_cumulative_wheel_revs        = 0;
    m_auto_calibration_in_progress = false;
}
#endif // CSCS_MOCK_ENABLE


/**@brief Function accel_csc_meas_timeout_handler.
 */
void accel_csc_meas_timeout_handler(void * p_context)
{
    uint32_t        err_code;
    ble_cscs_meas_t cscs_measurement;

    if (ble_connection_status())
    {
		UNUSED_PARAMETER(p_context);
#ifdef CSCS_MOCK_ENABLE
		csc_sim_measurement(&cscs_measurement);
#else
		err_code = accel_csc_measurement(&cscs_measurement);
		if( err_code == NRF_SUCCESS)
#endif
		{
			ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_periph_handles();
			for (uint8_t i = 0; i < conn_handles.len; i++)
			{
				m_cscs.conn_handle = conn_handles.conn_handles[i];
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
		}
    }
}

/**@brief Function accel_csc_meas_timeout_handler.
 */
uint32_t accel_csc_meas_timeout_handler2(ble_cscs_meas_t p_context)
{
    uint32_t        err_code = NRF_SUCCESS;

    if (ble_connection_status() && start_flag)
    {
    	ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_periph_handles();
 		while(!complete_flag);
 		complete_flag = false;
    	for (uint8_t i = 0; i < conn_handles.len; i++)
    	{
   			m_cscs.conn_handle = conn_handles.conn_handles[i];
    		err_code = ble_cscs_measurement_send(&m_cscs, &p_context);
			if ((err_code != NRF_SUCCESS) &&
					(err_code != NRF_ERROR_INVALID_STATE) &&
					(err_code != NRF_ERROR_RESOURCES) &&
					(err_code != NRF_ERROR_BUSY) &&
					(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
			)
			{
				APP_ERROR_HANDLER(err_code);
				//NRF_LOG_INFO("accel_csc_meas_timeout_handler2: error: %x", err_code);
    		}
    	}
    }
    return err_code;
}
/**
 * @}
 */

