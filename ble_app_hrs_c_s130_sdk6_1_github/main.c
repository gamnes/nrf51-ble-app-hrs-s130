/*
 * Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/** @example Board/nrf6310/s120/experimental/ble_app_hrs_c/main.c
 *
 * @brief This example uses ble_app_hrs_c for S120 and ble_app_hrs for S110 to show how the S130 can be used. 
 *        The example works as the following:
 *        The central part of the application can receive heart rate belt data, heart rate value, battery level, 
 *        sensor location, etc, from a peer peripheral*. 
 *        The peripheral part of the application will transmit the heart rate value that the central part of the 
 *        application has received, to a peer central.
 *        Peer peripheral --(hear rate sensor data )--> central --(heart rate value)-- peripheral --(heart rate value)
 *        --> peer central.
 *        *The central will connect to the first heart rate sensor it finds with RSSI higher than RSSI_CRITERIA
 *
 *        The example also shows transmission of data in the other direction:
 *        Peer central --(battery level)--> peripheral --(battery level)-- central
 *        --(battery level)--> peer peripheral*
 *        *The peer peripheral will receive the battery level value from the central provided that its 
 *         battery level characteristic is writable.
 * 
 *        Note: The S110 part of the application does not implement/support bonding.
 *
 * This file contains the source code for a sample heart rate collector.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_db_discovery_s130.h"
#include "softdevice_handler.h"
#include "app_util.h"
#include "app_error.h"
#include "ble_advdata_parser.h"
#include "boards.h"
#include "nrf6350.h"
#include "nrf_gpio.h"
#include "pstorage.h"
#include "device_manager_s130.h"
#include "app_trace.h"
#include "ble_hrs_c.h"
#include "ble_bas_c.h"
#include "app_util.h"

// Peripheral includes
#include "ble_bas_s130.h"
#include "ble_hrs_s130.h"
#include "ble_dis.h"
#include "ble_sensorsim.h"
#include "app_timer.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params_s130.h"
#include "ble_hci.h"
#include "nrf_delay.h"


// Peripheral settings
#define PERIPHERAL_ADV_LED_PIN_NO                   LED_0                                          /**< Is on when device is scanning. */
#define PERIPHERAL_CONNECTED_LED_PIN_NO             LED_0                                          /**< Is on when device has connected. */

#define PERIPHERAL_BONDMNGR_DELETE_BUTTON_PIN_NO   BUTTON_1

#define PERIPHERAL_DEVICE_NAME                          "S130_GG"                               /**< Name of device. Will be included in the advertising data. */
#define PERIPHERAL_MANUFACTURER_NAME                    "Gamnes"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define PERIPHERAL_APP_ADV_INTERVAL                     40                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define PERIPHERAL_APP_ADV_TIMEOUT_IN_SECONDS           0 //180                                        /**< The advertising timeout in units of seconds. */


#define PERIPHERAL_APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define PERIPHERAL_APP_TIMER_MAX_TIMERS                 5                                          /**< Maximum number of simultaneously created timers. */
#define PERIPHERAL_APP_TIMER_OP_QUEUE_SIZE              4                                          /**< Size of timer operation queues. */

#define PERIPHERAL_BATTERY_LEVEL_MEAS_INTERVAL          APP_TIMER_TICKS(2000, PERIPHERAL_APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define PERIPHERAL_MIN_BATTERY_LEVEL                    81                                         /**< Minimum simulated battery level. */
#define PERIPHERAL_MAX_BATTERY_LEVEL                    100                                        /**< Maximum simulated battery level. */
#define PERIPHERAL_BATTERY_LEVEL_INCREMENT              1                                          /**< Increment between each simulated battery level measurement. */

#define PERIPHERAL_HEART_RATE_MEAS_INTERVAL             APP_TIMER_TICKS(1000, PERIPHERAL_APP_TIMER_PRESCALER) /**< Heart rate measurement interval (ticks). */
#define PERIPHERAL_MIN_HEART_RATE                       140                                        /**< Minimum heart rate as returned by the simulated measurement function. */
#define PERIPHERAL_MAX_HEART_RATE                       300                                        /**< Maximum heart rate as returned by the simulated measurement function. */
#define PERIPHERAL_HEART_RATE_INCREMENT                 10                                         /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define PERIPHERAL_RR_INTERVAL_INTERVAL                 APP_TIMER_TICKS(300, PERIPHERAL_APP_TIMER_PRESCALER)  /**< RR interval interval (ticks). */
#define PERIPHERAL_MIN_RR_INTERVAL                      100                                        /**< Minimum RR interval as returned by the simulated measurement function. */
#define PERIPHERAL_MAX_RR_INTERVAL                      500                                        /**< Maximum RR interval as returned by the simulated measurement function. */
#define PERIPHERAL_RR_INTERVAL_INCREMENT                1                                          /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define PERIPHERAL_SENSOR_CONTACT_DETECTED_INTERVAL     APP_TIMER_TICKS(5000, PERIPHERAL_APP_TIMER_PRESCALER) /**< Sensor Contact Detected toggle interval (ticks). */

#define PERIPHERAL_MIN_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds). */
#define PERIPHERAL_MAX_CONN_INTERVAL                    MSEC_TO_UNITS(1000, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define PERIPHERAL_SLAVE_LATENCY                        0                                          /**< Slave latency. */
#define PERIPHERAL_CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)           /**< Connection supervisory timeout (4 seconds). */

#define PERIPHERAL_FIRST_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(5000, PERIPHERAL_APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define PERIPHERAL_NEXT_CONN_PARAMS_UPDATE_DELAY        APP_TIMER_TICKS(5000, PERIPHERAL_APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define PERIPHERAL_MAX_CONN_PARAMS_UPDATE_COUNT         3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define PERIPHERAL_SEC_PARAM_TIMEOUT                    30                                         /**< Timeout for Pairing Request or Security Request (in seconds). */
#define PERIPHERAL_SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define PERIPHERAL_SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define PERIPHERAL_SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define PERIPHERAL_SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define PERIPHERAL_SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define PERIPHERAL_SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//static uint16_t                              m_peripheral_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
//static ble_gap_sec_params_t                  m_peripheral_sec_params;                              /**< Security requirements for this application. */
static ble_gap_adv_params_t                  m_peripheral_adv_params;                              /**< Parameters to be passed to the stack when starting advertising. */
static ble_bas_t                             m_peripheral_bas;                                     /**< Structure used to identify the battery service. */
static ble_hrs_t                             m_peripheral_hrs;                                     /**< Structure used to identify the heart rate service. */
//static bool                                  m_peripheral_rr_interval_enabled = true;              /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

//static ble_sensorsim_cfg_t                   m_peripheral_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
//static ble_sensorsim_state_t                 m_peripheral_battery_sim_state;                       /**< Battery Level sensor simulator state. */
//static ble_sensorsim_cfg_t                   m_peripheral_heart_rate_sim_cfg;                      /**< Heart Rate sensor simulator configuration. */
//static ble_sensorsim_state_t                 m_peripheral_heart_rate_sim_state;                    /**< Heart Rate sensor simulator state. */
//static ble_sensorsim_cfg_t                   m_peripheral_rr_interval_sim_cfg;                     /**< RR Interval sensor simulator configuration. */
//static ble_sensorsim_state_t                 m_peripheral_rr_interval_sim_state;                   /**< RR Interval sensor simulator state. */

//static app_timer_id_t                        m_peripheral_battery_timer_id;                        /**< Battery timer. */
//static app_timer_id_t                        m_peripheral_heart_rate_timer_id;                     /**< Heart rate measurement timer. */
//static app_timer_id_t                        m_peripheral_rr_interval_timer_id;                    /**< RR interval timer. */
//static app_timer_id_t                        m_peripheral_sensor_contact_timer_id;                 /**< Sensor contact detected timer. */


static bool                                  m_peripheral_start_advertising = false;                        /**< Variable setting if advertising should start or not. */
static bool                                  m_central_is_scanning          = false;                        /**< Variable telling if the central is scanning or not. */



// ##### Central settings ####

#define BOND_DELETE_ALL_BUTTON_ID  BUTTON_1                           /**< Button used for deleting all bonded centrals during startup. */

#define SCAN_LED_PIN_NO                  LED_1                                          /**< Is on when device is scanning. */
#define CONNECTED_LED_PIN_NO             LED_1                                          /**< Is on when device has connected. */
#define RSSI_CRITERIA                    -50                         /**< Minimum RSSI value for peer peripheral. */        

#define APPL_LOG                         app_trace_log                                  /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */

#define SEC_PARAM_BOND             0                                   /**< Perform bonding. */
#define SEC_PARAM_MITM             0                                 /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES  BLE_GAP_IO_CAPS_NONE               /**< No I/O capabilities. */
#define SEC_PARAM_OOB              0                                  /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE     7                                  /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE     16                                 /**< Maximum encryption key size. */

#define SCAN_INTERVAL              0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                0x0050                             /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONNECTION_INTERVAL    MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   /**< Determines maximum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL    MSEC_TO_UNITS(30, UNIT_1_25_MS)    /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY              0                                  /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 millisecond. */

#define TARGET_UUID                0x180D                             /**< Target device name that application is looking for. */
#define MAX_PEER_COUNT             DEVICE_MANAGER_MAX_CONNECTIONS     /**< Maximum number of peer's application intends to manage. */
#define UUID16_SIZE                2                                  /**< Size of 16 bit UUID */

/**@breif Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST,SRC)                                                                  \
        do                                                                                       \
        {                                                                                        \
            (*(DST)) = (SRC)[1];                                                                 \
            (*(DST)) <<= 8;                                                                      \
            (*(DST)) |= (SRC)[0];                                                                \
        } while(0)

/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct
{
    uint8_t     * p_data;                                         /**< Pointer to data. */
    uint16_t      data_len;                                       /**< Length of data. */
}data_t;

typedef enum
{
    BLE_NO_SCAN,                                                  /**< No advertising running. */
    BLE_WHITELIST_SCAN,                                           /**< Advertising with whitelist. */
    BLE_FAST_SCAN,                                                /**< Fast advertising running. */
} ble_advertising_mode_t;

//static ble_db_discovery_t           m_ble_db_discovery;                  /**< Structure used to identify the DB Discovery module. */
//static ble_hrs_c_t                  m_ble_hrs_c;                         /**< Structure used to identify the heart rate client module. */
//static ble_bas_c_t                  m_ble_bas_c;                         /**< Structure used to identify the Battery Service client module. */
static ble_gap_scan_params_t        m_scan_param;                        /**< Scan parameters requested for scanning and connection. */
static dm_application_instance_t    m_dm_app_id;                         /**< Application identifier. */
//static dm_handle_t                  m_dm_device_handle;                  /**< Device Identifier identifier. */
//static uint8_t                      m_peer_count = 0;                    /**< Number of peer's connected. */
static uint8_t                      m_scan_mode;                         /**< Scan mode used by application. */

//static bool                         m_memory_access_in_progress = false; /**< Flag to keep track of ongoing operations on persistent memory. */


static void scan_start(void);

#define APPL_LOG                        app_trace_log             /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */

// WARNING: The following macro MUST be un-defined (by commenting out the definition) if the user
// does not have a nRF6350 Display unit. If this is not done, the application will not work.
//#define APPL_LCD_PRINT_ENABLE                                     /**< In case you do not have a functional display unit, disable this flag and observe trace on UART. */

#ifdef APPL_LCD_PRINT_ENABLE

#define APPL_LCD_CLEAR                  nrf6350_lcd_clear         /**< Macro to clear the LCD display.*/
#define APPL_LCD_WRITE                  nrf6350_lcd_write_string  /**< Macro to write a string to the LCD display.*/

#else // APPL_LCD_PRINT_ENABLE

#define APPL_LCD_WRITE(...)             true                      /**< Macro to clear the LCD display defined to do nothing when @ref APPL_LCD_PRINT_ENABLE is not defined.*/
#define APPL_LCD_CLEAR(...)             true                      /**< Macro to write a string to the LCD display defined to do nothing when @ref APPL_LCD_PRINT_ENABLE is not defined.*/

#endif // APPL_LCD_PRINT_ENABLE


/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    APPL_LOG("[APPL]: ASSERT: %s, %d, error 0x%08x\r\n", p_file_name, line_num, error_code);
    nrf_gpio_pin_set(LED_0);
    nrf_gpio_pin_set(LED_1);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    while(line_num)
    {
        (void) p_file_name;
    }
    //NVIC_SystemReset();
}


/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for starting advertising.
 */
static void peripheral_advertising_start(void)
{   
    uint32_t err_code;
    
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_peripheral_adv_params, 0, sizeof(m_peripheral_adv_params));

    m_peripheral_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_peripheral_adv_params.p_peer_addr = NULL;                           // Undirected advertisement.
    m_peripheral_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_peripheral_adv_params.interval    = PERIPHERAL_APP_ADV_INTERVAL;
    m_peripheral_adv_params.timeout     = PERIPHERAL_APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&m_peripheral_adv_params);
    APP_ERROR_CHECK(err_code);
    
    m_peripheral_start_advertising = false;
    nrf_gpio_pin_set(PERIPHERAL_ADV_LED_PIN_NO);
}



/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *  been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt) {
    //on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt) {

}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the LEDs initialization. 
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(LED_0);
    nrf_gpio_cfg_output(LED_1);
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    // Set Wakeup and Bonds Delete buttons as wakeup sources.
    nrf_gpio_cfg_sense_input(BOND_DELETE_ALL_BUTTON_ID,
                             BUTTON_PULL,
                             NRF_GPIO_PIN_SENSE_LOW);
}


/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init();
    APP_ERROR_CHECK(err_code);
}


/**@breif Function to start scanning.
 */
static void scan_start(void)
{
    ble_gap_whitelist_t   whitelist;
    ble_gap_addr_t        * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    ble_gap_irk_t         * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];
    uint32_t              err_code;
    uint32_t              count;

    // Verify if there is any flash access pending, if yes delay starting scanning until 
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);
    
    if (count != 0)
    {
        //m_memory_access_in_progress = true;
        return;
    }
    
    // Initialize whitelist parameters.
    whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
    whitelist.irk_count  = 0;
    whitelist.pp_addrs   = p_whitelist_addr;
    whitelist.pp_irks    = p_whitelist_irk;

    // Request creating of whitelist.
    err_code = dm_whitelist_create(&m_dm_app_id,&whitelist);
    APP_ERROR_CHECK(err_code);

    if (((whitelist.addr_count == 0) && (whitelist.irk_count == 0)) ||
         (m_scan_mode != BLE_WHITELIST_SCAN))
    {
        // No devices in whitelist, hence non selective performed.
        m_scan_param.active       = 0;            // Active scanning set.
        m_scan_param.selective    = 0;            // Selective scanning not set.
        m_scan_param.interval     = SCAN_INTERVAL;// Scan interval.
        m_scan_param.window       = SCAN_WINDOW;  // Scan window.
        m_scan_param.p_whitelist  = NULL;         // No whitelist provided.
        m_scan_param.timeout      = 0x0000;       // No timeout.
    }
    else
    {
        // Selective scanning based on whitelist first.
        m_scan_param.active       = 0;            // Active scanning set.
        m_scan_param.selective    = 1;            // Selective scanning not set.
        m_scan_param.interval     = SCAN_INTERVAL;// Scan interval.
        m_scan_param.window       = SCAN_WINDOW;  // Scan window.
        m_scan_param.p_whitelist  = &whitelist;   // Provide whitelist.
        m_scan_param.timeout      = 0x001E;       // 30 seconds timeout.

        // Set whitelist scanning state.
        m_scan_mode = BLE_WHITELIST_SCAN;
    }

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    APPL_LOG("[APPL]: Scan start error code: 0x%x \r\n", err_code);
    APP_ERROR_CHECK(err_code);
    
    bool lcd_write_status = APPL_LCD_WRITE("Scanning", 8, LCD_UPPER_LINE, 0);
    if (!lcd_write_status)
    {
        APPL_LOG("[APPL]: LCD Write failed!\r\n");
    }
    m_central_is_scanning = true;
    nrf_gpio_pin_set(SCAN_LED_PIN_NO);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void peripheral_gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)PERIPHERAL_DEVICE_NAME,
                                          strlen(PERIPHERAL_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
} 

static uint32_t do_temperature_measurement(void) {
    uint32_t retval = 0xFE000E74;

    if (nrf_gpio_pin_read(BUTTON_1)) {
        retval = 0xFF000E74;
    }
    
    return retval;
    
    /*
    
    int32_t temp = 0x00000136; // 97 is 37 deg(?) 
    uint32_t err_code;
    err_code = sd_temp_get(&temp);
    APP_ERROR_CHECK(err_code);
    //if (nrf_gpio_pin_read(BUTTON_1)) {
    if (nrf_gpio_pin_read(INPUT_PIN_SENSOR)) {
        temp = 0x00000065; // 50 is 10 deg
    }
    temp = (temp / 4) * 100;
    int8_t exponent = -2;
    return ((exponent & 0xFF) << 24) | (temp & 0x00FFFFFF);
    */
}

/* This function measures the battery voltage using the bandgap as a reference.
 * 3.6 V will return 100 %, so depending on battery voltage, it might need scaling. */
static uint32_t do_battery_measurement(void) {
    uint8_t retval = 0xAA;
    
    if (nrf_gpio_pin_read(BUTTON_1)) {
    //if (nrf_gpio_pin_read(INPUT_PIN_SENSOR)) {
        retval = 0x24;
    }

    return retval;
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void peripheral_advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_service_data_t service_data[2];
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    uint32_t temperature;
    temperature = do_temperature_measurement();
    service_data[0].service_uuid = BLE_UUID_HEALTH_THERMOMETER_SERVICE;
    service_data[0].data.p_data = (uint8_t *) &temperature;
    service_data[0].data.size = sizeof(temperature);
    
    uint8_t battery = do_battery_measurement();
    service_data[1].service_uuid = BLE_UUID_BATTERY_SERVICE;
    service_data[1].data.p_data = &battery;
    service_data[1].data.size = sizeof(battery);

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.p_service_data_array = service_data;
    advdata.service_data_count = 2;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void peripheral_services_init(void)
{
    uint32_t       err_code;
    ble_hrs_init_t hrs_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    uint8_t        body_sensor_location;

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_peripheral_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm); // todo : battery level service writable
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_peripheral_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)PERIPHERAL_MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}



int main(void)
{   
    // Initialization of various modules.
    uint32_t err_code;
    
    app_trace_init();
    leds_init();
    buttons_init();
    ble_stack_init();
    db_discovery_init();
    
    
    peripheral_gap_params_init();
    peripheral_advertising_init();
    peripheral_services_init();
    
    peripheral_advertising_start();

    // Start scanning for peripherals and initiate connection
    // with devices that advertise Heart Rate UUID.
    scan_start();
    m_peripheral_start_advertising = false;
    for (;;)
    {
        power_manage();
        
        // Start advertising
        if( m_peripheral_start_advertising == true)
        {
            if (m_central_is_scanning == true)
            {
                err_code = sd_ble_gap_scan_stop();
                APP_ERROR_CHECK(err_code);
                nrf_gpio_pin_clear(SCAN_LED_PIN_NO);
            }
                
            peripheral_advertising_start();
            if (m_central_is_scanning == true)
            {
                scan_start(); // only if the device was already scanning.
            }
 
        }
    }
}


