/*
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"

#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "counter.h"
#include "general.h"
#include "hb_ui_service.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"


#define LED_ON                          0
#define LED_OFF                         1
#define ON                              0
#define OFF                             1
#define MANUAL                          0
#define AUTO                            1

//************************************************
// define the command values to be sent to Tilter
#define TILT_UP                         1
#define TILT_DN                         2
#define TILT_STOP                       3
#define STORE_CAL_ANGLE                 10
#define DRIVE_SADDLE_TO_CAL_ANGLE       11
#define AUTO_MODE                       12

//************************************************
// define the buttons and led for Nordic DK 
#define LED_GRN                         NRF_GPIO_PIN_MAP(0,17)                 // all purpose led
#define LED_RED                         NRF_GPIO_PIN_MAP(0,18)                 // all purpose led
#define TILT_UP_BTN                     NRF_GPIO_PIN_MAP(0,13)                 // up
#define TILT_DN_BTN                     NRF_GPIO_PIN_MAP(0,14)                 // down
#define CAL_BTN                         NRF_GPIO_PIN_MAP(0,15)                // CAL button function
#define AUTO_BTN                        NRF_GPIO_PIN_MAP(0,16)                // Auto button function

//************************************************
/* define the buttons and led for hu_ui
#define LED_GRN                         NRF_GPIO_PIN_MAP(0,26)                 // all purpose led
#define LED_RED                         NRF_GPIO_PIN_MAP(0,27)                 // all purpose led
#define TILT_UP_BTN                     NRF_GPIO_PIN_MAP(0,05)                 // up
#define TILT_DN_BTN                     NRF_GPIO_PIN_MAP(0,06)                 // down
#define CAL_BTN                         NRF_GPIO_PIN_MAP(0,14)                // CAL button function
#define AUTO_BTN                        NRF_GPIO_PIN_MAP(0,15)                // Auto button function
*/
//************************************************
// define button press interval times to enable different commands
#define SW_TIME_TILT_MODE_AUTO          1000                                    /**< millisec from mid btn press-to-release for client to enter auto mode */
#define SW_TIME_BATT_CHK                4000                                    /**< millisec from mid btn press-to-release for peripheral battery check */
#define SW_TIME_DRIVE_SADDLE            1000                                    /**< millisec from mid btn press-to-release for client to drive saddle tilt */
#define SW_TIME_STORE_CAL_ANGLE         4000                                    /**< millisec from mid btn press-to-release for cpu deep sleep enable */

#define DEVICE_NAME                     "HB_UI"                                 /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "TBW"                                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define SLEEP_TIME_OUT                  10000                                    // millisec before sleep mode if no button pressed

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event 50 msec until a button is reported as pushed (in number of timer ticks). */

#define APP_ADV_DURATION                    18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

//#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(400, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE                     0                                       /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
BLE_LBS_DEF(m_lbs); 


APP_TIMER_DEF(m_delay_timer);                                                   /**< create delay timer instance. determines LED on/off time */
APP_TIMER_DEF(m_sleep_timer);

static uint8_t bike_data_array[20];                           /**< mother board accel array xmitted to cell phone */
static uint8_t saddle_data_array[20];                         /**< daughter board accel array xmitted to cell phone */
static uint16_t index = 20;                                   /**< index for data_array */

static nrf_ble_gatt_t m_gatt;

static bool m_connected;

static bool delay_tmr_busy = false;                                             /**< the delay timer is busy blink the led */
static uint8_t num_blinks;
static uint16_t blink_period = 250;                                             /**< led on/off period */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};


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


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    printf("Erase bonds!\n ");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */

 static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

/////////////////////////////////////////////
//          advertising pkt build
/////////////////////////////////////////////
    ble_uuid_t adv_uuids[] =  {
                              {LBS_UUID_SERVICE, m_lbs.uuid_type}
                              };
   
    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

/////////////////////////////////////////////
//          scan response pkt build
/////////////////////////////////////////////
    memset(&srdata, 0, sizeof(srdata));

    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);

    srdata.uuids_complete.p_uuids  = adv_uuids;

    // NOTE: any print statements in this routine causes an error=7 in
    // sub: err_code = ble_advdata_encode(...)

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    //printf("err_code %x \n", err_code);
    APP_ERROR_CHECK(err_code);

   err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    //printf("err_code %x \n", err_code);
    APP_ERROR_CHECK(err_code);



    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    ret_code_t           err_code;

    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
    num_blinks = 2;
    blink_led();
    }
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
            advertising_start(false);
            break;

        default:
            break;
    }
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

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
 /*
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        printf("GATT ATT MTU on connection 0x%x changed to %d.\n ",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }

    ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}
*/

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 *//*
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}
*/

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


/**@brief Function for handling Relay Client write events to the HB_UI LED characteristic.
 * Data sent to HB_UI from Relay
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 * led_state can take on vals from 0 to 0xff
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t m_tilter_cmd_byte)
{
    ret_code_t err_code;

    printf("cmd byte from tilter relay = %d \n", m_tilter_cmd_byte);
/*    
    switch(m_tilter_cmd_byte)
    {  
        case SOME_COMMAND_FROM_TILTER:
        {
             m_tilt_mode = MANUAL;
             accel_smpl_timer_stop();
             motor_drive_saddle_up();
             printf("manual mode - sdl up \n");
        } break;
   }
*/ 
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_lbs_init_t     init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);


    // Initialize LBS
    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
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
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
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
            printf("Fast advertising.\n ");
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;
/*
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
*/
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            num_blinks = 4;
            blink_led();
            printf("Connected.\n ");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
    // *********************************
    start_sleep_tmr(SLEEP_TIME_OUT);
    // *********************************            
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            num_blinks = 1;
            blink_led();
            printf("Disconnected, reason %d.\n ",
            p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            printf("PHY update request.\n ");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            printf("GATT Client Timeout.\n ");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            printf("GATT Server Timeout.\n ");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            printf("BLE_GAP_EVT_SEC_PARAMS_REQUEST\n ");
            break;
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            printf("BLE_GAP_EVT_AUTH_KEY_REQUEST\n ");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            printf("BLE_GAP_EVT_LESC_DHKEY_REQUEST\n ");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             printf("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x\n ",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
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




////////////////////////////////////////////////// TIMER STUFF ///////////////////////////////////////////////

//simple delay timer using timer
//
void m_delay_timer_handler(void * p_context)
{
    uint8_t i;
    uint32_t err_code;

    if ( num_blinks == 1)
    {
        nrf_gpio_pin_write(LED_RED, LED_OFF);
        delay_tmr_busy = false;        // no longer using the timer
    }
    else
    {
        if (num_blinks % 2 == 0) // is num_blinks EVEN?  % is mod operator
        {
            nrf_gpio_pin_write(LED_RED, LED_ON);   // if yes turn on LED
            //printf("num_blinks even = %d \n", num_blinks % 2);
        }
        else
        {
            nrf_gpio_pin_write(LED_RED, LED_OFF);   // if not turn off LED
            //printf("num_blinks odd = %d \n", num_blinks % 2);
        }
          
        num_blinks--;
        //printf("num_blinks= %d \n", num_blinks);
        err_code = app_timer_start(m_delay_timer, APP_TIMER_TICKS(blink_period), NULL); // start the delay timer
        APP_ERROR_CHECK(err_code);
    }
}


void blink_led()
{
    uint32_t err_code;
    if (!delay_tmr_busy)
    {
        num_blinks = num_blinks * 2;
        delay_tmr_busy = true;      // use delay timer to give blinking led
        err_code = app_timer_start(m_delay_timer, APP_TIMER_TICKS(1), NULL); // start the delay timer
                                                                        // to call the tmr handler after 1msec
        APP_ERROR_CHECK(err_code);
    }
}


// if timer "times out" means that the server did not connect with the client during the 
// scan_timeout_timer_timeout period
// the device is put into deep sleep to save the battery. TILT_DN_BTN
// brings the device out of deep sleep which resets the Nordic hardware cpu

void m_sleep_timer_handler(void * p_context)
{
    int8_t blink_cycles;
    //send_byte_to_tilter(TILT_STOP);
    printf("GOING TO SLEEP \n");
    sd_power_system_off();  // enter deep sleep to save battery
    // config Tilt_dn button to bring the device out of deep sleep
    /*
    nrf_gpio_pin_write(LED, LED_ON);
    nrf_delay_ms(500);
    nrf_gpio_pin_write(LED, LED_OFF);
    nrf_delay_ms(500);
    nrf_gpio_pin_write(LED, LED_ON);
    nrf_delay_ms(500);
    nrf_gpio_pin_write(LED, LED_OFF);

    nrf_gpio_cfg_sense_input(TILT_DN_BTN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);  //hb_ui pulldown
    //nrf_delay_ms(1000);        // workaround for system bug
    
    */
}


// called each time a button is pressed. this allows battery savings if the system isn't being used
// if the timer expires then sleep mode is entered
//
void start_sleep_tmr(int time_period)
{

    ret_code_t err_code;
    err_code = app_timer_start(m_sleep_timer, APP_TIMER_TICKS(time_period), NULL);
    APP_ERROR_CHECK(err_code);
}

void stop_sleep_tmr()
{
    ret_code_t err_code;
    err_code = app_timer_stop(m_sleep_timer);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sleep_timer, APP_TIMER_MODE_SINGLE_SHOT, m_sleep_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_delay_timer, APP_TIMER_MODE_SINGLE_SHOT, m_delay_timer_handler);
    APP_ERROR_CHECK(err_code);
}

////////////////////////////////////////////////// END TIMER STUFF ////////////////////////////////////////////




////////////////////////////////////////////////// BUTTON STUFF ///////////////////////////////////////////////


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
      ret_code_t err_code;
      printf("pin no. %d \n", pin_no);
      printf("btn action. %d \n", button_action);
          num_blinks = 3;
    //blink_led();
      switch (pin_no)
      {
          case TILT_UP_BTN:

              if(button_action == APP_BUTTON_PUSH) // 1 = button press
              { 
                printf("btn pressed. \n");
                stop_sleep_tmr();  // stop sleep timer and wait for btn release before restarting
                send_byte_to_tilter(TILT_UP);  // send cmd to client
                nrf_gpio_pin_write(LED_RED, LED_ON);      // press btn = LED on
                
              }
              else  // 0 = button released
              { 
                printf("btn released \n");
                start_sleep_tmr(SLEEP_TIME_OUT);  // system is being used, reset timeout period
                send_byte_to_tilter(TILT_STOP);  // send cmd to client
                nrf_gpio_pin_write(LED_RED, LED_OFF);      // release btn = LED off
                
              }
              break;

          case TILT_DN_BTN:
              if(button_action == APP_BUTTON_PUSH)
              {
                stop_sleep_tmr();  // stop sleep timer and wait for btn release before restarting
                send_byte_to_tilter(TILT_DN);  // send cmd to client
                nrf_gpio_pin_write(LED_RED, LED_ON);      // press btn = LED on
              }
              else
              {
                start_sleep_tmr(SLEEP_TIME_OUT);  // system is being used, reset timeout period
                send_byte_to_tilter(TILT_STOP);  // send cmd to client
                nrf_gpio_pin_write(LED_RED, LED_OFF);      // release btn = LED off
              }
              break;

          case CAL_BTN:       //CAL button enables one of two functions depending on the button press duration 
                              // 1. if: 0 < counter < 1 sec
                              //  send a cmd to Tilter to drive saddle tilt to calibrated position  
                              // 2. if: 1 < counter < 4 sec
                              //  send a cmd to Tilter to save current saddle tilt to flash

            if(button_action == APP_BUTTON_PUSH)
                {
                    // Start the  up counter.  When the btn is released we will measure the delta time
                    // from button press to button release. the elapsed time will determine what
                    // mode to execute
                    stop_sleep_tmr();  // stop sleep timer and wait for btn release before restarting
                    counter_start(); //start counting by 1msec ticks to meas the CAL button hold-down time
                    nrf_gpio_pin_write(LED_RED, LED_ON);      // press btn = LED on
                }

                else
                {
                    nrf_gpio_pin_write(LED_RED, LED_OFF);      // release btn = LED off
                    start_sleep_tmr(SLEEP_TIME_OUT);  // system is being used, reset timeout period
                    counter_stop();
                    uint32_t time_ms = counter_get();  // read raw counter val (in millisec). how long 
                                                       // since the CAL btn was pressed?


                    printf("============================= \n");
                    printf("CAL bt duration press duration time: %d milliseconds elapsed.\n" , time_ms);
                    printf("============================= \n") ;

                    if(time_ms < SW_TIME_DRIVE_SADDLE)
                    {
                      send_byte_to_tilter(DRIVE_SADDLE_TO_CAL_ANGLE);  // send cmd to client
                      // printf("tilt saddle to calibrated angle cmd sent to Client. \n \n");

                    }

                   else if(time_ms < SW_TIME_STORE_CAL_ANGLE)
                   {
                      send_byte_to_tilter(STORE_CAL_ANGLE);  // send cmd to client
                      // printf("write cal angle to flash cmd sent to Client. \n \n");
                   }

                   else
                   {
                      // printf("button press duration greater than %d milliseconds \n \n" , SW_TIME_STORE_CAL_ANGLE);
                   }
                }      
                break;

          case AUTO_BTN:      // AUTO button enables one of two functions depending on the button press duration 
                              // 1. if: 0 < counter < 1 sec
                              //  send a cmd to Tilter to switch to AUTO mode 
                              // 2. if: 1 < counter < 4 sec
                              //  read battery voltage and blink LED according to its charge level
            nrf_gpio_pin_write(LED_RED, LED_ON);      // press btn = LED on

            if(button_action == APP_BUTTON_PUSH)
                {
                    // printf("auto btn pressed. \n");
                    stop_sleep_tmr();  // stop sleep timer and wait for btn release before restarting
                    counter_start(); //start counting by 1msec ticks to meas the auto button hold-down time
                }

                else
                {
                    nrf_gpio_pin_write(LED_RED, LED_OFF);      // release btn = LED off
                    start_sleep_tmr(SLEEP_TIME_OUT);  // system is being used, reset timeout period
                    counter_stop();
                    uint32_t time_ms = counter_get();  // read raw counter val (in millisec). how long 
                                                       // since the CAL btn was pressed?
                    // printf("============================= \n");
                    // printf("AUTO btn press duration time: %d milliseconds elapsed.\n" , time_ms);
                    // printf("============================= \n") ;

                    if(time_ms < SW_TIME_TILT_MODE_AUTO)
                    {
                      send_byte_to_tilter(AUTO_MODE);  // send cmd to client
                      // printf("AUTO mode cmd sent to Client \n \n");
                    }

                   else if(time_ms < SW_TIME_BATT_CHK)
                   {
                      printf("check local battery. \n \n");
                   }

                   else
                   {
                      stop_sleep_tmr();  // stopping the timer enables it to accept a new value
                      start_sleep_tmr(SLEEP_TIME_OUT);
                      // printf("btn press time >  %d millisec \n", SW_TIME_BATT_CHK);
                   }
                }      
                break;

          default:
              APP_ERROR_HANDLER(pin_no);
              break;

           break;
      }

}

/**@brief Function for writing one byte to the client based upon what button the user pressed

 */

void send_byte_to_tilter(uint32_t cmd)
  {
      int8_t b = cmd;
      ret_code_t err_code;
      int8_t blink_cycles;

      err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, b); /**< send byte command to client */
      if (err_code != NRF_SUCCESS &&
          err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
          err_code != NRF_ERROR_INVALID_STATE &&
          err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
      {
          APP_ERROR_CHECK(err_code);
      }
      printf("sent switch to tilter %d \n", cmd);
  }


/**@brief Function for initializing the button handler module.
 */
static void led_and_buttons_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {TILT_UP_BTN, false, NRF_GPIO_PIN_PULLUP, button_event_handler},  //false , HB_UI PULLDOWN
        {TILT_DN_BTN, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
        {CAL_BTN, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
        {AUTO_BTN, false, NRF_GPIO_PIN_PULLUP, button_event_handler}
    };
    err_code = app_button_init(buttons, sizeof(buttons)/sizeof(buttons[0]), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_button_enable(); // enable all buttons in buttons[] to call button_event_handler
    APP_ERROR_CHECK(err_code);

    nrf_gpio_cfg_output(LED_RED);       // Right Angle LED


    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);

    nrf_gpio_pin_write(LED_RED, LED_OFF);   // start with led off


}

////////////////////////////////////////////////// END BUTTON STUFF //////////////////////////////////////////



/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    // must use (NRF_LOG_PROCESS() == false) or pgm freezes in debug mode
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
    counter_init();
    timers_init();
    led_and_buttons_init(&erase_bonds);
    power_management_init();
        
    ble_stack_init();
    gap_params_init();
    gatt_init();

    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();
    advertising_start(erase_bonds);
;
    // Start execution.

    num_blinks = 3;
    blink_led();
    //start_sleep_tmr(SLEEP_TIME_OUT); // 5 sec for testing troubleshooting 5/6/2024
    printf("hb_ui example started.\n ");

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


