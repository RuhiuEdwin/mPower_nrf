/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
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

/**
 * @brief Multiperipheral Sample Application main file.
 *
 * This file contains the source code for a sample server application with multiple peripheral connections using the LED Button service.
 */

#include "nordic_common.h"
#include "nrf.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_drv_gpiote.h"

#include "app_button.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_lbs.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_service.h"
#include "usb_port.h"

#define DEVICE_NAME "mPower"               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME "Are Consulting" /**< Manufacturer. Will be passed to Device Information Service. */
#define MP_DEVICE_ID 0x11                  /**< Id of device. */

#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */
#define LINK_TOTAL NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + \
                     NRF_SDH_BLE_CENTRAL_LINK_COUNT

#define ADVERTISING_LED BSP_BOARD_LED_0 /**< Is on when device is advertising. */
#define CONNECTED_LED BSP_BOARD_LED_1   /**< Is on when device has connected. */
#define LEDBUTTON_LED BSP_BOARD_LED_2   /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON BSP_BUTTON_0   /**< Button that will trigger the notification event with the LED Button Service */

#define APP_ADV_INTERVAL 300                                   /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS) /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(20000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                        /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//pin change handler
#ifdef BSP_BUTTON_0
#define PIN_IN_0 BSP_BUTTON_0
#endif
#ifdef BSP_BUTTON_1
#define PIN_IN_1 BSP_BUTTON_1
#endif
#ifdef BSP_BUTTON_2
#define PIN_IN_2 BSP_BUTTON_2
#endif
#ifdef BSP_BUTTON_3
#define PIN_IN_3 BSP_BUTTON_3
#endif
#ifndef PIN_IN_0
#error "Please indicate input pin"
#endif
#define PIN_IN_START PIN_IN_0
#define PIN_IN_STOP PIN_IN_3

#ifdef BSP_LED_0
#define PIN_OUT_0 BSP_LED_0
#endif
#ifdef BSP_LED_1
#define PIN_OUT_1 BSP_LED_1
#endif
#ifdef BSP_LED_2
#define PIN_OUT_2 BSP_LED_2
#endif
#ifdef BSP_LED_3
#define PIN_OUT_3 BSP_LED_3
#endif
#ifndef PIN_OUT_0
#error "Please indicate output pin"
#endif

void inPinHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    //NRF_LOG_INFO("inPinHandler: pin=%d, action=%d", pin, action);
    //onUsbPortChange(pin, action);
}

/**
 * @brief Function for configuring: P0_xx pin for input, P0_20 pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */

static void gpio_init(void) {
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;
//  nrf_drv_gpiote_in_config_t in_config = {
//    .sense = NRF_GPIOTE_POLARITY_TOGGLE,  //NRF_GPIOTE_POLARITY_LOTOHI,
//    .pull = NRF_GPIO_PIN_PULLUP,          //NRF_GPIO_PIN_NOPULL
//    .is_watcher = false,
//    .hi_accuracy = false
//  };

  err_code = nrf_drv_gpiote_out_init(IC_SRCLK_P0_8, &out_config);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_gpiote_out_init(IC_RCLK_P0_9, &out_config);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_gpiote_out_init(IC_SER_P0_10, &out_config);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_gpiote_out_init(SHIFT_REGISTER_P0_18, &out_config);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_gpiote_out_init(SHIFT_REGISTER_P0_19, &out_config);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_gpiote_in_init(SHIFT_REGISTER_P0_20, &in_config, inPinHandler);
  APP_ERROR_CHECK(err_code);
  //nrf_gpio_cfg_input(SHIFT_REGISTER_P0_20, NRF_GPIO_PIN_NOPULL);

  nrf_drv_gpiote_in_event_enable(SHIFT_REGISTER_P0_20, true);
  //NRF_LOG_INFO("gpio_init: end");
}

/**@brief   Priority of the application BLE event handler.
 * @note    You shouldn't need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO 3

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;           /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];            /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded scan data. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
  {
    .adv_data =
      {
        .p_data = m_enc_advdata,
        .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX},
    .scan_rsp_data =
      {
        .p_data = m_enc_scan_response_data,
        .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX

      }};

// Declare a service structure for our application
ble_mp_t m_ble_mp;
NRF_BLE_GATT_DEF(m_gatt);                              /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< Context for the Queued Write module.*/


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
  NRF_LOG_ERROR("assert_nrf_callback");
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void) {
  bsp_board_init(BSP_INIT_LEDS);
}

// Declare an app_timer id variable and define our timer interval
APP_TIMER_DEF(m_mp_char_timer_id);
#define MP_CHAR_TIMER_INTERVAL APP_TIMER_TICKS(1000) // 1000 ms intervals
static uint8_t port_status_announcer_counter = MP_SEND_PORT_STATUS_INTERVAL;

/**@brief Function for Timer event handler
 */
static void timer_timeout_handler(void *p_context) {
  checkUsbPorts();
  if (port_status_announcer_counter == 0) {
    sendPortStatusToAll(&m_ble_mp);
    port_status_announcer_counter = MP_SEND_PORT_STATUS_INTERVAL;
  } else {
    port_status_announcer_counter--;
  }
}


/**@brief Function for starting timers.
 */
  static void application_timers_start(void) {
    // Start the MP timer
    app_timer_start(m_mp_char_timer_id, MP_CHAR_TIMER_INTERVAL, NULL);
  }

  /**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
  static void timers_init(void) {
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Initiate timer
    app_timer_create(&m_mp_char_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
  }

  /**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
  static void gap_params_init(void) {
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
      (const uint8_t *)DEVICE_NAME,
      strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
  }

  /**@brief Function for initializing the GATT module.
 */
  static void gatt_init(void) {
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
  }

  /**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
  static void advertising_init(void) {
    ret_code_t err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;
    ble_gap_adv_params_t adv_params;

    ble_uuid_t adv_uuids[] = {{BLE_UUID_MP_SERVICE, m_ble_mp.uuid_type}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    // Start advertising.
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.p_peer_addr = NULL;
    adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
    adv_params.interval = APP_ADV_INTERVAL;

    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.duration = APP_ADV_DURATION;
    adv_params.primary_phy = BLE_GAP_PHY_1MBPS;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
  }

  /**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
  static void nrf_qwr_error_handler(uint32_t nrf_error) {
    NRF_LOG_ERROR("nrf_qwr_error_handler");
    APP_ERROR_HANDLER(nrf_error);
  }

  /**@brief Function for initializing services that will be used by the application.
 */
  static void services_init(void) {

    m_ble_mp.location_id = MP_DEVICE_ID;
    ret_code_t err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module instances.
    qwr_init.error_handler = nrf_qwr_error_handler;

    for (uint32_t i = 0; i < LINK_TOTAL; i++) {
      err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
      APP_ERROR_CHECK(err_code);
    }

    // Initialize the services used by the application.
    err_code = mpServiceInit(&m_ble_mp);
    APP_ERROR_CHECK(err_code);

    //diff
    ble_conn_state_init();
  }

  /**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
  static void conn_params_error_handler(uint32_t nrf_error) {
    NRF_LOG_ERROR("conn_params_error_handler");
    APP_ERROR_HANDLER(nrf_error);
  }

  /**@brief Function for initializing the Connection Parameters module.
 */
  static void conn_params_init(void) {
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = true;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
  }

  /**@brief Function for starting advertising.
 */
  static void advertising_start(void) {
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);

    APP_ERROR_CHECK(err_code);
    //bsp_board_led_on(ADVERTISING_LED);
  }

  /**@brief Function for handling the Connected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
  static void on_connected(const ble_gap_evt_t *const p_gap_evt) {
    ret_code_t err_code;
    uint32_t periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    NRF_LOG_INFO("Connection with link 0x%x established.", p_gap_evt->conn_handle);

    // Assign connection handle to available instance of QWR module.
    for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++) {
      if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID) {
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], p_gap_evt->conn_handle);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("Connection handle=%0x", p_gap_evt->conn_handle);
        break;
      }
    }

#if DIFF    
    err_code = app_button_enable();
    NRF_LOG_INFO("app_button_enable, ret=%0x", err_code);
    APP_ERROR_CHECK(err_code);
#endif

    // Update LEDs
    //TODO turn on LED on connect ????
    //bsp_board_led_on(CONNECTED_LED);
    if (periph_link_cnt == NRF_SDH_BLE_PERIPHERAL_LINK_COUNT) {
      bsp_board_led_off(ADVERTISING_LED);
    } else {
      // Continue advertising. More connections can be established because the maximum link count has not been reached.
      advertising_start();
    }
  }

  /**@brief Function for handling the Disconnected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
  static void on_disconnected(ble_gap_evt_t const *const p_gap_evt) {
    ret_code_t err_code;
    uint32_t periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
      p_gap_evt->conn_handle,
      p_gap_evt->params.disconnected.reason);

    if (periph_link_cnt == 0) {
      bsp_board_led_off(CONNECTED_LED);
      // ignore any error - APP_ERROR_CHECK(err_code);
    }

    if (periph_link_cnt == (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - 1)) {
      // Advertising is not running when all connections are taken, and must therefore be started.
      advertising_start();
    }
    NRF_LOG_INFO("on_disconnected leave");
  }

  /**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
  static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
      on_connected(&p_ble_evt->evt.gap_evt);
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      on_disconnected(&p_ble_evt->evt.gap_evt);
      break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      // Pairing not supported
      err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
        BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
        NULL,
        NULL);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
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

    default:
      // No implementation needed.
      break;
    }
  }

  /**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
  static void ble_stack_init(void) {
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

    // Call MP service onBleEvent() to do housekeeping of ble connections related to our service and characteristics
    NRF_SDH_BLE_OBSERVER(m_mp_service_observer, APP_BLE_OBSERVER_PRIO, onBleEvent, (void *)&m_ble_mp);
  }

  /**@brief Function for initializing the logging module.
 */
  static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
  }

  /**@brief Function for initializing power management.
 */
  static void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
  }

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

/**@brief Function for application main entry.
 */
int main(void) {

  // Initialize.
  log_init();
  //pin_change init
  gpio_init();
  timers_init();
  leds_init();
  power_management_init();
  ble_stack_init();
  gap_params_init();
  gatt_init();

  services_init();
  advertising_init();

  initUsbPorts();

  conn_params_init();

  // Start execution.
  NRF_LOG_INFO("%s started.", DEVICE_NAME);
  advertising_start();
  application_timers_start();

  // Enter main loop.
  for (;;) {
    idle_state_handle();
  }
}

    /**
 * @}
 */