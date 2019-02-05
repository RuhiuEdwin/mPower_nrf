#include "app_error.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include <stdint.h>
#include <string.h>

#include "ble_service.h"
#include "usb_port.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   ble_mp      MP Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void onConnect(ble_mp_t *p_ble_mp, ble_evt_t const *p_ble_evt) {
  p_ble_mp->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   ble_mp      MP Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void onDisconnect(ble_mp_t *p_ble_mp, ble_evt_t const *p_ble_evt) {
  UNUSED_PARAMETER(p_ble_evt);
  p_ble_mp->conn_handle = BLE_CONN_HANDLE_INVALID;
  onBleDisconnect(p_ble_evt);
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   ble_mp      MP Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void onWrite(ble_mp_t *p_ble_mp, ble_evt_t const *p_ble_evt) {
  ble_gatts_evt_write_t *p_evt_write = (ble_gatts_evt_write_t *)&p_ble_evt->evt.gatts_evt.params.write;

  if ((p_evt_write->handle == p_ble_mp->cmd_char_handles.value_handle)) {
    onNewCommand(p_ble_evt);
  }
}

//  Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic
void onBleEvent(ble_evt_t const *p_ble_evt, void *p_context) {
  ble_mp_t *p_ble_mp = (ble_mp_t *)p_context;

  NRF_LOG_INFO("ble_mp_service_on_ble_evt: event=%d", p_ble_evt->header.evt_id);

  // Implement switch case handling BLE events related to our service.
  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    onConnect(p_ble_mp, p_ble_evt);
    break;
  case BLE_GAP_EVT_DISCONNECTED:
    onDisconnect(p_ble_mp, p_ble_evt);
    break;
  case BLE_GATTS_EVT_WRITE:
    // Edgar: Write event
    NRF_LOG_INFO("Write operation performed, char_handle=%0x", p_ble_evt->evt.gatts_evt.params.write.handle);
    onWrite(p_ble_mp, p_ble_evt);
    break;
  default:
    // No implementation needed.
    break;
  }
}

/**@brief Function for adding the LED Characteristic.
 *
 * @param[in] p_ble_mp      USB Service structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t addCmdCharacteristic(ble_mp_t *p_ble_mp) {
  /*
    uint32_t            err_code;
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_MP_BASE;
    char_uuid.uuid      = BLE_UUID_MP_CMD_CHAR;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    NRF_LOG_INFO("cmd_char_uuid=%0x, type=%x\n", char_uuid.uuid, char_uuid.type);
    APP_ERROR_CHECK(err_code);  

    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = true;
    char_md.char_props.write = true;

    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;

    // TODO: vise location_id 
    uint8_t value[5]            = {0x12,0x34,0x56,0x78};
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = sizeof(value);
    attr_char_value.p_value     = value;

    err_code = sd_ble_gatts_characteristic_add(p_ble_mp->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_ble_mp->cmd_char_handles);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
*/

  uint32_t err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t attr_char_value;
  ble_uuid_t ble_uuid;
  ble_gatts_attr_md_t attr_md;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.read = 1;
  char_md.char_props.write = 1;
  char_md.p_char_user_desc = NULL;
  char_md.p_char_pf = NULL;
  char_md.p_user_desc_md = NULL;
  char_md.p_cccd_md = NULL;
  char_md.p_sccd_md = NULL;

  //err_code = sd_ble_uuid_vs_add(&ble_uuid, &ble_uuid.type);
  ble_uuid.type = p_ble_mp->uuid_type;
  ble_uuid.uuid = BLE_UUID_MP_CMD_CHAR;
  NRF_LOG_INFO("cmd_char ble_uuid=%0x, type=%0x", ble_uuid.uuid, ble_uuid.type);

  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen = 0;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;

  uint8_t value[1];
  value[0] = p_ble_mp->location_id;
  attr_char_value.p_value = value;
  attr_char_value.init_len = sizeof(value);
  attr_char_value.max_len = BLE_MAX_MESSAGE_SIZE;

  return sd_ble_gatts_characteristic_add(p_ble_mp->service_handle,
    &char_md,
    &attr_char_value,
    &p_ble_mp->cmd_char_handles);
}

/**@brief Function for adding the Alert Characteristic.
 *
 * @param[in] p_ble_mp      USB Service structure.
 * @param[in] p_mp_init USB Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t addAlertCharacteristic(ble_mp_t *p_ble_mp) {
  /*
    uint32_t            err_code;
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_MP_BASE;
    char_uuid.uuid      = BLE_UUID_MP_ALERT_CHAR;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    NRF_LOG_INFO("alert_char_uuid=%0x, type=%x", char_uuid.uuid, char_uuid.type);
    APP_ERROR_CHECK(err_code);  

    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = true;
    char_md.char_props.write = true;

    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;

    uint8_t value[5]            = {0x12,0x34,0x56,0x78};
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = sizeof(value);
    attr_char_value.p_value     = NULL; //value;

    err_code = sd_ble_gatts_characteristic_add(p_ble_mp->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_ble_mp->alert_char_handles);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;

*/
  uint32_t err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t attr_char_value;
  ble_uuid_t ble_uuid;
  ble_gatts_attr_md_t attr_md;

  memset(&cccd_md, 0, sizeof(cccd_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.read = 1;
  char_md.char_props.notify = 1;
  char_md.p_char_user_desc = NULL;
  char_md.p_char_pf = NULL;
  char_md.p_user_desc_md = NULL;
  char_md.p_cccd_md = &cccd_md;
  char_md.p_sccd_md = NULL;

  //err_code = sd_ble_uuid_vs_add(&ble_uuid, &ble_uuid.type);
  ble_uuid.type = p_ble_mp->uuid_type;
  ble_uuid.uuid = BLE_UUID_MP_ALERT_CHAR;
  NRF_LOG_INFO("alert_char ble_uuid=%0x, type=%0x", ble_uuid.uuid, ble_uuid.type);

  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen = 0;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;

  uint8_t value[1];
  value[0] = p_ble_mp->location_id;
  attr_char_value.p_value = value;
  attr_char_value.init_len = sizeof(value);
  attr_char_value.max_len = BLE_MAX_MESSAGE_SIZE;

  return sd_ble_gatts_characteristic_add(p_ble_mp->service_handle,
    &char_md,
    &attr_char_value,
    &p_ble_mp->alert_char_handles);
}

uint32_t mpServiceInit(ble_mp_t *p_ble_mp) {
  uint32_t err_code;
  ble_uuid_t ble_uuid;

  // Set 'current' service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
  p_ble_mp->conn_handle = BLE_CONN_HANDLE_INVALID;

  // Add service.
  ble_uuid128_t base_uuid = {BLE_UUID_MP_BASE};
  err_code = sd_ble_uuid_vs_add(&base_uuid, &p_ble_mp->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_ble_mp->uuid_type;
  ble_uuid.uuid = BLE_UUID_MP_SERVICE;

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ble_mp->service_handle);
  VERIFY_SUCCESS(err_code);

  // Add characteristics.
  err_code = addCmdCharacteristic(p_ble_mp);
  VERIFY_SUCCESS(err_code);

  err_code = addAlertCharacteristic(p_ble_mp);
  VERIFY_SUCCESS(err_code);

  return NRF_SUCCESS;
}

/**@brief Function for writing to the ALERT characteristic of all connected clients.
 *
 * @return If successful NRF_SUCCESS is returned. Otherwise, the error code
 */
void sendPortStatusToAll(ble_mp_t *p_ble_mp) {
  ret_code_t err_code;
  ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_periph_handles();
  uint16_t alert_handler = p_ble_mp->alert_char_handles.value_handle;
  uint8_t ackMsg[BLE_MAX_MESSAGE_SIZE];
  uint8_t byte_counter = 0;
  uint8_t ret = MP_SUCCESS;
  uint8_t port_status;

  memset(&ackMsg, 0, sizeof(port_status));
  ackMsg[byte_counter++] = p_ble_mp->location_id;
  ackMsg[byte_counter++] = MP_PORT_STATUS_ALERT;

  for (uint8_t i = MP_FIRST_USB_PORT_NUMBER; i <= MP_MAX_USB_PORT_NUMBER; i++) {
    ret = getPortStatus(i, &port_status);
    if (ret == MP_SUCCESS) {
      ackMsg[byte_counter++] = port_status;
    }
  }

  for (uint8_t i = 0; i < conn_handles.len; i++) {
    sendNotification(alert_handler, conn_handles.conn_handles[i], &ackMsg, byte_counter);
  }
}

void sendNotification(uint16_t charHandle, uint16_t connHandle, uint32_t *p_data, uint8_t length) {
  uint16_t len = length;
  ble_gatts_hvx_params_t hvx_params;

  if (connHandle != BLE_CONN_HANDLE_INVALID) {
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = charHandle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = p_data;

    NRF_LOG_INFO("sendNotification: connHandle=%0x, len=%d, value=%0x", connHandle, len, *p_data);

    sd_ble_gatts_hvx(connHandle, &hvx_params);
  }
}
