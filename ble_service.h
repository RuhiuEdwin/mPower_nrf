
#ifndef OUR_SERVICE_H__
#define OUR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"


// FROM_SERVICE_TUTORIAL: Defining 16-bit service and 128-bit base UUIDs
// UUID: 0000f00d-6cc7-425b-ad67-fcafcda33672 // 128-bit base UUID
#define BLE_UUID_MP_BASE        {0x72, 0x36, 0xA3, 0xAF, 0xCD, 0xFC, 0x67, 0xAD,\
                                 0x5B, 0x42, 0xC7, 0x6C, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
#define BLE_UUID_MP_SERVICE     0xF00D                                                                   // Just a random, but recognizable value

// Defining 16-bit characteristic UUID
#define BLE_UUID_MP_CMD_CHAR    0xBEEE  // Just a random, but recognizable value
#define BLE_UUID_MP_ALERT_CHAR  0xBEEF  // Just a random, but recognizable value

#define BLE_MAX_MESSAGE_SIZE    31

#define MP_SEND_PORT_STATUS_INTERVAL 5    // Send USB port status every n seconds

// This structure contains various status information for our service.
// The name is based on the naming convention used in Nordics SDKs.
// 'ble’ indicates that it is a Bluetooth Low Energy relevant structure and 
// NB! Multiple Connections - THE CONNECTION HANDLE, CONN_HANDLE, IS THERE TO KEEP TRACK OF THE CURRENT CONNECTION
typedef struct
{
  uint16_t conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
  uint16_t service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t cmd_char_handles;
  ble_gatts_char_handles_t alert_char_handles;
  uint8_t uuid_type; /**< UUID type for the MP Service. */
  uint8_t location_id;
} ble_mp_t;

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void onBleEvent(ble_evt_t const *p_ble_evt, void *p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
uint32_t mpServiceInit(ble_mp_t *p_ble_mp);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_our_service                     Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */

void sendNotification(uint16_t charHandle, uint16_t connHandle, uint32_t *p_data, uint8_t length);

void sendPortStatusToAll(ble_mp_t *p_ble_mp);

#endif /* _ OUR_SERVICE_H__ */