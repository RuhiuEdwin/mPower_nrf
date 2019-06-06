#include "app_error.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "nrf_gpio.h"

#include "app_error.h"
#include "ble_srv_common.h"
#include "bsp_btn_ble.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "sensorsim.h"
#include <stdint.h>
#include <string.h>

#include "ble_service.h"
#include "usb_port.h"

extern ble_mp_t m_ble_mp;
void clearOutputPins(bool boot);

UsbPort *usbPorts[MP_MAX_USB_PORT_NUMBER + 1];

#define MP_PORT   0
#define MP_STATE  1

//Pins 1-16: p8, p7, p6, p5, p4, p3, p2, p1, status_led1, status_led2, status_led3, power1, power2, power3, p10, p9.
static uint8_t pinOutputMap[16][2] = {  // [MP_PORT (=USB port number)][MP_STATE (=on/off)]
  {8, 0}, {7, 0}, {6, 0}, {5, 0}, {4, 0}, {3, 0}, {2, 0}, {1, 0},
  {LED_1, 0}, {LED_2, 0}, {LED_3, 0}, {0, 0}, {0, 0}, {0, 0}, {10, 0}, {9, 0}
};

// Output_X_active - oaX, Output_X_fault = ofX
//Pins 1-24: oa5, oa6, oa7, oa8, oa4, oa3, oa2, oa1, 
//           of3, of4, of5, of6, of1, of2, oa10, oa9,
//           -, -, -, -, of10, of9, of8, of7
static uint8_t pinInputMap[24][2] = {  // [MP_PORT (=USB port number)][MP_STATE (=on/off)]
  {5, 0}, {6, 0}, {7, 0}, {8, 0}, {4, 0}, {3, 0}, {2, 0}, {1, 0},
  {13, 0}, {14, 0}, {15, 0}, {16, 0}, {11, 0}, {12, 0}, {10, 0}, {9, 0},
  {0 /*test*/, 0}, {0, 0}, {0, 0}, {0, 0}, {20, 0}, {19, 0}, {18, 0}, {17, 0}
};

// verdiene over 10 er Output X fault

void initUsbPorts() {
  for (int i = MP_FIRST_USB_PORT_NUMBER; i <= MP_MAX_USB_PORT_NUMBER; i++) {
    usbPorts[i] = malloc(sizeof(UsbPort));
    memset(usbPorts[i], 0, sizeof(UsbPort));
    usbPorts[i]->connHandle = BLE_CONN_HANDLE_INVALID;
  }
  clearOutputPins(true);
}

uint8_t getPort(uint8_t port, UsbPort *data) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  uint32_t *p = usbPorts[port];
  return MP_SUCCESS;
}

uint8_t initPortStatus(uint8_t port, UsbPortStatus status, int16_t ticks, uint16_t handle) {
  NRF_LOG_INFO("setting port %x %x", port, status);
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }

  usbPorts[port]->status = status;
  usbPorts[port]->remainingChargeTicks = ticks;
  usbPorts[port]->connHandle = handle;
  return MP_SUCCESS;
}

uint8_t getPortStatus(uint8_t port, UsbPortStatus *status) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  *status = usbPorts[port]->status;
  return MP_SUCCESS;
}

uint8_t setPortStatus(uint8_t port, UsbPortStatus status) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  usbPorts[port]->status = status;
  return MP_SUCCESS;
}

uint8_t getState(uint8_t port, uint8_t *state) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  *state = usbPorts[port]->state;
  return MP_SUCCESS;
}

uint8_t setState(uint8_t port, uint8_t state) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  usbPorts[port]->state = state;
  return MP_SUCCESS;
}

uint8_t getConnHandle(uint8_t port, uint16_t *handle) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    *handle = BLE_CONN_HANDLE_INVALID;
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  *handle = usbPorts[port]->connHandle;
  return MP_SUCCESS;
}

uint8_t setConnHandle(uint8_t port, uint16_t handle) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  usbPorts[port]->connHandle = handle;
  return MP_SUCCESS;
}

uint8_t getRamainingChargeTicks(uint8_t port, int16_t *ticks) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  *ticks = usbPorts[port]->remainingChargeTicks;
  return MP_SUCCESS;
}

uint8_t setRamainingChargeTicks(uint8_t port, int16_t ticks) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  usbPorts[port]->remainingChargeTicks = ticks;
  return MP_SUCCESS;
}

uint8_t decrementChargingTicks(uint8_t port, int16_t *ticks) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  if (usbPorts[port]->status != MP_AVAILABLE) {
    --usbPorts[port]->remainingChargeTicks;
  }
  *ticks = usbPorts[port]->remainingChargeTicks;
  return MP_SUCCESS;
}

/*
  allocateFreePort
  return: <port number> if a port is available and sets the port status to FREE_CHARGE
          ERROR_NO_AVAILABLE_PORT if no ports are available
*/
uint8_t allocateFreePort(uint8_t *port) {

  for (int i = MP_FIRST_USB_PORT_NUMBER; i <= MP_MAX_USB_PORT_NUMBER; i++) {
    bool free = false;
    if (usbPorts[i]->status == MP_QUARANTINE && usbPorts[i]->state != MP_OUTPUT_X_ACTIVE)
      free = true;
    if (!free && usbPorts[i]->status == MP_AVAILABLE)
      free = true;
    if (free) {
      usbPorts[i]->status = MP_FREE_CHARGE;
      *port = i;
      return MP_SUCCESS;
    }
  }
  return MP_ERROR_NO_AVAILABLE_PORT;
}

void clearOutputPins(bool boot) {
  nrf_gpio_pin_clear(IC_SER_P0_10);
  for (uint8_t i = 0; i < 16 ; i++) {
    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_set(IC_SRCLK_P0_8);
    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_clear(IC_SRCLK_P0_8);
  }
  if (boot) {
    // Set RCLK high/low to write the shift register
    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_set(IC_RCLK_P0_9);
    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_clear(IC_RCLK_P0_9);
  }
}

uint8_t turnLedOnOff(uint8_t led, uint8_t onOff) {
  NRF_LOG_INFO("turnLedOnOff: led=%d, on/off=%d", led, onOff);
  if (led < LED_START || led > LED_STOP) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }

  clearOutputPins(false);
  nrf_gpio_pin_clear(IC_SRCLK_P0_8);
  nrf_gpio_pin_clear(IC_RCLK_P0_9);
  nrf_gpio_pin_clear(IC_SER_P0_10);

  for (uint8_t i = 0; i < 16; i++) {
    if (pinOutputMap[i][MP_PORT] >= MP_FIRST_USB_PORT_NUMBER && pinOutputMap[i][MP_PORT] <= MP_MAX_USB_PORT_NUMBER) {
      uint8_t status;

      if (getPortStatus(pinOutputMap[i][MP_PORT], &status) != MP_SUCCESS)
        break;
      if (status == MP_FREE_CHARGE || status == MP_ACTIVE_CHARGE) {
        // Set SER high
        pinOutputMap[i][MP_STATE] = MP_POWER_ON;
      }
      else {
        // Set SER low
        pinOutputMap[i][MP_STATE] = MP_POWER_OFF;
      }
    }
    else if (pinOutputMap[i][MP_PORT] >= LED_START && pinOutputMap[i][MP_PORT] <= LED_STOP) {
      if (pinOutputMap[i][MP_PORT] == led) {
        pinOutputMap[i][MP_STATE] = onOff;
      }
      else {
        // TODO: get LED state
        //pinOutputMap[i][MP_STATE] = getLedState(led);
      }
    }
    //NRF_LOG_INFO("turnLedOnOff: pin=%d, on/off=%d", i, pinOutputMap[i][MP_STATE]);
    if (pinOutputMap[i][MP_STATE] == MP_POWER_ON){
      nrf_delay_ms(MP_GPIO_DELAY);
      nrf_gpio_pin_set(IC_SER_P0_10);
    }

    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_set(IC_SRCLK_P0_8);
    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_clear(IC_SRCLK_P0_8);

    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_clear(IC_SER_P0_10);
  }

  nrf_delay_ms(MP_GPIO_DELAY);
  nrf_gpio_pin_set(IC_RCLK_P0_9);
  nrf_delay_ms(MP_GPIO_DELAY);
  nrf_gpio_pin_clear(IC_RCLK_P0_9);

  return MP_SUCCESS;
}

uint8_t turnPowerOnOff(uint8_t port, uint8_t onOff) {
  //NRF_LOG_INFO("turnPowerOnOff: port=%d, on/off=%d", port, onOff);
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }

  clearOutputPins(false);
  nrf_gpio_pin_clear(IC_SRCLK_P0_8);
  nrf_gpio_pin_clear(IC_RCLK_P0_9);
  nrf_gpio_pin_clear(IC_SER_P0_10);

  for (uint8_t i = 0; i < 16; i++) {
    if (pinOutputMap[i][MP_PORT] == port) {
      pinOutputMap[i][MP_STATE] = onOff;
    }
    else if (pinOutputMap[i][MP_PORT] >= MP_FIRST_USB_PORT_NUMBER && pinOutputMap[i][MP_PORT] <= MP_MAX_USB_PORT_NUMBER){
      uint8_t status;

      if (getPortStatus(pinOutputMap[i][MP_PORT], &status) != MP_SUCCESS)
        break;
      if (status == MP_FREE_CHARGE || status == MP_ACTIVE_CHARGE) {
        // Set SER high
        pinOutputMap[i][MP_STATE] = MP_POWER_ON;
      } else {
        // Set SER low
        pinOutputMap[i][MP_STATE] = MP_POWER_OFF;
      }
    }
    else if (pinOutputMap[i][MP_PORT] >= LED_START && pinOutputMap[i][MP_PORT] <= LED_STOP) {
      // TODO: get LED state
      //pinOutputMap[i][MP_STATE] = getLedState(led);
    }
    //NRF_LOG_INFO("turnPowerOnOff: pin=%d, on/off=%d", i, pinOutputMap[i][MP_STATE]);
    if (pinOutputMap[i][MP_STATE] == MP_POWER_ON){
      nrf_delay_ms(MP_GPIO_DELAY);
      nrf_gpio_pin_set(IC_SER_P0_10);
    }

    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_set(IC_SRCLK_P0_8);
    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_clear(IC_SRCLK_P0_8);

    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_clear(IC_SER_P0_10);
  }

  nrf_delay_ms(MP_GPIO_DELAY);
  nrf_gpio_pin_set(IC_RCLK_P0_9);
  nrf_delay_ms(MP_GPIO_DELAY);
  nrf_gpio_pin_clear(IC_RCLK_P0_9);

  return MP_SUCCESS;
}

uint8_t freeUsbPort(uint8_t port) {
  //turn off power of USB port and send BLE notification
  uint8_t status;
  uint16_t connHandle;
  uint16_t charHandle = m_ble_mp.alert_char_handles.value_handle; // Should be alert charact4eristic
  uint32_t alertMsg;
  uint8_t len = 4;
  uint8_t ret = MP_SUCCESS;

  NRF_LOG_INFO("timer: stop charging port=%d", port);
  ret = turnPowerOnOff(port, MP_POWER_OFF);

  ret = getConnHandle(port, &connHandle);
  if (ret == MP_SUCCESS) {
    if (connHandle != BLE_CONN_HANDLE_INVALID) {
      // send USB port disconnect
      alertMsg = ((port << 8) | MP_USB_PORT_CHARGING_STOPPED) << 8 | m_ble_mp.location_id;
      sendNotification(charHandle, connHandle, &alertMsg, len);
    }
  } else {
    NRF_LOG_INFO("freeUsbPort: getConnHandle failed withe error code %d", ret);
  }

  ret = getPortStatus(port, &status);
  if (ret == MP_SUCCESS && status == MP_FREE_CHARGE)
    initPortStatus(port, MP_QUARANTINE, MP_QUARANTINE_TIME, BLE_CONN_HANDLE_INVALID);
  else
    initPortStatus(port, MP_AVAILABLE, 0, BLE_CONN_HANDLE_INVALID);
}

void resetInputShiftRegister() {
  // Reset to start on 'Output 1 active' - set P0.19 low - high 
  nrf_delay_ms(MP_GPIO_DELAY);
  nrf_gpio_pin_clear(SHIFT_REGISTER_P0_19);
  nrf_delay_ms(MP_GPIO_DELAY);
  nrf_gpio_pin_set(SHIFT_REGISTER_P0_19);
  // Repeat sequence n times to ensure reset is done 
  nrf_delay_ms(MP_GPIO_DELAY);
  nrf_gpio_pin_clear(SHIFT_REGISTER_P0_19);
  nrf_delay_ms(MP_GPIO_DELAY);
  nrf_gpio_pin_set(SHIFT_REGISTER_P0_19);
  nrf_delay_ms(MP_GPIO_DELAY);
  nrf_gpio_pin_clear(SHIFT_REGISTER_P0_19);
  nrf_delay_ms(MP_GPIO_DELAY);
  nrf_gpio_pin_set(SHIFT_REGISTER_P0_19);
}

void toggleInputShiftRegister(uint8_t port) {
  resetInputShiftRegister();
  for (uint8_t i = 0; i < 24; i++) {
    // Toggle P0.18 high/low port-1 times to select port
    if (pinInputMap[i][MP_PORT] == port)
      break;
    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_set(SHIFT_REGISTER_P0_18);
    nrf_delay_ms(MP_GPIO_DELAY);
    nrf_gpio_pin_clear(SHIFT_REGISTER_P0_18);
  }
}

// Check "Output X fault"
uint8_t checkUsbPortFault(uint8_t port) {
//  for (uint8_t i = 0; i < 24; i++) {
//    // Toggle P0.18 high/low port-1 times to select port
//    if (pinInputMap[i][MP_PORT] == port + MP_OUTPUT_X_FAULT)
//      break;
//    nrf_delay_ms(MP_GPIO_DELAY);
//    nrf_gpio_pin_set(SHIFT_REGISTER_P0_18);
//    nrf_delay_ms(MP_GPIO_DELAY);
//    nrf_gpio_pin_clear(SHIFT_REGISTER_P0_18);
//  }
  toggleInputShiftRegister(port + MP_OUTPUT_X_FAULT);
  nrf_delay_ms(1);
  uint8_t fault = nrf_gpio_pin_read(SHIFT_REGISTER_P0_20);
  // Returns 1 if "Output X fault", else 0
  return fault;
}

int8_t readUsbPortState(uint8_t port) {
  int8_t state = 0;

  uint8_t fault = checkUsbPortFault(port);
  if (fault == 0) {
    toggleInputShiftRegister(port);
    nrf_delay_ms(1);
    // Returns 0 (low) if "Output X active" is not active, and 1 (high) if active
    state = nrf_gpio_pin_read(SHIFT_REGISTER_P0_20);
    //if (state != 0)
    //  NRF_LOG_INFO("readUsbPortState: port %d Output_X_active", port);
  } else {
    //NRF_LOG_INFO("readUsbPortState: port %d Output_X_fault", port);
    // Output X fault, return -1
    state = MP_STATE_OUTPUT_X_FAULT;
  }
  return state;
}

static uint8_t pollCounter = 0;

void _checkUsbPorts() {
  // Check if any USB ports have used up free charging time / charging time
  UsbPortStatus status;
  int16_t ticks = 0;
  uint8_t ret;

  // Check if any USB ports should be freed because a timeout periode has been reached.
  // A timeout periode can be
  //    - a paid charge time periode
  //    - a free charge time periode
  //    - a quarantine time periode
  for (uint8_t port = MP_FIRST_USB_PORT_NUMBER; port <= MP_MAX_USB_PORT_NUMBER; port++) {
    ret = decrementChargingTicks(port, &ticks);
    if (ret == MP_SUCCESS && ticks < 0) {
      freeUsbPort(port);
    }
  }

  // Check USB port state each n seconds
  if (pollCounter++ == 2) {
    pollCounter = 0;
    for (uint8_t port = MP_FIRST_USB_PORT_NUMBER; port <= MP_MAX_USB_PORT_NUMBER; port++) {
      uint8_t ret = getPortStatus(port, &status);

      // MUST turn power off to read port state (Output_X_Active)
      turnPowerOnOff(port, MP_POWER_OFF);

      int8_t state = readUsbPortState(port);
      if (state != MP_STATE_OUTPUT_X_FAULT) {
        setState(port, state);

        switch (status) {
        case MP_AVAILABLE:
          if (state == MP_OUTPUT_X_ACTIVE) {
            // Turn power on for a free charge periode
            initPortStatus(port, MP_FREE_CHARGE, MP_FREE_CHARGE_TIME, BLE_CONN_HANDLE_INVALID);
            turnPowerOnOff(port, MP_POWER_ON);
            getRamainingChargeTicks(port, &ticks);
            NRF_LOG_INFO("checkUsbPorts: port %d MP_AVAILABLE, device connected", port);
          }
          break;
        case MP_ACTIVE_CHARGE:
          getRamainingChargeTicks(port, &ticks);
          if (state == MP_OUTPUT_X_ACTIVE) {
            NRF_LOG_INFO("checkUsbPorts: port %d MP_ACTIVE_CHARGE, device connected, %d seconds left", port, ticks);
            turnPowerOnOff(port, MP_POWER_ON);
          } else {
            NRF_LOG_INFO("checkUsbPorts: port %d MP_ACTIVE_CHARGE, device disconnected, %d seconds left", port, ticks);
            //freeUsbPort(port);
          }
          break;
        case MP_FREE_CHARGE:
          getRamainingChargeTicks(port, &ticks);
          if (state == MP_OUTPUT_X_ACTIVE) {
            NRF_LOG_INFO("checkUsbPorts: port %d MP_FREE_CHARGE, device connected, %d seconds left", port, ticks);
          } else {
            NRF_LOG_INFO("checkUsbPorts: port %d MP_FREE_CHARGE, device disconnected, %d seconds left", port, ticks);
            freeUsbPort(port);
          }
          break;
        case MP_QUARANTINE:
          getRamainingChargeTicks(port, &ticks);
          NRF_LOG_INFO("checkUsbPorts: port %d MP_QUARANTINE, %d seconds left", port, ticks);
          break;
        default:
          break;
        }
      } else {
        NRF_LOG_INFO("checkUsbPorts: port %d Output_X_Fault", port);
      }
    }
  }

}

uint8_t getStateFromInputMap(port) {
  for (uint8_t i = 0; i < 24; i++) {
   if (pinInputMap[i][MP_PORT] == port)
     return pinInputMap[i][MP_STATE];
  }
  return 0;
}

void checkUsbPorts() {
  UsbPortStatus status;
  int16_t ticks;
  uint8_t ret;

  // Check if any USB ports should be freed because a timeout periode has been reached.
  // A timeout periode can be
  //    - a paid charge time periode
  //    - a free charge time periode
  //    - a quarantine time periode
  for (uint8_t port = MP_FIRST_USB_PORT_NUMBER; port <= MP_MAX_USB_PORT_NUMBER; port++) {
    ret = decrementChargingTicks(port, &ticks);
    if (ret == MP_SUCCESS && ticks < 0) {
      freeUsbPort(port);
    }
  }

  if (pollCounter++ == 4) {
    uint16_t ticks;
    pollCounter = 0;

    // Read out the current port states (Output_X_Active)
    resetInputShiftRegister();
    for (uint8_t i = 0; i < 24; i++) {
      uint8_t port = pinInputMap[i][MP_PORT];
      
      if (port >= MP_FIRST_USB_PORT_NUMBER && port <= MP_MAX_USB_PORT_NUMBER) {
        // MUST turn power off to read state (Output_X_Active)
        turnPowerOnOff(port, MP_POWER_OFF);
      }
      
      // Returns 0 (low), if "Output X active" is not activated, and 1 (high) if activated
      pinInputMap[i][MP_STATE] = nrf_gpio_pin_read(SHIFT_REGISTER_P0_20);
      
      // Toggle P0.18 high/low to select next port
      nrf_delay_ms(MP_GPIO_DELAY);
      nrf_gpio_pin_set(SHIFT_REGISTER_P0_18);
      nrf_delay_ms(MP_GPIO_DELAY);
      nrf_gpio_pin_clear(SHIFT_REGISTER_P0_18);
    }

    for (uint8_t port = MP_FIRST_USB_PORT_NUMBER; port <= MP_MAX_USB_PORT_NUMBER; port++) {
      uint8_t ret = getPortStatus(port, &status);
      // Check if port has fault (Output_X_Fault)
      ret = getStateFromInputMap(port + MP_OUTPUT_X_FAULT);
      if (ret == 0) {
        int8_t state = getStateFromInputMap(port);
        setState(port, state);

        switch (status) {
        case MP_AVAILABLE:
          if (state == MP_OUTPUT_X_ACTIVE) {
            // Turn power on for a free charge periode
            initPortStatus(port, MP_FREE_CHARGE, MP_FREE_CHARGE_TIME, BLE_CONN_HANDLE_INVALID);
            turnPowerOnOff(port, MP_POWER_ON);
            getRamainingChargeTicks(port, &ticks);
            NRF_LOG_INFO("checkUsbPorts: port %d MP_AVAILABLE, device connected", port);
          }
          break;
        case MP_ACTIVE_CHARGE:
          getRamainingChargeTicks(port, &ticks);
          if (state == MP_OUTPUT_X_ACTIVE) {
            NRF_LOG_INFO("checkUsbPorts: port %d MP_ACTIVE_CHARGE, device connected, %d seconds left", port, ticks);
            turnPowerOnOff(port, MP_POWER_ON);
          } else {
            NRF_LOG_INFO("checkUsbPorts: port %d MP_ACTIVE_CHARGE, device disconnected, %d seconds left", port, ticks);
            //freeUsbPort(port);
          }
          break;
        case MP_FREE_CHARGE:
          getRamainingChargeTicks(port, &ticks);
          if (state == MP_OUTPUT_X_ACTIVE) {
            NRF_LOG_INFO("checkUsbPorts: port %d MP_FREE_CHARGE, device connected, %d seconds left", port, ticks);
          } else {
            NRF_LOG_INFO("checkUsbPorts: port %d MP_FREE_CHARGE, device disconnected, %d seconds left", port, ticks);
            freeUsbPort(port);
          }
          break;
        case MP_QUARANTINE:
          getRamainingChargeTicks(port, &ticks);
          NRF_LOG_INFO("checkUsbPorts: port %d MP_QUARANTINE, %d seconds left", port, ticks);
          break;
        default:
          break;
        }
      } else {
        NRF_LOG_INFO("checkUsbPorts: port %d Output_X_Fault", port);
      }
    }
  }

}


void onNewCommand(ble_evt_t const *p_ble_evt) {
  // Write event - decode the data set by client:
  // 1st byte: command - 0=off, 1=on
  // 2nd byte: port number
  //           legal port number is: 1 - MAX_USB_PORT_NUMBER

  //ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

  uint16_t connHandle = p_ble_evt->evt.gap_evt.conn_handle;
  uint16_t charHandle = m_ble_mp.alert_char_handles.value_handle;
  uint8_t *pData = (uint8_t *)p_ble_evt->evt.gatts_evt.params.write.data;
  uint16_t length = p_ble_evt->evt.gatts_evt.params.write.len;
  uint8_t command = 0xff;
  uint8_t port = MP_CHOOSE_AVAILABLE_PORT;
  uint32_t ackMsg = 0;
  uint8_t len = 4;
  uint8_t ret = MP_SUCCESS;
  UsbPortStatus status;

  NRF_LOG_INFO("onNewCommand() enter");

  if (length >= 2) {
    command = pData[0];
    port = pData[1];
    if (port == 0)
      port = MP_CHOOSE_AVAILABLE_PORT;
    NRF_LOG_INFO("data received: length=%d, command=0x%x, port=0x%x", length, command, port);
  } else if (length == 1) {
    command = pData[0];
    port = MP_CHOOSE_AVAILABLE_PORT;

    NRF_LOG_INFO("data received: length=%0x, command=0x%x, port=allocate", length, command);
  } else {
    NRF_LOG_INFO("data received: length=%d, illegal command=0x%x", length, command);
    ackMsg = ((MP_ILLEGAL_COMMAND << 8) | MP_ERROR) << 8 | m_ble_mp.location_id;
    sendNotification(charHandle, connHandle, &ackMsg, len);
    return;
  }

  if (port == MP_CHOOSE_AVAILABLE_PORT) {
    ret = allocateFreePort(&port);
  }

  if (ret != MP_SUCCESS) {
    NRF_LOG_INFO("No available port")
    ackMsg = ((MP_ERROR_NO_AVAILABLE_PORT << 8) | MP_ERROR) << 8 | m_ble_mp.location_id;
    sendNotification(charHandle, connHandle, &ackMsg, len);
    return;
  }

  ret = getPortStatus(port, &status);

  if (ret != MP_SUCCESS) {
    NRF_LOG_INFO("Get status on port 0x%x failed, %s", port, ret)
  } else {
    if (command == MP_TURN_USB_POWER_ON) {
      NRF_LOG_INFO("Turn power ON on port 0x%x", port);

      if (status != MP_AVAILABLE && status != MP_FREE_CHARGE) {
        NRF_LOG_INFO("Illegal port status, status=%d", status)
        ret = MP_ERROR_ILLEGAL_PORT_STATUS;
      } else {
        ret = turnPowerOnOff(port, MP_POWER_ON);

        // update port status
        // TODO CHANGE TIME TO ACTUAL CHARGE TIME
        ret = initPortStatus(port, MP_ACTIVE_CHARGE, MP_CHARGE_TIME, connHandle);
      }

    } else if (command == MP_TURN_USB_POWER_OFF) {
      NRF_LOG_INFO("Turn power OFF on port 0x%x", port);
      ret = turnPowerOnOff(port, MP_POWER_OFF);

      // update port status
      //ret = initPortStatus(port, MP_AVAILABLE, 0, connHandle);
      ret = freeUsbPort(port);

    } else {
      NRF_LOG_INFO("Illegal command 0x%x", command);
      ret = MP_ILLEGAL_COMMAND;
    }
  }

  // Send ack/nack message
  if (ret == MP_SUCCESS) {
    // send ack
    NRF_LOG_INFO("Send ack port=0x%x", port);
    ackMsg = (port << 8) | MP_SUCCESS;
  } else {
    // send nack
    NRF_LOG_INFO("Send nack port=0x%x, ret=0x%x", port, ret);
    ackMsg = (ret << 8) | MP_ERROR;
  }
  ackMsg = (ackMsg << 8) | m_ble_mp.location_id;
  sendNotification(charHandle, connHandle, &ackMsg, len);

  // TODO testStuff status
  ret = getPortStatus(port, &status);
  NRF_LOG_INFO("Port status is %x", status);

  NRF_LOG_INFO("onNewCommand() leave");
}

void _onUsbPortChange(uint8_t pin, uint8_t action) {
  // TODO
  // Routine is currently disabled (nor called)
  // Could poll port states as in checkUsbPorts()
  uint16_t connHandle = NULL;
  uint16_t charHandle = m_ble_mp.alert_char_handles.value_handle;
  uint32_t ackMsg = 0;
  uint8_t len = 4;
  uint8_t ret = MP_SUCCESS;
  UsbPortStatus status;

  NRF_LOG_INFO("onUsbPortChange() enter, pin=%0x, action=%0x", pin, action);


  switch(action) {
  case NRF_GPIOTE_POLARITY_LOTOHI:
    break;

  case NRF_GPIOTE_POLARITY_HITOLO:
    break;

  case NRF_GPIOTE_POLARITY_TOGGLE:
  default:
    break;
  }

  NRF_LOG_INFO("onUsbPortChange() leave");
}

void onBleDisconnect(ble_evt_t const *p_ble_evt) {
  // Remvove connection handle from UsbPort
  for (int i = MP_FIRST_USB_PORT_NUMBER; i <= MP_MAX_USB_PORT_NUMBER; i++) {
    if (usbPorts[i]->connHandle == p_ble_evt->evt.gap_evt.conn_handle) {
      usbPorts[i]->connHandle = BLE_CONN_HANDLE_INVALID;
    }
  }
}