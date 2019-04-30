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

//Pins 1-16: p8, p7, p6, p5, p4, p3, p2, p1, status_led1, status_led2, status_led3, power1, power2, power3, p10, p9.
static uint8_t pin_map[16][2] = {  // [USB port number][on/off]
  {8, 0}, {7, 0}, {6, 0}, {5, 0}, {4, 0}, {3, 0}, {2, 0}, {1, 0},
  {LED_1, 0}, {LED_2, 0}, {LED_3, 0}, {0, 0}, {0, 0}, {0, 0}, {10, 0}, {9, 0}
};


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
    if (usbPorts[i]->status == MP_AVAILABLE) {
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
    if (pin_map[i][0] >= MP_FIRST_USB_PORT_NUMBER && pin_map[i][0] <= MP_MAX_USB_PORT_NUMBER){
      uint8_t _status;

      if (getPortStatus(pin_map[i][0], &_status) != MP_SUCCESS)
        break;
      if (_status == MP_FREE_CHARGE || _status == MP_ACTIVE_CHARGE) {
        // Set SER high
        pin_map[i][1] = MP_POWER_ON;
      }
      else {
        // Set SER low
        pin_map[i][1] = MP_POWER_OFF;
      }
    }
    else if (pin_map[i][0] >= LED_START && pin_map[i][0] <= LED_STOP) {
      if (pin_map[i][0] == led) {
        pin_map[i][1] = onOff;
      }
      else {
        // TODO: get LED state
        //pin_map[i][1] = getLedState(led);
      }
    }
    //NRF_LOG_INFO("turnLedOnOff: pin=%d, on/off=%d", i, pin_map[i][1]);
    if (pin_map[i][1] == MP_POWER_ON){
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
  NRF_LOG_INFO("turnPowerOnOff: port=%d, on/off=%d", port, onOff);
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }

  clearOutputPins(false);
  nrf_gpio_pin_clear(IC_SRCLK_P0_8);
  nrf_gpio_pin_clear(IC_RCLK_P0_9);
  nrf_gpio_pin_clear(IC_SER_P0_10);

  for (uint8_t i = 0; i < 16; i++) {
    if (pin_map[i][0] == port) {
      pin_map[i][1] = onOff;
    }
    else if (pin_map[i][0] >= MP_FIRST_USB_PORT_NUMBER && pin_map[i][0] <= MP_MAX_USB_PORT_NUMBER){
      uint8_t _status;

      if (getPortStatus(pin_map[i][0], &_status) != MP_SUCCESS)
        break;
      if (_status == MP_FREE_CHARGE || _status == MP_ACTIVE_CHARGE) {
        // Set SER high
        pin_map[i][1] = MP_POWER_ON;
      } else {
        // Set SER low
        pin_map[i][1] = MP_POWER_OFF;
      }
    }
    else if (pin_map[i][0] >= LED_START && pin_map[i][0] <= LED_STOP) {
      // TODO: get LED state
      //pin_map[i][1] = getLedState(led);
    }
    //NRF_LOG_INFO("turnPowerOnOff: pin=%d, on/off=%d", i, pin_map[i][1]);
    if (pin_map[i][1] == MP_POWER_ON){
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
  initPortStatus(port, MP_AVAILABLE, 0, BLE_CONN_HANDLE_INVALID);
}

void resetInputShiftRegister() {
  // Reset to start on 'Output 1 active' - set P0.19 low - high 
  nrf_delay_ms(1);
  nrf_gpio_pin_clear(SHIFT_REGISTER_P0_19);
  nrf_delay_ms(1);
  nrf_gpio_pin_set(SHIFT_REGISTER_P0_19);
  // Repeat sequence to ensure reset is done 
    nrf_delay_ms(1);
  nrf_gpio_pin_clear(SHIFT_REGISTER_P0_19);
  nrf_delay_ms(1);
  nrf_gpio_pin_set(SHIFT_REGISTER_P0_19);
  nrf_delay_ms(1);
  nrf_gpio_pin_clear(SHIFT_REGISTER_P0_19);
  nrf_delay_ms(1);
  nrf_gpio_pin_set(SHIFT_REGISTER_P0_19);
}

void toggleInputShiftRegister(uint8_t port) {
  resetInputShiftRegister();
  for (uint8_t i = 1; i < port; i++) {
    // Toggle P0.18 high/low port-1 times to select port
    nrf_delay_ms(1);
    nrf_gpio_pin_set(SHIFT_REGISTER_P0_18);
    nrf_delay_ms(1);
    nrf_gpio_pin_clear(SHIFT_REGISTER_P0_18);
  }
}

// Check "Output X fault"
uint8_t checkUsbPortFault(uint8_t port) {
  //resetInputShiftRegister();
  toggleInputShiftRegister(port+10);
  uint8_t fault_status = nrf_gpio_pin_read(SHIFT_REGISTER_P0_20);
  // Returns 0 if "Output X fault" not has fault.
  return fault_status;
}

int8_t readUsbPortStatus(uint8_t port) {
  int8_t port_status = 0;

  //resetInputShiftRegister();
  port_status = checkUsbPortFault(port);
  if (port_status == 0) {
    toggleInputShiftRegister(port);
    // Returns 0 (low), if "Output X active" is not activated, and 1 (high) if activated
    nrf_delay_ms(1);
    uint8_t pin_status = nrf_gpio_pin_read(SHIFT_REGISTER_P0_20);
  } else {
    NRF_LOG_INFO("readUsbPortStatus fault: port %d - status %d", port, port_status);
    // Output X fault, return -1
    port_status = -1;
  }
  return port_status;
}

static uint8_t poll_counter = 0;

void checkUsbPorts() {
  // Check if any USB ports have used up free charging time / charging time
  uint8_t ret;
  int16_t ticks;

  for (uint8_t port = MP_FIRST_USB_PORT_NUMBER; port <= MP_MAX_USB_PORT_NUMBER; port++) {
    ret = decrementChargingTicks(port, &ticks);
    if (ret == MP_SUCCESS && ticks < 0) {
      freeUsbPort(port);
    }
  }

  // Just for testing
  //turnLedOnOff(LED_1, poll_counter % 2);

  if (poll_counter++ == 2) {
    poll_counter = 0;
    for (uint8_t port = MP_FIRST_USB_PORT_NUMBER; port <= MP_MAX_USB_PORT_NUMBER; port++) {
      int8_t port_status = readUsbPortStatus(port);
      //NRF_LOG_INFO("checkUsbPorts: port %d - status %d", port, port_status);
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
  UsbPortStatus portStatus;

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

  ret = getPortStatus(port, &portStatus);

  if (ret != MP_SUCCESS) {
    NRF_LOG_INFO("Get status on port 0x%x failed, %s", port, ret)
  } else {
    if (command == MP_TURN_USB_POWER_ON) {
      NRF_LOG_INFO("Turn power ON on port 0x%x", port);

      if (portStatus != MP_AVAILABLE && portStatus != MP_FREE_CHARGE) {
        NRF_LOG_INFO("Illegal port status, portstatus=%d", portStatus)
        ret = MP_ERROR_ILLEGAL_PORT_STATUS;
      } else {
        ret = turnPowerOnOff(port, MP_POWER_ON);

        // update port status
        // TODO CHANGE TIME TO ACTUAL CHARGE TIME
        ret = initPortStatus(port, MP_ACTIVE_CHARGE, MP_TEST_TIME, connHandle);
      }

    } else if (command == MP_TURN_USB_POWER_OFF) {
      NRF_LOG_INFO("Turn power OFF on port 0x%x", port);
      ret = turnPowerOnOff(port, MP_POWER_OFF);

      // update port status
      // TODO CHANGE TIME TO ACTUAL CHARGE TIME
      //ret = initPortStatus(port, MP_AVAILABLE, MP_TEST_TIME, connHandle);
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
  ret = getPortStatus(port, &portStatus);
  NRF_LOG_INFO("Port status is %x", portStatus);

  NRF_LOG_INFO("onNewCommand() leave");
}

void _onUsbPortChange(uint8_t port, uint8_t action) {
  //TODO add USB status, ack message, ports, turn off power, turn on power, change USB status, get USB status
  uint16_t connHandle = NULL;
  uint16_t charHandle = m_ble_mp.alert_char_handles.value_handle;
  uint32_t ackMsg = 0;
  uint8_t len = 4;
  uint8_t ret = MP_SUCCESS;
  UsbPortStatus portStatus;

  NRF_LOG_INFO("onUsbChange() enter, port=%0x, action=%0x", port, action);

  ret = getPortStatus(port, &portStatus);

  if (ret != MP_SUCCESS) {
    NRF_LOG_INFO("Get port status failed, %s", ret)
  } else {
    ret = getConnHandle(port, &connHandle);

    switch(action) {
    case NRF_GPIOTE_POLARITY_LOTOHI:
      switch(portStatus) {
      case MP_AVAILABLE:
        // Connected - activate free charging
        ret = turnPowerOnOff(port, MP_POWER_ON);
        ret = initPortStatus(port, MP_FREE_CHARGE, MP_TEST_TIME, connHandle);
        NRF_LOG_INFO("Power LOTOHI on port %d ON - MP_AVAILABLE->MP_FREE_CHARGE", port);
        break;

      case MP_ACTIVE_CHARGE:
        // Connected - already paid
        ret = turnPowerOnOff(port, MP_POWER_ON);
        NRF_LOG_INFO("Power LOTOHI on port %d ON - MP_AVAILABLE->MP_FREE_CHARGE", port);
        break;

      case MP_FREE_CHARGE:
        // Illegal state
        NRF_LOG_INFO("Power LOTOHI on port %d ON - MP_FREE_CHARGE", port);
        break;

      case MP_FREE_CHARGE_NOT_AVAILABLE:
      default:
        // Free port
        ret = turnPowerOnOff(port, MP_POWER_OFF);
        NRF_LOG_INFO("Power LOTOHI on port %d ON - MP_FREE_CHARGE_NOT_AVAILABLE", port);
        break;
      }
      break;

    case NRF_GPIOTE_POLARITY_HITOLO:
      switch(portStatus) {
      case MP_AVAILABLE:
        // Illegal state
        break;

      case MP_ACTIVE_CHARGE:
        // Disconnected - paid for
        break;

      case MP_FREE_CHARGE:
        // Disconnected - not paid for
        // Should block port for some time to prevent misuse
        break;

      case MP_FREE_CHARGE_NOT_AVAILABLE:
      default:
        // Free blocking
        break;
      }
      break;

    case NRF_GPIOTE_POLARITY_TOGGLE:
    default:
      switch(portStatus) {
      case MP_AVAILABLE:
        // Going from idle to active
        break;

      case MP_ACTIVE_CHARGE:
        // Disconnected - paid for
        break;

      case MP_FREE_CHARGE:
        // Disconnected - not paid for
        // Should block port for some time to prevent misuse
        break;

      case MP_FREE_CHARGE_NOT_AVAILABLE:
      default:
        // Free blocking
        break;
      }

      break;
    }
  }

  // Send ack/nack message
  if (ret == MP_SUCCESS) {
    // send ack
    ackMsg = (port << 8) | MP_SUCCESS;
  } else {
    // send nack
    ackMsg = (ret << 8) | MP_ERROR;
  }

  NRF_LOG_INFO("onUsbChange() leave");
}

void onUsbPortChange(uint8_t port, uint8_t action) {
  //TODO add USB status, ack message, ports, turn off power, turn on power, change USB status, get USB status
  uint16_t connHandle = NULL;
  uint16_t charHandle = m_ble_mp.alert_char_handles.value_handle;
  uint32_t ackMsg = 0;
  uint8_t len = 4;
  uint8_t ret = MP_SUCCESS;
  UsbPortStatus portStatus;

  NRF_LOG_INFO("onUsbChange() enter, port=%0x, action=%0x", port, action);

  ret = getPortStatus(port, &portStatus);

  if (ret != MP_SUCCESS) {
    NRF_LOG_INFO("Get port status failed, %s", ret)
  } else {
    ret = getConnHandle(port, &connHandle);

    // update port status
    // TODO CHANGE TIME TO ACTUAL CHARGE TIME
    if (portStatus == MP_AVAILABLE) {
      NRF_LOG_INFO("Turn power ON on port %x", port);
      ret = turnPowerOnOff(port, MP_POWER_ON);

      ret = initPortStatus(port, MP_FREE_CHARGE, MP_TEST_TIME, connHandle);
      NRF_LOG_INFO("Set portStatus to MP_FREE_CHARGE");

    } else if (portStatus == MP_ACTIVE_CHARGE) {
      NRF_LOG_INFO("Turn power OFF on port %x", port);

      ret = turnPowerOnOff(port, MP_POWER_OFF);

      ret = freeUsbPort(port);
      NRF_LOG_INFO("Set portStatus to MP_AVAILABLE");

    } else if (portStatus == MP_FREE_CHARGE) {
      NRF_LOG_INFO("Turn power OFF on port %x", port);

      ret = turnPowerOnOff(port, MP_POWER_OFF);

      ret = freeUsbPort(port);
      NRF_LOG_INFO("Set portStatus to MP_FREE_CHARGE_NOT_AVAILABLE");
    }
  }
  // Send ack/nack message
  if (ret == MP_SUCCESS) {
    // send ack
    ackMsg = (port << 8) | MP_SUCCESS;
  } else {
    // send nack
    ackMsg = (ret << 8) | MP_ERROR;
  }
  ackMsg = (ackMsg << 8) | m_ble_mp.location_id;

  if (connHandle != NULL) {
    sendNotification(charHandle, connHandle, &ackMsg, len);
  } else {
    ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_periph_handles();
    // TODO: Send endring i port-status til alle som er oppkoblet
    for (uint8_t i = 0; i < conn_handles.len; i++) {
      sendNotification(charHandle, conn_handles.conn_handles[i], &ackMsg, len);
    }
  }
  // TODO testStuff status
  ret = getPortStatus(port, &portStatus);
  NRF_LOG_INFO("Port status is %x", portStatus);

  NRF_LOG_INFO("onUsbChange() leave");
}

void onBleDisconnect(ble_evt_t const *p_ble_evt) {
  // Remvove connection handle from UsbPort
  for (int i = MP_FIRST_USB_PORT_NUMBER; i <= MP_MAX_USB_PORT_NUMBER; i++) {
    if (usbPorts[i]->connHandle == p_ble_evt->evt.gap_evt.conn_handle) {
      usbPorts[i]->connHandle = BLE_CONN_HANDLE_INVALID;
    }
  }
}