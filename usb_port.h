#ifndef OUR_PORT_H
#define OUR_PORT_H

#include "nrf_drv_gpiote.h"


typedef enum {
  MP_AVAILABLE = 0,
  MP_FREE_CHARGE,              // Port busy but not yet paid for
  MP_ACTIVE_CHARGE,            // Port active; port busy and paid for
  MP_FREE_CHARGE_NOT_AVAILABLE // port busy; free charging time expired
} UsbPortStatus;

typedef struct {
  UsbPortStatus status;
  int16_t remainingChargeTicks;
  int16_t notAvailableTicks;
  uint16_t connHandle;
} UsbPort;

/** MP_COMMAND_xx Command definitions
 **/
#define MP_COMMAND_BASE_NUM (0x0) /// Command base
#define MP_TURN_USB_POWER_OFF (MP_COMMAND_BASE_NUM + 0)
#define MP_TURN_USB_POWER_ON (MP_COMMAND_BASE_NUM + 1)
#define MP_CHOOSE_AVAILABLE_PORT (MP_COMMAND_BASE_NUM + 255)

/** MP_ALERT_xx Alert definitions
 **/
#define MP_ALERT_BASE_NUM (0x0) /// Alert base
#define MP_USB_PORT_CONNECTED         (MP_ALERT_BASE_NUM + 2) 
#define MP_USB_PORT_DISCONNECTED      (MP_ALERT_BASE_NUM + 3) 
#define MP_USB_PORT_CHARGING_STARTED  (MP_ALERT_BASE_NUM + 4) 
#define MP_USB_PORT_CHARGING_STOPPED  (MP_ALERT_BASE_NUM + 5) 
#define MP_PORT_STATUS_ALERT          (MP_ALERT_BASE_NUM + 6)

/** MP_ERRORS_xx Error Codes definitions
 **/
#define MP_ERROR_BASE_NUM (0xE0) /// Error base
#define MP_SUCCESS (0)
#define MP_ERROR (1)
#define MP_ILLEGAL_COMMAND (MP_ERROR_BASE_NUM + 2)
#define MP_ERROR_ILLEGAL_PORT_NUMBER (MP_ERROR_BASE_NUM + 3)
#define MP_ERROR_ILLEGAL_PORT_STATUS (MP_ERROR_BASE_NUM + 4)
#define MP_ERROR_NO_AVAILABLE_PORT (MP_ERROR_BASE_NUM + 5)

//#define MAX_PORT 10
#define MP_FIRST_USB_PORT_NUMBER 1
#define MP_MAX_USB_PORT_NUMBER 4

//TODO Test data
#define MP_TEST_TIME                        (0.5*60)  // 6 sek
#define MP_MAX_CHARGE_TIME                  (MP_TEST_TIME)//(60*60) // 1 time
#define MP_MAX_FREE_TIME                    (MP_TEST_TIME)//(5*60)  // 5 min
#define MP_FREE_CHARGE_NOT_AVAILABLE_TIME   (MP_TEST_TIME)//(60*60) // 1 time

#define MP_POWER_ON 1
#define MP_POWER_OFF 0

void initUsbPorts();
uint8_t getPort(uint8_t port, UsbPort *data);
uint8_t initPortStatus(uint8_t port, UsbPortStatus status, int16_t ticks, uint16_t handle);
uint8_t getPortStatus(uint8_t port, UsbPortStatus *status);
uint8_t setPortStatus(uint8_t port, UsbPortStatus status);
uint8_t getConnHandle(uint8_t port, uint16_t *handle);
uint8_t setConnHandle(uint8_t port, uint16_t handle);
uint8_t getRamainingChargeTicks(uint8_t port, int16_t *ticks);
uint8_t setRamainingChargeTicks(uint8_t port, int16_t ticks);
uint8_t decrementChargingTicks(uint8_t port, int16_t *ticks);
uint8_t allocateFreePort(uint8_t *port);
uint8_t turnOnOffPower(uint8_t port, uint8_t onOff);

void onNewCommand(ble_evt_t const *p_ble_evt);
void onBleDisconnect(ble_evt_t const *p_ble_evt);
uint8_t freeUsbPort(uint8_t port);
void checkUsbPorts();
void onUsbPortChange(uint8_t port, uint8_t action);

#endif /* OUR_PORT_H */