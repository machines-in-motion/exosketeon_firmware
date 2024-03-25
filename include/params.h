#ifndef __PARAMS__
#define __PARAMS__
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#define KNEE_ACTUATOR_ID 0x123
#define KNEE_DAMPING_CONSTANT 0.02

#define KNEE_ACTUATOR_IDX 0
#define MAX_MOTOR_CURRENT 6.0 // max cuurent in the motor

#define CONTROL_FREQUENCY 1000
#define SAFETY_COUNTER_LIMIT 100 // max number of times a message is missed before which safety controller is called

// A Mac address to be used for the leg.
byte mac_address[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
// The IP address of the leg:
IPAddress ip(192, 168, 123, 10);
unsigned int localPort = 5000;      // DAQ local port to listen on

const uint32_t MOTION_CMD=0x01;
const uint8_t MOTION_STATE=0x01;

enum motor_mode {
  SAFETY_MODE = 0,
  ACTIVE_MODE = 1
};

#endif