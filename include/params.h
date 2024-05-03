#ifndef __PARAMS__
#define __PARAMS__
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#define KNEE_ACTUATOR_ID 0x124
#define MOTOR_DAMPING_CONSTANT 0.1

#define KNEE_ACTUATOR_IDX 0
#define MAX_MOTOR_TORQUE 6.0 // max torque in the motor
#define MAX_JOINT_LIMIT 1.8 //0 // max angle in the motor
#define MIN_JOINT_LIMIT -1.8 // min angle in the motor


#define CONTROL_FREQUENCY 1000
#define SAFETY_COUNTER_LIMIT 100 // max number of times a message is missed before which safety controller is called

// IMU details
#define CAN_QUAT_ADDRESS   0x0
#define CAN_STATUS_ADDRESS 0x1
#define uint_range 32768.0

#define IMU_HAND_ADDRESS 0x120
#define IMU_BASE_ADDRESS 0x122
#define IMU_SHOULDER_ADDRESS 0x121


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

const float forward_kinematics[3] = {-0.16600942,  0.73458596,  1.78947858};
const float jacobian[2] = {2*-0.16600942,  0.73458596};

#endif