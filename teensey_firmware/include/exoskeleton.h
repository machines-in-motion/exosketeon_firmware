// Contains exo skeleton specific functions and variables

#include "params.h"
#include "bno_manager.h"
#include "sharedMemory.h"

// Instantiate required objects:
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2; // The CAN2 driver

IntervalTimer daqLoopCallbackTimer;             // A periodic timer for sending commands evert 1ms 
EthernetUDP Udp;
uint32_t latest_cmd_stamp; 

char packetBuffer[1024];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";        // a string to send back

volatile int shm_updated = 0;

// Actuator stuff
ODRI2ActuatorManager actuator_manager;          // The actuator manger class for managing the communication with the motor drivers
int actuator_mode = 0;
bool recieved_first_command = 0;

const float compute_exoskeleton_joint_angle(const float motor_angle){
  return forward_kinematics[0]*std::pow(motor_angle,2) + forward_kinematics[1]*std::pow(motor_angle,1) + forward_kinematics[2];
}

const float compute_exoskeleton_joint_velocity(const float motor_angle, const float motor_velocity){
  return (jacobian[0]*std::pow(motor_angle,1) + jacobian[1])*motor_velocity;
}

const float compute_exoskeleton_motor_torque(const float motor_angle, const float joint_torque){
  return (jacobian[0]*std::pow(motor_angle,1) + jacobian[1])*joint_torque;
}


void check_safety( const ActuatorState_t state){
  // if((millis() - latest_cmd_stamp) > SAFETY_COUNTER_LIMIT){
  //   actuator_mode = 0;

  // }
  if((state.q) > MAX_JOINT_LIMIT || MIN_JOINT_LIMIT > (state.q)){
    actuator_mode = 0;
  }
  else{
    actuator_mode = 1;
  }
}


// callback to process the responses from the actuators on the CAN2 bus
void ActuatorCanSniff(const CAN_message_t &msg) {
  actuator_manager.decodeResponse(msg);
}


// IMU stuff
BnoManager imu_base(IMU_BASE_ADDRESS);
BnoManager imu_hand(IMU_HAND_ADDRESS);
BnoManager imu_shoulder(IMU_SHOULDER_ADDRESS);


void IMUCanSniff(const CAN_message_t &msg) {

  imu_base.process_response(msg);
  imu_base.update_shm(shm_state_t.data.quaternion_base, shm_state_t.data.base_quality);
  imu_shoulder.process_response(msg);
  imu_shoulder.update_shm(shm_state_t.data.quaternion_shoulder, shm_state_t.data.shoulder_quality);
  imu_hand.process_response(msg);
  imu_hand.update_shm(shm_state_t.data.quaternion_hand, shm_state_t.data.hand_quality);

  // Serial.println("IMU_BASE");
  // Serial.print(shm_state_t.data.quaternion_base[0], 2);
  // Serial.print(F(","));
  // Serial.print(shm_state_t.data.quaternion_base[1], 2);
  // Serial.print(F(","));
  // Serial.print(shm_state_t.data.quaternion_base[2], 2);
  // Serial.print(F(","));
  // Serial.print(shm_state_t.data.quaternion_base[3], 2);
  // Serial.print(F(","));
  // float norm = 0;
  // for(unsigned i =0; i < 4; ++i){
  //   norm +=  imu_base.quat[i]*imu_base.quat[i];
  // };
  // Serial.print(sqrt(norm));
  // Serial.print(F(","));
  // Serial.print(shm_state_t.data.base_quality);
  // Serial.print(F(","));
  // Serial.println();

  // Serial.println("IMU_SHOULDER");
  // Serial.print(imu_shoulder.quat[0], 2);
  // Serial.print(F(","));
  // Serial.print(imu_shoulder.quat[1], 2);
  // Serial.print(F(","));
  // Serial.print(imu_shoulder.quat[2], 2);
  // Serial.print(F(","));
  // Serial.print(imu_shoulder.quat[3], 2);
  // Serial.print(F(","));
  // norm = 0;
  // for(unsigned i =0; i < 4; ++i){
  //   norm += imu_shoulder.quat[i]*imu_shoulder.quat[i];
  // };
  // Serial.print(sqrt(norm));
  // Serial.print(F(","));
  // Serial.print(shm_state_t.data.shoulder_quality);
  // Serial.print(F(","));
  // Serial.println();

  // Serial.println("IMU_HAND");
  // Serial.print(imu_hand.quat[0], 2);
  // Serial.print(F(","));
  // Serial.print(imu_hand.quat[1], 2);
  // Serial.print(F(","));
  // Serial.print(imu_hand.quat[2], 2);
  // Serial.print(F(","));
  // Serial.print(imu_hand.quat[3], 2);
  // Serial.print(F(","));
  // norm = 0;
  // for(unsigned i =0; i < 4; ++i){
  //   norm +=  imu_hand.quat[i]*imu_hand.quat[i];
  // };
  // Serial.print(sqrt(norm));
  // Serial.print(F(","));
  // Serial.print(shm_state_t.data.hand_quality);
  // Serial.print(F(","));
  // Serial.println();
}