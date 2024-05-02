#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include "odri_actuator.h"
#include "params.h"
#include "sharedMemory.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

// TO be cleaned
int counter = 0;


// IMU 
Adafruit_BNO055 bno_shoulder = Adafruit_BNO055(56, 0x28, &Wire);
Adafruit_BNO055 bno_base = Adafruit_BNO055(55, 0x28, &Wire1);
Adafruit_BNO055 bno_wrist = Adafruit_BNO055(55, 0x28, &Wire2);

imu::Quaternion imu_shoulder_ori;
imu::Quaternion imu_base_ori; 
imu::Quaternion imu_wrist_ori; 

// Instantiate required objects:
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2; // The CAN2 driver

IntervalTimer daqLoopCallbackTimer;             // A periodic timer for sending commands evert 1ms 
ODRI2ActuatorManager actuator_manager;          // The actuator manger class for managing the communication with the motor drivers
EthernetUDP Udp;
uint32_t latest_cmd_stamp; 
int actuator_mode = 0;
bool recieved_first_command = 0;

char packetBuffer[1024];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";        // a string to send back

volatile int shm_updated = 0;

// callback to process the responses from the actuators on the CAN2 bus
void can2Callback(const CAN_message_t &msg) {
  actuator_manager.decodeResponse(msg);
}

float computePD(const JointCmd_t cmd, const JointState_t state)
{
  float kp = cmd.kp;
  float kv = cmd.kv;
  float q_des = cmd.q_des;
  float dq_des = cmd.dq_des;

  // compute the contrl law (note that the unit of u is in amps)
  double u = kp*(q_des  - state.motor_q) + kv*(dq_des - state.motor_dq) + cmd.tau_ff;
  u = std::max(-MAX_MOTOR_TORQUE, std::min(u, MAX_MOTOR_TORQUE));
  return u;
}

float computeDampingCommand(const JointState_t state, const float damping_constant)
{
  double u = damping_constant*(0 - state.motor_dq);
  u = std::max(-MAX_MOTOR_TORQUE, std::min(u, MAX_MOTOR_TORQUE));
  return u;
}

void check_safety( const ActuatorState_t state){
  if((millis() - latest_cmd_stamp) > SAFETY_COUNTER_LIMIT){
    actuator_mode = 0;

  }
  if((state.q) > MAX_JOINT_LIMIT || MIN_JOINT_LIMIT > (state.q)){
    actuator_mode = 0;
  }
  else{
    actuator_mode = 1;
  }
}

const float compute_exoskeleton_joint_angle(const float motor_angle){
  return forward_kinematics[0]*std::pow(motor_angle,2) + forward_kinematics[1]*std::pow(motor_angle,1) + forward_kinematics[2];
}

const float compute_exoskeleton_joint_velocity(const float motor_angle, const float motor_velocity){
  return (jacobian[0]*std::pow(motor_angle,1) + jacobian[1])*motor_velocity;
}

// callback that runs priodically every 1ms 
void daqLoopCallback(void)
{
  const ActuatorState_t state = actuator_manager.getLatestState(KNEE_ACTUATOR_ID);
  const JointCmd_t cmd = shm_cmd.data[KNEE_ACTUATOR_IDX];
  
  //updating the imu state at 100 ms
  if (counter > (CONTROL_FREQUENCY/BNO055_SAMPLERATE_DELAY_MS)){

    if(bno_shoulder.getQuat().magnitude() > 1e-6){
      imu_shoulder_ori = bno_shoulder.getQuat();
      // imu_shoulder_ori.normalize();
    }

    if(bno_base.getQuat().magnitude() > 1e-6){
      imu_base_ori = bno_base.getQuat();
      // imu_base_ori.normalize();
    }
    if(bno_wrist.getQuat().magnitude() > 1e-6){
      imu_wrist_ori = bno_wrist.getQuat();
      // imu_wrist_ori.normalize();
    }

    Serial.println("BASE IMU");
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno_base.getCalibration(&system, &gyro, &accel, &mag);

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
    Serial.println("");

    Serial.println("SHUOLDER IMU");
    system = gyro = accel = mag = 0;
    bno_shoulder.getCalibration(&system, &gyro, &accel, &mag);

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
    Serial.println("");

    Serial.println("WRIST IMU");
    system = gyro = accel = mag = 0;
    bno_wrist.getCalibration(&system, &gyro, &accel, &mag);

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
    Serial.println("");


    counter = 0;
  }

  shm_state.data[KNEE_ACTUATOR_IDX].motor_q = state.q - cmd.q_offset;
  shm_state.data[KNEE_ACTUATOR_IDX].motor_dq = state.dq;
  shm_state.data[KNEE_ACTUATOR_IDX].q = compute_exoskeleton_joint_angle(shm_state.data[KNEE_ACTUATOR_IDX].motor_q);
  shm_state.data[KNEE_ACTUATOR_IDX].dq = compute_exoskeleton_joint_velocity(shm_state.data[KNEE_ACTUATOR_IDX].motor_q, shm_state.data[KNEE_ACTUATOR_IDX].motor_dq);
  
  
  shm_state.data[KNEE_ACTUATOR_IDX].shoulder_ori[0]=imu_shoulder_ori.x();
  shm_state.data[KNEE_ACTUATOR_IDX].shoulder_ori[1]=imu_shoulder_ori.y();
  shm_state.data[KNEE_ACTUATOR_IDX].shoulder_ori[2]=imu_shoulder_ori.z();
  shm_state.data[KNEE_ACTUATOR_IDX].shoulder_ori[3]=imu_shoulder_ori.w();

  shm_state.data[KNEE_ACTUATOR_IDX].base_ori[0]=imu_base_ori.x();
  shm_state.data[KNEE_ACTUATOR_IDX].base_ori[1]=imu_base_ori.y();
  shm_state.data[KNEE_ACTUATOR_IDX].base_ori[2]=imu_base_ori.z();
  shm_state.data[KNEE_ACTUATOR_IDX].base_ori[3]=imu_base_ori.w();

  shm_state.data[KNEE_ACTUATOR_IDX].wrist_ori[0]=imu_wrist_ori.x();
  shm_state.data[KNEE_ACTUATOR_IDX].wrist_ori[1]=imu_wrist_ori.y();
  shm_state.data[KNEE_ACTUATOR_IDX].wrist_ori[2]=imu_wrist_ori.z();
  shm_state.data[KNEE_ACTUATOR_IDX].wrist_ori[3]=imu_wrist_ori.w();

  check_safety(state);
  // compute the internal PD
  switch(actuator_mode){
    case ACTIVE_MODE:
    {
      double u = computePD(shm_cmd.data[KNEE_ACTUATOR_IDX], shm_state.data[KNEE_ACTUATOR_IDX]);
      auto msg = actuator_manager.command2Message(KNEE_ACTUATOR_ID, u);
      Can2.write(msg);
      break;
    }
    case SAFETY_MODE:
    {
      double u = computeDampingCommand(shm_state.data[KNEE_ACTUATOR_IDX], MOTOR_DAMPING_CONSTANT);
      auto msg = actuator_manager.command2Message(KNEE_ACTUATOR_ID, u);
      Can2.write(msg);
      break;
    }
  }
  shm_updated = 1;
  Can2.events();
  counter += 1;

}

float readJointStates(ActuatorState_t state)
{
  return state.q;
}
void setup() {
  // Initialize the shared memory
  for(int i=0; i<NUM_ACTUATORS; i++)
  {
    shm_cmd.data[i].q_des=0.;
    shm_cmd.data[i].dq_des=0.;
    shm_cmd.data[i].kp=0.;
    shm_cmd.data[i].kv=0.;
    shm_cmd.data[i].tau_ff=0.;
    shm_cmd.data[i].q_offset=0.;


    shm_state.data[i].q = 0.;
    shm_state.data[i].dq = 0.;
  }
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("starting setup.");

  Serial2.begin(115200);
  while (!Serial2) delay(10);  // wait for serial port to open!

  // Enable the CAN2 bus with interrupt 
  Can2.begin();
  Can2.setBaudRate(1000000);
  Can2.setMaxMB(16);
  Can2.enableFIFO();
  Can2.enableFIFOInterrupt();
  Can2.onReceive(can2Callback);
  Can2.mailboxStatus();
  // Add the knee actuator to the actuator manager
  actuator_manager.registerActuator(KNEE_ACTUATOR_ID);
  
  // start the Ethernet
  Ethernet.begin(mac_address, ip);

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start UDP
  Udp.begin(localPort);
  actuator_mode = 1;
  Serial.println("motor is ready.");

  // Start IMU
  if (!bno_base.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 base detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  // bno_base.setExtCrystalUse(true);
  bno_base.setMode(OPERATION_MODE_MAGGYRO);
  delay(1000);

  if (!bno_shoulder.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 shoulder detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  // bno_shoulder.setExtCrystalUse(true);
  bno_shoulder.setMode(OPERATION_MODE_MAGGYRO);
  delay(1000);

  if (!bno_wrist.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 shoulder detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  // bno_wrist.setExtCrystalUse(true);
  bno_wrist.setMode(OPERATION_MODE_MAGGYRO);
  delay(1000);

  // shm periodic timer (1000us)  
  daqLoopCallbackTimer.begin(daqLoopCallback, CONTROL_FREQUENCY); // 1000Hz Callback

}

uint8_t udp_cmd_buffer[128];
int incomingByte = 0;
void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Udp.read(udp_cmd_buffer, packetSize);
    switch(udp_cmd_buffer[0])
    {
      case MOTION_CMD:
        if(packetSize==sizeof(shm_cmd.buffer)+4)
        {
          latest_cmd_stamp = millis();
          memcpy((uint8_t *)shm_cmd.buffer, (uint8_t *) &udp_cmd_buffer[4], sizeof(shm_cmd.buffer));
          // send a motion state response to the IP address and port that sent us the motion command
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write(&MOTION_STATE, 1);
          Udp.write(shm_state.buffer, sizeof(shm_state.buffer));
          Udp.endPacket();
        }
        break;
      }
    }

}


