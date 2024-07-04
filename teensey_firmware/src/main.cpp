#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <Wire.h>
#include <SPI.h>

#include "odri_actuator.h"
#include "sharedMemory.h"
#include "exoskeleton.h"

float computePD(const JointCmd cmd, const JointState state)
{
  // compute the contrl law (note that the unit of u is in amps)
  // double u = compute_exoskeleton_motor_torque(state.motor_q, cmd.tau_ff);
  double u = cmd.tau_ff;
  // Serial.println(u);
  Serial.println(cmd.tau_ff);
  u = std::max(-MAX_MOTOR_TORQUE, std::min(u, MAX_MOTOR_TORQUE));
  return u;
}

float computeDampingCommand(const JointState state, const float damping_constant)
{
  double u = damping_constant*(0 - state.motor_dq);
  u = std::max(-MAX_MOTOR_TORQUE, std::min(u, MAX_MOTOR_TORQUE));
  return u;
}

// callback that runs priodically every 1ms 
void daqLoopCallback(void)
{
  const ActuatorState_t state = actuator_manager.getLatestState(KNEE_ACTUATOR_ID);
  shm_state_t.data.motor_q = state.q - shm_cmd_t.data[KNEE_ACTUATOR_IDX].q_offset;
  shm_state_t.data.motor_dq = state.dq;
  shm_state_t.data.q = compute_exoskeleton_joint_angle(shm_state_t.data.motor_q);
  shm_state_t.data.dq = compute_exoskeleton_joint_velocity(shm_state_t.data.motor_q, shm_state_t.data.motor_dq);
  // check_safety(state);
  // compute the internal PD
  switch(actuator_mode){
    case ACTIVE_MODE:
    {
      double u = computePD(shm_cmd_t.data[KNEE_ACTUATOR_IDX], shm_state_t.data);
      auto msg = actuator_manager.command2Message(KNEE_ACTUATOR_ID, u);
      Can1.write(msg);
      break;
    }
    case SAFETY_MODE:
    {
      double u = computeDampingCommand(shm_state_t.data, MOTOR_DAMPING_CONSTANT);
      auto msg = actuator_manager.command2Message(KNEE_ACTUATOR_ID, u);
      Can1.write(msg);
      break;
    }
  }

}

float readJointStates(ActuatorState_t state)
{
  return state.q;
}
void setup() {
  // Initialize the shared memory
  for(int i=0; i<NUM_ACTUATORS; i++)
  {
    shm_cmd_t.data[i].q_des=0.;
    shm_cmd_t.data[i].dq_des=0.;
    shm_cmd_t.data[i].kp=0.;
    shm_cmd_t.data[i].kv=0.;
    shm_cmd_t.data[i].tau_ff=0.;
    shm_cmd_t.data[i].q_offset=0.;

    shm_state_t.data.q = 0.;
    shm_state_t.data.dq = 0.;
  }
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("starting setup.");

  Serial2.begin(115200);
  while (!Serial2) delay(10);  // wait for serial port to open!

  // Enable the Can1 bus with interrupt 
  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(ActuatorCanSniff);
  Can1.mailboxStatus();

  // Enable the CAN2 bus with interrupt 
  Can2.begin();
  Can2.setBaudRate(1000000);
  Can2.setMaxMB(16);
  Can2.enableFIFO();
  Can2.enableFIFOInterrupt();
  Can2.onReceive(IMUCanSniff);
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

  // shm periodic timer (1000us)  
  daqLoopCallbackTimer.begin(daqLoopCallback, CONTROL_FREQUENCY); // 1000Hz Callback

}

uint8_t udp_cmd_buffer[256];
int incomingByte = 0;
void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("packet rechieved");
    Udp.read(udp_cmd_buffer, packetSize);
    switch(udp_cmd_buffer[0])
    {
      case MOTION_CMD:
        if(packetSize==sizeof(shm_cmd_t.buffer)+4)
        {
          latest_cmd_stamp = millis();
          memcpy((uint8_t *)shm_cmd_t.buffer, (uint8_t *) &udp_cmd_buffer[4], sizeof(shm_cmd_t.buffer));
          // send a motion state response to the IP address and port that sent us the motion command
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write(&MOTION_STATE, 1);
          Udp.write(shm_state_t.buffer, sizeof(shm_state_t.buffer));
          Udp.endPacket();
        }
        break;
      }
    }

  Can1.events();
  Can2.events();
  delayMicroseconds(100);

}


