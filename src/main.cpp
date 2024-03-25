#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include "odri_actuator.h"
#include "params.h"
#include "sharedMemory.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

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

float computePD(JointCmd_t cmd, JointState_t state)
{
  float kp = cmd.kp;
  float kv = cmd.kv;
  float q_des = cmd.q_des;// A*sin(millis()/T);
  float dq_des = cmd.dq_des;//(A/T)*cos(millis()/T);
  // compute the contrl law (note that the unit of u is in amps)
  double u = kp*(q_des - cmd.q_offset - state.q) + kv*(dq_des - state.dq) + cmd.tau_ff;
  u = std::max(-MAX_MOTOR_CURRENT, std::min(u, MAX_MOTOR_CURRENT));
  return u;
}

float computeDampingCommand(JointState_t state, float damping_constant)
{
  double u = damping_constant*(0 - state.dq);
  u = std::max(-MAX_MOTOR_CURRENT, std::min(u, MAX_MOTOR_CURRENT));
  return u;
}

void check_safety( const ActuatorState_t state){
  if((millis() - latest_cmd_stamp) > SAFETY_COUNTER_LIMIT && recieved_first_command){
    actuator_mode = 0;
  }
  else{
    actuator_mode = 1;
  }
}

// callback that runs priodically every 1ms 
void daqLoopCallback(void)
{
  const ActuatorState_t state = actuator_manager.getLatestState(KNEE_ACTUATOR_ID);
  shm_state.data[KNEE_ACTUATOR_IDX].q = state.q;
  shm_state.data[KNEE_ACTUATOR_IDX].dq = state.dq;
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
      double u = computeDampingCommand(shm_state.data[KNEE_ACTUATOR_IDX], KNEE_DAMPING_CONSTANT);
      auto msg = actuator_manager.command2Message(KNEE_ACTUATOR_ID, u);
      Can2.write(msg);
      break;
    }
  }
  shm_updated = 1;
  Can2.events();
}

float readJointStates(ActuatorState_t state)
{
  return state.q;
}
void setup() {
  // Initialize the shared memory
  for(int i=0; i<NUM_ACTUATORS; i++)
  {
    shm_cmd.data[i].kp=0.;
    shm_cmd.data[i].kv=0.;
    shm_cmd.data[i].dq_des=0.;
    shm_cmd.data[i].dq_des=0.;
    shm_state.data[i].q = 0.;
    shm_state.data[i].dq = 0.;
  }
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("starting setup.");

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
  
  // shm periodic timer (1000us)
  daqLoopCallbackTimer.begin(daqLoopCallback, CONTROL_FREQUENCY); // 1000Hz Callback

  // start the Ethernet
  Ethernet.begin(mac_address, ip);

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start UDP
  Udp.begin(localPort);
  actuator_mode = 1;
  Serial.println("motor is ready.");
}

uint8_t udp_cmd_buffer[128];
void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Udp.read(udp_cmd_buffer, UDP_TX_PACKET_MAX_SIZE);
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
          // Serial.println("A valid motion command was received");
        }
        break;
      }
    }
}


