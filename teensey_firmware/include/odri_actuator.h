#ifndef __ODRI_ACTUATOR__
#define __ODRI_ACTUATOR__

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <vector>
#include <map>
#include <math.h>
#include <functional>

#define KP_MAX 50
#define KV_MAX 10
#define Q_MAX  62830
#define Q_DOT_MAX  10000
#define I_MAX 10
#define ADC_REFERENCE_VOLTAGE  3.3
#define AMPLIFICATION_GAIN            9.14
#define RSHUNT                        0.003
#define CURRENT_CONV_FACTOR		(uint16_t)((65536.0 * RSHUNT * AMPLIFICATION_GAIN)/ADC_REFERENCE_VOLTAGE)
#define MOTOR_CONSTANT 1.44 // identified with test
#define GEAR_RATIO 9


// Define the type required for holding the responses from the actuators 
struct ActuatorState{
  double temperature; 
  double q;
  double dq;
  double tau_est; 
  double q_offset;
};

typedef struct ActuatorState ActuatorState_t;
void printCanMessage(const CAN_message_t &msg)
{
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  }
  Serial.println(' ');
}
class ODRI2ActuatorManager
{
  public:
    ODRI2ActuatorManager()
    {
      _idx=0;
      print_counter = 0;
    }
    
    void registerActuator(uint16_t id)
    {
      idMap[id] = _idx;
      _idx++;
      ActuatorState_t state;
      _actuators_states.push_back(state);
      _id_list.push_back(id);
    }

    void decodeResponse(const CAN_message_t &msg)
    {
      union ActuatorCANResponse
      {
        struct
        {
          float q;
          int16_t q_dot;
          int16_t tau;
        }data;
        uint8_t buffer[8];
      }_response;
      for ( uint8_t i = 0; i < msg.len; i++ ) 
          _response.buffer[i] = msg.buf[i];

        _actuators_states[msg.id].q = (_response.data.q)/GEAR_RATIO;
        _actuators_states[msg.id].dq = (((float)_response.data.q_dot/32700.) *Q_DOT_MAX)/GEAR_RATIO;
        _actuators_states[msg.id].tau_est = _response.data.tau; //TODO: covert to the non-raw version
    }

    CAN_message_t command2Message(uint16_t id, float torque)
    {
      
      CAN_message_t _msg;
      union{
        float current;
        uint8_t buff[4];
      }cmd;
      cmd.current = torque/MOTOR_CONSTANT;
      _msg.buf[0] =   cmd.buff[0];
      _msg.buf[1] =   cmd.buff[1];
      _msg.buf[2] =   cmd.buff[2];
      _msg.buf[3] =   cmd.buff[3];
      _msg.buf[4] =  0;
      _msg.buf[5] =  0;
      _msg.buf[6] =  0;
      _msg.buf[7] =  0;
      _msg.id = id;
      return _msg;
    }
    
    ActuatorState_t getLatestState(uint16_t id)
    {
      return _actuators_states[id];
    }
  
  private:
    std::map<uint16_t, uint16_t> idMap; 
    std::vector<ActuatorState_t> _actuators_states;
    std::vector<uint16_t> _id_list;
    uint16_t _idx;
    int print_counter;
};


#endif