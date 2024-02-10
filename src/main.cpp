#include "Gemini_Teensy41.h"
#include <FlexCAN_T4.h>
#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>

// *** Begin for RingBuf *** //
#include "SdFat.h"
#include "RingBuf.h"

// *** Begin for Ros *** //
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include <sensor_msgs/msg/joint_state.h>

// ROS2 Messages
#include <geometry_msgs/msg/wrench_stamped.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include <sensor_msgs/msg/joint_state.h>


#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#include "HX711.h"
#include <FlexCAN_T4.h>
#define LED_PIN 13 // LED turns on when the microros agent is connected to the board
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

#define PPR 4096
#define MISTAKE 10000 // will be fixed

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// ROBOT PARAMS
#define NB_MOTORS 2 // number of motors

// ROS MESSAGE DATA
sensor_msgs__msg__JointState motor_state[NB_MOTORS];
rcl_publisher_t state_publisher;
rcl_subscription_t motor_cmd_subsriber;
std_msgs__msg__Int32 motor_cmd_msg;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
bool micro_ros_init_successful;

rcl_publisher_t wrench_publisher;

geometry_msgs__msg__WrenchStamped wrench_msg;
// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3
int32_t motor_cmd = 0;


// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#define LOG_INTERVAL_USEC 2000          // Interval between points for 25 ksps.
#define LOG_FILE_SIZE 220 * 500 * 600   // Size to log 220 byte lines at 500 Hz for more than 600 seconds = 66,000,000 bytes.
#define RING_BUF_CAPACITY 250 * 500 * 1 // Space to hold more than xx ms of data for yy byte lines at zz ksps.
#define LOG_FILENAME "Log_file.csv"

/*Canbus Setup*/
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;
/*Canbus Setup*/

uint32_t ID_offset = 0x140;

///////////////////////////////////////////////////////
// Setting Parameters

int assist_mode = 2;

double Gain_L = 1;
double Gain_R = 1;
double Gain_common = 1.0;

int current_limitation = 5;  // [Amp]
double JOINT_LB = -100;
double JOINT_UB = 100;

///////////////////////////////////////////////////////

String mode = "start";

double Gear_ratio = 6; // The actuator gear ratio, will influence actuator angle and angular velocity
double Torque_const = 1.67;
uint16_t Maxspeed_position = 500;

int Stop_button = 0;    // Stop function
int saveDataFlag = 1;

uint32_t Motor_ID1 = 2; //1 Motor Can Bus ID, left, loadcell: port 1, positive current = extension // updated on 2022-04-01 2
uint32_t Motor_ID2 = 1; //2 Motor Can Bus ID, right, loadcell: port 2, positive current = flexion // updated on 2022-04-01 3
int CAN_ID = 3;         // CAN port from Teensy

float arm_elevation_L;
float arm_elevation_R;

float radius_motor_pulley = 0.046; // [m]

Gemini_Teensy41 m1(Motor_ID1, CAN_ID, Gear_ratio, Maxspeed_position); //Create motor object 
Gemini_Teensy41 m2(Motor_ID2, CAN_ID, Gear_ratio, Maxspeed_position); //Create motor object

double Fsample = 300;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
double Fsample_ble = 100;    // [Hz] Bluetooth sending data frequency
unsigned long current_time = 0;
unsigned long previous_time = 0;                                                        // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                                    // used to control the Bluetooth communication frequency
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);               // used to control the teensy controller frequency
unsigned long Tinterval_ble_microsecond = (unsigned long)(1000000 / Fsample_ble);       // used to control the Bluetooth communication frequency

double Cur_command_L = 0;
double Cur_command_R = 0;
double torque_command_L = 0;
double torque_command_R = 0;

// PD Control
double des_pos = 0;
double des_vel = 0;

// Data logging
int isLogging = 0;
int taskIdx = 1;
int conditionIdx = 1; // baseline=1, sham=2, powered=3
int trialIdx = 1;

#define LED_PIN 13 // LED turns on when the microros agent is connected to the board
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void motor_cmd_callback(const void * msgin)
{  
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
  motor_cmd = msg->data;
}

// ROS Functions

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "exoskeleton_node", "", &support));

  // create publisher

  RCCHECK(rclc_publisher_init_default(
  &state_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "motor_state")); 


  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &motor_cmd_subsriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "motor_cmd1"));

  // create executor
  
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_cmd_subsriber, &motor_cmd_msg, &motor_cmd_callback, ON_NEW_DATA));
  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&state_publisher, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}


void initial_CAN()
{
  //initial CAN Bus
  Can3.begin();
  Can3.setBaudRate(1000000);
  //delay(3000);
  //pinMode(28, OUTPUT);
  //digitalWrite(28, LOW);
  delay(500);
  Serial.println("Can bus setup done...");
  delay(500);
}

void setup()
{
  // put your setup code here, to run once:
  delay(100);
  Serial.begin(115200);  //used for communication with computer.
  set_microros_serial_transports(Serial);
  delay(2000);
  Serial5.begin(115200); //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
   
  initial_CAN();
  delay(100);
  
  m1.init_motor(); // Start the CAN bus communication & Motor Control
  delay(100);
  m2.init_motor(); // Start the CAN bus communication & Motor Control
  delay(300);
    
  current_time = micros();
  previous_time = current_time;
  previous_time_ble = current_time;

  // allocating memory for joint message
  motor_state->position.size = NB_MOTORS;
  motor_state->position.capacity = NB_MOTORS;    
  motor_state->position.data = (double*) malloc(motor_state->position.capacity * sizeof(double));

  motor_state->velocity.size = NB_MOTORS;
  motor_state->velocity.capacity = NB_MOTORS;
  motor_state->velocity.data = (double*) malloc(motor_state->position.capacity * sizeof(double));

  pinMode(LED_PIN, OUTPUT);
  state = WAITING_AGENT;

}

// safety functions

void Cur_limitation()
{
  //************* Current limitation *************//

  Cur_command_L = min(+current_limitation, Cur_command_L);
  Cur_command_L = max(-current_limitation, Cur_command_L);
  Cur_command_R = min(+current_limitation, Cur_command_R);
  Cur_command_R = max(-current_limitation, Cur_command_R);
}

bool are_joint_limits_exceeded()
{
  if (m1.motorAngle < JOINT_LB ||  m1.motorAngle > JOINT_UB){
    return true;
  }
  else if (m2.motorAngle < JOINT_LB ||  m2.motorAngle > JOINT_UB){
    return true;
  }
  return false;
}

void reset_motor_angle()
{
  for (int i = 0; i < 20; i++)
  {
    m1.read_multi_turns_angle();
    delay(10);
    m1.receive_CAN_data();
    m1.motorAngle_offset = m1.motorAngle_raw;
    
    m2.read_multi_turns_angle();
    delay(10);
    m2.receive_CAN_data();
    m2.motorAngle_offset = m2.motorAngle_raw;
  }
}

void Compute_Cur_Commands()
{
  if (assist_mode == 1)
  {
    mode = "Damping Mode";
    Cur_command_L = 0.01*(0 - m1.speed_value);
    Cur_command_R =0.01*(0 - m2.speed_value);
  }
  else if (assist_mode == 2)
  {
    mode = "Torque Control Mode";
    // des_pos = 500 * sin(2 * PI * current_time / 50000000);
    Cur_command_L = motor_cmd; //0.05 * (des_pos - m1.motorAngle) + 0.01*(0 - m1.speed_value) ;  // [Amp]
  }  
  else if (assist_mode == 3)
  {
    mode = "Zero Mode";
    Cur_command_L = 0;
    Cur_command_R = 0;
  
  }
}

//**************Plot Data*****************//

void print_Current_Data()
{
    Serial.print(Cur_command_L); Serial.print("   ");
    Serial.print(Cur_command_R); Serial.print("   ");
//    Serial.println();
}

void print_Torque_Data()
{
    Serial.print(torque_command_L); Serial.print("   ");
    Serial.print(torque_command_R); Serial.print("   ");
//    Serial.println();
}

void print_data()
{
  Serial.print(des_pos); Serial.print("  ");
  // Serial.print(Cur_command_R); Serial.print("  ");
  Serial.print(m1.motorAngle); Serial.print("  "); // angle unit: degree 
  // Serial.print(m2.motorAngle); Serial.print("  "); // angle unit: degree
  Serial.print(m1.speed_value); Serial.print("  "); // angle velocity unit: degree per second
  //Serial.print(m2.speed_value); Serial.print("  "); // angle velocity unit: degree per second
  Serial.println();
}

void CurrentControl()
{
  current_time = micros();             //query current time (microsencond)

  //********* use to control the teensy controller frequency **********//
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    if (Stop_button) //stop
    {
      Serial.println("In STOP ...");
      assist_mode = 3;
    }
    else if (are_joint_limits_exceeded()){
      Serial.println("Joint Limits exceeded ...");
      assist_mode = 1;
    }

    Compute_Cur_Commands();
    Cur_limitation();

    int64_t time_nano = rmw_uros_epoch_nanos()%1000000000;
    int64_t time_sec = rmw_uros_epoch_millis()/1e3;

    motor_state->header.stamp.nanosec = time_nano;
    motor_state->header.stamp.sec = time_sec;
    
    motor_state->position.data[0]= m1.motorAngle;
    motor_state->position.data[1]= m2.motorAngle;

    motor_state->velocity.data[0]= m1.speed_value;
    motor_state->velocity.data[1]= m2.speed_value;
  
    rcl_publish(&state_publisher, &motor_state, NULL);

    m1.send_current_command(Cur_command_L);
    m1.receive_CAN_data();
    delay(1);
    
    m1.read_multi_turns_angle(); //read angle and angular velocity
    m1.receive_CAN_data();
    delay(1);
    
    // m2.send_current_command(Cur_command_R);
    // m2.receive_CAN_data();
    delay(1);

    m2.read_multi_turns_angle(); //read angle and angular velocity
    m2.receive_CAN_data();
    delay(1);
    
    previous_time = current_time; //reset previous control loop time
  }
}

void loop()
{

  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      // Agent time synchronization
      EXECUTE_EVERY_N_MS(2000, rmw_uros_sync_session(1000););
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        
        //send the command to the motor driver
        CurrentControl();
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}