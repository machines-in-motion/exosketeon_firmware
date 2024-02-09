#include "Gemini_Teensy41.h"
#include <FlexCAN_T4.h>
#include <Arduino.h>
#include <SPI.h>
#include "WL_IMU.h"
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

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#define LOG_INTERVAL_USEC 2000          // Interval between points for 25 ksps.
#define LOG_FILE_SIZE 220 * 500 * 600   // Size to log 220 byte lines at 500 Hz for more than 600 seconds = 66,000,000 bytes.
#define RING_BUF_CAPACITY 250 * 500 * 1 // Space to hold more than xx ms of data for yy byte lines at zz ksps.
#define LOG_FILENAME "Log_file.csv"
#define NB_MOTORS 2 // number of motors;

SdFs sd;
FsFile file;

RingBuf<FsFile, RING_BUF_CAPACITY> rb; // RingBuf for File type FsFile.

int stopFlag = 0;
// *** End for RingBuf *** //

/*Canbus Setup*/
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
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
IMU imu;                                                      //Create IMU object see WL_IMU.h

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

//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy

int angle_L_ble = 0;                //left knee angle
int angle_R_ble = 0;                //right knee angle
int current_command_L_ble = 0; //current reference(A) for inner loop current control
int current_command_R_ble = 0;
int current_L_ble = 0;
int current_R_ble = 0;
int torque_command_L_ble = 0; //total torque reference(N-m)
int torque_command_R_ble = 0;
int torque_L_ble = 0;             //actual torque(N-m)(measured by torque sensor)
int torque_R_ble = 0;
int motor_angle_L_ble = 0;
int motor_angle_R_ble = 0;
int motor_speed_L_ble = 0;
int motor_speed_R_ble = 0;

//***********ROS 2 Communication*********//
// Microros functions
rcl_subscription_t motor_cmd_subsriber;
std_msgs__msg__Float64 motor_cmd_msg;
sensor_msgs__msg__JointState motor_state[3];

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t state_publisher;
bool micro_ros_init_successful;

float_t motor_cmd = 0;

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

//***********High Level Communication*********//

char data_serial_highlevel[101] = {0};
char data_highlevel_rx[7] = {0}; //data high level communication
char Highlevel_Data_Length_Send = 101;

int LK_highlevel = 0;
int RK_highlevel = 0;
int TK_highlevel = 0;
int LT_highlevel = 0;
int RT_highlevel = 0;
int LS_highlevel = 0;
int RS_highlevel = 0;

double Left_Torque_Command;
double Right_Torque_Command;
int Left_Torque = 0;
int Right_Torque = 0;

double relTime = 0.0;
int SDcounter = 0;
int SDtotalCounter = 0;
String dataBuffer = "";
#define charBufferSize 100
char charBuffer[charBufferSize];
long charBufferPosition = 0;

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
    
  // imu.INIT(); //Initialize IMU;
  // delay(500);
  // imu.INIT_MEAN();
  
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

void SDCardSaveToFile()
{
  rb.sync();
  file.truncate();
  file.rewind();
  file.close();
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


void SDCardSetup(const char* logFileName)
{
  sd.remove(logFileName);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG))
  {
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(logFileName, O_RDWR | O_CREAT | O_TRUNC))
  {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE))
  {
    Serial.println("preAllocate failed\n");
    file.close();
    return;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.print("Data logging initialized. File name = ");
  Serial.println(logFileName);
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


void send_ble_Data()
{
  angle_L_ble = arm_elevation_L * 100;
  angle_R_ble = arm_elevation_R * 100;
  current_command_L_ble = Cur_command_L * 100; // Gui flexion is negative
  current_command_R_ble = Cur_command_R * 100;  // Gui flexion is negative  
  current_L_ble = m1.iq_A * 100;
  current_R_ble = m2.iq_A * 100;
  torque_command_L_ble = torque_command_L * 100; // Gui flexion is negative
  torque_command_R_ble = torque_command_R * 100; // Gui flexion is negative
  torque_L_ble = m1.iq_A * Torque_const * 100; // +m1.iq_A * 0.232 * 9 * 100; //torque_sensor1.torque[0]   // motor torque constant = 0.232 Nm/A, gear ratio = 9;
  torque_R_ble = m2.iq_A * Torque_const * 100; // -m2.iq_A * 0.232 * 9 * 100; //torque_sensor1.torque[1]   // motor torque constant = 0.232 Nm/A, gear ratio = 9;
  motor_angle_L_ble = m1.motorAngle * 100; // [deg]
  motor_angle_R_ble = m2.motorAngle * 100; // [deg]
  motor_speed_L_ble = m1.speed_value / 180 * 3.1415 * 100; // [rad/s]
  motor_speed_R_ble = m2.speed_value / 180 * 3.1415 * 100; // [rad/s]

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0] = 165;
  data_ble[1] = 90;
  data_ble[2] = datalength_ble;
  
  data_ble[3] = angle_L_ble;
  data_ble[4] = angle_L_ble >> 8;
  data_ble[5] = angle_R_ble;
  data_ble[6] = angle_R_ble >> 8;
  data_ble[7] = current_command_L_ble;
  data_ble[8] = current_command_L_ble >> 8;
  data_ble[9] = current_command_R_ble;
  data_ble[10] = current_command_R_ble >> 8;
  data_ble[11] = current_L_ble;
  data_ble[12] = current_L_ble >> 8;
  data_ble[13] = current_R_ble;
  data_ble[14] = current_R_ble >> 8;
  data_ble[15] = torque_command_L_ble;
  data_ble[16] = torque_command_L_ble >> 8;
  data_ble[17] = torque_command_R_ble;
  data_ble[18] = torque_command_R_ble >> 8;
  data_ble[19] = torque_L_ble;
  data_ble[20] = torque_L_ble >> 8;
  data_ble[21] = torque_R_ble;
  data_ble[22] = torque_R_ble >> 8;
  data_ble[23] = motor_angle_L_ble;
  data_ble[24] = motor_angle_L_ble >> 8;
  data_ble[25] = motor_angle_R_ble;
  data_ble[26] = motor_angle_R_ble >> 8;
  data_ble[27] = motor_speed_L_ble;
  data_ble[28] = motor_speed_L_ble >> 8;
  data_ble[29] = motor_speed_R_ble;
  data_ble[30] = motor_speed_R_ble >> 8;

  Serial5.write(data_ble, datalength_ble);
}

void receive_ble_Data()
{
  if (Serial5.available() >= 20)
  {
    // Serial.println("-------------New data received-------------------");
    data_rs232_rx[0] = Serial5.read();
    if (data_rs232_rx[0] == 165)
    {
      data_rs232_rx[1] = Serial5.read();
      if (data_rs232_rx[1] == 90)
      {
        data_rs232_rx[2] = Serial5.read();
        if (data_rs232_rx[2] == 20)
        {
          Serial5.readBytes(&data_rs232_rx[3], 17);
          if (data_rs232_rx[3] == 0)
          {
            Stop_button = int(data_rs232_rx[4]);
            if (Stop_button)
            {
              Serial.println("STOP button pressed");
            }
            else
            {
              Serial.println("START button pressed");
            }
          }
          else if (data_rs232_rx[3] == 1)
          {
            assist_mode = int(data_rs232_rx[4]);
            Serial.print("Mode: ");
            // Serial.println(assist_mode);
          }
          else if (data_rs232_rx[3] == 2)
          {
            Gain_L = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            Serial.print("Left gain: ");
            // Serial.println(Gain_L);
          }
          else if (data_rs232_rx[3] == 3)
          {
            Gain_R = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            Serial.print("Right gain: ");
            // Serial.println(Gain_R);
          }
          else if (data_rs232_rx[3] == 4)
          { 
            Gain_common = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            Serial.print("Common gain: ");
            // Serial.println(Gain_common);
          }
          else if (data_rs232_rx[3] == 5)
          {
          }
          else if (data_rs232_rx[3] == 6)
          {
          }
          else if (data_rs232_rx[3] == 7)
          {
          }
          else if (data_rs232_rx[3] == 10)
          {
            reset_motor_angle();
            Serial.println("The angle of motor has been reset");
            imu.INIT_MEAN();
            Serial.println("The angle of IMUs has been reset");
          }
          else if (data_rs232_rx[3] == 20)
          {
            isLogging = int(data_rs232_rx[7]);
            if (isLogging == 1)
            {
              taskIdx = int(data_rs232_rx[4]);
              conditionIdx = int(data_rs232_rx[5]);
              trialIdx = int(data_rs232_rx[6]);
              String taskName;
              if (taskIdx == 1)
              {
                taskName = "task_A";
              }
              else if (taskIdx == 2)
              {
                taskName = "task_B";
              }
              String stringOne = taskName + '-';

              String conditionName;
              if (conditionIdx == 1)
              {
                conditionName = "Baseline";
              }
              else if (conditionIdx == 2)
              {
                conditionName = "Sham";
              }
              else if (conditionIdx == 3)
              {
                conditionName = "Powered";
              }
              else
              {
                conditionName = "Others";
              }
              String stringTwo = stringOne + conditionName + "-Trial";
              String logFileName = stringTwo + String(trialIdx) + ".csv";
              SDCardSetup(logFileName.c_str());
              relTime = 0.0;
              Serial.println("Data logging started......");
            }
            else if (isLogging == 0)
            {
              SDCardSaveToFile();
              Serial.println("Data logging stopped......");
            }
          }
        }
      }
    }
  }
}

//******************Receive high level controller Command****************//
void receive_serial_Data_Highlevel()
{
  if (Serial.available() >= 7)
  {
    //Serial.println("receive");
    data_highlevel_rx[0] = Serial.read();
    if (data_highlevel_rx[0] == 165)
    {
      data_highlevel_rx[1] = Serial.read();
      if (data_highlevel_rx[1] == 90)
      {
        Serial.readBytes(&data_highlevel_rx[2], 5); //int data_rs232_rx[7]
        //Highlevel_Data_Length_Receive = int(data_highlevel_rx[2]); // read data length
        Left_Torque_Command = ((int16_t)(((int16_t)data_highlevel_rx[3]) | ((int16_t)data_highlevel_rx[4] << 8))); Left_Torque_Command = Left_Torque_Command / 100;
        Right_Torque_Command = ((int16_t)(((int16_t)data_highlevel_rx[5]) | ((int16_t)data_highlevel_rx[6] << 8))); Right_Torque_Command = Right_Torque_Command / 100;
      }
    }
  }
}

//******************Send high level controller Command****************//
void send_serial_Data_Highlevel()
{
  LK_highlevel = imu.LKx * 100;
  RK_highlevel = imu.RKx * 100;
  TK_highlevel = imu.TKx * 100;
  LT_highlevel = imu.LTx * 100;
  RT_highlevel = imu.RTx * 100;
  LS_highlevel = imu.LSx * 100;
  RS_highlevel = imu.RSx * 100;
  Left_Torque  = +m1.iq_A * Torque_const * 100; //torque_sensor1.torque[0] * 100;
  Right_Torque = -m2.iq_A * Torque_const * 100; //torque_sensor1.torque[1] * 100;

  data_serial_highlevel[0] = 165;
  data_serial_highlevel[1] = 90;
  data_serial_highlevel[2] = Highlevel_Data_Length_Send;
  
  data_serial_highlevel[3] = Left_Torque >> 8;
  data_serial_highlevel[4] = Left_Torque;
  data_serial_highlevel[5] = Right_Torque >> 8;
  data_serial_highlevel[6] = Right_Torque;
  data_serial_highlevel[7] = LK_highlevel >> 8;
  data_serial_highlevel[8] = LK_highlevel;
  data_serial_highlevel[9] = RK_highlevel >> 8;
  data_serial_highlevel[10] = RK_highlevel;
  data_serial_highlevel[11] = LT_highlevel >> 8;
  data_serial_highlevel[12] = LT_highlevel;
  data_serial_highlevel[29] = RT_highlevel >> 8;
  data_serial_highlevel[30] = RT_highlevel;
  data_serial_highlevel[47] = LS_highlevel >> 8;
  data_serial_highlevel[48] = LS_highlevel;
  data_serial_highlevel[65] = RS_highlevel >> 8;
  data_serial_highlevel[66] = RS_highlevel;
  data_serial_highlevel[83] = TK_highlevel >> 8;
  data_serial_highlevel[84] = TK_highlevel;

  Serial.write(data_serial_highlevel, Highlevel_Data_Length_Send);
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
  else if (assist_mode == 4) //Serial communication with High-level system
  {
    mode = "High Level Control";
    send_serial_Data_Highlevel(); 
    receive_serial_Data_Highlevel();
    Cur_command_L =  Left_Torque_Command / Torque_const;
    Cur_command_R = Right_Torque_Command / Torque_const;
  }
}


//*** Ringbuf ***//
void logData3()
{
  // Max RingBuf used bytes. Useful to understand RingBuf overrun.
  size_t maxUsed = 0;

  // Min spare micros in loop.
  //int32_t minSpareMicros = INT32_MAX;

  // Start time.
  //uint32_t logTime = micros();
  // Log data until Serial input or file full.
  //  while (!Serial.available()) {
  // Amount of data in ringBuf.
  size_t n = rb.bytesUsed();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20))
  {
    Serial.println("File full - quitting.");
    return; // break;
  }
  if (n > maxUsed)
  {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy())
  {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512))
    {
      Serial.println("writeOut failed");
      return; //break;
    }
  }
  // Time for next point.
  //  logTime += LOG_INTERVAL_USEC;
  //  int32_t spareMicros = logTime - micros();
  //  if (spareMicros < minSpareMicros) {
  //    minSpareMicros = spareMicros;
  //  }
  //  if (spareMicros <= 0) {
  //    Serial.print("Rate too fast ");
  //    Serial.println(spareMicros);
  //    break;
  //  }
  // Wait until time to log data.
  //  while (micros() < logTime) {}

  // Read ADC0 - about 17 usec on Teensy 4, Teensy 3.6 is faster.
  //  uint16_t adc = analogRead(0);
  // Print spareMicros into the RingBuf as test data.
     rb.print(relTime); rb.write(" ");
     rb.print(imu.TKx); rb.write(" ");
     rb.print(imu.TKy); rb.write(" ");
     rb.print(imu.TKz); rb.write(" ");
     rb.print(imu.LTx); rb.write(" ");
     rb.print(imu.LTy); rb.write(" ");
     rb.print(imu.LTz); rb.write(" ");
     rb.print(imu.RTx); rb.write(" ");
     rb.print(imu.RTy); rb.write(" ");
     rb.print(imu.RTz); rb.write(" ");
     rb.print(imu.LSx); rb.write(" ");
     rb.print(imu.LSy); rb.write(" ");
     rb.print(imu.LSz); rb.write(" ");
     rb.print(imu.RSx); rb.write(" ");
     rb.print(imu.RSy); rb.write(" ");
     rb.print(imu.RSz); rb.write(" ");
     rb.print(imu.TKAVx); rb.write(" ");
     rb.print(imu.TKAVy); rb.write(" ");
     rb.print(imu.TKAVz); rb.write(" ");
     rb.print(imu.LTAVx); rb.write(" ");
     rb.print(imu.LTAVy); rb.write(" ");
     rb.print(imu.LTAVz); rb.write(" ");
     rb.print(imu.RTAVx); rb.write(" ");
     rb.print(imu.RTAVy); rb.write(" ");
     rb.print(imu.RTAVz); rb.write(" ");
     rb.print(imu.LSAVx); rb.write(" ");
     rb.print(imu.LSAVy); rb.write(" ");
     rb.print(imu.LSAVz); rb.write(" ");
     rb.print(imu.RSAVx); rb.write(" ");
     rb.print(imu.RSAVy); rb.write(" ");
     rb.print(imu.RSAVz); rb.write(" ");
     rb.print(Cur_command_L); rb.write(" ");
     rb.print(Cur_command_R); rb.write(" ");
     rb.print(m1.iq_A); rb.write(" ");
     rb.print(m2.iq_A); rb.write(" ");
     rb.print(m1.motorAngle); rb.write(" ");
     rb.print(m2.motorAngle); rb.write(" ");
     rb.print(m1.speed_value); rb.write(" ");
     rb.print(m2.speed_value); rb.write(" "); 
     rb.println();
     //rb.print("\n");
  // Print adc into RingBuf.
  //  rb.println(adc);
  if (rb.getWriteError())
  {
    // Error caused by too few free bytes in RingBuf.
    Serial.println("WriteError");
    return; //break;
  }
  
}

//**************Wait functions*****************//

void custom_wait()
{
//  delay(1);
  for (int  qwe = 0; qwe < 1e3; qwe++)
    {
      //for (int aaa = 0; aaa < 1e1; aaa++){}
    }
}
void custom_wait2()
{
  delay(1);
//  for (int  qwe = 0; qwe < 1e8; qwe++)
//    {
//      for (int aaa = 0; aaa < 1e8; aaa++){}
//    }
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

void print_IMU_Data()
{

     Serial.print(imu.LSx); Serial.print("   ");
     Serial.print(imu.LSy); Serial.print("   ");
     Serial.print(imu.LSz); Serial.print("   ");
     Serial.print(imu.RSx); Serial.print("   ");
     Serial.print(imu.RSy); Serial.print("   ");
     Serial.print(imu.RSz); Serial.print("   ");
     Serial.print(imu.LSAVx); Serial.print("   ");
     Serial.print(imu.LSAVy); Serial.print("   ");
     Serial.print(imu.LSAVz); Serial.print("   ");
     Serial.print(imu.RSAVx); Serial.print("   ");
     Serial.print(imu.RSAVy); Serial.print("   ");
     Serial.print(imu.RSAVz); Serial.print("   ");
//     Serial.println("   ");
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
  imu.READ();                          //Check if IMU data available and read it. the sample rate is 100 hz
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

    // print_data();

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
    custom_wait();
    m1.receive_CAN_data();
    custom_wait2();
    
    m1.read_multi_turns_angle(); //read angle and angular velocity
    custom_wait();
    m1.receive_CAN_data();
    custom_wait2();
    
    // m2.send_current_command(Cur_command_R);
    // custom_wait();
    // m2.receive_CAN_data();
    // custom_wait();

    // m2.read_multi_turns_angle(); //read angle and angular velocity
    // custom_wait();
    // m2.receive_CAN_data();
    // custom_wait();
    
    // if (isLogging)
    // {
    //   logData3();
    // }

    previous_time = current_time; //reset previous control loop time
    relTime += Tinterval_microsecond / 1000;
  }

  //********* use to control the Bluetooth communication frequency **********//
  if (current_time - previous_time_ble > Tinterval_ble_microsecond)
  {
    receive_ble_Data();
    send_ble_Data(); // send the BLE data
    print_data();
    previous_time_ble = current_time;
        
  }
  if (Serial.available())
  {
    char cmd = (char)Serial.read();
    if (cmd == 's')
    {
      stopFlag = 1;
      Serial.println("Stopped");
      if (saveDataFlag)
      {
        SDCardSaveToFile();
      }
    }
  }
}

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

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

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// ROBOT PARAMS
#define NB_MOTORS 1 // number of motors


// CAN driver link functions
struct DriverCommand {
  uint8_t kp;
  uint8_t kv;
  uint32_t q_d;
  int16_t q_dot_d;
  int16_t tau_ff;
};

union StatePacket{
	struct{
		int32_t q;
		int16_t q_dot;
		int16_t tau;
	}data;
	uint8_t buffer[8];
};

union  StatePacket   can1_states[NB_MOTORS];
struct DriverCommand can1_cmds[NB_MOTORS];
CAN_message_t can1_cmd_msgs[NB_MOTORS];


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


void canSniff(const CAN_message_t &msg) {
  // print_counter++;
  // if(print_counter>50)
  // {
  //   print_counter =0;
  //   if(false)
  //   {
  //     Serial.print("MB "); Serial.print(msg.mb);
  //     // Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  //     // Serial.print("  LEN: "); Serial.print(msg.len);
  //     // Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  //     // Serial.print(" TS: "); Serial.print(msg.timestamp);
  //     Serial.print(" ID: "); Serial.print(msg.id, HEX);
  //     // Serial.print(" Buffer: ");
  //     for ( uint8_t i = 0; i < msg.len; i++ ) {
  //       // Serial.print(msg.buf[i], HEX); Serial.print(" ");
  //       can1_states[0].buffer[i] = msg.buf[i];
  //     }
  //     Serial.print(" q: "); Serial.print(can1_states[0].data.q);
  //     Serial.print(" q_dot: "); Serial.print(can1_states[0].data.q_dot);
  //     Serial.print(" tau: "); Serial.print(can1_states[0].data.tau);
  //     Serial.println();
  //   }
  //   else
  //   {
  //     for ( uint8_t i = 0; i < msg.len; i++ ) {
  //       can1_states[0].buffer[i] = msg.buf[i];
  //     }
  //     // Serial.print(can1_states[0].data.q);
  //     Serial.print(","); Serial.print(can1_states[0].data.q_dot);
  //     Serial.print(","); Serial.print(can1_states[0].data.tau);
  //     Serial.println();
  //   }
  // }
}

void packCmdMessage(CAN_message_t *msg, struct DriverCommand *cmd)
{
  uint32_t q_d = 2097151 + cmd->q_d;
  int16_t q_dot_d = 511 + cmd->q_dot_d;
  msg->buf[0]=cmd->kp;
  msg->buf[1]=cmd->kv;
  msg->buf[7] = 0xff&cmd->tau_ff;
  msg->buf[6] = cmd->tau_ff >> 8;
  msg->buf[2] = (q_dot_d>>2)&0xff;
  msg->buf[3] = (q_dot_d& 0b11)<<6;
  msg->buf[3] |= ((q_d >> (16)) & 0b00111111);
  msg->buf[4]  = (q_d >> (8))&0xff;
  msg->buf[5]  = (q_d)&0xff;
}

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;
bool transmit_flag = false;
void daq_transaction_callback()
{
  transmit_flag = true;
}

IntervalTimer daqLoggerTimer;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 18;
const int LOADCELL_SCK_PIN = 19;
HX711 scale;

void motor_cmd_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
  motor_cmd = msg->data;
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "odri2-node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
  &state_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "motor_state")); 
  
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &motor_cmd_subsriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor_cmd"));

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

  rcl_publisher_fini(&wrench_publisher, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

float convertToGrams(long raw)
{
  return -(float)(raw)*0.0100619-803.009;
}

void setup() {

  #if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  #else
  byte local_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };
  IPAddress local_ip(192, 168, 1, 177);
  IPAddress agent_ip(192, 168, 1, 113);
  size_t agent_port = 8888;
  set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip, agent_port);
  #endif
  pinMode(LED_PIN, OUTPUT);
  state = WAITING_AGENT;

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  //Initilize the connection to the motor driver
  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();
  // daqLoggerTimer.begin(daq_transaction_callback, 100);
}

void loop() {
  long reading = 0;
  Can0.events();
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
        
        if (scale.is_ready()) {
          reading = scale.read();
          int64_t time_nano = rmw_uros_epoch_nanos()%1000000000;
          int64_t time_sec = rmw_uros_epoch_millis()/1e3;
          wrench_msg.header.stamp.nanosec = time_nano;
          wrench_msg.header.stamp.sec = time_sec;
          wrench_msg.wrench.torque.z = 0.8936*(convertToGrams(reading)*(0.125*0.5)/100*1.15)+0.08;//m(g)*L(m)/100(g -> N)
          rcl_publish(&wrench_publisher, &wrench_msg, NULL);
          //send the command to the motor driver
          can1_cmds[0].q_d = 0;
          can1_cmds[0].q_dot_d = 0;
          can1_cmds[0].tau_ff = motor_cmd;
          can1_cmds[0].kp = 0;
          can1_cmds[0].kv = 0;    
          packCmdMessage(&can1_cmd_msgs[0], &can1_cmds[0]);
          can1_cmd_msgs[0].id = 0x123;
          Can0.write(can1_cmd_msgs[0]);
        }
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


void loop()
{
  while (stopFlag)
  {  };

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