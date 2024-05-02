#ifndef __SHM__
#define __SHM__
#define NUM_ACTUATORS 1

struct JointCmd
{
    float q_des;
    float dq_des;
    float kp;
    float kv;
    float tau_ff;
    float q_offset;
};

struct JointState
{
    float q;
    float dq;
    float tau_est;
    float temp;
    float motor_q;
    float motor_dq;
    float base_ori[4];
    float shoulder_ori[4];
    float wrist_ori[4];

};

typedef struct JointCmd JointCmd_t;
typedef struct JointState JointState_t;

typedef struct SharedMemory SharedMemory_t;


union{
 JointCmd_t data[NUM_ACTUATORS];
 unsigned char buffer[NUM_ACTUATORS*sizeof(JointCmd_t)];
}shm_cmd;

union{
 JointState_t data[NUM_ACTUATORS];
 unsigned char buffer[NUM_ACTUATORS*sizeof(JointState_t)];
}shm_state;

#endif