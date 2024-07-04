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
    //motor related data
    float q;
    float dq;
    float motor_q;
    float motor_dq;
    // imu related data
    float quaternion_base[4];
    float quaternion_shoulder[4];
    float quaternion_hand[4];
    float base_quality;
    float shoulder_quality;
    float hand_quality;
};

union shm_cmd{
 JointCmd data[NUM_ACTUATORS];
 unsigned char buffer[NUM_ACTUATORS*sizeof(JointCmd)];
};

union shm_state{
 JointState data;
 unsigned char buffer[sizeof(JointState)];
};

shm_cmd shm_cmd_t;
shm_state shm_state_t;

#endif