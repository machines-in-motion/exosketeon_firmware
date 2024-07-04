#ifndef __USER_PARAMS__
#define __USER_PARAMS__

/***************** MOTOR PARAMETERS  ******************************/

#define POLE_PAIR_NUM          21 /* Number of motor pole pairs */
#define RS                     0.33 /* Stator resistance , ohm*/
#define LS                     1.4e-4
/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define NOMINAL_CURRENT         16328
#define MOTOR_MAX_SPEED_RPM     1194 /*!< Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT  7.4 /*!< Volts RMS ph-ph /kRPM */
/***************** MOTOR SENSORS PARAMETERS  ******************************/
#define M1_ENCODER_PPR             1024  // Number of pulses per revolution


/***************** INTERFACE PARAMETERS  ******************************/
#define CAN_ID 0x124

/***************** DRIVER PARAMETERS  ******************************/
#define KP_MAX 50
#define KV_MAX 10
#define Q_MAX  62830
#define Q_DOT_MAX  10000
#define I_MAX 10

#endif