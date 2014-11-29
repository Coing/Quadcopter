#ifndef __CONTROL_STABLIZE_H__
#define __CONTROL_STABLIZE_H__

#include "stm32f10x.h"


#include "arhs_types.h"

#include "PPM.h"

void Control_angle(T_float_angle *att_in,Axis3i16 *gyr_in, T_RC_Data *rc_in, u8 armed);

void PID_SET(float KP,float KI,float KD,float YAW_KP,float YAW_KI,float YAW_KD);


/**
 * IMU update frequency dictates the overall update frequency.
 */
#define IMU_UPDATE_FREQ   500
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)



//#define PID_ROLL_RATE_KP  70.0
//#define PID_ROLL_RATE_KI  0.0
//#define PID_ROLL_RATE_KD  0.0
//#define PID_ROLL_RATE_INTEGRATION_LIMIT    100.0

//#define PID_PITCH_RATE_KP  70.0
//#define PID_PITCH_RATE_KI  0.0
//#define PID_PITCH_RATE_KD  0.0
//#define PID_PITCH_RATE_INTEGRATION_LIMIT   100.0

//#define PID_YAW_RATE_KP  50.0
//#define PID_YAW_RATE_KI  25.0
//#define PID_YAW_RATE_KD  0.0
//#define PID_YAW_RATE_INTEGRATION_LIMIT     500.0

#endif

