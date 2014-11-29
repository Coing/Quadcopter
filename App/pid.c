/**
 *
 * pid.c - implementation of the PID regulator
 */
#include "stm32f10x.h"
#include <math.h>

#include "pid.h"
#include "led.h"

#include "control_stabilize.h"



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

//#define PID_ROLL_KP  3.5
//#define PID_ROLL_KI  2.0
//#define PID_ROLL_KD  0.0
//#define PID_ROLL_INTEGRATION_LIMIT    20.0

//#define PID_PITCH_KP  3.5
//#define PID_PITCH_KI  2.0
//#define PID_PITCH_KD  0.0
//#define PID_PITCH_INTEGRATION_LIMIT   20.0

//#define PID_YAW_KP  0.0
//#define PID_YAW_KI  0.0
//#define PID_YAW_KD  0.0
//#define PID_YAW_INTEGRATION_LIMIT     360.0



void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;
  pid->dt        = dt;
}

//float pidUpdate(PidObject* pid, const float measured, const bool updateError)
//{
//    float output;

//    if (updateError)
//    {
//        pid->error = pid->desired - measured;
//    }

//    pid->integ += pid->error * pid->dt;
//    if (pid->integ > pid->iLimit)
//    {
//        pid->integ = pid->iLimit;
//    }
//    else if (pid->integ < pid->iLimitLow)
//    {
//        pid->integ = pid->iLimitLow;
//    }

//    pid->deriv = (pid->error - pid->prevError) / pid->dt;

//    pid->outP = pid->kp * pid->error;
//    pid->outI = pid->ki * pid->integ;
//    pid->outD = pid->kd * pid->deriv;

//    output = pid->outP + pid->outI + pid->outD;

//    pid->prevError = pid->error;

//    return output;
//}

/*----------------------------------------------pid输出更新------------------------------------------*/
//输入参数：pid结构体指针，测量值 ,期望值
//输出：pid输出
float pidUpdate(PidObject* pid, const float measured,float expect,float gyro)
{
  float output = 0;
  //static float lastoutput=0;

  pid->desired=expect;			 				//获取期望角度

  pid->error = pid->desired - measured;	 	  //偏差：期望-测量值
  
  pid->integ += pid->error * IMU_UPDATE_DT;	  //偏差积分
 
  if (pid->integ > pid->iLimit)				  //作积分限制
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)
  {
    pid->integ = -pid->iLimit;
  }				 

 // pid->deriv = (pid->error - pid->prevError) / IMU_UPDATE_DT;		//微分	 应该可用陀螺仪角速度代替
  pid->deriv = -gyro;
//  if(fabs(pid->error)>2)									//pid死区
//  {
//		  pid->outP = pid->kp * pid->error;								 //方便独立观察
//		  pid->outI = pid->ki * pid->integ;
//		  pid->outD = pid->kd * pid->deriv;
//		
//		  output = (pid->kp * pid->error) +
//		           (pid->ki * pid->integ) +
//		           (pid->kd * pid->deriv);
//  }
//  else
//  {
//  		  output=lastoutput;
//  }

			pid->outP = pid->kp * pid->error;								 //方便独立观察
		  pid->outI = pid->ki * pid->integ;
		  pid->outD = pid->kd * pid->deriv;
		
		  output = (pid->kp * pid->error) +
		           (pid->ki * pid->integ) +
		           (pid->kd * pid->deriv);
  pid->prevError = pid->error;							 		//更新前一次偏差
  //lastoutput=output;

  return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->iLimit = limit;
}


void pidSetIntegralLimitLow(PidObject* pid, const float limitLow) {
    pid->iLimitLow = limitLow;
}

void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
  pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
  return pid->desired;
}

bool pidIsActive(PidObject* pid)
{
  bool isActive = TRUE;

  if (pid->kp < 0.0001 && pid->ki < 0.0001 && pid->kd < 0.0001)
  {
    isActive = FALSE;
  }

  return isActive;
}

void pidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}
void pidSetDt(PidObject* pid, const float dt) {
    pid->dt = dt;
}
