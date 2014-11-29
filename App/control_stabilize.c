#include "stm32f10x.h"
#include "math.h"

#include "moto.h"
#include "arhs_types.h"
#include "PPM.h"

#include "control_stabilize.h"
#include "pid.h"

#include "altitude.h"

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);






PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;


float pid_roll,pid_pitch,pid_yaw;

static vs16 motorPowerM1 = 0,motorPowerM2 = 0,motorPowerM3 = 0,motorPowerM4 = 0;
float yaw_p;
float PID_YAW_p;
float PID_YAW_d;

//初始化PID PID参数待改

#define PID_ROLL_KP  10.0						//15.5
#define PID_ROLL_KI  0.0								
#define PID_ROLL_KD  2.5			//5.0
#define PID_ROLL_INTEGRATION_LIMIT    200.0				//积分限制

#define PID_PITCH_KP  10.0							//15.5
#define PID_PITCH_KI  0.0								//1.0
#define PID_PITCH_KD  2.5								//5.0
#define PID_PITCH_INTEGRATION_LIMIT   400.0

#define PID_YAW_KP  1.5	//1.5  //1
#define PID_YAW_KI  0.0
#define PID_YAW_KD  -1.0	//-1.0
#define PID_YAW_INTEGRATION_LIMIT     100.0

/************************************************************************************************
函数名：void Control_angle(T_float_angle *att_in,Axis3i16 *gyr_in, T_RC_Data *rc_in, u8 armed)
说明：四旋翼控制函数，用于PWM计算和输出, 仿照匿名，待改进，改成rate，angle
入口：T_float_angle *att_in	期望姿态角
			Axis3i16 *gyr_in			陀螺仪数据
			T_RC_Data *rc_in			遥控器数据
			u8 armed							是否解锁
出口：无
备注：当前控制环为姿态控制环，没有加 速率控制环，待改进  主要应用PID
************************************************************************************************/
void Control_angle(T_float_angle *att_in,Axis3i16 *gyr_in, T_RC_Data *rc_in, u8 armed)
{
	 float EXP_rol=0,EXP_pit=0,EXP_yaw=0;
		EXP_rol = ((float)rc_in->ROLL-1500)/20.0;     //-25~25?? 500/20 小于500  3D特技是要无死角..再说吧
		EXP_pit = ((float)rc_in->PITCH-1500)/20.0;
	  //EXP_yaw = ((float)rc_in->YAW-1500)/20.0 ;						//-125~125	
		EXP_yaw = rc_in->YAW-1500;						//-125~125
	

	
		if(EXP_yaw<50&&EXP_yaw>-50)
		{
			  EXP_yaw=0;
		}
		else
		{EXP_yaw=EXP_yaw/4;}				//-125~125度
	
		gyr_in->x *= 0.0610351f;				//归一，统一单位，忘了..
		gyr_in->y *= 0.0610351f;
		gyr_in->z *= 0.0610351f;
	
	// --- PID运算-->这里的微分D运算并非传统意义上的利用前一次的误差减去上一次的误差得来
	// --- 而是直接利用陀螺仪的值来替代微分项,这样的处理非常好,因为巧妙利用了硬件设施,陀螺仪本身就是具有增量的效果

		pid_roll = pidUpdate(&pidRoll,att_in->rol,EXP_rol,gyr_in->x);
		pid_pitch = pidUpdate(&pidPitch,att_in->pit,EXP_pit,gyr_in->y);
		
		pid_yaw = pidUpdate(&pidYaw,att_in->yaw,EXP_yaw,gyr_in->z);
		
		//if((armed == 1)&&(att_in->rol < 50)&&(att_in->pit) < 50   && (rc_in->THROTTLE)>1000 )
		if((armed == 1)&&(rc_in->THROTTLE)>1000 )
		{
			//distributePower(rc_in->THROTTLE,0,pid_pitch,0);
			//distributePower(rc_in->THROTTLE,pid_roll,0,0);
			//distributePower(rc_in->THROTTLE,pid_roll,pid_pitch,0);
		distributePower(rc_in->THROTTLE,pid_roll,pid_pitch,pid_yaw);
	 	//distributePower(rc_in->THROTTLE,0,0,pid_yaw);

		}
		else
		{
			distributePower(900,0,0,0);
		}
 
}









static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
//#ifdef QUAD_FORMATION_X
//  roll = roll >> 1;
//  pitch = pitch >> 1;
//  motorPowerM1 = limitThrust(thrust - roll + pitch + yaw);
//  motorPowerM2 = limitThrust(thrust - roll - pitch - yaw);
//  motorPowerM3 =  limitThrust(thrust + roll - pitch + yaw);
//  motorPowerM4 =  limitThrust(thrust + roll + pitch - yaw);
//#else // QUAD_FORMATION_NORMAL
//  motorPowerM1 = limitThrust(thrust + pitch + yaw);
//  motorPowerM2 = limitThrust(thrust - roll - yaw);
//  motorPowerM3 =  limitThrust(thrust - pitch + yaw);
//  motorPowerM4 =  limitThrust(thrust + roll - yaw);
//#endif
	
	motorPowerM1 = (thrust + pitch + yaw);
  motorPowerM2 = (thrust - roll - yaw);
  motorPowerM3 =  (thrust - pitch + yaw);
  motorPowerM4 =  (thrust + roll - yaw);
	if(motorPowerM1<0)	motorPowerM1 = 0;													//天啊，你怎么不限制值啊，限0啊，%>_<%
	if(motorPowerM2<0)	motorPowerM2 = 0;
	if(motorPowerM3<0)	motorPowerM3 = 0;
	if(motorPowerM4<0)	motorPowerM4 = 0;
	//1-3测试，pitch
//	motorPowerM1 = thrust + pitch ;
//	motorPowerM2 = thrust - roll ;
//	motorPowerM3 =  thrust - pitch ;
//	motorPowerM4 =  thrust + roll ;

	//Moto_Pwm_Update(motorPowerM1,motorPowerM2,motorPowerM3,motorPowerM4);
	//Moto_Pwm_Update(motorPowerM1,0,motorPowerM3,0);
	//Moto_Pwm_Update(0,motorPowerM2,0,motorPowerM4);
	/*m1 5%转，m2，m3 6%转 m4 6.4%转*/
	Moto_Pwm_Update(motorPowerM1,motorPowerM2,motorPowerM3,motorPowerM4);

}


void controllerInit()
{

  pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, IMU_UPDATE_DT);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, IMU_UPDATE_DT);
  pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, IMU_UPDATE_DT);
	
	
  pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);

}



//待做，加入在线调试，usmart
void Control_PID_SET(float KP,float KI,float KD,float YAW_KP,float YAW_KI,float YAW_KD)
{
	pidSetKp(&pidRoll, (float)KP/1.0);
	pidSetKi(&pidRoll, (float)KI/1.0);
	pidSetKd(&pidRoll, (float)KD/1.0);
	pidSetKp(&pidPitch, (float)KP/1.0);
	pidSetKi(&pidPitch, (float)KI/1.0);
	pidSetKd(&pidPitch, (float)KD/1.0);
	pidSetKp(&pidYaw, (float)YAW_KP/1.0);
	pidSetKi(&pidYaw, (float)YAW_KI/1.0);
	pidSetKd(&pidYaw, (float)YAW_KD/1.0);
}
