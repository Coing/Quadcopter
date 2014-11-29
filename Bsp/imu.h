#ifndef _IMU_H_
#define _IMU_H_

//#include "control.h"
#include "stm32f10x.h"
#include "arhs_types.h"




//extern float 	AngleOffset_Rol,AngleOffset_Pit; 

//void Prepare_Data(T_int16_xyz *acc_in,T_int16_xyz *acc_out);

//���Ľ��� ����ֿ� ahrs��imu ���� imu���ˣ�
void AHRSupdate(Axis3i16 *gyr, Axis3i16 *acc,Axis3i16 *mag, T_float_angle *angle);
				
void IMUupdate(Axis3i16 *gyr, Axis3i16 *acc, T_float_angle *angle);
				


#endif
