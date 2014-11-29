#ifndef _DATA_TRANSFER_H_
#define _DATA_TRANSFER_H_

#include "arhs_types.h"
#include "imu.h"


extern u8 Send_Status,Send_Senser;
extern T_float_angle 	Att_Angle;	//ATT函数计算出的姿态角


void Data_Exchange(void);

void Data_Send_Status(void);	
void Data_Send_Senser(void);	

extern Axis3i16 		Acc,Gyr;	//两次综合后的传感器数据
extern Axis3i16   magT;

extern uint16_t  sonarAlt; 

#endif
