#ifndef __ALTITUDE_H__
#define __ALTITUDE_H__

#include "stm32f10x.h"
#include "pid.h"

void filter_hcsr( float alt);

extern unsigned int ur_time;	//时间
//extern float rang;		//距离
//extern float speed;		//垂直速度
extern u8 altHold;

extern PidObject altHoldPID;

typedef struct
{
	unsigned int flag;
	unsigned int  count;

	float  distance;	   		//测量出的实际距离(m)
	float  altitude;			//通过角度计算后得到的高度 （此值为飞行器与水平地面的实际距离）
	float  speed;
	
//	struct _temperature temperature;  //气温
} _ult_data;

extern _ult_data ult_data;


float stabilizerAltHoldUpdate(u8 mode, int16_t THROTTLE);

float ult_pid_computer(float exp_height, float current_height,  float current_speed);


void AltControlInit(void);

#endif


