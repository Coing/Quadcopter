#ifndef __ALTITUDE_H__
#define __ALTITUDE_H__

#include "stm32f10x.h"
#include "pid.h"

void filter_hcsr( float alt);

extern unsigned int ur_time;	//ʱ��
//extern float rang;		//����
//extern float speed;		//��ֱ�ٶ�
extern u8 altHold;

extern PidObject altHoldPID;

typedef struct
{
	unsigned int flag;
	unsigned int  count;

	float  distance;	   		//��������ʵ�ʾ���(m)
	float  altitude;			//ͨ���Ƕȼ����õ��ĸ߶� ����ֵΪ��������ˮƽ�����ʵ�ʾ��룩
	float  speed;
	
//	struct _temperature temperature;  //����
} _ult_data;

extern _ult_data ult_data;


float stabilizerAltHoldUpdate(u8 mode, int16_t THROTTLE);

float ult_pid_computer(float exp_height, float current_height,  float current_speed);


void AltControlInit(void);

#endif


