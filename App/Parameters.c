#include "stm32f10x.h"
#include "Parameters.h"



//2�����ϵط��õ���ȫ�ֱ�������ʵ���� ����һ���ļ��ģ����Ľ�

int32_t Gyro_xoffset = 0,Gyro_yoffset = 0,Gyro_zoffset = 0;
int32_t Accel_xoffset = 0,Accel_yoffset = 0,Accel_zoffset = 0;


Axis3i16   	magT;

