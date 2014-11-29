#include "stm32f10x.h"
#include "Parameters.h"



//2个以上地方用到的全局变量，其实可以 放在一个文件的，待改进

int32_t Gyro_xoffset = 0,Gyro_yoffset = 0,Gyro_zoffset = 0;
int32_t Accel_xoffset = 0,Accel_yoffset = 0,Accel_zoffset = 0;


Axis3i16   	magT;

