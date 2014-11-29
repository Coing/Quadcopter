#ifndef  __PARAMETER_H_
#define  __PARAMETER_H_

#include "stm32f10x.h"
#include "arhs_types.h"





/*

						希望做成 串级控制  

				内环  500HZ											外环	  200HZ(预计)
			roll   PI												pitch		PID
rate{	pitch	 PI			angle(attitude){	Roll		PID
			yaw    PI												yaw			0(no control)


*/










//加速度计，陀螺仪 零偏 ，该改成 结构体的，放在他所在的文件 static类型。 凌乱

extern int32_t Gyro_xoffset ,Gyro_yoffset ,Gyro_zoffset ;
extern int32_t Accel_xoffset ,Accel_yoffset ,Accel_zoffset ;


//磁力计数据
extern Axis3i16   magT;


/*
	磁力计矫正有关参数
*/
typedef struct {
			 float x0;
			 float y0;
			 float a;
			 float b;
} Ellipse;
 
#define Ellipse_To_Round_XY 1.0243902				//xsf = 210/205       Ysf = 1 以y为基准
#define Ellipse_To_Round_YZ 0.53283767      //zsf = (Yma - Ymin) / (Zmax- Zmin)


		//RC limit
#define THROTTLE_MIN_RANGE_MIN 900				//确定真的有值
#define	THROTTLE_MIN_RANGE_MAX 1000			  //最小值区间
#define YAW_MIN_RANGE_MIN 		 900
#define YAW_MIN_RANGE_MAX			 1100
#define YAW_LIMIT_MAX			 		 1900


#endif
