#ifndef  __PARAMETER_H_
#define  __PARAMETER_H_

#include "stm32f10x.h"
#include "arhs_types.h"





/*

						ϣ������ ��������  

				�ڻ�  500HZ											�⻷	  200HZ(Ԥ��)
			roll   PI												pitch		PID
rate{	pitch	 PI			angle(attitude){	Roll		PID
			yaw    PI												yaw			0(no control)


*/










//���ٶȼƣ������� ��ƫ ���øĳ� �ṹ��ģ����������ڵ��ļ� static���͡� ����

extern int32_t Gyro_xoffset ,Gyro_yoffset ,Gyro_zoffset ;
extern int32_t Accel_xoffset ,Accel_yoffset ,Accel_zoffset ;


//����������
extern Axis3i16   magT;


/*
	�����ƽ����йز���
*/
typedef struct {
			 float x0;
			 float y0;
			 float a;
			 float b;
} Ellipse;
 
#define Ellipse_To_Round_XY 1.0243902				//xsf = 210/205       Ysf = 1 ��yΪ��׼
#define Ellipse_To_Round_YZ 0.53283767      //zsf = (Yma - Ymin) / (Zmax- Zmin)


		//RC limit
#define THROTTLE_MIN_RANGE_MIN 900				//ȷ�������ֵ
#define	THROTTLE_MIN_RANGE_MAX 1000			  //��Сֵ����
#define YAW_MIN_RANGE_MIN 		 900
#define YAW_MIN_RANGE_MAX			 1100
#define YAW_LIMIT_MAX			 		 1900


#endif
