/******************** (C) COPYRIGHT **********************************************
 * 文件名  ：main.c
 * 描述    : PWM实验     
 * 实验平台：自制飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：星辰
**********************************************************************************/
#include "stm32f10x.h"
#include "math.h"
#include "bsp.h"



/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 */
 


Axis3i16 		Acc,Gyr;	//两次综合后的传感器数据

 
Axis3i16			Acc_AVG;
//IIR滤波
Axis3i32   MYaccelStoredFilterValues;
uint8_t    MYimuAccLpfAttFactor;
/*-----姿态角--------------------------------------
 g_Pitch 		范围-180°~+180° -->上正下负
 g_Roll 		范围-90°~+90°		-->左负右正
 g_Yaw 			范围-180°~+180° -->逆时针为正,顺时针为负
-------------------------------------------------*/
T_float_angle 		Att_Angle;	//ATT函数计算出的姿态角

// 磁力计 矫正 参数
Ellipse EllipseFIT;

T_RC_Data 			Rc_PPM;		//遥控通道数据


u8 Send_Status,Send_Senser;

void Delay_(u16 nms)
{	
	uint16_t i,j;
	for(i=0;i<nms;i++)
		for(j=0;j<8500;j++);
};

	static u8 altHoldCounter = 0;
	static u8	sonar_data_cnt = 0;			//更新数据
	//测距精度可达高到3mm，2位有效吧那就，还有一位估读
	uint16_t  sonarAlt = 0;           //to think about the unit    这个不能改成float吗？？？
	static uint16_t Alt_ExpThrottle = 0;

int main(void)
{
	
	EllipseFIT.x0 =  -10;	
	EllipseFIT.y0 = -15;
	EllipseFIT.a = 210;
	EllipseFIT.b = 205;
	
	static u8 att_cnt=0;
	static u8 rc_cnt=0;

	Axis3i16 mpu6050_dataacc1,mpu6050_datagyr1,mpu6050_dataacc2,mpu6050_datagyr2;
	static u8 senser_cnt=0,status_cnt=0;
	
	static u8 cnt_arm=0;
	
	u8 ARMED = 0;
	
	//IIR 滤波
	MYimuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;
	
	SYS_INIT_OK=0;
	Board_init();
	
	Moto_Pwm_Update(0,0,0,0);
	delay_ms(1000);
	Moto_Pwm_Update(900,900,900,900);
	delay_ms(1000);
	delay_ms(1000);

	
	controllerInit();
	AltControlInit();

	
	SYS_INIT_OK=1;


	
	while (1)
	{	
		//这里面要做一大堆控制了.. 
		if(FLAG_ATT)							//1ms进入一次
		{
			FLAG_ATT = 0;
			att_cnt++;
			rc_cnt++;
			
			
			if(rc_cnt==10)  //10ms  100hz
			{
				rc_cnt = 0;
				
				Rc_GetValue(&Rc_PPM);
				//ARMED = 0;
				

				//加锁
				if( (Rc_PPM.THROTTLE < THROTTLE_MIN_RANGE_MAX && Rc_PPM.THROTTLE > THROTTLE_MIN_RANGE_MIN) && (Rc_PPM.YAW > YAW_LIMIT_MAX) )
				{
					cnt_arm++;
					if(cnt_arm == 5)//快点
					{
						ARMED = 0;
						LED1_Y = 1;		//灭灯 黄灯
					  cnt_arm = 0;
					}
					
				}
				//THROTTLE：960左右 , YAW : 1007左右  解锁
				else if( (Rc_PPM.THROTTLE < THROTTLE_MIN_RANGE_MAX && Rc_PPM.THROTTLE > THROTTLE_MIN_RANGE_MIN) && (Rc_PPM.YAW < YAW_MIN_RANGE_MAX && Rc_PPM.YAW > YAW_MIN_RANGE_MIN))
				{
					cnt_arm++;
					if(cnt_arm == 20)
					{
						ARMED = 1;
						LED1_Y = !LED1_Y;		//亮灯 黄灯 闪
					  cnt_arm = 0;
					}
				}
				else
				{
					cnt_arm = 0;
				}
				
				//altHold = 0;
				//定高标志位放在这里吗？？
				if((Rc_PPM.AUX4 < 1500)&&(Rc_PPM.AUX4 > 900))
				{
					altHold = 1;
				}
				else if(Rc_PPM.AUX4 > 1500)  //直接放外面就行了
				{
					altHold = 0;
				}
				
				
				if( ARMED == 1)
				{
					LED1_Y = 0;		//已解锁 黄灯亮
				}
				else
				{
					LED1_Y = 1;		//未解锁 黄灯灭
				}
			
				
			}
			
			if(att_cnt==1) //1ms  1000hz   其实一次就行了，算了先这样吧
			{	
				mpu6050GetMotion6(&mpu6050_dataacc1.x, &mpu6050_dataacc1.y, &mpu6050_dataacc1.z, &mpu6050_datagyr1.x, &mpu6050_datagyr1.y, &mpu6050_datagyr1.z);
				mpu6050_dataacc1.x = mpu6050_dataacc1.x-Accel_xoffset;
				mpu6050_dataacc1.y = mpu6050_dataacc1.y-Accel_yoffset;
				mpu6050_dataacc1.z = mpu6050_dataacc1.z-Accel_zoffset;
				mpu6050_datagyr1.x = mpu6050_datagyr1.x-Gyro_xoffset;
				mpu6050_datagyr1.y = mpu6050_datagyr1.y-Gyro_yoffset;
				mpu6050_datagyr1.z = mpu6050_datagyr1.z-Gyro_zoffset;
			}
			else				  //2ms  500hz		
			{
				att_cnt = 0;
				mpu6050GetMotion6(&mpu6050_dataacc2.x, &mpu6050_dataacc2.y, &mpu6050_dataacc2.z, &mpu6050_datagyr2.x, &mpu6050_datagyr2.y, &mpu6050_datagyr2.z);
				mpu6050_dataacc2.x = mpu6050_dataacc2.x-Accel_xoffset;
				mpu6050_dataacc2.y = mpu6050_dataacc2.y-Accel_yoffset;
				mpu6050_dataacc2.z = mpu6050_dataacc2.z-Accel_zoffset;
				mpu6050_datagyr2.x = mpu6050_datagyr2.x-Gyro_xoffset;
				mpu6050_datagyr2.y = mpu6050_datagyr2.y-Gyro_yoffset;
				mpu6050_datagyr2.z = mpu6050_datagyr2.z-Gyro_zoffset;
				
				Acc.x = (mpu6050_dataacc1.x+mpu6050_dataacc2.x)/2;
				Acc.y = (mpu6050_dataacc1.y+mpu6050_dataacc2.y)/2;
				Acc.z = (mpu6050_dataacc1.z+mpu6050_dataacc2.z)/2;
				Gyr.x = (mpu6050_datagyr1.x+mpu6050_datagyr2.x)/2;
				Gyr.y = (mpu6050_datagyr1.y+mpu6050_datagyr2.y)/2;
				Gyr.z = (mpu6050_datagyr1.z+mpu6050_datagyr2.z)/2;

				Prepare_Data(&Acc,&Acc_AVG);	//滑动滤波
				//IIR滤波
//				imuAccIIRLPFilter(&Acc, &Acc_AVG, &MYaccelStoredFilterValues,
//                    (int32_t)MYimuAccLpfAttFactor);
				
				/***************************************/
				/*磁力计暂时未用， 受电池干扰严重 */
				hmc5883lGetHeading(&magT.x, &magT.y, &magT.z);
				
				magT.x = (magT.x - EllipseFIT.x0)*Ellipse_To_Round_XY ;
				magT.y = magT.y - EllipseFIT.y0 ;
				magT.z = magT.z*Ellipse_To_Round_YZ;
				/***************************************/
				/* 姿态解算 */
				IMUupdate(&Gyr,&Acc_AVG,&Att_Angle);
				
			

			altHoldCounter++;
			if(altHoldCounter >= 5 )   //10ms  100hz计算altpid
			{
				altHoldCounter = 0;
				Alt_ExpThrottle = stabilizerAltHoldUpdate(altHold,Rc_PPM.THROTTLE);
				
			}

      //if( (altHold) && (sonarAlt < 28000)&&(sonarAlt > 3000) )//2.5m,这个范围内定高 0.3-2.8m
      if( (altHold)&&(sonarAlt < 28000))
			{
        // Use thrust from altitude Calculate 
				Rc_PPM.THROTTLE = Alt_ExpThrottle;
      }
				
				
				
				
				/* 输出控制 */
				Control_angle(&Att_Angle,&Gyr,&Rc_PPM,ARMED);
				
				senser_cnt++;			//每2ms +1
				status_cnt++;
				if(senser_cnt==5) //10ms  100hz 发送状态
				{
					senser_cnt = 0;
					Send_Senser = 1;
					Data_Exchange();
				}
				if(status_cnt==5) //10ms  100hz 发送状态
				{
					status_cnt = 0;
					Send_Status = 1;
					Data_Exchange();
				}
				
			}

		
			
			sonar_data_cnt++;
			if(sonar_data_cnt >= 60)  //60ms  1/60(hz)  超声波pdf说明 至少60ms
			{
				LED0_R = !LED0_R ;//  程序在正常运行  先放这里吧,不然频率太高了
				
				sonar_data_cnt = 0;
				hcsr04_get_distance(&sonarAlt);
				/* 滤波 ， 加 处理 */
				filter_hcsr(sonarAlt);
				
				
				
			}
			
		
			
			
			
			
		}
		

	}
}






