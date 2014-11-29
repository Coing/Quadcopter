/******************** (C) COPYRIGHT **********************************************
 * �ļ���  ��main.c
 * ����    : PWMʵ��     
 * ʵ��ƽ̨�����Ʒ�����
 * ��汾  ��ST3.5.0
 * ����    ���ǳ�
**********************************************************************************/
#include "stm32f10x.h"
#include "math.h"
#include "bsp.h"



/*
 * ��������main
 * ����  ��������
 * ����  ����
 * ���  ����
 */
 


Axis3i16 		Acc,Gyr;	//�����ۺϺ�Ĵ���������

 
Axis3i16			Acc_AVG;
//IIR�˲�
Axis3i32   MYaccelStoredFilterValues;
uint8_t    MYimuAccLpfAttFactor;
/*-----��̬��--------------------------------------
 g_Pitch 		��Χ-180��~+180�� -->�����¸�
 g_Roll 		��Χ-90��~+90��		-->������
 g_Yaw 			��Χ-180��~+180�� -->��ʱ��Ϊ��,˳ʱ��Ϊ��
-------------------------------------------------*/
T_float_angle 		Att_Angle;	//ATT�������������̬��

// ������ ���� ����
Ellipse EllipseFIT;

T_RC_Data 			Rc_PPM;		//ң��ͨ������


u8 Send_Status,Send_Senser;

void Delay_(u16 nms)
{	
	uint16_t i,j;
	for(i=0;i<nms;i++)
		for(j=0;j<8500;j++);
};

	static u8 altHoldCounter = 0;
	static u8	sonar_data_cnt = 0;			//��������
	//��ྫ�ȿɴ�ߵ�3mm��2λ��Ч���Ǿͣ�����һλ����
	uint16_t  sonarAlt = 0;           //to think about the unit    ������ܸĳ�float�𣿣���
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
	
	//IIR �˲�
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
		//������Ҫ��һ��ѿ�����.. 
		if(FLAG_ATT)							//1ms����һ��
		{
			FLAG_ATT = 0;
			att_cnt++;
			rc_cnt++;
			
			
			if(rc_cnt==10)  //10ms  100hz
			{
				rc_cnt = 0;
				
				Rc_GetValue(&Rc_PPM);
				//ARMED = 0;
				

				//����
				if( (Rc_PPM.THROTTLE < THROTTLE_MIN_RANGE_MAX && Rc_PPM.THROTTLE > THROTTLE_MIN_RANGE_MIN) && (Rc_PPM.YAW > YAW_LIMIT_MAX) )
				{
					cnt_arm++;
					if(cnt_arm == 5)//���
					{
						ARMED = 0;
						LED1_Y = 1;		//��� �Ƶ�
					  cnt_arm = 0;
					}
					
				}
				//THROTTLE��960���� , YAW : 1007����  ����
				else if( (Rc_PPM.THROTTLE < THROTTLE_MIN_RANGE_MAX && Rc_PPM.THROTTLE > THROTTLE_MIN_RANGE_MIN) && (Rc_PPM.YAW < YAW_MIN_RANGE_MAX && Rc_PPM.YAW > YAW_MIN_RANGE_MIN))
				{
					cnt_arm++;
					if(cnt_arm == 20)
					{
						ARMED = 1;
						LED1_Y = !LED1_Y;		//���� �Ƶ� ��
					  cnt_arm = 0;
					}
				}
				else
				{
					cnt_arm = 0;
				}
				
				//altHold = 0;
				//���߱�־λ���������𣿣�
				if((Rc_PPM.AUX4 < 1500)&&(Rc_PPM.AUX4 > 900))
				{
					altHold = 1;
				}
				else if(Rc_PPM.AUX4 > 1500)  //ֱ�ӷ����������
				{
					altHold = 0;
				}
				
				
				if( ARMED == 1)
				{
					LED1_Y = 0;		//�ѽ��� �Ƶ���
				}
				else
				{
					LED1_Y = 1;		//δ���� �Ƶ���
				}
			
				
			}
			
			if(att_cnt==1) //1ms  1000hz   ��ʵһ�ξ����ˣ�������������
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

				Prepare_Data(&Acc,&Acc_AVG);	//�����˲�
				//IIR�˲�
//				imuAccIIRLPFilter(&Acc, &Acc_AVG, &MYaccelStoredFilterValues,
//                    (int32_t)MYimuAccLpfAttFactor);
				
				/***************************************/
				/*��������ʱδ�ã� �ܵ�ظ������� */
				hmc5883lGetHeading(&magT.x, &magT.y, &magT.z);
				
				magT.x = (magT.x - EllipseFIT.x0)*Ellipse_To_Round_XY ;
				magT.y = magT.y - EllipseFIT.y0 ;
				magT.z = magT.z*Ellipse_To_Round_YZ;
				/***************************************/
				/* ��̬���� */
				IMUupdate(&Gyr,&Acc_AVG,&Att_Angle);
				
			

			altHoldCounter++;
			if(altHoldCounter >= 5 )   //10ms  100hz����altpid
			{
				altHoldCounter = 0;
				Alt_ExpThrottle = stabilizerAltHoldUpdate(altHold,Rc_PPM.THROTTLE);
				
			}

      //if( (altHold) && (sonarAlt < 28000)&&(sonarAlt > 3000) )//2.5m,�����Χ�ڶ��� 0.3-2.8m
      if( (altHold)&&(sonarAlt < 28000))
			{
        // Use thrust from altitude Calculate 
				Rc_PPM.THROTTLE = Alt_ExpThrottle;
      }
				
				
				
				
				/* ������� */
				Control_angle(&Att_Angle,&Gyr,&Rc_PPM,ARMED);
				
				senser_cnt++;			//ÿ2ms +1
				status_cnt++;
				if(senser_cnt==5) //10ms  100hz ����״̬
				{
					senser_cnt = 0;
					Send_Senser = 1;
					Data_Exchange();
				}
				if(status_cnt==5) //10ms  100hz ����״̬
				{
					status_cnt = 0;
					Send_Status = 1;
					Data_Exchange();
				}
				
			}

		
			
			sonar_data_cnt++;
			if(sonar_data_cnt >= 60)  //60ms  1/60(hz)  ������pdf˵�� ����60ms
			{
				LED0_R = !LED0_R ;//  ��������������  �ȷ������,��ȻƵ��̫����
				
				sonar_data_cnt = 0;
				hcsr04_get_distance(&sonarAlt);
				/* �˲� �� �� ���� */
				filter_hcsr(sonarAlt);
				
				
				
			}
			
		
			
			
			
			
		}
		

	}
}






