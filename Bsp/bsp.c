#include "stm32f10x.h"
#include "bsp.h"

u8 SYS_INIT_OK = 0;
__IO u8 FLAG_ATT=0;


void Board_init(void)
{
	NVIC_Configuration();															//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	
	measure_time_init(72);														//SYSTICK��ʼ����systickΪ72/8M
	
	
	
	Uart1_Init(115200);																//���ڳ�ʼ������û��RX�ж�,�������ż���DMA
	
	LED_Init();																				//LED��ʼ����������Ϊ�ر�����������
	
	Moto_Init();																			//PWM��ʼ��
	

//	delay_ms(20);
//	Moto_Pwm_Update(1000,0,1000,0);
//	delay_ms(20);
//	Moto_Pwm_Update(1200,0,1200,0);
//	delay_ms(200);
	
	TIM1_Int_Init(500,72);														//TIM1��ʼ����0.5ms�ж�һ��,1Mhz�ļ���Ƶ��  
	
	PPM_Init();																				//PPM�����ʼ��
	
	i2cdevInit(I2C1);																	//��ʼ��I2C
  mpu6050Init(I2C1);																//��ʼ��mpu6050��ַ
  if (mpu6050TestConnection() == TRUE)							
  {
    printf("MPU6050 I2C connection [OK].\r\n");
  }
  else
  {
    printf("MPU6050 I2C connection [FAIL].\r\n");
  }
	mpu6050Reset();
  delay_ms(50);
  // Activate MPU6050
  mpu6050SetSleepEnabled(FALSE);
  // Enable temp sensor
  mpu6050SetTempSensorEnabled(TRUE);
  // Disable interrupts
  mpu6050SetIntEnabled(FALSE);
  // Connect the HMC5883L to the main I2C bus ����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
  mpu6050SetI2CBypassEnabled(TRUE);
  // Set x-axis gyro as clock source
  mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
  // Set gyro full scale range
  mpu6050SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  // Set accelerometer full scale range
  mpu6050SetFullScaleAccelRange(MPU6050_ACCEL_FS_4);
	// Set output rate (1): 1000 / (1 + 1) = 500Hz
  mpu6050SetRate(1);
  // Set digital low-pass bandwidth
  mpu6050SetDLPFMode(MPU6050_DLPF_BW_42);


	hmc5883lInit(I2C1);
  if (hmc5883lTestConnection() == TRUE)
  {
    //isHmc5883lPresent = TRUE;
    printf("HMC5883 I2C connection [OK].\r\n");
  }
  else
  {
    printf("HMC5883L I2C connection [FAIL].\r\n");
  }
	if (ms5611Init(I2C1) == TRUE)
  {
    //isMs5611Present = TRUE;
    printf("MS5611 I2C connection [OK].\r\n");
  }
  else
  {
    printf("MS5611 I2C connection [FAIL].\r\n");
  }
	mpu6050SelfTest();
	hmc5883lSelfTest();
	ms5611SelfTest();
	
	arhs_Calibrated();												//��ʱֻ��mpu6050, ��ʵ������bias ƫ�� 
	
	/*���������2��O(��_��)O����~���ѳ��������жϵȼ�����һ�£���˵֮ǰ�����˰ɡ�������*/
	hcsr04_init();//1.Ӳ������������	�ĳ�float�����������Կ��� ֻ��ʹ��int 2.ʱ��ʱ����,���ػ����ǵ�Դ����
}
