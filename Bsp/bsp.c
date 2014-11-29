#include "stm32f10x.h"
#include "bsp.h"

u8 SYS_INIT_OK = 0;
__IO u8 FLAG_ATT=0;


void Board_init(void)
{
	NVIC_Configuration();															//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	
	measure_time_init(72);														//SYSTICK初始化，systick为72/8M
	
	
	
	Uart1_Init(115200);																//串口初始化，还没加RX中断,可以试着加入DMA
	
	LED_Init();																				//LED初始化，可能以为回被用做其他了
	
	Moto_Init();																			//PWM初始化
	

//	delay_ms(20);
//	Moto_Pwm_Update(1000,0,1000,0);
//	delay_ms(20);
//	Moto_Pwm_Update(1200,0,1200,0);
//	delay_ms(200);
	
	TIM1_Int_Init(500,72);														//TIM1初始化，0.5ms中断一次,1Mhz的计数频率  
	
	PPM_Init();																				//PPM解码初始化
	
	i2cdevInit(I2C1);																	//初始化I2C
  mpu6050Init(I2C1);																//初始化mpu6050地址
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
  // Connect the HMC5883L to the main I2C bus 主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
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
	
	arhs_Calibrated();												//暂时只是mpu6050, 其实就是求bias 偏置 
	
	/*完美解决了2，O(∩_∩)O哈哈~，把超声波的中断等级设置一下，话说之前设置了吧。。算了*/
	hcsr04_init();//1.硬件出错。。？？	改成float。。单独测试可以 只能使用int 2.时不时死机,严重怀疑是电源问题
}
