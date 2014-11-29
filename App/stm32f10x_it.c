/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h" 
#include "bsp.h"
#include "i2croutines.h"
#include "i2cdev.h"

//#include "STM32_I2C.h"

extern u16 ms1_cnt;


void I2C1_EV_IRQHandler(void)
{
  i2cInterruptHandlerI2c1();
}

void I2C1_ER_IRQHandler(void)
{
  i2cErrorInterruptHandlerI2c1();
}
///*=====================================================================================================*/
///*=====================================================================================================*/
//void DMA1_Channel6_IRQHandler( void )
//{
//	I2C1_Send_DMA_IRQ();
//}
///*=====================================================================================================*/
///*=====================================================================================================*/
//void DMA1_Channel7_IRQHandler( void )
//{
//	I2C1_Recv_DMA_IRQ();
//}


void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
	Moto_Pwm_Update(900,900,900,900);
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  Moto_Pwm_Update(900,900,900,900);
	while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 
void SysTick_Handler(void)
{
}

void USART1_IRQHandler(void)
{
	Uart1_IRQ();
	/*
	u8 c;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 	
	    c=USART1->DR;
	  	printf("%c",c);    //将接受到的数据直接返回打印
	} 
	  */
}

//static uint8_t buffer[14];

void TIM1_UP_IRQHandler(void)   //TIM1中断 0.5MS一次主中断
{
	static u8 ms1_cnt=0;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
			ms1_cnt++;
			if(ms1_cnt==2)																//每两次中断执行一次,1ms
			{
				ms1_cnt = 0;
				
				FLAG_ATT = 1;
			}
//	i2cdevInit(I2C1);		
//	mpu6050Init(I2C1);		
//	if (mpu6050TestConnection() == TRUE)
//  {
//    printf("MPU6050 I2C connection [OK].\r\n");
//  }
//  else
//  {
//    printf("MPU6050 I2C connection [FAIL].\r\n");
//  }
//			if(SYS_INIT_OK == 1)
//			{
//		delay_ms(10);
//		i2cdevRead(I2C1, 0x68, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
//		delay_ms(10);
//			}

		}
		//printf("MPU6050 I2C connection [FAIL].\r\n");
		//Data_Exchange();
}



/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
