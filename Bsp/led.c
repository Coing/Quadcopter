/***********************************************
 * 文件名  ：led.c
 * 描述    ：led 应用函数库         
 * 硬件连接：-----------------
 *          |   PA8 - LED0     |
 *					|	  PD2 - LED1     |
 *           ----------------- 
 * 库版本  ：ST3.5.0
**************************************************/
#include "stm32f10x.h"
#include "led.h"


/*
 * 函数名：LED_INIT
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void LED_Init(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启GPIOD的外设时钟*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE); 

	/*开启GPIOA的外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	/*选择要控制的GPIOB引脚*/															   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	

	/*设置引脚模式为通用推挽输出*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*设置引脚速率为50MHz */   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*调用库函数，初始化GPIOA*/
  GPIO_Init(GPIOA, &GPIO_InitStructure);		  

	/*选择要控制的GPIOD引脚*/	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	
	/*设置引脚速率为50MHz */ 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	
	/*设置引脚模式为通用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	
	/*调用库函数，初始化GPIOA*/
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* 关闭led0灯	*/
	GPIO_SetBits(GPIOA, GPIO_Pin_8);	
	
	/* 关闭led1灯	*/
	GPIO_SetBits(GPIOD, GPIO_Pin_2); 
}


void Delay_ms_led(u16 nms)
{	
	uint16_t i,j;
	for(i=0;i<nms;i++)
		for(j=0;j<8500;j++);
}



void LED_Flash(void)
{
	//打开所有的led灯
	LEDALL_ON;
	
	//延时一下
	Delay_ms_led(100);
	
	//关闭所有的led灯
	LEDALL_OFF;
	
	//延时一下
	Delay_ms_led(100);
	
	//打开所有的led灯
	LEDALL_ON;
	
	//延时一下
	Delay_ms_led(100);
	
	//关闭所有的led灯
	LEDALL_OFF;
	
	//延时一下
	Delay_ms_led(100);
	
	//打开所有的led灯
	LEDALL_ON;
	
	//延时一下
	Delay_ms_led(100);
	
	//关闭所有的led灯
	LEDALL_OFF;
	
	//延时一下
	Delay_ms_led(100);
	
	//打开所有的led灯
	LEDALL_ON;

	//延时一下
	Delay_ms_led(100);
	
	//关闭所有的led灯
	LEDALL_OFF;

	//延时一下
	Delay_ms_led(100);

	//打开所有的led灯
	LEDALL_ON;

	//延时一下
	Delay_ms_led(100);
	
	//关闭所有的led灯
	LEDALL_OFF;

	//延时一下
	Delay_ms_led(100);
	
	//打开所有的led灯
	LEDALL_ON;
	
	//延时一下
	Delay_ms_led(100);
	
	//关闭所有的led灯
	LEDALL_OFF;
	
	//延时一下
	Delay_ms_led(100);
}

