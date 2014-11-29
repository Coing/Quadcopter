/***********************************************
 * �ļ���  ��led.c
 * ����    ��led Ӧ�ú�����         
 * Ӳ�����ӣ�-----------------
 *          |   PA8 - LED0     |
 *					|	  PD2 - LED1     |
 *           ----------------- 
 * ��汾  ��ST3.5.0
**************************************************/
#include "stm32f10x.h"
#include "led.h"


/*
 * ��������LED_INIT
 * ����  ������LED�õ���I/O��
 * ����  ����
 * ���  ����
 */
void LED_Init(void)
{		
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����GPIOD������ʱ��*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE); 

	/*����GPIOA������ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	/*ѡ��Ҫ���Ƶ�GPIOB����*/															   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	

	/*��������ģʽΪͨ���������*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*������������Ϊ50MHz */   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*���ÿ⺯������ʼ��GPIOA*/
  GPIO_Init(GPIOA, &GPIO_InitStructure);		  

	/*ѡ��Ҫ���Ƶ�GPIOD����*/	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	
	/*������������Ϊ50MHz */ 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	
	/*��������ģʽΪͨ���������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	
	/*���ÿ⺯������ʼ��GPIOA*/
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* �ر�led0��	*/
	GPIO_SetBits(GPIOA, GPIO_Pin_8);	
	
	/* �ر�led1��	*/
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
	//�����е�led��
	LEDALL_ON;
	
	//��ʱһ��
	Delay_ms_led(100);
	
	//�ر����е�led��
	LEDALL_OFF;
	
	//��ʱһ��
	Delay_ms_led(100);
	
	//�����е�led��
	LEDALL_ON;
	
	//��ʱһ��
	Delay_ms_led(100);
	
	//�ر����е�led��
	LEDALL_OFF;
	
	//��ʱһ��
	Delay_ms_led(100);
	
	//�����е�led��
	LEDALL_ON;
	
	//��ʱһ��
	Delay_ms_led(100);
	
	//�ر����е�led��
	LEDALL_OFF;
	
	//��ʱһ��
	Delay_ms_led(100);
	
	//�����е�led��
	LEDALL_ON;

	//��ʱһ��
	Delay_ms_led(100);
	
	//�ر����е�led��
	LEDALL_OFF;

	//��ʱһ��
	Delay_ms_led(100);

	//�����е�led��
	LEDALL_ON;

	//��ʱһ��
	Delay_ms_led(100);
	
	//�ر����е�led��
	LEDALL_OFF;

	//��ʱһ��
	Delay_ms_led(100);
	
	//�����е�led��
	LEDALL_ON;
	
	//��ʱһ��
	Delay_ms_led(100);
	
	//�ر����е�led��
	LEDALL_OFF;
	
	//��ʱһ��
	Delay_ms_led(100);
}

