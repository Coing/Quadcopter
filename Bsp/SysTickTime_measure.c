/*---------------------------------------------------------------*
*time_measure.h for AFC V0.2
*ע�⣺
*	   ��.h�ļ������ڲ���ϵͳ������ʱ��
*ռ��Ӳ����timer5		
*��ʼ������void start_measure_time(void)
*ֹͣ����: unsigned int stop_measure_time(void)	   
*����ʱ�䣺0~65535us
*2011.10.20
*by majianjia in fantasticlab.blog.163.com
*----------------------------------------------------------------*/
#include"stm32f10x.h"
#include "SysTickTime_measure.h" 

//9-28 ̫�ݳ��ˣ���systick������ʱ��
//10-3 ��������ʱ���ܣ����ǲ���ʱ�����ʱ����һ���ú������ʱһ���ðɡ�����

static u8  fac_us=0;//us��ʱ������
static u16 fac_ms=0;//ms��ʱ������
//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void measure_time_init(u8 SYSCLK)
{
//	SysTick->CTRL&=0xfffffffb;//bit2���,ѡ���ⲿʱ��  HCLK/8
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	fac_us=SYSCLK/8;		    
	fac_ms=(u16)fac_us*1000;
}
 
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864ms 
void start_measure_time(void)
{	 		  	  
//	u32 temp;		   
	SysTick->LOAD=(u32)0xffffff;					//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL=0x01;           //��ʼ���� 
//  do
//	{
//		temp=SysTick->CTRL;
//	}
//	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
//	SysTick->CTRL=0x00;       //�رռ�����
//	SysTick->VAL =0X00;       //��ռ�����	
}

//��bug ������ ���ܳ�ʱ
//unsigned int stop_measure_time(void)
float stop_measure_time(void)
{
	float time;
	unsigned int temp;
	temp = (unsigned int)SysTick->VAL;
	//temp = SysTick_GetCounter
	time = ((u32)0xffffff - temp);
	
	time = time/9.0;
	//time = time/(9.0*1000000);						// 1/9Mhzÿ�� 
	//time = time*0.1;
	
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����
	
	return time;		//us���ص�λ
}


//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864 
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL=0x01 ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
}   
//��ʱnus
//nusΪҪ��ʱ��us��.		    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01 ;      //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}




