#include "stm32f10x.h"
#include "misc.h"
#include "usart.h"



u8 TxBuffer[0xff];
u8 TxCounter=0;
u8 count=0; 
u8 Rx_Buf[2][32];	//两个32字节的串口接收缓存
u8 Rx_Act=0;		//正在使用的buf号
u8 Rx_Adr=0;		//正在接收第几字节
u8 Rx_Ok0 = 0;
u8 Rx_Ok1 = 0;

static void Uart1_Put_Char(unsigned char DataToSend);

//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法 配置选项*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
*/


void Uart1_Init(u32 bound)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* config USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	USART_DeInit(USART1);  //复位串口1
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9
 
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10
	  
	/* USART1 mode config */
	 USART_InitStructure.USART_BaudRate = bound;																				//一般设置为9600;
	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;												//字长为8位数据格式
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;															//一个停止位
	 USART_InitStructure.USART_Parity = USART_Parity_No;																//无奇偶校验位
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//无硬件数据流控制
	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;										//收发模式

   USART_Init(USART1, &USART_InitStructure); 																					//初始化串口
   //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);																			//开启中断
   USART_Cmd(USART1, ENABLE);                    																			//使能串口 
	 
	 		//串口
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


static void Uart1_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;  
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}
void Uart1_Put_String(unsigned char *Str)
{
	//判断Str指向的数据是否有效.
	while(*Str)
	{
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')Uart1_Put_Char(0x0d);
		else if(*Str=='\n')Uart1_Put_Char(0x0a);
			else Uart1_Put_Char(*Str);
	//指针++ 指向下一个字节.
	Str++;
	}
}
void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num)
{
	for(u8 i=0;i<data_num;i++)
		TxBuffer[count++] = *(DataToSend+i);
	if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}




void Uart1_IRQ(void)
{
	if(USART1->SR & USART_IT_ORE)
	{
		USART1->SR;
	}
	//发送中断
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		USART1->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE中断
			//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
		}
	}
	//接收中断 (接收寄存器非空) 
	if(USART1->SR & (1<<5))//if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    
	{
//		u8 com_data = USART1->DR;
//		if(Rx_Adr==0)		//寻找帧头0X8A
//		{
//			if(com_data==0x8A)	//接收数据如果是0X8A,则写入缓存
//			{
//				Rx_Buf[Rx_Act][0] = com_data;
//				Rx_Adr = 1;
//			}
//		}
//		else		//正在接收数据
//		{
//			Rx_Buf[Rx_Act][Rx_Adr] = com_data;
//			Rx_Adr ++;
//		}
//		if(Rx_Adr==32)		//数据接收完毕
//		{
//			Rx_Adr = 0;
//			if(Rx_Act)	
//			{ 
//				Rx_Act = 0; 			//切换缓存
//				Rx_Ok1 = 1;
//			}
//			else 				
//			{
//				Rx_Act = 1;
//				Rx_Ok0 = 1;
//			}
//		}
	}
}
