#include "stm32f10x.h"
#include "misc.h"
#include "usart.h"



u8 TxBuffer[0xff];
u8 TxCounter=0;
u8 count=0; 
u8 Rx_Buf[2][32];	//����32�ֽڵĴ��ڽ��ջ���
u8 Rx_Act=0;		//����ʹ�õ�buf��
u8 Rx_Adr=0;		//���ڽ��յڼ��ֽ�
u8 Rx_Ok0 = 0;
u8 Rx_Ok1 = 0;

static void Uart1_Put_Char(unsigned char DataToSend);

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ��� ����ѡ��*/
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
	
	USART_DeInit(USART1);  //��λ����1
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA9
 
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA10
	  
	/* USART1 mode config */
	 USART_InitStructure.USART_BaudRate = bound;																				//һ������Ϊ9600;
	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;												//�ֳ�Ϊ8λ���ݸ�ʽ
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;															//һ��ֹͣλ
	 USART_InitStructure.USART_Parity = USART_Parity_No;																//����żУ��λ
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//��Ӳ������������
	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;										//�շ�ģʽ

   USART_Init(USART1, &USART_InitStructure); 																					//��ʼ������
   //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);																			//�����ж�
   USART_Cmd(USART1, ENABLE);                    																			//ʹ�ܴ��� 
	 
	 		//����
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
	//�ж�Strָ��������Ƿ���Ч.
	while(*Str)
	{
	//�Ƿ��ǻس��ַ� �����,������Ӧ�Ļس� 0x0d 0x0a
	if(*Str=='\r')Uart1_Put_Char(0x0d);
		else if(*Str=='\n')Uart1_Put_Char(0x0a);
			else Uart1_Put_Char(*Str);
	//ָ��++ ָ����һ���ֽ�.
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
	//�����ж�
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		USART1->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־          
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�ж�
			//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
		}
	}
	//�����ж� (���ռĴ����ǿ�) 
	if(USART1->SR & (1<<5))//if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    
	{
//		u8 com_data = USART1->DR;
//		if(Rx_Adr==0)		//Ѱ��֡ͷ0X8A
//		{
//			if(com_data==0x8A)	//�������������0X8A,��д�뻺��
//			{
//				Rx_Buf[Rx_Act][0] = com_data;
//				Rx_Adr = 1;
//			}
//		}
//		else		//���ڽ�������
//		{
//			Rx_Buf[Rx_Act][Rx_Adr] = com_data;
//			Rx_Adr ++;
//		}
//		if(Rx_Adr==32)		//���ݽ������
//		{
//			Rx_Adr = 0;
//			if(Rx_Act)	
//			{ 
//				Rx_Act = 0; 			//�л�����
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
