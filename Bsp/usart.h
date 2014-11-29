#ifndef __USART1_H_
#define	__USART1_H_

#include "stm32f10x.h"
#include <stdio.h>

void Uart1_Init(u32 bound);
//void Uart1_Put_String(unsigned char *Str);
//uint8_t Uart1_Put_Char(unsigned char DataToSend);
//uint8_t Uart1_Put_Int16(uint16_t DataToSend);
//uint8_t Uart1_Put_Float(float DataToSend);


int fputc(int ch, FILE *f);




void Uart1_Put_String(unsigned char *Str);
void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num);
void Uart1_IRQ(void);

#endif /* __USART1_H */
