#ifndef __LED_H_
#define	__LED_H_

#include "stm32f10x.h"
#include "stm32f1_system.h"


#define LED_ON  0
#define LED_OFF 1

//λ�� ����
#define LED0_R  	PAout(8)		// PA8 LED0 red  
#define LED1_Y 		PDout(2)		// PD2 LED1 Yellow
#define LEDALL_OFF  LED0_R=LED_OFF; LED1_Y=LED_OFF;
#define LEDALL_ON 	LED0_R=LED_ON ; LED1_Y=LED_ON ;

/* �⺯��  ���� δ����,Ӧ��û����
#define LED0_OFF  	GPIO_SetBits(GPIOA, GPIO_Pin_8)     //LED0��
#define LED0_ON 		GPIO_ResetBits(GPIOA, GPIO_Pin_8)	  //LED0��
#define LED1_OFF  	GPIO_SetBits(GPIOD, GPIO_Pin_2)		  //LED1��
#define LED1_ON 		GPIO_ResetBits(GPIOD, GPIO_Pin_2)		//LED1��
#define LEDALL_OFF  LED0_OFF;LED1_OFF;									//ȫ��
#define LEDALL_ON 	LED0_ON;LED1_ON;										//ȫ��

*/

void LED_Init(void);
void LED_Flash(void);

#endif /* __LED_H */
