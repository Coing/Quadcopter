#ifndef _BSP_MOTO_H_
#define _BSP_MOTO_H_
#include "stm32f10x.h"

#define Moto_PwmMax 2400

void Moto_Pwm_Update(u16 MOTO1_PWM,u16 MOTO2_PWM,u16 MOTO3_PWM,u16 MOTO4_PWM);
void Moto_Init(void);

#endif
