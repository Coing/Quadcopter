#ifndef _PWM_H_
#define _PWM_H_

#include "stm32f10x.h"


// RX1  TIM3_CH1 PC6 
// RX2  TIM3_CH2 PC7
// RX3  TIM3_CH3 PC8 
// RX4  TIM3_CH4 PC9 
// RX5  TIM4_CH3 PB8 
// RX6  TIM4_CH4 PB9 

typedef struct {
         int16_t CH1;
         int16_t CH2;
         int16_t CH3;
				 int16_t CH4;
				 int16_t CH5;
				 int16_t CH6;
				 int16_t CH7;
				 int16_t CH8;
} _PPM_Channel;


typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
				int16_t AUX6;}T_RC_Data;

void PPM_Init(void);


void Rc_GetValue(T_RC_Data *temp);
				
#endif
