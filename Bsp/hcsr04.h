#ifndef __HCSR04_H_
#define	__HCSR04_H_

#include "stm32f10x.h"

void hcsr04_init(void);
void hcsr04_get_distance(volatile uint16_t* distance);



#endif
