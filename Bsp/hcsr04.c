#include "stm32f10x.h"
#include "hcsr04.h"

#include "SysTickTime_measure.h" 
#include "STM32_DELAY.h"

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When trigged it sends out a series of 40KHz ultrasonic pulses and receives
 * echo froman object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

static uint16_t trigger_pin;
static uint16_t echo_pin;
static uint32_t exti_line;
static uint8_t exti_pin_source;
static IRQn_Type exti_irqn;

//static uint32_t last_measurement;
static volatile uint16_t* distance_ptr;

    float timing_stop, temp;

void ECHO_EXTI_IRQHandler(void)
{
    static uint32_t timing_start;
//    float timing_stop, temp;
    if(GPIO_ReadInputDataBit(GPIOB, echo_pin) != 0)
		{			
			start_measure_time();
			//timing_start = micros();
			//printf("start_measure_time\r\n");
		}
		else 
    {
        timing_stop = stop_measure_time();
			//printf("stop_measure_time\r\n");
        //if(timing_stop > timing_start) 
				if(timing_stop > 0) 
        {
					
            // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
            // The ping travels out and back, so to find the distance of the
            // object we take half of the distance traveled.
            //
            // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59
            //int32_t pulse_duration = timing_stop - timing_start;          
            //*distance_ptr = pulse_duration / 59 ;
						float pulse_duration = timing_stop - timing_start;  
//						temp = pulse_duration / 59 ;
//					temp = temp*1000000;
					*distance_ptr = pulse_duration / 59 * 100;
					timing_stop =0;
						//*distance_ptr = pulse_duration / 59 ;
        }
    }

    EXTI_ClearITPendingBit(exti_line);   
}

//void EXTI1_IRQHandler(void)
//{
//    ECHO_EXTI_IRQHandler();
//}

void EXTI15_10_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}

void hcsr04_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTIInit;
		NVIC_InitTypeDef NVIC_InitStructure; 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    //enable AFIO for EXTI support - already done is drv_system.c
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph, ENABLE); 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
//    switch(config)
//    {
//    case sonar_pwm56:
//        trigger_pin = GPIO_Pin_8;   // PWM5 (PB8) - 5v tolerant
//        echo_pin = GPIO_Pin_9;      // PWM6 (PB9) - 5v tolerant
//        exti_line = EXTI_Line9;
//        exti_pin_source = GPIO_PinSource9;
//        exti_irqn = EXTI9_5_IRQn;
//        break;
//    case sonar_rc78:    
//        trigger_pin = GPIO_Pin_0;   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
//        echo_pin = GPIO_Pin_1;      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
//        exti_line = EXTI_Line1;
//        exti_pin_source = GPIO_PinSource1;
//        exti_irqn = EXTI1_IRQn;
//        break;
//    }
		trigger_pin = GPIO_Pin_14;   // PWM5 (PB8) - 5v tolerant
		echo_pin = GPIO_Pin_15;      // PWM6 (PB9) - 5v tolerant
		exti_line = EXTI_Line15;
		exti_pin_source = GPIO_PinSource15;
		exti_irqn = EXTI15_10_IRQn;
    
    // tp - trigger pin 
    GPIO_InitStructure.GPIO_Pin = trigger_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // ep - echo pin
    GPIO_InitStructure.GPIO_Pin = echo_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // setup external interrupt on echo pin
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, exti_pin_source);

    EXTI_ClearITPendingBit(exti_line);
       
    EXTIInit.EXTI_Line = exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;    
    EXTI_Init(&EXTIInit);    

			//3.1
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		NVIC_EnableIRQ(exti_irqn);

    //last_measurement = millis() - 60; // force 1st measurement in hcsr04_get_distance()
}

// distance calculation is done asynchronously, using interrupt
void hcsr04_get_distance(volatile uint16_t* distance)
{   
//    uint32_t current_time = millis();

//    if( current_time < (last_measurement + 60) )
//    {
//        // the repeat interval of trig signal should be greater than 60ms
//        // to avoid interference between connective measurements.
//        return;
//    }
        
    //last_measurement = current_time;
    distance_ptr = distance;

    GPIO_SetBits(GPIOB, trigger_pin);
    //  The width of trig signal must be greater than 10us
    //delay_us(11);
    Delay_1us(15);
		GPIO_ResetBits(GPIOB, trigger_pin);
}


