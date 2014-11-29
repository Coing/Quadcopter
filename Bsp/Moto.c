#include "moto.h"

int16_t MOTO1_PWM = 0;
int16_t MOTO2_PWM = 0;
int16_t MOTO3_PWM = 0;
int16_t MOTO4_PWM = 0;

void Moto_Pwm_Update(u16 MOTO1_PWM,u16 MOTO2_PWM,u16 MOTO3_PWM,u16 MOTO4_PWM)
{		
	if(MOTO1_PWM>Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;
	if(MOTO2_PWM>Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM>Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM>Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
//	if(MOTO1_PWM<0)	MOTO1_PWM = 0;													//天啊，你怎么不限制值啊，限0啊，%>_<%
//	if(MOTO2_PWM<0)	MOTO2_PWM = 0;
//	if(MOTO3_PWM<0)	MOTO3_PWM = 0;
//	if(MOTO4_PWM<0)	MOTO4_PWM = 0;
	
	TIM2->CCR1 = MOTO1_PWM;																			//装载计数值
	TIM2->CCR2 = MOTO2_PWM;
	TIM2->CCR3 = MOTO3_PWM;
	TIM2->CCR4 = MOTO4_PWM;
}

void Tim2_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;
	/* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
	- Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices
	
    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
	= 24 MHz / 1000 = 24 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
	----------------------------------------------------------------------- */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);						//PCLK1经过2倍频后作为TIM3的时钟源等于72MHz
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 1000000 ) - 1;		//计算分频值
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 20 * 1000 - 1 ;							//计数上线	 1MHZ/20000 = 50HZ PWM  20ms   设置自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	          //pwm时钟分频 72-1 即APB1总线 72分频 Tim2 1MHZ  设置预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;										//设置时钟分频系数，不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;			//向上计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);									//初始化TIM2
	
	/* PWM1 Mode configuration: Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;								//配置为PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;															//初始占空比为0
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;				//当定时器计数值小于CCR1_Val时为高电平
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);												//使能通道1
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);								//CH1 预装载使能 
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);												//使能通道2
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);								//CH2 预装载使能 
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);												//使能通道3
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);								//CH3 预装载使能 
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);												//使能通道4
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);								//CH3 预装载使能 
	TIM_ARRPreloadConfig(TIM2, ENABLE);															//使能TIM2重载寄存器ARR
	TIM_Cmd(TIM2, ENABLE);																					//使能定时器2
}


void Moto_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能电机用的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); 
	//设置电机使用到得管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	Tim2_init();	
}

