#include "PPM.h"
#include "usart.h"

static void Rc_DataPPM(void);


void PPM_Init(void)
{
		GPIO_InitTypeDef         GPIO_InitStructure = {0};
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};
	  TIM_ICInitTypeDef  			 TIM3_ICInitStructure = {0};
		TIM_ICInitTypeDef  			 TIM4_ICInitStructure = {0};
		NVIC_InitTypeDef 				 NVIC_InitStructure= {0}; 
	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	 // πƒ‹TIM4 ±÷” PB8-9
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	 // πƒ‹TIM3 ±÷”	PC6-9
 	  
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // πƒ‹GPIOB ±÷”
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);    // πƒ‹AFIOπ¶ƒ‹µƒ ±÷”
	  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);      //TIM3»´≤øπ¶ƒ‹÷ÿ”≥…‰
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  // πƒ‹GPIOC ±÷”
	 // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	 //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	  
	  // GPIOC≥ı ºªØ PC6-PC9
	  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;             //PB6 «Â≥˝÷Æ«∞…Ë÷√  
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;     			//œ¬¿≠ ‰»Î    
	  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIOC,GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);		
		
		// GPIOB≥ı ºªØ PB8,PB9
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;						//œ¬¿≠ ‰»Î    
		//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);	
		
		//≥ı ºªØ∂® ±∆˜3 TIM3	 
	  TIM_DeInit(TIM3);
		
		TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                      //…Ë∂®º∆ ˝∆˜◊‘∂Ø÷ÿ◊∞÷µ 
	  TIM_TimeBaseStructure.TIM_Prescaler =(72-1); 	                 	//‘§∑÷∆µ∆˜ 72   1Mhz£¨æ´»∑µΩ1us
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      		//…Ë÷√ ±÷”∑÷∏Ó:TDTS = Tck_tim
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  		//TIMœÚ…œº∆ ˝ƒ£ Ω
	  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);              		//∏˘æ›TIM_TimeBaseInitStruct÷–÷∏∂®µƒ≤Œ ˝≥ı ºªØTIMxµƒ ±º‰ª˘ ˝µ•Œª
    
		//≥ı ºªØ∂® ±∆˜4 TIM4	
	  TIM_DeInit(TIM4);
		
		TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                      //…Ë∂®º∆ ˝∆˜◊‘∂Ø÷ÿ◊∞÷µ 
	  TIM_TimeBaseStructure.TIM_Prescaler =(72-1); 	                 	//‘§∑÷∆µ∆˜ 72   1Mhz£¨æ´»∑µΩ1us
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      		//…Ë÷√ ±÷”∑÷∏Ó:TDTS = Tck_tim
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  		//TIMœÚ…œº∆ ˝ƒ£ Ω
	  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		
		//≥ı ºªØTIM3 ‰»Î≤∂ªÒ≤Œ ˝, ’‚º∏∏ˆÕ®µ¿ƒ‹≤ªƒ‹œÒGPIO“ª—˘£¨“ª∆ƒÿ£¨¥˝◊ˆ
	  TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	—°‘Ò ‰»Î∂À IC1”≥…‰µΩTI1…œ
  	TIM3_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   	 //…œ…˝—ÿ≤∂ªÒ
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //”≥…‰µΩTI1…œ
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //≈‰÷√ ‰»Î∑÷∆µ,≤ª∑÷∆µ 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ≈‰÷√ ‰»Î¬À≤®∆˜ ≤ª¬À≤®
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	
	  //≥ı ºªØTIM3 ch2 ‰»Î≤∂ªÒ≤Œ ˝
		TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2;                //CC1S=01 	—°‘Ò ‰»Î∂À IC2”≥…‰µΩTI1…œ
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //…œ…˝—ÿ≤∂ªÒ
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //”≥…‰µΩTI1…œ
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //≈‰÷√ ‰»Î∑÷∆µ,≤ª∑÷∆µ 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ≈‰÷√ ‰»Î¬À≤®∆˜ ≤ª¬À≤®
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
		
		//≥ı ºªØTIM3 ch3 ‰»Î≤∂ªÒ≤Œ ˝
		TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	—°‘Ò ‰»Î∂À IC3”≥…‰µΩTI1…œ
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //…œ…˝—ÿ≤∂ªÒ
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //”≥…‰µΩTI1…œ
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //≈‰÷√ ‰»Î∑÷∆µ,≤ª∑÷∆µ 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ≈‰÷√ ‰»Î¬À≤®∆˜ ≤ª¬À≤®
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
		
		//≥ı ºªØTIM3 ch4 ‰»Î≤∂ªÒ≤Œ ˝
		TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	—°‘Ò ‰»Î∂À IC4”≥…‰µΩTI1…œ
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //…œ…˝—ÿ≤∂ªÒ
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //”≥…‰µΩTI1…œ
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //≈‰÷√ ‰»Î∑÷∆µ,≤ª∑÷∆µ 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ≈‰÷√ ‰»Î¬À≤®∆˜ ≤ª¬À≤®
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	
		//≥ı ºªØTIM4 ch3 ‰»Î≤∂ªÒ≤Œ ˝,
		TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	—°‘Ò ‰»Î∂À IC3”≥…‰µΩTI1…œ
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //…œ…˝—ÿ≤∂ªÒ
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //”≥…‰µΩTI1…œ
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //≈‰÷√ ‰»Î∑÷∆µ,≤ª∑÷∆µ 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ≈‰÷√ ‰»Î¬À≤®∆˜ ≤ª¬À≤®
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
		
		//≥ı ºªØTIM4 ch4 ‰»Î≤∂ªÒ≤Œ ˝,
		TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	—°‘Ò ‰»Î∂À IC4”≥…‰µΩTI1…œ
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //…œ…˝—ÿ≤∂ªÒ
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //”≥…‰µΩTI1…œ
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //≈‰÷√ ‰»Î∑÷∆µ,≤ª∑÷∆µ 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ≈‰÷√ ‰»Î¬À≤®∆˜ ≤ª¬À≤®
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
  
		TIM_Cmd(TIM4,ENABLE ); 
		
	  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);        //≤ª‘ –Ì∏¸–¬÷–∂œ£¨‘ –ÌCC1IE,CC2IE,CC3IE,CC4IE≤∂ªÒ÷–∂œ	
	  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	  TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	  TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
		
		TIM_Cmd(TIM3,ENABLE ); 
		
		TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);					//≤ª‘ –Ì∏¸–¬÷–∂œ£¨‘ –ÌCC3IE,CC4IE≤∂ªÒ÷–∂œ	
	  TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;            //TIM4÷–∂œ
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //œ»’º”≈œ»º∂1º∂
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //¥””≈œ»º∂0º∂
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQÕ®µ¿±ª πƒ‹
		NVIC_Init(&NVIC_InitStructure); 
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;            //TIM3÷–∂œ
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //œ»’º”≈œ»º∂1º∂
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;         //¥””≈œ»º∂0º∂
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQÕ®µ¿±ª πƒ‹
		NVIC_Init(&NVIC_InitStructure); 
	
}


/**********************************
∏ªÀπ9 “£øÿ∆˜
÷‹∆⁄19.52ms
”––ßPWM∏ﬂµÁ∆Ω 1.040ms~1.860ms◊Û”“ ∂‘”¶ 0%~100%  1040~1860◊Û”“
***********************************/

static u16 			Rc_Pwm_In[6];  			//1040~1860


static u8 	TIM3CH1_CAPTURE_STA = 0;	//Time3Õ®µ¿1 ‰»Î≤∂ªÒ±Í÷æ£¨∏ﬂ¡ΩŒª◊ˆ≤∂ªÒ±Í÷æ£¨µÕ6Œª◊ˆ“Á≥ˆ±Í÷æ		
static u16 TIM3CH1_CAPTURE_UPVAL;
static u16 TIM3CH1_CAPTURE_DOWNVAL;

static u8 	TIM3CH2_CAPTURE_STA = 0;	//Time3Õ®µ¿2 ‰»Î≤∂ªÒ±Í÷æ£¨∏ﬂ¡ΩŒª◊ˆ≤∂ªÒ±Í÷æ£¨µÕ6Œª◊ˆ“Á≥ˆ±Í÷æ		
static u16 TIM3CH2_CAPTURE_UPVAL;
static u16 TIM3CH2_CAPTURE_DOWNVAL;

static u8 	TIM3CH3_CAPTURE_STA = 0;	//Time3Õ®µ¿3 ‰»Î≤∂ªÒ±Í÷æ£¨∏ﬂ¡ΩŒª◊ˆ≤∂ªÒ±Í÷æ£¨µÕ6Œª◊ˆ“Á≥ˆ±Í÷æ		
static u16 TIM3CH3_CAPTURE_UPVAL;
static u16 TIM3CH3_CAPTURE_DOWNVAL;

static u8 	TIM3CH4_CAPTURE_STA = 0;	//Time3Õ®µ¿4 ‰»Î≤∂ªÒ±Í÷æ£¨∏ﬂ¡ΩŒª◊ˆ≤∂ªÒ±Í÷æ£¨µÕ6Œª◊ˆ“Á≥ˆ±Í÷æ		
static u16 TIM3CH4_CAPTURE_UPVAL;
static u16 TIM3CH4_CAPTURE_DOWNVAL;

static u8 	TIM4CH3_CAPTURE_STA = 0;	//Time4Õ®µ¿3 ‰»Î≤∂ªÒ±Í÷æ£¨∏ﬂ¡ΩŒª◊ˆ≤∂ªÒ±Í÷æ£¨µÕ6Œª◊ˆ“Á≥ˆ±Í÷æ		
static u16 TIM4CH3_CAPTURE_UPVAL;
static u16 TIM4CH3_CAPTURE_DOWNVAL;

static u8 	TIM4CH4_CAPTURE_STA = 0;	//Time4Õ®µ¿4 ‰»Î≤∂ªÒ±Í÷æ£¨∏ﬂ¡ΩŒª◊ˆ≤∂ªÒ±Í÷æ£¨µÕ6Œª◊ˆ“Á≥ˆ±Í÷æ		
static u16 TIM4CH4_CAPTURE_UPVAL;
static u16 TIM4CH4_CAPTURE_DOWNVAL;



/*∂® ±∆˜3÷–∂œ∑˛ŒÒ≥Ã–Ú


			º∆À„∏ﬂµÁ∆Ω≥÷–¯ ±º‰
	 ----				 ----				          
	|		 |			|		 |
	|		 |			|		 |
	|		 |			|		 |
	|		 |			|		 |
	|		 |			|		 |			
---		 --------		 --------

*/
void TIM3_IRQHandler(void)
{ 
	
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)            //≤∂ªÒ1∑¢…˙≤∂ªÒ ¬º˛
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC1); 								 //«Â≥˝÷–∂œ±Í÷æŒª
			if(TIM3CH1_CAPTURE_STA&0X40)		                       	 //≤∂ªÒµΩ“ª∏ˆœ¬Ωµ—ÿ 		
			{	  			
				//TIM3CH1_CAPTURE_STA |= 0X80;		                     //±Íº«≥…π¶≤∂ªÒµΩ“ª¥Œ…œ…˝—ÿ
				TIM3CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM3);			 //ªÒ»°TIM3µƒÕ®µ¿1µƒ ‰»Î≤∂◊Ω÷µ
		   	if (TIM3CH1_CAPTURE_DOWNVAL < TIM3CH2_CAPTURE_UPVAL)	 // µ√µΩ∏ﬂµÁ∆Ω ≥÷–¯ ±º‰
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH1_CAPTURE_UPVAL + TIM3CH1_CAPTURE_DOWNVAL
					Rc_Pwm_In[0] = 0xffff - TIM3CH1_CAPTURE_UPVAL + TIM3CH1_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[0] = TIM3CH1_CAPTURE_DOWNVAL - TIM3CH1_CAPTURE_UPVAL ;
				}
				
				
				TIM3CH1_CAPTURE_STA = 0;															 //≤∂ªÒ±Í÷æŒª«Â¡„
				TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); 		 //CC1P=0 …Ë÷√Œ™…œ…˝—ÿ≤∂ªÒ				
				//printf("TIM3CH1:%d us\r\n",Rc_Pwm_In[0]);							 //for test
			}
			else  								                                 	//ªπŒ¥ø™ º,µ⁄“ª¥Œ≤∂ªÒ…œ…˝—ÿ£¨º«¬º¥À ±µƒ∂® ±∆˜º∆ ˝÷µ
			{
				TIM3CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM3);				//ªÒ»°…œ…˝—ÿ ˝æ›
				TIM3CH1_CAPTURE_STA|=0X40;		                        //±Íº«≤∂ªÒµΩ¡À…œ…˝—ÿ
		    TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 …Ë÷√Œ™œ¬Ωµ—ÿ≤∂ªÒ
			}		    
		}	

	
	 
	  if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)            //≤∂ªÒ2∑¢…˙≤∂ªÒ ¬º˛
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC2); 								 //«Â≥˝÷–∂œ±Í÷æŒª
			if(TIM3CH2_CAPTURE_STA&0X40)		                       	 //≤∂ªÒµΩ“ª∏ˆœ¬Ωµ—ÿ 		
			{	  			
				//TIM3CH2_CAPTURE_STA|=0X80;		                       	 //±Íº«≥…π¶≤∂ªÒµΩ“ª¥Œ…œ…˝—ÿ
				TIM3CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM3);			 //ªÒ»°TIM3µƒÕ®µ¿1µƒ ‰»Î≤∂◊Ω÷µ
		   	
				if (TIM3CH2_CAPTURE_DOWNVAL < TIM3CH2_CAPTURE_UPVAL)	 // µ√µΩ∏ﬂµÁ∆Ω ≥÷–¯ ±º‰
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH2_CAPTURE_UPVAL + TIM3CH2_CAPTURE_DOWNVAL
					Rc_Pwm_In[1] = 0xffff - TIM3CH2_CAPTURE_UPVAL + TIM3CH2_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[1] = TIM3CH2_CAPTURE_DOWNVAL - TIM3CH2_CAPTURE_UPVAL ;
				}
				
				TIM3CH2_CAPTURE_STA = 0;															 //≤∂ªÒ±Í÷æŒª«Â¡„
		   	TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Rising); 		 //CC1P=0 …Ë÷√Œ™…œ…˝—ÿ≤∂ªÒ
				//printf("TIM3CH2:%d us\r\n",Rc_Pwm_In[1]);
			}
			else  								                                 	 //ªπŒ¥ø™ º,µ⁄“ª¥Œ≤∂ªÒ…œ…˝—ÿ
			{
				TIM3CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM3);				//ªÒ»°…œ…˝—ÿ ˝æ›
				TIM3CH2_CAPTURE_STA|=0X40;		                        //±Íº«≤∂ªÒµΩ¡À…œ…˝—ÿ
		    TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 …Ë÷√Œ™œ¬Ωµ—ÿ≤∂ªÒ
			}		    
		}	
  	
    if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)            //≤∂ªÒ3∑¢…˙≤∂ªÒ ¬º˛
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC3); 								 //«Â≥˝÷–∂œ±Í÷æŒª
			if(TIM3CH3_CAPTURE_STA&0X40)		                         //≤∂ªÒµΩ“ª∏ˆœ¬Ωµ—ÿ 		
			{	  			
				TIM3CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM3);			 //ªÒ»°TIM3µƒÕ®µ¿1µƒ ‰»Î≤∂◊Ω÷µ
		   	
				if (TIM3CH3_CAPTURE_DOWNVAL < TIM3CH3_CAPTURE_UPVAL)	 // µ√µΩ∏ﬂµÁ∆Ω ≥÷–¯ ±º‰
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[2] = 0xffff - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[2] = TIM3CH3_CAPTURE_DOWNVAL - TIM3CH3_CAPTURE_UPVAL ;
				}
				
				
				TIM3CH3_CAPTURE_STA = 0;															 //≤∂ªÒ±Í÷æŒª«Â¡„
		   	TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Rising);     //CC1P=0 …Ë÷√Œ™…œ…˝—ÿ≤∂ªÒ
			}
			else  								                                   //ªπŒ¥ø™ º,µ⁄“ª¥Œ≤∂ªÒ…œ…˝—ÿ
			{
				TIM3CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM3);				//ªÒ»°…œ…˝—ÿ ˝æ›
				TIM3CH3_CAPTURE_STA|=0X40;		                        //±Íº«≤∂ªÒµΩ¡À…œ…˝—ÿ
		    TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 …Ë÷√Œ™œ¬Ωµ—ÿ≤∂ª
			}		    
		}	

    if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)            //≤∂ªÒ4∑¢…˙≤∂ªÒ ¬º˛
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC4); 								 //«Â≥˝÷–∂œ±Í÷æŒª
			if(TIM3CH4_CAPTURE_STA&0X40)		                         //≤∂ªÒµΩ“ª∏ˆœ¬Ωµ—ÿ 		
			{	  			
				TIM3CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM3);			 //ªÒ»°TIM3µƒÕ®µ¿1µƒ ‰»Î≤∂◊Ω÷µ
		   	
				if (TIM3CH4_CAPTURE_DOWNVAL < TIM3CH4_CAPTURE_UPVAL)	 // µ√µΩ∏ﬂµÁ∆Ω ≥÷–¯ ±º‰
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[3] = 0xffff - TIM3CH4_CAPTURE_UPVAL + TIM3CH4_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[3] = TIM3CH4_CAPTURE_DOWNVAL - TIM3CH4_CAPTURE_UPVAL ;
				}
				
				TIM3CH4_CAPTURE_STA = 0;															 //≤∂ªÒ±Í÷æŒª«Â¡„
		   	TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising);     //CC1P=0 …Ë÷√Œ™…œ…˝—ÿ≤∂ªÒ
			}else  								                                   //ªπŒ¥ø™ º,µ⁄“ª¥Œ≤∂ªÒ…œ…˝—ÿ
			{
				TIM3CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM3);				//ªÒ»°…œ…˝—ÿ ˝æ›
				TIM3CH4_CAPTURE_STA|=0X40;		                        //±Íº«≤∂ªÒµΩ¡À…œ…˝—ÿ
		    TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);		 //CC1P=1 …Ë÷√Œ™œ¬Ωµ—ÿ≤∂ªÒ
			}		    
		}		
}


//∂® ±∆˜4÷–∂œ∑˛ŒÒ≥Ã–Ú	  ¥˝≤‚  oK
void TIM4_IRQHandler(void)
{ 
	

		if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)            //≤∂ªÒ4∑¢…˙≤∂ªÒ ¬º˛
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC3); 								 //«Â≥˝÷–∂œ±Í÷æŒª
			if(TIM4CH3_CAPTURE_STA&0X40)		                         //≤∂ªÒµΩ“ª∏ˆœ¬Ωµ—ÿ 		
			{	  			
				TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);			 //ªÒ»°TIM3µƒÕ®µ¿1µƒ ‰»Î≤∂◊Ω÷µ
		   	
				if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)	 // µ√µΩ∏ﬂµÁ∆Ω ≥÷–¯ ±º‰
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[4] = 0xffff - TIM4CH3_CAPTURE_UPVAL + TIM4CH3_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[4] = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL ;
				}
				
				TIM4CH3_CAPTURE_STA = 0;															 //≤∂ªÒ±Í÷æŒª«Â¡„
		   	TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising);     //CC1P=0 …Ë÷√Œ™…œ…˝—ÿ≤∂ªÒ
			}else  								                                   //ªπŒ¥ø™ º,µ⁄“ª¥Œ≤∂ªÒ…œ…˝—ÿ
			{
				TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);				//ªÒ»°…œ…˝—ÿ ˝æ›
				TIM4CH3_CAPTURE_STA|=0X40;		                        //±Íº«≤∂ªÒµΩ¡À…œ…˝—ÿ
		    TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);		 //CC1P=1 …Ë÷√Œ™œ¬Ωµ—ÿ≤∂ªÒ
			}		    
		}

		if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)            //≤∂ªÒ4∑¢…˙≤∂ªÒ ¬º˛
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC4); 								 //«Â≥˝÷–∂œ±Í÷æŒª
			if(TIM4CH4_CAPTURE_STA&0X40)		                         //≤∂ªÒµΩ“ª∏ˆœ¬Ωµ—ÿ 		
			{	  			
				TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);			 //ªÒ»°TIM3µƒÕ®µ¿1µƒ ‰»Î≤∂◊Ω÷µ
		   	
				if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)	 // µ√µΩ∏ﬂµÁ∆Ω ≥÷–¯ ±º‰
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[5] = 0xffff - TIM4CH4_CAPTURE_UPVAL + TIM4CH4_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[5] = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL ;
				}
				
				TIM4CH4_CAPTURE_STA = 0;															 //≤∂ªÒ±Í÷æŒª«Â¡„
		   	TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising);     //CC1P=0 …Ë÷√Œ™…œ…˝—ÿ≤∂ªÒ
			}else  								                                   //ªπŒ¥ø™ º,µ⁄“ª¥Œ≤∂ªÒ…œ…˝—ÿ
			{
				TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);				//ªÒ»°…œ…˝—ÿ ˝æ›
				TIM4CH4_CAPTURE_STA|=0X40;		                        //±Íº«≤∂ªÒµΩ¡À…œ…˝—ÿ
		    TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);		 //CC1P=1 …Ë÷√Œ™œ¬Ωµ—ÿ≤∂ªÒ
			}		    
		}
		
		Rc_DataPPM();
}





//static T_RC_Data 	Rc_Data;//1000~2000
static T_RC_Data 	Rc_Data;//900~1950

static void Rc_DataPPM(void)
{
	Rc_Data.ROLL			=	Rc_Pwm_In[0];
	Rc_Data.PITCH			=	Rc_Pwm_In[1];
	Rc_Data.THROTTLE	=	Rc_Pwm_In[2];
	Rc_Data.YAW				=	Rc_Pwm_In[3];
	Rc_Data.AUX2			=	Rc_Pwm_In[4];
	Rc_Data.AUX4			=	Rc_Pwm_In[5];
}

void Rc_GetValue(T_RC_Data *temp)
{
	temp->THROTTLE	= Rc_Data.THROTTLE;
	temp->YAW				= Rc_Data.YAW;
	temp->ROLL			= Rc_Data.ROLL;
	temp->PITCH			= Rc_Data.PITCH;	
	temp->AUX2			=	Rc_Data.AUX2;
	temp->AUX4			= Rc_Data.AUX4;
	
}


