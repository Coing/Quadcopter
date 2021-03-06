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
	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	 //使能TIM4时钟 PB8-9
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	 //使能TIM3时钟	PC6-9
 	  
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIOB时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);    //使能AFIO功能的时钟
	  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);      //TIM3全部功能重映射
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //使能GPIOC时钟
	 // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	 //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	  
	  // GPIOC初始化 PC6-PC9
	  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;             //PB6 清除之前设置  
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;     			//下拉输入    
	  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIOC,GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);		
		
		// GPIOB初始化 PB8,PB9
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;						//下拉输入    
		//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);	
		
		//初始化定时器3 TIM3	 
	  TIM_DeInit(TIM3);
		
		TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                      //设定计数器自动重装值 
	  TIM_TimeBaseStructure.TIM_Prescaler =(72-1); 	                 	//预分频器 72   1Mhz，精确到1us
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      		//设置时钟分割:TDTS = Tck_tim
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  		//TIM向上计数模式
	  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);              		//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
    
		//初始化定时器4 TIM4	
	  TIM_DeInit(TIM4);
		
		TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                      //设定计数器自动重装值 
	  TIM_TimeBaseStructure.TIM_Prescaler =(72-1); 	                 	//预分频器 72   1Mhz，精确到1us
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      		//设置时钟分割:TDTS = Tck_tim
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  		//TIM向上计数模式
	  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		
		//初始化TIM3输入捕获参数, 这几个通道能不能像GPIO一样，一起呢，待做
	  TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   	 //上升沿捕获
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	
	  //初始化TIM3 ch2输入捕获参数
		TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2;                //CC1S=01 	选择输入端 IC2映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
		
		//初始化TIM3 ch3输入捕获参数
		TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	选择输入端 IC3映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
		
		//初始化TIM3 ch4输入捕获参数
		TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	选择输入端 IC4映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	
		//初始化TIM4 ch3输入捕获参数,
		TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	选择输入端 IC3映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
		
		//初始化TIM4 ch4输入捕获参数,
		TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	选择输入端 IC4映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
  
		TIM_Cmd(TIM4,ENABLE ); 
		
	  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);        //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断	
	  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	  TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	  TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
		
		TIM_Cmd(TIM3,ENABLE ); 
		
		TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);					//不允许更新中断，允许CC3IE,CC4IE捕获中断	
	  TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;            //TIM4中断
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级1级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //从优先级0级
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道被使能
		NVIC_Init(&NVIC_InitStructure); 
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;            //TIM3中断
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级1级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;         //从优先级0级
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道被使能
		NVIC_Init(&NVIC_InitStructure); 
	
}


/**********************************
富斯9 遥控器
周期19.52ms
有效PWM高电平 1.040ms~1.860ms左右 对应 0%~100%  1040~1860左右
***********************************/

static u16 			Rc_Pwm_In[6];  			//1040~1860


static u8 	TIM3CH1_CAPTURE_STA = 0;	//Time3通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
static u16 TIM3CH1_CAPTURE_UPVAL;
static u16 TIM3CH1_CAPTURE_DOWNVAL;

static u8 	TIM3CH2_CAPTURE_STA = 0;	//Time3通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志		
static u16 TIM3CH2_CAPTURE_UPVAL;
static u16 TIM3CH2_CAPTURE_DOWNVAL;

static u8 	TIM3CH3_CAPTURE_STA = 0;	//Time3通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志		
static u16 TIM3CH3_CAPTURE_UPVAL;
static u16 TIM3CH3_CAPTURE_DOWNVAL;

static u8 	TIM3CH4_CAPTURE_STA = 0;	//Time3通道4输入捕获标志，高两位做捕获标志，低6位做溢出标志		
static u16 TIM3CH4_CAPTURE_UPVAL;
static u16 TIM3CH4_CAPTURE_DOWNVAL;

static u8 	TIM4CH3_CAPTURE_STA = 0;	//Time4通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志		
static u16 TIM4CH3_CAPTURE_UPVAL;
static u16 TIM4CH3_CAPTURE_DOWNVAL;

static u8 	TIM4CH4_CAPTURE_STA = 0;	//Time4通道4输入捕获标志，高两位做捕获标志，低6位做溢出标志		
static u16 TIM4CH4_CAPTURE_UPVAL;
static u16 TIM4CH4_CAPTURE_DOWNVAL;



/*定时器3中断服务程序


			计算高电平持续时间
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
	
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)            //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC1); 								 //清除中断标志位
			if(TIM3CH1_CAPTURE_STA&0X40)		                       	 //捕获到一个下降沿 		
			{	  			
				//TIM3CH1_CAPTURE_STA |= 0X80;		                     //标记成功捕获到一次上升沿
				TIM3CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM3);			 //获取TIM3的通道1的输入捕捉值
		   	if (TIM3CH1_CAPTURE_DOWNVAL < TIM3CH2_CAPTURE_UPVAL)	 // 得到高电平 持续时间
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH1_CAPTURE_UPVAL + TIM3CH1_CAPTURE_DOWNVAL
					Rc_Pwm_In[0] = 0xffff - TIM3CH1_CAPTURE_UPVAL + TIM3CH1_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[0] = TIM3CH1_CAPTURE_DOWNVAL - TIM3CH1_CAPTURE_UPVAL ;
				}
				
				
				TIM3CH1_CAPTURE_STA = 0;															 //捕获标志位清零
				TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); 		 //CC1P=0 设置为上升沿捕获				
				//printf("TIM3CH1:%d us\r\n",Rc_Pwm_In[0]);							 //for test
			}
			else  								                                 	//还未开始,第一次捕获上升沿，记录此时的定时器计数值
			{
				TIM3CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM3);				//获取上升沿数据
				TIM3CH1_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		    TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		    
		}	

	
	 
	  if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)            //捕获2发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC2); 								 //清除中断标志位
			if(TIM3CH2_CAPTURE_STA&0X40)		                       	 //捕获到一个下降沿 		
			{	  			
				//TIM3CH2_CAPTURE_STA|=0X80;		                       	 //标记成功捕获到一次上升沿
				TIM3CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM3);			 //获取TIM3的通道1的输入捕捉值
		   	
				if (TIM3CH2_CAPTURE_DOWNVAL < TIM3CH2_CAPTURE_UPVAL)	 // 得到高电平 持续时间
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH2_CAPTURE_UPVAL + TIM3CH2_CAPTURE_DOWNVAL
					Rc_Pwm_In[1] = 0xffff - TIM3CH2_CAPTURE_UPVAL + TIM3CH2_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[1] = TIM3CH2_CAPTURE_DOWNVAL - TIM3CH2_CAPTURE_UPVAL ;
				}
				
				TIM3CH2_CAPTURE_STA = 0;															 //捕获标志位清零
		   	TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Rising); 		 //CC1P=0 设置为上升沿捕获
				//printf("TIM3CH2:%d us\r\n",Rc_Pwm_In[1]);
			}
			else  								                                 	 //还未开始,第一次捕获上升沿
			{
				TIM3CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM3);				//获取上升沿数据
				TIM3CH2_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		    TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		    
		}	
  	
    if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)            //捕获3发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC3); 								 //清除中断标志位
			if(TIM3CH3_CAPTURE_STA&0X40)		                         //捕获到一个下降沿 		
			{	  			
				TIM3CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM3);			 //获取TIM3的通道1的输入捕捉值
		   	
				if (TIM3CH3_CAPTURE_DOWNVAL < TIM3CH3_CAPTURE_UPVAL)	 // 得到高电平 持续时间
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[2] = 0xffff - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[2] = TIM3CH3_CAPTURE_DOWNVAL - TIM3CH3_CAPTURE_UPVAL ;
				}
				
				
				TIM3CH3_CAPTURE_STA = 0;															 //捕获标志位清零
		   	TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Rising);     //CC1P=0 设置为上升沿捕获
			}
			else  								                                   //还未开始,第一次捕获上升沿
			{
				TIM3CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM3);				//获取上升沿数据
				TIM3CH3_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		    TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕�
			}		    
		}	

    if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)            //捕获4发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC4); 								 //清除中断标志位
			if(TIM3CH4_CAPTURE_STA&0X40)		                         //捕获到一个下降沿 		
			{	  			
				TIM3CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM3);			 //获取TIM3的通道1的输入捕捉值
		   	
				if (TIM3CH4_CAPTURE_DOWNVAL < TIM3CH4_CAPTURE_UPVAL)	 // 得到高电平 持续时间
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[3] = 0xffff - TIM3CH4_CAPTURE_UPVAL + TIM3CH4_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[3] = TIM3CH4_CAPTURE_DOWNVAL - TIM3CH4_CAPTURE_UPVAL ;
				}
				
				TIM3CH4_CAPTURE_STA = 0;															 //捕获标志位清零
		   	TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising);     //CC1P=0 设置为上升沿捕获
			}else  								                                   //还未开始,第一次捕获上升沿
			{
				TIM3CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM3);				//获取上升沿数据
				TIM3CH4_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		    TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);		 //CC1P=1 设置为下降沿捕获
			}		    
		}		
}


//定时器4中断服务程序	  待测  oK
void TIM4_IRQHandler(void)
{ 
	

		if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)            //捕获4发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC3); 								 //清除中断标志位
			if(TIM4CH3_CAPTURE_STA&0X40)		                         //捕获到一个下降沿 		
			{	  			
				TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);			 //获取TIM3的通道1的输入捕捉值
		   	
				if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)	 // 得到高电平 持续时间
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[4] = 0xffff - TIM4CH3_CAPTURE_UPVAL + TIM4CH3_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[4] = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL ;
				}
				
				TIM4CH3_CAPTURE_STA = 0;															 //捕获标志位清零
		   	TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising);     //CC1P=0 设置为上升沿捕获
			}else  								                                   //还未开始,第一次捕获上升沿
			{
				TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);				//获取上升沿数据
				TIM4CH3_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		    TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);		 //CC1P=1 设置为下降沿捕获
			}		    
		}

		if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)            //捕获4发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC4); 								 //清除中断标志位
			if(TIM4CH4_CAPTURE_STA&0X40)		                         //捕获到一个下降沿 		
			{	  			
				TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);			 //获取TIM3的通道1的输入捕捉值
		   	
				if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)	 // 得到高电平 持续时间
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[5] = 0xffff - TIM4CH4_CAPTURE_UPVAL + TIM4CH4_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[5] = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL ;
				}
				
				TIM4CH4_CAPTURE_STA = 0;															 //捕获标志位清零
		   	TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising);     //CC1P=0 设置为上升沿捕获
			}else  								                                   //还未开始,第一次捕获上升沿
			{
				TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);				//获取上升沿数据
				TIM4CH4_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		    TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);		 //CC1P=1 设置为下降沿捕获
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


