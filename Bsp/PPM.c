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
	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	 //ʹ��TIM4ʱ�� PB8-9
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	 //ʹ��TIM3ʱ��	PC6-9
 	  
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //ʹ��GPIOBʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);    //ʹ��AFIO���ܵ�ʱ��
	  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);      //TIM3ȫ��������ӳ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //ʹ��GPIOCʱ��
	 // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	 //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	  
	  // GPIOC��ʼ�� PC6-PC9
	  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;             //PB6 ���֮ǰ����  
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;     			//��������    
	  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIOC,GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);		
		
		// GPIOB��ʼ�� PB8,PB9
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;						//��������    
		//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);	
		
		//��ʼ����ʱ��3 TIM3	 
	  TIM_DeInit(TIM3);
		
		TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                      //�趨�������Զ���װֵ 
	  TIM_TimeBaseStructure.TIM_Prescaler =(72-1); 	                 	//Ԥ��Ƶ�� 72   1Mhz����ȷ��1us
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      		//����ʱ�ӷָ�:TDTS = Tck_tim
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  		//TIM���ϼ���ģʽ
	  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);              		//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
    
		//��ʼ����ʱ��4 TIM4	
	  TIM_DeInit(TIM4);
		
		TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                      //�趨�������Զ���װֵ 
	  TIM_TimeBaseStructure.TIM_Prescaler =(72-1); 	                 	//Ԥ��Ƶ�� 72   1Mhz����ȷ��1us
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      		//����ʱ�ӷָ�:TDTS = Tck_tim
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  		//TIM���ϼ���ģʽ
	  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		
		//��ʼ��TIM3���벶�����, �⼸��ͨ���ܲ�����GPIOһ����һ���أ�����
	  TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  	TIM3_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   	 //�����ز���
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	
	  //��ʼ��TIM3 ch2���벶�����
		TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2;                //CC1S=01 	ѡ������� IC2ӳ�䵽TI1��
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //�����ز���
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
		
		//��ʼ��TIM3 ch3���벶�����
		TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	ѡ������� IC3ӳ�䵽TI1��
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //�����ز���
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
		
		//��ʼ��TIM3 ch4���벶�����
		TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	ѡ������� IC4ӳ�䵽TI1��
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //�����ز���
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	
		//��ʼ��TIM4 ch3���벶�����,
		TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	ѡ������� IC3ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //�����ز���
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
		
		//��ʼ��TIM4 ch4���벶�����,
		TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	ѡ������� IC4ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //�����ز���
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
  
		TIM_Cmd(TIM4,ENABLE ); 
		
	  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);        //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�	
	  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	  TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	  TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
		
		TIM_Cmd(TIM3,ENABLE ); 
		
		TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);					//����������жϣ�����CC3IE,CC4IE�����ж�	
	  TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;            //TIM4�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�1��
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //�����ȼ�0��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQͨ����ʹ��
		NVIC_Init(&NVIC_InitStructure); 
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;            //TIM3�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�1��
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;         //�����ȼ�0��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQͨ����ʹ��
		NVIC_Init(&NVIC_InitStructure); 
	
}


/**********************************
��˹9 ң����
����19.52ms
��ЧPWM�ߵ�ƽ 1.040ms~1.860ms���� ��Ӧ 0%~100%  1040~1860����
***********************************/

static u16 			Rc_Pwm_In[6];  			//1040~1860


static u8 	TIM3CH1_CAPTURE_STA = 0;	//Time3ͨ��1���벶���־������λ�������־����6λ�������־		
static u16 TIM3CH1_CAPTURE_UPVAL;
static u16 TIM3CH1_CAPTURE_DOWNVAL;

static u8 	TIM3CH2_CAPTURE_STA = 0;	//Time3ͨ��2���벶���־������λ�������־����6λ�������־		
static u16 TIM3CH2_CAPTURE_UPVAL;
static u16 TIM3CH2_CAPTURE_DOWNVAL;

static u8 	TIM3CH3_CAPTURE_STA = 0;	//Time3ͨ��3���벶���־������λ�������־����6λ�������־		
static u16 TIM3CH3_CAPTURE_UPVAL;
static u16 TIM3CH3_CAPTURE_DOWNVAL;

static u8 	TIM3CH4_CAPTURE_STA = 0;	//Time3ͨ��4���벶���־������λ�������־����6λ�������־		
static u16 TIM3CH4_CAPTURE_UPVAL;
static u16 TIM3CH4_CAPTURE_DOWNVAL;

static u8 	TIM4CH3_CAPTURE_STA = 0;	//Time4ͨ��3���벶���־������λ�������־����6λ�������־		
static u16 TIM4CH3_CAPTURE_UPVAL;
static u16 TIM4CH3_CAPTURE_DOWNVAL;

static u8 	TIM4CH4_CAPTURE_STA = 0;	//Time4ͨ��4���벶���־������λ�������־����6λ�������־		
static u16 TIM4CH4_CAPTURE_UPVAL;
static u16 TIM4CH4_CAPTURE_DOWNVAL;



/*��ʱ��3�жϷ������


			����ߵ�ƽ����ʱ��
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
	
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)            //����1���������¼�
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC1); 								 //����жϱ�־λ
			if(TIM3CH1_CAPTURE_STA&0X40)		                       	 //����һ���½��� 		
			{	  			
				//TIM3CH1_CAPTURE_STA |= 0X80;		                     //��ǳɹ�����һ��������
				TIM3CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM3);			 //��ȡTIM3��ͨ��1�����벶׽ֵ
		   	if (TIM3CH1_CAPTURE_DOWNVAL < TIM3CH2_CAPTURE_UPVAL)	 // �õ��ߵ�ƽ ����ʱ��
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH1_CAPTURE_UPVAL + TIM3CH1_CAPTURE_DOWNVAL
					Rc_Pwm_In[0] = 0xffff - TIM3CH1_CAPTURE_UPVAL + TIM3CH1_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[0] = TIM3CH1_CAPTURE_DOWNVAL - TIM3CH1_CAPTURE_UPVAL ;
				}
				
				
				TIM3CH1_CAPTURE_STA = 0;															 //�����־λ����
				TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); 		 //CC1P=0 ����Ϊ�����ز���				
				//printf("TIM3CH1:%d us\r\n",Rc_Pwm_In[0]);							 //for test
			}
			else  								                                 	//��δ��ʼ,��һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM3CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM3);				//��ȡ����������
				TIM3CH1_CAPTURE_STA|=0X40;		                        //��ǲ�����������
		    TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}		    
		}	

	
	 
	  if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)            //����2���������¼�
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC2); 								 //����жϱ�־λ
			if(TIM3CH2_CAPTURE_STA&0X40)		                       	 //����һ���½��� 		
			{	  			
				//TIM3CH2_CAPTURE_STA|=0X80;		                       	 //��ǳɹ�����һ��������
				TIM3CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM3);			 //��ȡTIM3��ͨ��1�����벶׽ֵ
		   	
				if (TIM3CH2_CAPTURE_DOWNVAL < TIM3CH2_CAPTURE_UPVAL)	 // �õ��ߵ�ƽ ����ʱ��
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH2_CAPTURE_UPVAL + TIM3CH2_CAPTURE_DOWNVAL
					Rc_Pwm_In[1] = 0xffff - TIM3CH2_CAPTURE_UPVAL + TIM3CH2_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[1] = TIM3CH2_CAPTURE_DOWNVAL - TIM3CH2_CAPTURE_UPVAL ;
				}
				
				TIM3CH2_CAPTURE_STA = 0;															 //�����־λ����
		   	TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Rising); 		 //CC1P=0 ����Ϊ�����ز���
				//printf("TIM3CH2:%d us\r\n",Rc_Pwm_In[1]);
			}
			else  								                                 	 //��δ��ʼ,��һ�β���������
			{
				TIM3CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM3);				//��ȡ����������
				TIM3CH2_CAPTURE_STA|=0X40;		                        //��ǲ�����������
		    TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}		    
		}	
  	
    if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)            //����3���������¼�
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC3); 								 //����жϱ�־λ
			if(TIM3CH3_CAPTURE_STA&0X40)		                         //����һ���½��� 		
			{	  			
				TIM3CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM3);			 //��ȡTIM3��ͨ��1�����벶׽ֵ
		   	
				if (TIM3CH3_CAPTURE_DOWNVAL < TIM3CH3_CAPTURE_UPVAL)	 // �õ��ߵ�ƽ ����ʱ��
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[2] = 0xffff - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[2] = TIM3CH3_CAPTURE_DOWNVAL - TIM3CH3_CAPTURE_UPVAL ;
				}
				
				
				TIM3CH3_CAPTURE_STA = 0;															 //�����־λ����
		   	TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Rising);     //CC1P=0 ����Ϊ�����ز���
			}
			else  								                                   //��δ��ʼ,��һ�β���������
			{
				TIM3CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM3);				//��ȡ����������
				TIM3CH3_CAPTURE_STA|=0X40;		                        //��ǲ�����������
		    TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز��
			}		    
		}	

    if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)            //����4���������¼�
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC4); 								 //����жϱ�־λ
			if(TIM3CH4_CAPTURE_STA&0X40)		                         //����һ���½��� 		
			{	  			
				TIM3CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM3);			 //��ȡTIM3��ͨ��1�����벶׽ֵ
		   	
				if (TIM3CH4_CAPTURE_DOWNVAL < TIM3CH4_CAPTURE_UPVAL)	 // �õ��ߵ�ƽ ����ʱ��
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[3] = 0xffff - TIM3CH4_CAPTURE_UPVAL + TIM3CH4_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[3] = TIM3CH4_CAPTURE_DOWNVAL - TIM3CH4_CAPTURE_UPVAL ;
				}
				
				TIM3CH4_CAPTURE_STA = 0;															 //�����־λ����
		   	TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising);     //CC1P=0 ����Ϊ�����ز���
			}else  								                                   //��δ��ʼ,��һ�β���������
			{
				TIM3CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM3);				//��ȡ����������
				TIM3CH4_CAPTURE_STA|=0X40;		                        //��ǲ�����������
		    TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);		 //CC1P=1 ����Ϊ�½��ز���
			}		    
		}		
}


//��ʱ��4�жϷ������	  ����  oK
void TIM4_IRQHandler(void)
{ 
	

		if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)            //����4���������¼�
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC3); 								 //����жϱ�־λ
			if(TIM4CH3_CAPTURE_STA&0X40)		                         //����һ���½��� 		
			{	  			
				TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);			 //��ȡTIM3��ͨ��1�����벶׽ֵ
		   	
				if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)	 // �õ��ߵ�ƽ ����ʱ��
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[4] = 0xffff - TIM4CH3_CAPTURE_UPVAL + TIM4CH3_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[4] = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL ;
				}
				
				TIM4CH3_CAPTURE_STA = 0;															 //�����־λ����
		   	TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising);     //CC1P=0 ����Ϊ�����ز���
			}else  								                                   //��δ��ʼ,��һ�β���������
			{
				TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);				//��ȡ����������
				TIM4CH3_CAPTURE_STA|=0X40;		                        //��ǲ�����������
		    TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);		 //CC1P=1 ����Ϊ�½��ز���
			}		    
		}

		if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)            //����4���������¼�
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC4); 								 //����жϱ�־λ
			if(TIM4CH4_CAPTURE_STA&0X40)		                         //����һ���½��� 		
			{	  			
				TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);			 //��ȡTIM3��ͨ��1�����벶׽ֵ
		   	
				if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)	 // �õ��ߵ�ƽ ����ʱ��
				{
					//Rc_Pwm_In[0] = 65536 - TIM3CH3_CAPTURE_UPVAL + TIM3CH3_CAPTURE_DOWNVAL
					Rc_Pwm_In[5] = 0xffff - TIM4CH4_CAPTURE_UPVAL + TIM4CH4_CAPTURE_DOWNVAL ;
				}
				else
				{
					Rc_Pwm_In[5] = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL ;
				}
				
				TIM4CH4_CAPTURE_STA = 0;															 //�����־λ����
		   	TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising);     //CC1P=0 ����Ϊ�����ز���
			}else  								                                   //��δ��ʼ,��һ�β���������
			{
				TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);				//��ȡ����������
				TIM4CH4_CAPTURE_STA|=0X40;		                        //��ǲ�����������
		    TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);		 //CC1P=1 ����Ϊ�½��ز���
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


