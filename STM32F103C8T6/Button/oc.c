#include "oc.h"

void tim2_init()        
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;   

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 ;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	TIM_TimeBaseInitStruct.TIM_Period = 99;              
	TIM_TimeBaseInitStruct.TIM_Prescaler  = 71;          
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��PWM1ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //�����������
	//TIM_OC1Init(TIM2, &TIM_OCInitStruct);                      //��ʼ������Ƚϲ���
	TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	//TIM_OC3Init(TIM2, &TIM_OCInitStruct);
	//TIM_OC4Init(TIM2, &TIM_OCInitStruct);
	
	//TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //CH1ʹ��Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void tim3_init()        
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;   

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	
	TIM_TimeBaseInitStruct.TIM_Period = 19999;              
	TIM_TimeBaseInitStruct.TIM_Prescaler  = 71;          
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��PWM1ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //�����������
	TIM_OC1Init(TIM3, &TIM_OCInitStruct);                      //��ʼ������Ƚϲ���
	TIM_OC2Init(TIM3, &TIM_OCInitStruct);
	TIM_OC3Init(TIM3, &TIM_OCInitStruct);
	TIM_OC4Init(TIM3, &TIM_OCInitStruct);
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH1ʹ��Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}

void tim4_init()        
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;   

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_Period = 19999;              
	TIM_TimeBaseInitStruct.TIM_Prescaler  = 71;          
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��PWM1ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //�����������
	TIM_OC1Init(TIM4, &TIM_OCInitStruct);                      //��ʼ������Ƚϲ���
	TIM_OC2Init(TIM4, &TIM_OCInitStruct);
	TIM_OC3Init(TIM4, &TIM_OCInitStruct);
	TIM_OC4Init(TIM4, &TIM_OCInitStruct);
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //CH1ʹ��Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

void pwm_init()
{
	tim2_init();
	tim3_init();
	tim4_init();
}

