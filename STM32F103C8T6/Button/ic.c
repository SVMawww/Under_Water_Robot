#include "ic.h"

static void* TIMx                 [4] = {TIM3 					,TIM4 					  ,TIM5 					,TIM8 					};
static u32   RCC_APB1Periph_TIMx  [4] = {RCC_APB1Periph_TIM3 	,RCC_APB1Periph_TIM4 	  ,RCC_APB1Periph_TIM5 	    ,RCC_APB2Periph_TIM8 	};

static void* GPIOx                [4] = {GPIOB                  ,GPIOD                    ,GPIOA                    ,GPIOC                  };                       
static u32   RCC_APB2Periph_GPIOx [4] = {RCC_APB2Periph_GPIOB   ,RCC_APB2Periph_GPIOD     ,RCC_APB2Periph_GPIOA     ,RCC_APB2Periph_GPIOC   };                       
static u32   A_GPIO_Pin_x         [4] = {GPIO_Pin_4             ,GPIO_Pin_12              ,GPIO_Pin_0               ,GPIO_Pin_6             };                       
static u32   B_GPIO_Pin_x         [4] = {GPIO_Pin_5             ,GPIO_Pin_13              ,GPIO_Pin_1             	,GPIO_Pin_7				}; 

static void gpio_init(int i)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	if(i == 0)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
	}
	else if(i == 1)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
	}
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx[i], ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = A_GPIO_Pin_x[i] | B_GPIO_Pin_x[i];
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx[i], &GPIO_InitStructure);
}

static void tim_init(int i)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	
	if(i == 3)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	else
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIMx[i], ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIMx[i], &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIMx[i], TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIMx[i], &TIM_ICInitStructure);
}

void ic_init(int i)
{	
	i--;
	gpio_init(i);
	tim_init(i)	;
 
	TIM_Cmd(TIMx[i], ENABLE);
}
int ic_getdata(int i)
{
	int encoder_value;
	i--;
	encoder_value = -((short)(((TIM_TypeDef *)TIMx[i])->CNT));
	TIM_SetCounter(TIMx[i], 65535);
	return encoder_value;
}
