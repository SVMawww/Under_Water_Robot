#include "gpio.h"

#define GPI_USE GPIO_Pin_All

#define GPO_USE GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6

static void(* i0)(void);

void gpi_init( void(* i0_)(void) )
{
    GPIO_InitTypeDef GPIO_InitStructure;    
	EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
	i0 = i0_;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOF, ENABLE );
	
	GPIO_InitStructure.GPIO_Pin = GPI_USE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init( GPIOF, &GPIO_InitStructure );
	
	if(GPI_USE & GPIO_Pin_0) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource0 );
	if(GPI_USE & GPIO_Pin_1) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource1 );
	if(GPI_USE & GPIO_Pin_2) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource2 );
	if(GPI_USE & GPIO_Pin_3) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource3 );
	if(GPI_USE & GPIO_Pin_4) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource4 );
	if(GPI_USE & GPIO_Pin_5) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource5 );
	if(GPI_USE & GPIO_Pin_6) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource6 );
	if(GPI_USE & GPIO_Pin_7) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource7 );
	if(GPI_USE & GPIO_Pin_8) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource8 );
	if(GPI_USE & GPIO_Pin_9) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource9 );
	if(GPI_USE & GPIO_Pin_10) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource10 );
	if(GPI_USE & GPIO_Pin_11) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource11 );
	if(GPI_USE & GPIO_Pin_12) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource12 );
	if(GPI_USE & GPIO_Pin_13) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource13 );
	if(GPI_USE & GPIO_Pin_14) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource14 );
	if(GPI_USE & GPIO_Pin_15) GPIO_EXTILineConfig( GPIO_PortSourceGPIOF, GPIO_PinSource15 );
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | 
									EXTI_Line1 |
									EXTI_Line2 |
									EXTI_Line3 |
									EXTI_Line4 |
									EXTI_Line5 |
									EXTI_Line6 |
									EXTI_Line7 |
									EXTI_Line8 |
									EXTI_Line9 |
									EXTI_Line10 |
									EXTI_Line11 |
									EXTI_Line12 |
									EXTI_Line13 |
									EXTI_Line14 |
									EXTI_Line15 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init( &EXTI_InitStructure );
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_Init( &NVIC_InitStructure );	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Init( &NVIC_InitStructure );	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init( &NVIC_InitStructure );	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init( &NVIC_InitStructure );	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init( &NVIC_InitStructure );	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_Init( &NVIC_InitStructure );	
}

void EXTI15_10_IRQHandler(void)
{
	EXTI_ClearITPendingBit( (uint32_t)0xFFFFF );
	i0();
}
void EXTI9_5_IRQHandler(void)
{
	EXTI_ClearITPendingBit( (uint32_t)0xFFFFF );
	i0();
}
void EXTI0_IRQHandler(void)
{
	EXTI_ClearITPendingBit( (uint32_t)0xFFFFF );
	i0();
}
void EXTI1_IRQHandler(void)
{
	EXTI_ClearITPendingBit( (uint32_t)0xFFFFF );
	i0();
}
void EXTI2_IRQHandler(void)
{
	EXTI_ClearITPendingBit( (uint32_t)0xFFFFF );
	i0();
}
void EXTI3_IRQHandler(void)
{
	EXTI_ClearITPendingBit( (uint32_t)0xFFFFF );
	i0();
}
void EXTI4_IRQHandler(void)
{
	EXTI_ClearITPendingBit( (uint32_t)0xFFFFF );
	i0();
}


void gpo_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	 

	GPIO_InitStructure.GPIO_Pin = GPO_USE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE, GPO_USE);
}
