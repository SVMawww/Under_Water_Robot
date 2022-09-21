#include "uart.h"

#include <stdio.h>

static void*	USARTx					[5]	=	{	USART1,					USART2,					USART3,					0,						0						};
static u32		RCC_APB1Periph_USARTx	[5]	=	{	0,						RCC_APB1Periph_USART2,	RCC_APB1Periph_USART3,	0,						0						};
static u32		RCC_APB2Periph_USARTx	[5]	=	{	RCC_APB2Periph_USART1,	0,						0,						0,						0						};

static u32		GPIO_Remap_USARTx		[5]	=	{	0,						0,						0,						0,						0						};

static void*	GPIOx					[5]	=	{	GPIOA,					GPIOA,					GPIOB,					0,						0						};
static u32		RCC_APB2Periph_GPIOx	[5]	=	{	RCC_APB2Periph_GPIOA,	RCC_APB2Periph_GPIOA,	RCC_APB2Periph_GPIOB,	0,						0						};
static u32		T_GPIO_Pin_x			[5]	=	{	GPIO_Pin_9,				GPIO_Pin_2,				GPIO_Pin_10,			0,						0						};
static u32		R_GPIO_Pin_x			[5]	=	{	GPIO_Pin_10,			GPIO_Pin_3,				GPIO_Pin_11,			0,						0						};

static u8		USARTx_IRQn				[5]	=	{	USART1_IRQn,			USART2_IRQn,			USART3_IRQn,			0,						0						};
				
static u32		RCC_AHBPeriph_DMAx		[5]	=	{	RCC_AHBPeriph_DMA1,		RCC_AHBPeriph_DMA1,		RCC_AHBPeriph_DMA1,		0,						0						};
static void*	T_DMA_Channel_x			[5]	=	{	DMA1_Channel4,			DMA1_Channel7,			DMA1_Channel2,			0,						0						};
static void*	R_DMA_Channel_x			[5]	=	{	DMA1_Channel5,			DMA1_Channel6,			DMA1_Channel3,			0,						0						};


#define 	 RX_PRE_MAX					200
static u8	 rx_pre						[5]	[RX_PRE_MAX] = {0};

static u32   bauds						[5] = 	{0,0,0,0,0};
static void* tx_ptrs 					[5] = 	{0,0,0,0,0};
static u32 	 tx_lens                	[5] = 	{0,0,0,0,0};
static void* rx_ptrs 					[5] = 	{0,0,0,0,0};
static u32 	 rx_lens 					[5] = 	{0,0,0,0,0};
static int*  rx_flags 					[5] = 	{0,0,0,0,0};
//------------------------------------------------------------------------------------------//
static void gpio_init(int i)
{
    GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_PinRemapConfig(GPIO_Remap_USARTx[i], ENABLE);
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOx[i], ENABLE );

	GPIO_InitStruct.GPIO_Pin = T_GPIO_Pin_x[i];
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx[i], &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = R_GPIO_Pin_x[i];
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOx[i], &GPIO_InitStruct);
}

static void nvic_init(int i)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn[i];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
}

static void dma_Tinit(int i)
{
	DMA_InitTypeDef DMA_InitStruct;	
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMAx[i], ENABLE);

	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)(tx_ptrs[i]);
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&((USART_TypeDef *)USARTx[i])->DR);
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	
	DMA_InitStruct.DMA_BufferSize = 0;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( T_DMA_Channel_x[i], &DMA_InitStruct);
	
	DMA_Cmd( T_DMA_Channel_x[i], ENABLE);
}	
static void dma_Rinit(int i)
{	
	DMA_InitTypeDef DMA_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMAx[i], ENABLE);
	
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)(rx_pre[i]);
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&((USART_TypeDef *)USARTx[i])->DR);
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	
	DMA_InitStruct.DMA_BufferSize = RX_PRE_MAX;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( R_DMA_Channel_x[i], &DMA_InitStruct);
		
	DMA_Cmd( R_DMA_Channel_x[i], ENABLE);
}
static void usart_init(int i)
{
    USART_InitTypeDef USART_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USARTx[i], ENABLE);		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USARTx[i], ENABLE);

	USART_InitStruct.USART_BaudRate = bauds[i];
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USARTx[i], &USART_InitStruct);
}

//------------------------------------------------------------------------------------------//
void uart_init(int i, u32 baud, void* tx_ptr, u32 tx_len, void* rx_ptr, u32 rx_len, int* rx_flag)
{
	i--;
	
	bauds[i] = baud;
	tx_ptrs[i] = tx_ptr;
	rx_ptrs[i] = rx_ptr;
	tx_lens[i] = tx_len;
	rx_lens[i] = rx_len;
	rx_flags[i] = rx_flag;
	
	nvic_init(i);
	if(T_DMA_Channel_x[i] != 0) dma_Tinit(i);
	if(R_DMA_Channel_x[i] != 0) dma_Rinit(i);
	usart_init(i);
	gpio_init(i);
	
	USART_ITConfig(USARTx[i], USART_IT_IDLE, ENABLE);
	USART_DMACmd(USARTx[i], USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USARTx[i], ENABLE);
}
//------------------------------------------------------------------------------------------//
void uart_singletrans(int i)
{
    i--;
	
	if(DMA_GetCurrDataCounter(T_DMA_Channel_x[i])) while(1);
    DMA_Cmd(T_DMA_Channel_x[i], DISABLE);
	DMA_SetCurrDataCounter( T_DMA_Channel_x[i], tx_lens[i] ); 
    DMA_Cmd(T_DMA_Channel_x[i], ENABLE);
}
//------------------------------------------------------------------------------------------//
//读取串口数据时，先将其置入缓冲数组（而不是直接提取分析串口数据），避免中断过长导致的时序紊乱
static void USARTx_IRQHandler( int i )
{
    u8 cnt = 0;
	u16 j = 0;

	i--;
	if( USART_GetITStatus( USARTx[i], USART_IT_IDLE ) != RESET )
    {
        USART_ReceiveData( USARTx[i] );	//读取数据注意：这句必须要，否则不能够清除空闲中断标志位。
        DMA_Cmd( R_DMA_Channel_x[i], DISABLE );
		
        cnt = RX_PRE_MAX - DMA_GetCurrDataCounter(  R_DMA_Channel_x[i] ); 
		*(int*)rx_flags[i] = cnt;
		for( j = 0; j < cnt && j < rx_lens[i]; j++ )
			((char* )rx_ptrs[i])[j] = rx_pre[i][j];
		
        USART_ClearITPendingBit( USARTx[i], USART_IT_IDLE );
		
        DMA_SetCurrDataCounter( R_DMA_Channel_x[i], RX_PRE_MAX );
		DMA_Cmd(R_DMA_Channel_x[i], ENABLE);
    }
}

void USART1_IRQHandler( void )
{
    USARTx_IRQHandler( 1 );
}

void USART2_IRQHandler( void )
{
    USARTx_IRQHandler( 2 );
}

void USART3_IRQHandler( void )
{
    USARTx_IRQHandler( 3 );
}

void UART4_IRQHandler( void )
{
    USARTx_IRQHandler( 4 );
}
//------------------------------------------------------------------------------------------//
//串口1提供printf功能
int fputc(int ch, FILE* f)
{      
    USART_SendData(USART1, (uint8_t)ch);
    while( !USART_GetFlagStatus(USART1, USART_FLAG_TC) );	
	return ch;
}
