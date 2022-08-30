#include "ccd.h"

#include "sys.h"
#include "delay.h"
#include "oled.h"

#define TSL_SI    PAout(3)   //SI  
#define TSL_CLK   PAout(2)   //CLK

u32 ccd_middle, black_cnt; 
//**************************************************************************
#define 	FILTER_NUM					5			//均值滤波数(必须是奇数)
#define		VALID_WIDTH					111			//有效像素点( 必须是奇数 && <=127 )

#define		Yuzhi_Queue_SIZE			5

#define		ADV_MIN				(64 - (VALID_WIDTH - 1) / 2)
#define		ADV_MAX				(64 + (VALID_WIDTH - 1) / 2)
#define		CCD_VALUE_SIZE		VALID_WIDTH + 1 - FILTER_NUM

static u16 ADV[128] = {0};
static void ccd_calculator()
{
	u8 i = 0, j = 0;
	
	static u8 first = 0, sixcnt = 0;
	
	static u16 CCD_MAX, CCD_MIN, CCD_Yuzhi;
	
	static u16 Yuzhi_Queue[Yuzhi_Queue_SIZE] = {0};
	static u8 Yuzhi_Queue_index = 0;
	static float Yuzhi_Queue_Av = 0;
	
	u16 ccd_value[CCD_VALUE_SIZE];
	u32 tmp = 0;
	
	CCD_MAX = 0;
	CCD_MIN = 0xFFFF;
	
	black_cnt = 0;
	ccd_middle = 0;
	for(i = 0; i < CCD_VALUE_SIZE; i++)
	{
		tmp = 0;
		for(j = 0; j < FILTER_NUM; j++)
			tmp += ADV[ADV_MIN + i + j];
		ccd_value[i] = tmp / FILTER_NUM;
		
		if(ccd_value[i] > CCD_MAX) 
			CCD_MAX = ccd_value[i];
		else if(ccd_value[i] < CCD_MIN) 
			CCD_MIN = ccd_value[i];
		
		if(ccd_value[i] < Yuzhi_Queue_Av)
		{
			black_cnt++;
			ccd_middle += i;
		}	
	}
	ccd_middle /= black_cnt;
	 
	CCD_Yuzhi = CCD_MAX / 2 + 3;
	if(CCD_Yuzhi > CCD_MAX) CCD_Yuzhi = CCD_MAX;

	Yuzhi_Queue_Av += (CCD_Yuzhi - Yuzhi_Queue[Yuzhi_Queue_index]) * 1.0 / Yuzhi_Queue_SIZE;
	Yuzhi_Queue[Yuzhi_Queue_index] =  CCD_Yuzhi;
	Yuzhi_Queue_index = (Yuzhi_Queue_index == Yuzhi_Queue_SIZE - 1) ? 0 :Yuzhi_Queue_index + 1;		
	
	
	if(first == 0)
	{
		first = 1;
		OLED_ShowString(00,10,"MID");
		OLED_ShowString(28,10,"NUM");
		OLED_ShowString(56,10,"MAX");
		OLED_ShowString(84,10,"MIN");
		OLED_ShowString(112,10,"YU");
	}
	sixcnt = (sixcnt == 6 - 1) ? 0 : sixcnt + 1;
	
	if(sixcnt == 0)
		for(i = 0;i < CCD_VALUE_SIZE; i++)
		{
			if(ccd_value[i] < Yuzhi_Queue_Av) 
				for(j = 0;j < 8; j++)
					OLED_DrawPoint(i, j, 1);
			else 
				for(j = 0;j < 8; j++)
					OLED_DrawPoint(i, j, 0);
		}
	else if(sixcnt == 1)
	{
		OLED_ShowNumber(00,20, ccd_middle, 3, 12);
		OLED_ShowNumber(28,20, black_cnt, 3, 12);
		OLED_ShowNumber(56,20, CCD_MAX, 3, 12);
		OLED_ShowNumber(84,20, CCD_MIN, 3, 12);
		OLED_ShowNumber(109,20,Yuzhi_Queue_Av, 3, 12);
	}
	else if(sixcnt >= 2 && sixcnt <= 5)
		OLED_Refresh_Gram0(sixcnt + 2);
}
//***************************************************

static void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++);
}

static u16 Get_Adc(u8 ch)   
{
	//设置转换序列	  		 
	ADC1->SQR3&=0XFFFFFFE0;//规则序列1 通道ch
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<22;       //启动规则转换通道 
	while(!(ADC1->SR&1<<1));//等待转换结束	 	   
	return ADC1->DR;		//返回adc值	
}

void ccd_getdata(void) 
{
	u8 i=0, tslp=0;
	TSL_CLK=1;
	TSL_SI=0; 
	Dly_us();
      
	TSL_SI=1; 
	TSL_CLK=0;
	Dly_us();
      
	TSL_CLK=1;
	TSL_SI=0;
	Dly_us(); 
	for(i=0;i<128;i++)
	{ 
		TSL_CLK=0; 
		Dly_us();  //调节曝光时间
		Dly_us();  //调节曝光时间
		ADV[tslp]=(Get_Adc(8))>>4;
		++tslp;
		TSL_CLK=1;
		Dly_us();
	}  
	ccd_calculator();
}

void ccd_init(void)
{
	//先初始化IO口
	RCC->APB2ENR|=1<<2;    //使能PORTA口时钟 
 	RCC->APB2ENR|=1<<3;    //使能PORTB口时钟 
	GPIOB->CRL&=0XFFFFFFF0;//PB0 anolog输入
 	 
	GPIOA->CRL&=0XFFFF00FF;//PA2 3 
	GPIOA->CRL|=0X00002200;//PA2 3 推挽输出 2MHZ   
	 
	RCC->APB2ENR|=1<<9;    //ADC1时钟使能	  
	RCC->APB2RSTR|=1<<9;   //ADC1复位
	RCC->APB2RSTR&=~(1<<9);//复位结束	    
	RCC->CFGR&=~(3<<14);   //分频因子清零	
	//SYSCLK/DIV2=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!
	//否则将导致ADC准确度下降! 
	RCC->CFGR|=2<<14;      	 

	ADC1->CR1&=0XF0FFFF;   //工作模式清零
	ADC1->CR1|=0<<16;      //独立工作模式  
	ADC1->CR1&=~(1<<8);    //非扫描模式	  
	ADC1->CR2&=~(1<<1);    //单次转换模式
	ADC1->CR2&=~(7<<17);	   
	ADC1->CR2|=7<<17;	   //软件控制转换  
	ADC1->CR2|=1<<20;      //使用用外部触发(SWSTART)!!!	必须使用一个事件来触发
	ADC1->CR2&=~(1<<11);   //右对齐	 
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1&=0<<20;     //1个转换在规则序列中 也就是只转换规则序列1 			   
	//设置通道7的采样时间
	ADC1->SMPR2&=0XF0FFFFFF;//通道8采样时间清空	  
	ADC1->SMPR2|=7<<24;     //通道8  239.5周期,提高采样时间可以提高精确度	 

	ADC1->CR2|=1<<0;	    //开启AD转换器	 
	ADC1->CR2|=1<<3;        //使能复位校准  
	while(ADC1->CR2&1<<3);  //等待校准结束 			 
    //该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。 		 
	ADC1->CR2|=1<<2;        //开启AD校准	   
	while(ADC1->CR2&1<<2);  //等待校准结束
	delay_ms(1);
}
