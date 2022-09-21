#include "ccd.h"

#include "sys.h"
#include "delay.h"
#include "oled.h"

#define TSL_SI    PAout(3)   //SI  
#define TSL_CLK   PAout(2)   //CLK

u32 ccd_middle, black_cnt; 
//**************************************************************************
#define 	FILTER_NUM					5			//��ֵ�˲���(����������)
#define		VALID_WIDTH					111			//��Ч���ص�( ���������� && <=127 )

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
	//����ת������	  		 
	ADC1->SQR3&=0XFFFFFFE0;//��������1 ͨ��ch
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<22;       //��������ת��ͨ�� 
	while(!(ADC1->SR&1<<1));//�ȴ�ת������	 	   
	return ADC1->DR;		//����adcֵ	
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
		Dly_us();  //�����ع�ʱ��
		Dly_us();  //�����ع�ʱ��
		ADV[tslp]=(Get_Adc(8))>>4;
		++tslp;
		TSL_CLK=1;
		Dly_us();
	}  
	ccd_calculator();
}

void ccd_init(void)
{
	//�ȳ�ʼ��IO��
	RCC->APB2ENR|=1<<2;    //ʹ��PORTA��ʱ�� 
 	RCC->APB2ENR|=1<<3;    //ʹ��PORTB��ʱ�� 
	GPIOB->CRL&=0XFFFFFFF0;//PB0 anolog����
 	 
	GPIOA->CRL&=0XFFFF00FF;//PA2 3 
	GPIOA->CRL|=0X00002200;//PA2 3 ������� 2MHZ   
	 
	RCC->APB2ENR|=1<<9;    //ADC1ʱ��ʹ��	  
	RCC->APB2RSTR|=1<<9;   //ADC1��λ
	RCC->APB2RSTR&=~(1<<9);//��λ����	    
	RCC->CFGR&=~(3<<14);   //��Ƶ��������	
	//SYSCLK/DIV2=12M ADCʱ������Ϊ12M,ADC���ʱ�Ӳ��ܳ���14M!
	//���򽫵���ADC׼ȷ���½�! 
	RCC->CFGR|=2<<14;      	 

	ADC1->CR1&=0XF0FFFF;   //����ģʽ����
	ADC1->CR1|=0<<16;      //��������ģʽ  
	ADC1->CR1&=~(1<<8);    //��ɨ��ģʽ	  
	ADC1->CR2&=~(1<<1);    //����ת��ģʽ
	ADC1->CR2&=~(7<<17);	   
	ADC1->CR2|=7<<17;	   //�������ת��  
	ADC1->CR2|=1<<20;      //ʹ�����ⲿ����(SWSTART)!!!	����ʹ��һ���¼�������
	ADC1->CR2&=~(1<<11);   //�Ҷ���	 
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1&=0<<20;     //1��ת���ڹ��������� Ҳ����ֻת����������1 			   
	//����ͨ��7�Ĳ���ʱ��
	ADC1->SMPR2&=0XF0FFFFFF;//ͨ��8����ʱ�����	  
	ADC1->SMPR2|=7<<24;     //ͨ��8  239.5����,��߲���ʱ�������߾�ȷ��	 

	ADC1->CR2|=1<<0;	    //����ADת����	 
	ADC1->CR2|=1<<3;        //ʹ�ܸ�λУ׼  
	while(ADC1->CR2&1<<3);  //�ȴ�У׼���� 			 
    //��λ��������ò���Ӳ���������У׼�Ĵ�������ʼ�����λ��������� 		 
	ADC1->CR2|=1<<2;        //����ADУ׼	   
	while(ADC1->CR2&1<<2);  //�ȴ�У׼����
	delay_ms(1);
}
