//***************************************************//
//�ӿڣ���ȴ�������PA3
//      ��ݮ�ɵ�RX��PA9��STM32��TX��
//      ��ݮ�ɵ�TX��PA10��STM32��RX��
//		���������������PA6 PA7
//		����ˮƽ��������PB6 PB7 PB8 PB9���������������������Ӳ����ʱ������������
// 
//***************************************************//
//STM32F103ʵ�ֹ��� 1.��ȴ�����
//                  2.����ٶȿ����� * 6
//
//�������������ʽ���ַ��������£�
//init_identity, init_pwm, pwm1, pwm2, pwm3, pwm4, pwm5, pwm6
//
//��init_identity = 1 ~ 6 ʱ��Ϊ������������ģʽ:      init_identity, init_pwm       ������
//��init_identity = ����ʱ��  Ϊ������������ģʽ��pwm1, pwm2, pwm3, pwm4, pwm5, pwm6 ������
//***************************************************//

#include "include.h"

int init_identity = 0, init_pwm = 0;
int pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0, pwm5 = 0, pwm6 = 0, catchball = 0;

int depth = 0;	int depth1 = 0, depth2 = 0;
int temp = 0;


int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);//����JTAG ���� SWD
	delay_init();
	////   		    
	pwm_init();
	mpu__init();

	trans_others_init(1, 115200); 
	trans_others_init(2, 115200);
	
	while(1)
	{	
		if(trans_others_R(2, &temp, 0, &depth1, &depth2, 0, 0, 0, 0, 0, 0))
		{
			if(depth < 0)
				depth = 100 * depth1 - depth2;
			else
				depth = 100 * depth1 + depth2;
			
			
			
			mpu_getdata();
			
	        printf("depth = %d, pitch = %d\r\n", depth, (int)(100.0  * pitch));
		}

		if(trans_others_R(1, &init_identity, &init_pwm, &pwm1, &pwm2, &pwm3, &pwm4, &pwm5, &pwm6, &catchball, 0))
		{
			if(catchball == 0)
			{
				TIM_SetCompare2(TIM2, 0);
			}
		    else if(catchball == 1)
			{
				GPIO_ResetBits(GPIOA, GPIO_Pin_0);
				GPIO_SetBits(GPIOA, GPIO_Pin_4);
				TIM_SetCompare2(TIM2, 50);
			}
		    else if(catchball == 2)
			{
				GPIO_SetBits(GPIOA, GPIO_Pin_0);
				GPIO_ResetBits(GPIOA, GPIO_Pin_4);
				TIM_SetCompare2(TIM2, 50);
			}
			
//			printf("\r\n%d\r\n", init_identity);
//			printf("%d\r\n", init_pwm);
//			printf("%d\r\n", pwm1);
//			printf("%d\r\n", pwm2);
//			printf("%d\r\n", pwm3);
//			printf("%d\r\n", pwm4);
//			printf("%d\r\n", pwm5);
//			printf("%d\r\n", pwm6);
			
			if(init_identity == 1)
				TIM_SetCompare1(TIM3, init_pwm);
			else if(init_identity == 2)
				TIM_SetCompare2(TIM3, init_pwm);
			else if(init_identity == 3)
				TIM_SetCompare1(TIM4, init_pwm);
			else if(init_identity == 4)
				TIM_SetCompare2(TIM4, init_pwm);
			else if(init_identity == 5)
				TIM_SetCompare3(TIM4, init_pwm);
			else if(init_identity == 6)
				TIM_SetCompare4(TIM4, init_pwm);
			else
		    {
			    if(pwm1 == 0) pwm1 = 1500;
				if(pwm2 == 0) pwm2 = 1500;
				if(pwm3 == 0) pwm3 = 1500;
				if(pwm4 == 0) pwm4 = 1500;
				if(pwm5 == 0) pwm5 = 1500;
				if(pwm6 == 0) pwm6 = 1500;
				
				TIM_SetCompare1(TIM3, pwm1);
				TIM_SetCompare2(TIM3, pwm2);
				TIM_SetCompare1(TIM4, pwm3);
				TIM_SetCompare2(TIM4, pwm4);
				TIM_SetCompare3(TIM4, pwm5);
				TIM_SetCompare4(TIM4, pwm6);
			}
		}
		
		//mpu_getdata();
		//delay_ms(100);	
	    //printf("depth = %d, pitch = %d\r\n", depth, (int)pitch);
	}
}