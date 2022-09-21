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
int pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0, pwm5 = 0, pwm6 = 0;

int depth = 0;	int depth1 = 0, depth2 = 0;


int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);//����JTAG ���� SWD
	delay_init();
	////   		    
	pwm_init();
	trans_others_init(1, 115200);
	trans_others_init(2, 115200);

	while(1)
	{
		if(trans_others_R(2, 0, 0, &depth1, &depth2, 0, 0, 0, 0, 0, 0))
		{
			if(depth < 0)
				depth = 100 * depth1 - depth2;
			else
				depth = 100 * depth1 + depth2;
		}

		if(trans_others_R(1, &init_identity, &init_pwm, &pwm1, &pwm2, &pwm3, &pwm4, &pwm5, &pwm6, 0, 0))
		{
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
			    TIM_SetCompare1(TIM3, pwm1);
				TIM_SetCompare2(TIM3, pwm2);
				TIM_SetCompare1(TIM4, pwm3);
				TIM_SetCompare2(TIM4, pwm4);
				TIM_SetCompare3(TIM4, pwm5);
				TIM_SetCompare4(TIM4, pwm6);
			}
		}

		printf("depth = %d\r\n", depth);
		
		delay_ms(100);
	}
}