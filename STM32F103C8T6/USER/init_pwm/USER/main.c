//***************************************************//
//�ӿڣ���ȴ�������PA3
//      ��ݮ�ɵ�RX��PA9��STM32��TX��
//      ��ݮ�ɵ�TX��PA10��STM32��RX��
//		���������������PA6 PA7
//		����ˮƽ��������PB6 PB7 PB8 PB9���������������������Ӳ����ʱ������������
//
//***************************************************//
//STM32F103ʵ�ֹ��� 1.��ȿ�������PID��Ҫ�Ե���
//                  2.xy�ٶȿ�����
//                  2.��ת�ٶȿ�����
//
//�������������ʽ���ַ��������£�
//P_depth	I_depth		D_depth		a_depth 	v_x 	v_y		v_p 	pwm_max
//
//���У���v_x��ֵΪ0~100�����������ʻ��
//		         100~200��������ǰ��ʻ��
//���У���v_y��ֵΪ0~100������������ʻ��
//		         100~200������������ʻ��
//���У���v_p��ֵΪ0~100����������ʱ����ת��
//		         100~200��������˳ʱ����ת��
//���У�pwm_max = 100ʱ�����������������Դﵽ��ǿ����100%��
//		pwm_max = xʱ�����������������Դﵽx%��
//
//***************************************************//
int init_identity = 0;
int init_pwm = 0;

#include "include.h"

int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);//����JTAG ���� SWD
	delay_init();
	////   		    
	pwm_init();
	trans_others_init(1, 115200);

	while(1)
	{	
		if(trans_others_R(1, &init_identity, &init_pwm, 0,0,0,0,0,0,0,0))
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
			printf("data got!!!");
		}
	}
}
