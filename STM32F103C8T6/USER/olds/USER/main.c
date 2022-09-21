
//*************************** ��һ�� *********************************

#if 0
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

#include "include.h"

PID depth_pid;
int v_x = 0, v_y = 0, v_p = 0, pwm_max = 0;

int pitch = 0, roll = 0, yaw = 0;

//************** Parameter of depth_pid *************//
int P_depth = 0, I_depth = 0, D_depth = 0;
int a_depth = 0;		

float depth;	int depth1 = 0, depth2 = 0;

float pwm_depth = 0;

//***************************************************//
int pwm_limit(int in)
{
	if(in < -pwm_max) in = -pwm_max;
	else if(in > pwm_max) in = pwm_max;
	return in * 5 + 1500;
}

int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);//����JTAG ���� SWD
	delay_init();
	////   		    
	PID_init(&depth_pid, 0, 0, 0, 1);
	pwm_init();
	trans_others_init(1, 115200);
	trans_others_init(2, 115200);

	while(1)
	{
		if(trans_others_R(2, 0, 0, &depth1, &depth2, 0, 0, 0, 0, 0, 0))
		{
			depth = 100 * depth1 + depth2;
		}
		if(trans_others_R(1, &P_depth, &I_depth, &D_depth, &a_depth, &v_x, &v_y, &v_p, &pwm_max, 0, 0))
		{
			PID_init(&depth_pid, P_depth, I_depth, D_depth, 0);
			printf("\r\n\r\nstm32 receives a piece of message:\r\n");
			printf("--  P_depth = %5d  --\r\n", P_depth);
			printf("--  I_depth = %5d  --\r\n", I_depth);
			printf("--  D_depth = %5d  --\r\n", D_depth);
			printf("--  a_depth = %5d  --\r\n", a_depth);
			printf("--    v_x   = %5d  --\r\n", v_x);
			printf("--    v_y   = %5d  --\r\n", v_y);
			printf("--    v_p   = %5d  --\r\n", v_p);
			printf("--  pwm_max = %5d  --\r\n\r\n\r\n", pwm_max);
		}
		printf("depth = %.2f\r\n", depth);
		
		pwm_depth = PID_calc1(&depth_pid, depth, a_depth);
		TIM_SetCompare1(TIM3, pwm_limit(pwm_depth));
		TIM_SetCompare2(TIM3, pwm_limit(pwm_depth));
		
		TIM_SetCompare1(TIM4, pwm_limit(v_x + v_y + v_p));
		TIM_SetCompare2(TIM4, pwm_limit(v_x - v_y - v_p));
		TIM_SetCompare3(TIM4, pwm_limit(v_x + v_y - v_p));
		TIM_SetCompare4(TIM4, pwm_limit(v_x - v_y + v_p));
		
		delay_ms(100);
	}
}

#endif


//*************************** init �� *********************************
#if 0

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



#endif
