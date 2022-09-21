
//*************************** 第一版 *********************************

#if 0
//***************************************************//
//接口：深度传感器：PA3
//      树莓派的RX：PA9（STM32的TX）
//      树莓派的TX：PA10（STM32的RX）
//		控制深度螺旋桨：PA6 PA7
//		控制水平螺旋桨：PB6 PB7 PB8 PB9（具体连接情况，在连接硬件的时候配合软件调）
//
//***************************************************//
//STM32F103实现功能 1.深度控制器（PID需要自调）
//                  2.xy速度控制器
//                  2.旋转速度控制器
//
//串口数据输入格式（字符串）如下：
//P_depth	I_depth		D_depth		a_depth 	v_x 	v_y		v_p 	pwm_max
//
//其中：当v_x的值为0~100，船体向后行驶，
//		         100~200，船体向前行驶。
//其中：当v_y的值为0~100，船体向右行驶，
//		         100~200，船体向左行驶。
//其中：当v_p的值为0~100，船体向逆时针旋转，
//		         100~200，船体向顺时针旋转。
//其中：pwm_max = 100时，螺旋桨驱动力可以达到最强，即100%。
//		pwm_max = x时，螺旋桨驱动力可以达到x%。
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
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);//禁用JTAG 启用 SWD
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


//*************************** init 版 *********************************
#if 0

//***************************************************//
//接口：深度传感器：PA3
//      树莓派的RX：PA9（STM32的TX）
//      树莓派的TX：PA10（STM32的RX）
//		控制深度螺旋桨：PA6 PA7
//		控制水平螺旋桨：PB6 PB7 PB8 PB9（具体连接情况，在连接硬件的时候配合软件调）
//
//***************************************************//
//STM32F103实现功能 1.深度控制器（PID需要自调）
//                  2.xy速度控制器
//                  2.旋转速度控制器
//
//串口数据输入格式（字符串）如下：
//P_depth	I_depth		D_depth		a_depth 	v_x 	v_y		v_p 	pwm_max
//
//其中：当v_x的值为0~100，船体向后行驶，
//		         100~200，船体向前行驶。
//其中：当v_y的值为0~100，船体向右行驶，
//		         100~200，船体向左行驶。
//其中：当v_p的值为0~100，船体向逆时针旋转，
//		         100~200，船体向顺时针旋转。
//其中：pwm_max = 100时，螺旋桨驱动力可以达到最强，即100%。
//		pwm_max = x时，螺旋桨驱动力可以达到x%。
//
//***************************************************//
int init_identity = 0;
int init_pwm = 0;

#include "include.h"

int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);//禁用JTAG 启用 SWD
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
