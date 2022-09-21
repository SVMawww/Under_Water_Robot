#include "pwm.h"

#define PWM1_MS 20

#define PWM11_PHASE_INIT 	10
#define PWM11_PHASE_TOP 	15
#define PWM11_PHASE_BUTTOM 	5

#define PWM12_PHASE_INIT 	10
#define PWM12_PHASE_TOP 	15
#define PWM12_PHASE_BUTTOM 	5

#define PWM13_PHASE_INIT 	10
#define PWM13_PHASE_TOP 	15
#define PWM13_PHASE_BUTTOM 	5

#define PWM14_PHASE_INIT 	10
#define PWM14_PHASE_TOP 	15
#define PWM14_PHASE_BUTTOM 	5


static u32 pwm11_compare = 0;
static u32 pwm12_compare = 0;
static u32 pwm13_compare = 0;
static u32 pwm14_compare = 0;

void pwm1_init()
{
	tim1_init(PWM1_MS);
	pwm11_compare = (u32)(PWM11_PHASE_INIT / 100.0 * 7000);
	pwm12_compare = (u32)(PWM12_PHASE_INIT / 100.0 * 7000);
	pwm13_compare = (u32)(PWM13_PHASE_INIT / 100.0 * 7000);
	pwm14_compare = (u32)(PWM14_PHASE_INIT / 100.0 * 7000);
	TIM_SetCompare1(TIM1, pwm11_compare);
	TIM_SetCompare2(TIM1, pwm12_compare);
	TIM_SetCompare3(TIM1, pwm13_compare);
	TIM_SetCompare4(TIM1, pwm14_compare);
}
void pwm11_set(float value)
{
	pwm11_compare = (u32)(value / 100.0 * 7000);
	TIM_SetCompare1(TIM1, pwm11_compare);
}
void pwm11_add(float value)
{
	pwm11_compare += 0.7 * (PWM11_PHASE_TOP - PWM11_PHASE_BUTTOM) * value;
	if(pwm11_compare < 70 * PWM11_PHASE_BUTTOM) 
		pwm11_compare = 70 * PWM11_PHASE_BUTTOM;
	else if(pwm11_compare > 70 * PWM11_PHASE_TOP) 
		pwm11_compare = 70 * PWM11_PHASE_TOP;
	TIM_SetCompare1(TIM1, pwm11_compare);
}
void pwm12_add(float value)
{
	pwm12_compare += 0.7 * (PWM12_PHASE_TOP - PWM12_PHASE_BUTTOM) * value;
	if(pwm12_compare < 70 * PWM12_PHASE_BUTTOM) 
		pwm12_compare = 70 * PWM12_PHASE_BUTTOM;
	else if(pwm12_compare > 70 * PWM12_PHASE_TOP) 
		pwm12_compare = 70 * PWM12_PHASE_TOP;
	TIM_SetCompare2(TIM1, pwm12_compare);
}
void pwm13_add(float value)
{
	pwm13_compare += 0.7 * (PWM13_PHASE_TOP - PWM13_PHASE_BUTTOM) * value;
	if(pwm13_compare < 70 * PWM13_PHASE_BUTTOM) 
		pwm13_compare = 70 * PWM13_PHASE_BUTTOM;
	else if(pwm13_compare > 70 * PWM13_PHASE_TOP) 
		pwm13_compare = 70 * PWM13_PHASE_TOP;
	TIM_SetCompare3(TIM1, pwm13_compare);
}
void pwm14_add(float value)
{
	pwm14_compare += 0.7 * (PWM14_PHASE_TOP - PWM14_PHASE_BUTTOM) * value;
	if(pwm14_compare < 70 * PWM14_PHASE_BUTTOM) 
		pwm14_compare = 70 * PWM14_PHASE_BUTTOM;
	else if(pwm14_compare > 70 * PWM14_PHASE_TOP) 
		pwm14_compare = 70 * PWM14_PHASE_TOP;
	TIM_SetCompare4(TIM1, pwm14_compare);
}



static float straight_value = 0;
static float side_value = 0;
static float spin_value = 0;

void move_init()
{
	tim1_init(1);
	tim2_init(1);
}

void straight_set(float value)
{
	straight_value = value;
	if(straight_value >= 0)
	{
		TIM_SetCompare1(TIM1, straight_value);
		TIM_SetCompare2(TIM1, 0);
		TIM_SetCompare3(TIM1, straight_value);
		TIM_SetCompare4(TIM1, 0);
		TIM_SetCompare1(TIM2, straight_value);
		TIM_SetCompare2(TIM2, 0);
		TIM_SetCompare3(TIM2, straight_value);
		TIM_SetCompare4(TIM2, 0);
	}
	else
	{
		TIM_SetCompare1(TIM1, 0);
		TIM_SetCompare2(TIM1, straight_value);
		TIM_SetCompare3(TIM1, 0);
		TIM_SetCompare4(TIM1, straight_value);
		TIM_SetCompare1(TIM2, 0);
		TIM_SetCompare2(TIM2, straight_value);
		TIM_SetCompare3(TIM2, 0);
		TIM_SetCompare4(TIM2, straight_value);	
	}	
}

void side_set(float value)
{
	side_value = value;
	if(side_value >= 0)
	{
		TIM_SetCompare1(TIM1, side_value);
		TIM_SetCompare2(TIM1, 0);
		TIM_SetCompare3(TIM1, 0);
		TIM_SetCompare4(TIM1, side_value);
		TIM_SetCompare1(TIM2, side_value);
		TIM_SetCompare2(TIM2, 0);
		TIM_SetCompare3(TIM2, 0);
		TIM_SetCompare4(TIM2, side_value);
	}
	else
	{
		TIM_SetCompare1(TIM1, 0);
		TIM_SetCompare2(TIM1, side_value);
		TIM_SetCompare3(TIM1, side_value);
		TIM_SetCompare4(TIM1, 0);
		TIM_SetCompare1(TIM2, 0);
		TIM_SetCompare2(TIM2, side_value);
		TIM_SetCompare3(TIM2, side_value);
		TIM_SetCompare4(TIM2, 0);	
	}
}

void spin_set(float value)
{
	spin_value = value;
	if(spin_value >= 0)
	{
		TIM_SetCompare1(TIM1, spin_value);
		TIM_SetCompare2(TIM1, 0);
		TIM_SetCompare3(TIM1, 0);
		TIM_SetCompare4(TIM1, spin_value);
		TIM_SetCompare1(TIM2, 0);
		TIM_SetCompare2(TIM2, spin_value);
		TIM_SetCompare3(TIM2, spin_value);
		TIM_SetCompare4(TIM2, 0);
	}
	else
	{
		TIM_SetCompare1(TIM1, 0);
		TIM_SetCompare2(TIM1, spin_value);
		TIM_SetCompare3(TIM1, spin_value);
		TIM_SetCompare4(TIM1, 0);
		TIM_SetCompare1(TIM2, spin_value);
		TIM_SetCompare2(TIM2, 0);
		TIM_SetCompare3(TIM2, 0);
		TIM_SetCompare4(TIM2, spin_value);	
	}
}

static float pwm_pre1 = 0;
static float pwm_pre2 = 0;
static float pwm_pre3 = 0;
static float pwm_pre4 = 0;

static float pwm_spin1 = 0;
static float pwm_spin2 = 0;
static float pwm_spin3 = 0;
static float pwm_spin4 = 0;

static float pwm_out1 = 0;
static float pwm_out2 = 0;
static float pwm_out3 = 0;
static float pwm_out4 = 0;

static void move_set()
{
	if(pwm_out1 >= 0)
		TIM_SetCompare1(TIM1, pwm_out1), TIM_SetCompare2(TIM1, 0);
	else
		TIM_SetCompare1(TIM1, 0),	  TIM_SetCompare2(TIM1, pwm_out1);
	if(pwm_out2 >= 0)
		TIM_SetCompare3(TIM1, pwm_out2), TIM_SetCompare4(TIM1, 0);
	else
		TIM_SetCompare3(TIM1, 0),     TIM_SetCompare4(TIM1, pwm_out2);
	if(pwm_out3 >= 0)
		TIM_SetCompare1(TIM2, pwm_out3), TIM_SetCompare2(TIM2, 0);
	else
		TIM_SetCompare1(TIM2, 0),     TIM_SetCompare2(TIM2, pwm_out3);		
	if(pwm_out4 >= 0)
		TIM_SetCompare3(TIM2, pwm_out4), TIM_SetCompare4(TIM2, 0);
	else
		TIM_SetCompare3(TIM2, 0),     TIM_SetCompare4(TIM2, pwm_out4);		
}

void straight_add(float f1, float f2, float f3,float f4)
{
	pwm_pre1 += f1; 
	pwm_pre2 -= f2; 
	pwm_pre3 -= f3; 
	pwm_pre4 += f4; 

	//move_set();
}

void side_add(float value)
{
	pwm_pre1 += value; 
	pwm_pre2 -= value; 
	pwm_pre3 += value; 
	pwm_pre4 -= value; 

	move_set();
}

void spin_add(float value, float limit)
{
	if(value > limit) value = limit;
	if(value < -limit) value = -limit;
	
	pwm_spin1 = -value;
	pwm_spin2 = -value;	
	pwm_spin3 = -value;
	pwm_spin4 = -value;	
	
	pwm_out1 = pwm_spin1 + pwm_pre1;
	if(pwm_out1 > 7000) pwm_out1 = 7000;
	else if(pwm_out1 < -7000) pwm_out1 = -7000;
	
	pwm_out2 = pwm_spin2 + pwm_pre2;
	if(pwm_out2 > 7000) pwm_out2 = 7000;
	else if(pwm_out2 < -7000) pwm_out2 = -7000;
	
	pwm_out3 = pwm_spin3 + pwm_pre3;
	if(pwm_out3 > 7000) pwm_out3 = 7000;
	else if(pwm_out3 < -7000) pwm_out3 = -7000;
	
	pwm_out4 = pwm_spin4 + pwm_pre4;
	if(pwm_out4 > 7000) pwm_out4 = 7000;
	else if(pwm_out4 < -7000) pwm_out4 = -7000;	
	
	move_set();
}

void stop()
{
	
	pwm_pre1 = 0;
	pwm_pre2 = 0;
	pwm_pre3 = 0;
	pwm_pre4 = 0;
	
	pwm_spin1 = 0;
	pwm_spin2 = 0;
	pwm_spin3 = 0;
	pwm_spin4 = 0;
	
	pwm_out1 = 0;
	pwm_out2 = 0;
	pwm_out3 = 0;
	pwm_out4 = 0;
}