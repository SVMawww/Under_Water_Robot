#ifndef _PWM_H_
#define _PWM_H_

#include "stm32f10x.h"

#include "oc.h"

void pwm1_init(void);

void pwm11_add(float value);
void pwm12_add(float value);
void pwm13_add(float value);
void pwm14_add(float value);


void move_init(void);
void straight_set(float value);
void straight_add(float f1, float f2, float f3,float f4);

void side_set(float value);
void side_add(float value);
void spin_set(float value);
void spin_add(float value, float limit);
void stop(void);

#endif
