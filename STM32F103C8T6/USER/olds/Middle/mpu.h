#ifndef _MPU_H_
#define _MPU_H_

#include "stm32f10x.h"

extern float pitch, roll, yaw;

void mpu__init(void);

void mpu_getdata(void);
	
#endif

