#ifndef _MAINCYCLE_H_
#define _MAINCYCLE_H_

#include "stm32f10x.h"

void maincycle_ms_init(uint16_t ms);    //ms取值范围20~60
#define  Maincycle_Handler                  TIM6_IRQHandler

void CPUoccupationRate_Calculatestart(void);
float CPUoccupationRate_Calculatefinish(void);

#endif
