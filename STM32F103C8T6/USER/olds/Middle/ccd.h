#ifndef __ADC_H
#define __ADC_H	

#include "stm32f10x.h"

extern u32 ccd_middle, black_cnt; 
extern u16 CCD_Yuzhi;

void ccd_init(void);
void ccd_getdata(void);

#endif 












