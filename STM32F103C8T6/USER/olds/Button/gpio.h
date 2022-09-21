#ifndef _GPIO_H_
#define _GPIO_H_

#include "stm32f10x.h"

void gpi_init( void(* i0_)(void) );
void gpo_init(void);

#endif
