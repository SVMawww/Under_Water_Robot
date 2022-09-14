#ifndef _UART_H_
#define _UART_H_

#include "stm32f10x.h"

void uart_init(int i, u32 baud, void* tx_ptr, u32 tx_len, void* rx_ptr, u32 rx_len, int* rx_flag);

void uart_singletrans(int i);

#endif
