#ifndef _TRANS_H_
#define _TRANS_H_

#include "stm32f10x.h"

#include "uart.h"

void trans_bt_init(int i, u32 baud);

void trans_bt_T(int i, 
				char c1, char c2, char c3,
				float f1, 	float f2, 	float f3, 	float f4, 	float f5,
				float f6, 	float f7, 	float f8, 	float f9, 	float f10,
				float f11, 	float f12, 	float f13, 	float f14, 	float f15,
				float f16, 	float f17, 	float f18, 	float f19, 	float f20,
				float f21, 	float f22, 	float f23, 	float f24, 	float f25,
				float f26, 	float f27, 	float f28, 	float f29, 	float f30 );
int trans_bt_R( int i, 
				char* c1, char* c2, char* c3,
				float* f1, 	float* f2, 	float* f3, 	float* f4, 	float* f5,
				float* f6, 	float* f7, 	float* f8, 	float* f9, 	float* f10,
				float* f11, float* f12, float* f13, float* f14, float* f15,
				float* f16, float* f17, float* f18, float* f19, float* f20,
				float* f21, float* f22, float* f23, float* f24, float* f25,
				float* f26, float* f27, float* f28, float* f29, float* f30 );

				
void trans_others_init(int i, u32 baud);

int trans_others_R(int i,
				   int* i1, int* i2, int* i3, int* i4, int* i5,
				   int* i6, int* i7, int* i8, int* i9, int* i10);
	
void trans_others_T(int i,
				    int i1, int i2, int i3, int i4, int i5,
				    int i6, int i7, int i8, int i9, int i10);
					
					
void trans_mpu_init(int i, u32 baud);

int trans_mpu_R( int i, 
				int* pitch, int* roll, 	int* yaw );
#endif
