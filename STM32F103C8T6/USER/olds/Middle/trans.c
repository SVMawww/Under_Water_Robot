#include "trans.h"

#include "math.h"

static struct
{
	char first; char c1; char c2; char c3;
	float f1; 	float f2; 	float f3; 	float f4; 	float f5;
	float f6; 	float f7; 	float f8; 	float f9; 	float f10;
	float f11; 	float f12; 	float f13; 	float f14; 	float f15;
	float f16; 	float f17; 	float f18; 	float f19; 	float f20;
	float f21; 	float f22; 	float f23; 	float f24; 	float f25;
	float f26; 	float f27; 	float f28; 	float f29; 	float f30;
	char check; char last;
}
trans_bt_Tbuff[4];

static struct
{
	char first; char c1; char c2; char c3;
	float f1; 	float f2; 	float f3; 	float f4; 	float f5;
	float f6; 	float f7; 	float f8; 	float f9; 	float f10;
	float f11; 	float f12; 	float f13; 	float f14; 	float f15;
	float f16; 	float f17; 	float f18; 	float f19; 	float f20;
	float f21; 	float f22; 	float f23; 	float f24; 	float f25;
	float f26; 	float f27; 	float f28; 	float f29; 	float f30;
	char check; char last;
}
trans_bt_Rbuff[4];

static int trans_bt_Rflag[4] = {0,0,0,0};

void trans_bt_init(int i, u32 baud)
{
	uart_init(i, baud, &trans_bt_Tbuff[i - 1], sizeof(trans_bt_Tbuff[i - 1]) - 2, 
					   &trans_bt_Rbuff[i - 1], sizeof(trans_bt_Rbuff[i - 1]) - 2, 
					   &trans_bt_Rflag[i - 1]);
	trans_bt_Tbuff[i - 1].first = 0xA5;
	trans_bt_Tbuff[i - 1].last = 0x5A;
}

void trans_bt_T(int i, 
				char c1, char c2, char c3,
				float f1, 	float f2, 	float f3, 	float f4, 	float f5,
				float f6, 	float f7, 	float f8, 	float f9, 	float f10,
				float f11, 	float f12, 	float f13, 	float f14, 	float f15,
				float f16, 	float f17, 	float f18, 	float f19, 	float f20,
				float f21, 	float f22, 	float f23, 	float f24, 	float f25,
				float f26, 	float f27, 	float f28, 	float f29, 	float f30 )
{
	char* charptr = (char* )&trans_bt_Tbuff[i - 1] + 1;
	uint8_t checkbyte = 0;
	uint16_t j =  sizeof(trans_bt_Tbuff[i - 1]) - 5;
	
	trans_bt_Tbuff[i - 1].c1 = c1; 
	trans_bt_Tbuff[i - 1].c2 = c2; 
	trans_bt_Tbuff[i - 1].c3 = c3;
	trans_bt_Tbuff[i - 1].f1 = f1;
	trans_bt_Tbuff[i - 1].f2 = f2;
	trans_bt_Tbuff[i - 1].f3 = f3;
	trans_bt_Tbuff[i - 1].f4 = f4;
	trans_bt_Tbuff[i - 1].f5 = f5;
	trans_bt_Tbuff[i - 1].f6 = f6;
	trans_bt_Tbuff[i - 1].f7 = f7;
	trans_bt_Tbuff[i - 1].f8 = f8;
	trans_bt_Tbuff[i - 1].f9 = f9;
	trans_bt_Tbuff[i - 1].f10 = f10;
	trans_bt_Tbuff[i - 1].f11 = f11;
	trans_bt_Tbuff[i - 1].f12 = f12;
	trans_bt_Tbuff[i - 1].f13 = f13;
	trans_bt_Tbuff[i - 1].f14 = f14;
	trans_bt_Tbuff[i - 1].f15 = f15;
	trans_bt_Tbuff[i - 1].f16 = f16;
	trans_bt_Tbuff[i - 1].f17 = f17;
	trans_bt_Tbuff[i - 1].f18 = f18;
	trans_bt_Tbuff[i - 1].f19 = f19;
	trans_bt_Tbuff[i - 1].f20 = f20;
	trans_bt_Tbuff[i - 1].f21 = f21;
	trans_bt_Tbuff[i - 1].f22 = f22;
	trans_bt_Tbuff[i - 1].f23 = f23;
	trans_bt_Tbuff[i - 1].f24 = f24;
	trans_bt_Tbuff[i - 1].f25 = f25;
	trans_bt_Tbuff[i - 1].f26 = f26;
	trans_bt_Tbuff[i - 1].f27 = f27;
	trans_bt_Tbuff[i - 1].f28 = f28;
	trans_bt_Tbuff[i - 1].f29 = f29;
	trans_bt_Tbuff[i - 1].f30 = f30;

	while( j-- )
	{
	    checkbyte += *charptr++;
	}
	*charptr = checkbyte;
	uart_singletrans(i);
}

int trans_bt_R( int i, 
				char* c1, char* c2, char* c3,
				float* f1, 	float* f2, 	float* f3, 	float* f4, 	float* f5,
				float* f6, 	float* f7, 	float* f8, 	float* f9, 	float* f10,
				float* f11, float* f12, float* f13, float* f14, float* f15,
				float* f16, float* f17, float* f18, float* f19, float* f20,
				float* f21, float* f22, float* f23, float* f24, float* f25,
				float* f26, float* f27, float* f28, float* f29, float* f30 )
{
	if(trans_bt_Rflag[i - 1] == sizeof(trans_bt_Rbuff[i - 1]) - 2)
	{
		if(c1 != 0) *c1 = trans_bt_Rbuff[i - 1].c1;
		if(c2 != 0) *c2 = trans_bt_Rbuff[i - 1].c2;
		if(c3 != 0) *c2 = trans_bt_Rbuff[i - 1].c3;
		if(f1 != 0) *f1 = trans_bt_Rbuff[i - 1].f1;
		if(f2 != 0) *f2 = trans_bt_Rbuff[i - 1].f2;
		if(f3 != 0) *f3 = trans_bt_Rbuff[i - 1].f3;
		if(f4 != 0) *f4 = trans_bt_Rbuff[i - 1].f4;
		if(f5 != 0) *f5 = trans_bt_Rbuff[i - 1].f5;
		if(f6 != 0) *f6 = trans_bt_Rbuff[i - 1].f6;
		if(f7 != 0) *f7 = trans_bt_Rbuff[i - 1].f7;
		if(f8 != 0) *f8 = trans_bt_Rbuff[i - 1].f8;
		if(f9 != 0) *f9 = trans_bt_Rbuff[i - 1].f9;
		if(f10 != 0)*f10 = trans_bt_Rbuff[i - 1].f10;
		if(f11 != 0)*f11 = trans_bt_Rbuff[i - 1].f11;
		if(f12 != 0)*f12 = trans_bt_Rbuff[i - 1].f12;
		if(f13 != 0)*f13 = trans_bt_Rbuff[i - 1].f13;
		if(f14 != 0)*f14 = trans_bt_Rbuff[i - 1].f14;
		if(f15 != 0)*f15 = trans_bt_Rbuff[i - 1].f15;
		if(f16 != 0)*f16 = trans_bt_Rbuff[i - 1].f16;
		if(f17 != 0)*f17 = trans_bt_Rbuff[i - 1].f17;
		if(f18 != 0)*f18 = trans_bt_Rbuff[i - 1].f18;
		if(f19 != 0)*f19 = trans_bt_Rbuff[i - 1].f19;
		if(f20 != 0)*f20 = trans_bt_Rbuff[i - 1].f20;
		if(f21 != 0)*f21 = trans_bt_Rbuff[i - 1].f21;
		if(f22 != 0)*f22 = trans_bt_Rbuff[i - 1].f22;
		if(f23 != 0)*f23 = trans_bt_Rbuff[i - 1].f23;
		if(f24 != 0)*f24 = trans_bt_Rbuff[i - 1].f24;
		if(f25 != 0)*f25 = trans_bt_Rbuff[i - 1].f25;
		if(f26 != 0)*f26 = trans_bt_Rbuff[i - 1].f26;
		if(f27 != 0)*f27 = trans_bt_Rbuff[i - 1].f27;
		if(f28 != 0)*f28 = trans_bt_Rbuff[i - 1].f28;
		if(f29 != 0)*f29 = trans_bt_Rbuff[i - 1].f29;
		if(f30 != 0)*f30 = trans_bt_Rbuff[i - 1].f30;
			
		trans_bt_Rflag[i - 1] = 0;
		return 1;
	}
	return 0;
}



//***************************************************//
#define TRANS_OTHERS_TBUFF_SIZE 52

char trans_others_Tbuff[4][TRANS_OTHERS_TBUFF_SIZE] = {0};
int  trans_others_TFlag[4] = {0};

#define TRANS_OTHERS_RBUFF_SIZE 50

char trans_others_Rbuff[4][TRANS_OTHERS_RBUFF_SIZE] = {0};
int  trans_others_RFlag[4] = {0};


void trans_others_init(int i, u32 baud)
{
	uart_init(i, baud, &trans_others_Tbuff[i - 1], TRANS_OTHERS_TBUFF_SIZE, 
	                   &trans_others_Rbuff[i - 1], TRANS_OTHERS_RBUFF_SIZE, 
	                   &trans_others_RFlag[i - 1]);
}

int trans_others_R(int i,
				   int* i1, int* i2, int* i3, int* i4, int* i5,
				   int* i6, int* i7, int* i8, int* i9, int* i10)
{
	int i_10[10] = {0};
	u8 i_10_index = 0;
	
	int j = 0, num_flag = 0, signal_flag = 1;
	
	if(trans_others_RFlag[i - 1])
	{
		for(j = 0; j < trans_others_RFlag[i - 1]; j++)
			if(trans_others_Rbuff[i - 1][j] <= '9' && trans_others_Rbuff[i - 1][j] >= '0')
			{
				i_10[i_10_index] = 10 * i_10[i_10_index] + (trans_others_Rbuff[i - 1][j] - '0') * signal_flag;
				num_flag = 1;
			}
			else if(trans_others_Rbuff[i - 1][j] == '-')
			{
				signal_flag = -1;
			}
			else
			{
				if(num_flag == 1)
					i_10_index++, num_flag = 0;
				if(i_10_index == 10) break;
			}
			
		if(i1 != 0) *i1 = i_10[0];
		if(i2 != 0) *i2 = i_10[1];
		if(i3 != 0) *i3 = i_10[2];
		if(i4 != 0) *i4 = i_10[3];
		if(i5 != 0) *i5 = i_10[4];
		if(i6 != 0) *i6 = i_10[5];
		if(i7 != 0) *i7 = i_10[6];
		if(i8 != 0) *i8 = i_10[7];
		if(i9 != 0) *i9 = i_10[8];
		if(i10 != 0) *i10 = i_10[9];
			
		trans_others_RFlag[i - 1] = 0;	
		return 1;
	}
	return 0;
}	
					
void trans_others_T(int i,
				   int i1, int i2, int i3, int i4, int i5,
				   int i6, int i7, int i8, int i9, int i10)	
{
	int i_10[10] = {0};
	
	int j = 0;
	
	i_10[0] = i1;
	i_10[1] = i2;
	i_10[2] = i3;
	i_10[3] = i4;
	i_10[4] = i5;
	i_10[5] = i6;
	i_10[6] = i7;
	i_10[7] = i8;
	i_10[8] = i9;
	i_10[9] = i10;
	
	trans_others_Tbuff[i - 1][0] = 0xaa;
	for(j = 0; j < 10; j++)
	{
		trans_others_Tbuff[i - 1][5 * j + 1] = ' ';	
		trans_others_Tbuff[i - 1][5 * j + 2] = ' ';
		trans_others_Tbuff[i - 1][5 * j + 3] = (unsigned char)(i_10[j]) / 100 + '0';
		trans_others_Tbuff[i - 1][5 * j + 4] = (unsigned char)(i_10[j]) / 10 % 10 + '0' ;
		trans_others_Tbuff[i - 1][5 * j + 5] = (unsigned char)(i_10[j]) % 10  + '0';
	}
	trans_others_Tbuff[i - 1][51] = 0xdd;
	uart_singletrans(i);
}			
