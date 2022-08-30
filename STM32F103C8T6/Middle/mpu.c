#include "mpu.h"

#include "ioi2c.h"
#include "mpu6050.h"
#include "delay.h"

float pitch = 0, roll = 0, yaw = 0;

void mpu__init()
{
	//delay_ms(200);
	IIC_Init();                     //=====IIC初始化    读取MPU6050数据
	//delay_ms(200);					//=====加延时避免初始化不成功
	MPU6050_initialize();           //=====MPU6050初始化
	delay_ms(200);	
	DMP_Init();                     //=====初始化DMP  
	delay_ms(200);
}

void mpu_getdata()
{	
	static float pitch_last1 = 0;
	static float pitch_last2 = 0;
	static float roll_last1 = 0;
	static float roll_last2 = 0;
	static float yaw_last1 = 0;
	static float yaw_last2 = 0;
	
	Read_DMP();
	pitch = Pitch;
	roll = Roll;
	yaw = Yaw;
	
//	if(pitch_last2 + Pitch - 2 * pitch_last1 > 5 || pitch_last2 + Pitch - 2 * pitch_last1 < -5)
//	{
//		pitch = pitch_last1 = pitch_last2;
//	}
//	else
//	{
//		pitch = pitch_last1;
//		pitch_last2 = pitch_last1;
//		pitch_last1 = Pitch;
//	}
//	
//	if(roll_last2 + Roll - 2 * roll_last1 > 5 || roll_last2 + Roll - 2 * roll_last1 < -5)
//	{
//		roll = roll_last1 = roll_last2;
//	}
//	else
//	{
//		roll = roll_last1;
//		roll_last2 = roll_last1;
//		roll_last1 = Roll;
//	}
//	
//	if(yaw_last2 + Yaw - 2 * yaw_last1 > 5 || yaw_last2 + Yaw - 2 * yaw_last1 < -5)
//	{
//		yaw = yaw_last1 = yaw_last2;
//	}
//	else
//	{
//		yaw = yaw_last1;
//		yaw_last2 = yaw_last1;
//		yaw_last1 = Yaw;
//	}	

}
