#include "pid.h"

void PID_init(PID* pid, float P, float I, float D, char zero)
{
    pid->P = P;
	pid->D = D;
	pid->I = I;
	if(zero == 1)
	{
		pid->LastError = 0;
		pid->PrevError = 0;
		pid->Integral = 0;	
	}
}

void PID_start(PID* pid)
{
    pid->LastError = 0;
	pid->PrevError = 0;
	pid->Integral = 0;
}

float PID_calc1(PID* pid, float NowPoint, float SetPoint)
{
    float iError, output; 
	iError = SetPoint - NowPoint;
//	if( pid->LastError - iError > I_limit && iError > 0)
//		;
//	else if( pid->LastError - iError < -I_limit && iError < 0)
//		;
//	else
		pid->Integral += iError;
	output = ( pid->P * iError ) + ( pid->I * pid->Integral ) + (pid->D * (iError - pid->LastError));
	pid->LastError = iError;
    return output;
}

float PID_calc2(PID* pid, float NowPoint, float SetPoint)
{
    float iError, output;
	iError = SetPoint - NowPoint;
	output =  pid->P * ( iError - pid->LastError) + 
            pid->I * ( iError ) + 
            pid->D * ( iError - 2 * pid->LastError + pid->PrevError );
	pid->PrevError = pid->LastError;
	pid->LastError = iError;
    return output;
}
