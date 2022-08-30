#include "stm32f10x.h"

//******************* Stdlib ************************//
#include "math.h"
#include <stdio.h>

//******************* SYSTEM ************************//
#include "delay.h"
#include "sys.h"
#include "pid.h"

//******************* Middle ************************//
#include "pwm.h"
#include "trans.h"

//******************* Button ************************//
#include "gpio.h"
#include "oc.h"

//***************************************************//


/*
			MPU6050*4	oledspi
			
			|			|
			|			|			uart		oc			ic			gpio
			|			|
			|			|			|			|			|			|			
			|			|			|			|			|			|
															|			|
ccd			mpu			oled		trans		pwm			|			|
															|			|
|			|			|			|			|			|			|
|			|			|			|			|			|			|

*/
