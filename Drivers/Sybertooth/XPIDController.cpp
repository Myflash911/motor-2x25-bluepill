/*
 * XPID.cpp
 *
 *  Created on: 22 авг. 2020 г.
 *      Author: Boris Pavlenko <borpavlenko@ispovedn1k.com>
 */

#include "minimath.h"
#include <stdlib.h>
#include <stdio.h>
#include "XPIDController.h"

XPIDController::XPIDController()
{


}

XPIDController::~XPIDController()
{

}


// target normalized to [minPosition : maxPosition].
int8_t XPIDController::CalcPID(uint32_t position, uint32_t target, uint16_t dt)
{
	int32_t error = target - position;
	float P_term = Kp * (float)error;
	integrated_error += error;
	float I_term = Ki * (float)integrated_error * (float)dt;
	float D_term = Kd *  ((float)(error - last_error)) / (float)dt;
	last_error = error;

	output =  (int8_t) constrain(Ka * (P_term + I_term + D_term), -127, 127);

	return output;
}


void XPIDController::setPID(PID_settings_p src)
{
	Ka = src->Ka;
	Kp = src->Kp;
	Ki = src->Ki;
	Kd = src->Kd;
	hitbox = src->hitbox;
}


void XPIDController::dumpPID(PID_settings_p dst)
{
	if (NULL != dst)
	{
		dst->Ka = Ka;
		dst->Kp = Kp;
		dst->Ki = Ki;
		dst->Kd = Kd;
		dst->hitbox = hitbox;
	}
}


void XPIDController::setDefault(void)
{
	Ka = 0.01;
	Kp = 0.1;
	Kd = 0.01;
	Ki = 0.00003;
	hitbox = 20;
}


int XPIDController::sprint(char* ptrc)
{
	return sprintf(ptrc, "\
|    Kp:  %1.3e |\r\n\
|    Ki:  %1.3e |\r\n\
|    Kd:  %1.3e |\r\n\
|    Ka:  %1.3e |\r\n\
|    hb:  %9d |\r\n",
	Kp, Ki, Kd, Ka, hitbox);
}
