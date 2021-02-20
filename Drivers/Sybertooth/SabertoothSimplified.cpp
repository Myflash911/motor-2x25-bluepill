/*
 * SabertoothSimplified.cpp
 *
 *  Created on: 22 авг. 2020 г.
 *      Author: Boris Pavlenko <borpavlenko@ispovedn1k.com>
 *
 * Управление драйвером двигателей Sabertooth в режиме Simplified Serial Mode.
 */

#include "SabertoothSimplified.h"
#include "minimath.h"
#include <stdlib.h>


SabertoothSimplified::SabertoothSimplified(port_write_function port_write)
{
	_port_write = port_write;
	motor1_inversion = eInversionOff;
	motor2_inversion = eInversionOff;
	threshold = 0;
}

SabertoothSimplified::~SabertoothSimplified() {}


void SabertoothSimplified::stop()
{
  _port_write(0);
}



void SabertoothSimplified::drive(MotorID_e motor, int8_t power)
{
	uint8_t magnitude, cmd = 0;
	magnitude = abs(power);

	if (threshold > 0)
	{
		if (magnitude < threshold)
		{
			magnitude = 0;
		}
		else if (magnitude < SLOWLEST)
		{
			magnitude = SLOWLEST;
		}
	}

#ifdef FASTEST
	if (magnitude != 0 && magnitude > FASTEST)
	{
		magnitude = FASTEST;
	}
#endif

	magnitude = magnitude >> 1;

	if (motor == eLeftMotor)
	{
		cmd = power < 0 ? SBT_M1_STOP - motor1_inversion*magnitude : SBT_M1_STOP + motor1_inversion*magnitude;
	}
	else
	{
		cmd = power < 0 ? SBT_M2_STOP - motor2_inversion*magnitude : SBT_M2_STOP + motor2_inversion*magnitude;
	}

	_port_write(cmd);
}
