/*
 * MotorUnit.cpp
 *
 *  Created on: Jan 2, 2021
 *      Author: borpa
 */

#include "MotorUnit.h"
#include "minimath.h"
#include <stdio.h>

#define AWAIT_FOR_DIRECTION			500	// ms
#define CALIBRATION_TIMEOUT			6000	// ms
#define CALIBRATION_SPEED			50
#define LINEAR_SPEED				40
#define STARTING_POS				500

#define isBlockedBWD			(flags & HIT_BWD)
#define isBlockedFWD			(flags & HIT_FWD)
#define isStoped				(flags & STOPED)



MotorUnit::MotorUnit(MotorID_e _id,
		XPIDController *_pid,
		SabertoothSimplified *ST,
		uint32_t (*getTick_function)(void))
{
	id = _id;
	pid = _pid;
	_getTick = getTick_function;
	lastError = NoError;
	limit_gap = 500;
	_ST = ST;
}

MotorUnit::~MotorUnit() {}


int8_t MotorUnit::GetSpeed(void)
{
	if (speedSource == Suspended)
	{
		output_speed = 0;
	}
	else if (speedSource == Calibration)
	{
		Calibrate();
		if (calibStage == Calibrated)
		{
			pid->integrated_error = 0;
			speedSource = PID;
		}
	}
	else if (speedSource == PID)
	{
		output_speed = pid->output;
	}
	else if (speedSource == LinearDriver)
	{
		if (target > *pPosition + pid->hitbox)
		{
			output_speed = LINEAR_SPEED;
		}
		else if (target < *pPosition - pid->hitbox)
		{
			output_speed = -LINEAR_SPEED;
		}
		else
		{
			output_speed = 0;
		}
	}
	else if (speedSource == Command)
	{
		output_speed = requested_speed;
	}

	if ((isBlockedBWD && output_speed < 0) ||  (isBlockedFWD && output_speed > 0) ||
			(minPosition > *pPosition && output_speed < 0) || (maxPosition < *pPosition && output_speed > 0))
	{
		output_speed = 0;
		//speedSource = Suspended;
		pid->integrated_error = 0;
		return 0;
	}

	return output_speed;
}


void MotorUnit::SetTarget(uint32_t _target)
{
	target = map(_target, minTarget, maxTarget, minPosition, maxPosition); // can be less then minPosition
	target = constrain(target, minPosition, maxPosition);  // for safe
}


void MotorUnit::SetTargetRange(uint32_t min, uint32_t max)
{
	minTarget = min;
	maxTarget = max;
}


void MotorUnit::RotateWithSpeed(int8_t speed)
{
	requested_speed = speed;
	speedSource = Command;
}


void MotorUnit::HappendEvent(uint8_t eventMSK, EventStatus_e st)
{
	if (st == OnEvent)
	{
		flags |= eventMSK;
	}
	else
	{
		flags &= ~eventMSK;
	}

	if (calibStage == AwaitLimitBWD || calibStage == AwaitForBWDRotaryDirection)
	{
		if ((eventMSK == HIT_BWD) && (st == OnEvent))
		{
			calibStage = HitLimitBWD;
		}
	}
	if (calibStage == AwaitLimitFWD || calibStage == AwaitForFWDRotaryDirection)
	{
		if ((eventMSK == HIT_FWD) && (st == OnEvent))
		{
			calibStage = HitLimitFWD;
		}
	}
}


void MotorUnit::Calibrate(void)
{
	if (calibStage == RequestedCalibration)
	{
		if (isBlockedBWD)
		{
			calibStage = HitLimitBWD;
			return;
		}
		output_speed = -CALIBRATION_SPEED;
		*pPosition = 48000; // preset
		oldPosition = *pPosition;
		maxPosition = 65000; // preset
		calibStage = AwaitForBWDRotaryDirection;
		start_tick = _getTick();
		return;
	}
	if (calibStage == AwaitForBWDRotaryDirection)
	{
		if ((_getTick() - start_tick) >= AWAIT_FOR_DIRECTION)
		{
			calibStage = VerifyEncoderBWDRotation;
		}
		return;
	}
	if (calibStage == VerifyEncoderBWDRotation)
	{
		if (oldPosition > *pPosition)
		{
			calibStage = AwaitLimitBWD;
		}
		else
		{
			calibStage = ErrorAborted;
			lastError = ErrorEncoderDirection;
			speedSource = Suspended;
			output_speed = 0;
		}
		return;
	}
	if (calibStage == AwaitLimitBWD)
	{
		if ((_getTick() - start_tick) >= CALIBRATION_TIMEOUT)
		{
			calibStage = ErrorAborted;
			lastError  = ErrorTimeout;
			speedSource = Suspended;
			output_speed = 0;
		}
		if (isBlockedBWD)
		{
			calibStage = HitLimitBWD;
		}
		return;
	}
	if (calibStage == HitLimitBWD)
	{
		start_tick = _getTick();
		output_speed = 0;
		calibStage = AwaitForStop;
		return;
	}
	if (calibStage == AwaitForStop)
	{
		if ((_getTick() - start_tick) >= AWAIT_FOR_DIRECTION)
		{
			*pPosition = STARTING_POS;
			oldPosition = STARTING_POS;
			minPosition = STARTING_POS + limit_gap;
			calibStage = AwaitForFWDRotaryDirection;
			output_speed = CALIBRATION_SPEED;
			start_tick = _getTick();
		}
		return;
	}
	if (calibStage == AwaitForFWDRotaryDirection)
	{
		if ((_getTick() - start_tick) >= AWAIT_FOR_DIRECTION)
		{
			calibStage = VerifyEncoderFWDRotation;
		}
		return;
	}
	if (calibStage == VerifyEncoderFWDRotation)
	{
		if (oldPosition < *pPosition)
		{
			calibStage = AwaitLimitFWD;
		}
		else
		{
			calibStage = ErrorAborted;
			lastError = ErrorEncoderDirection;
			speedSource = Suspended;
			output_speed = 0;
		}
		return;
	}
	if (calibStage == AwaitLimitFWD)
	{
		if ((_getTick() - start_tick) >= CALIBRATION_TIMEOUT)
		{
			calibStage = ErrorAborted;
			lastError = ErrorTimeout;
			speedSource = Suspended;
			output_speed = 0;
		}
		return;
	}
	if (calibStage == HitLimitFWD)
	{
		maxPosition = *pPosition - limit_gap;
		output_speed = -CALIBRATION_SPEED;
		target = (maxPosition+minPosition)/2;
		calibStage = AwaitCenter;
		return;
	}
	if (calibStage == AwaitCenter)
	{
		if (*pPosition <= target)
		{
			output_speed = 0;
			calibStage = Calibrated;
			lastError = NoError;
		}
	}
}


void MotorUnit::Drive(void)
{
	if (isStoped)
		_ST->drive(id, 0);
	else
		_ST->drive(id, output_speed);
}

void MotorUnit::CalcPID(uint16_t dt)
{
	if (isStoped)
	{
		pid->integrated_error = 0;
		return;
	}
	pid->CalcPID(*pPosition, target, dt);
}


/**
 * @descr: Disables any rotation until 'Resume'
 * Allows to stop and resume actions by basic commands.
 * This command does not change source of signal.
 */
void MotorUnit::Stop(void)
{
	flags |= STOPED;
}


void MotorUnit::Resume(void)
{
	flags &= ~STOPED;
}


void MotorUnit::RequestCalibration(void)
{
	Resume();
	speedSource = Calibration;
	calibStage = RequestedCalibration;
}


/**
 * Stops actions by changing source of signal to Zero.
 */
void MotorUnit::Suspend(void)
{
	speedSource = Suspended;
}


uint8_t MotorUnit::UsePID(void)
{
	if (calibStage == Calibrated)
	{
		pid->integrated_error = 0;
		speedSource = PID;
		return 1;
	}
	else
	{
		return 0;
	}
}


uint8_t MotorUnit::UseLinear(void)
{
	if (calibStage == Calibrated)
	{
		speedSource = LinearDriver;
		return 1;
	}
	else
	{
		return 0;
	}
}


int8_t MotorUnit::Rotate(int8_t speed)
{
	if (speedSource == Calibration)
	{
		return 0;
	}
	speedSource = Command;
	requested_speed = speed;
	return speed;
}



int MotorUnit::GetStatus(char* p)
{
	return sprintf(p,
"ID: %d\r\n\
flags: %d\r\n\
last err: %d\r\n\
calibStage: %d\r\n\
speedSource: %d\r\n\
out_sp: %d\r\n\
req_sp: %d\r\n\
min/max Pos: %lu/%lu\r\n\
target: %lu\r\n\
position: %lu\r\n\
differ: %ld\r\n\
", id, flags, lastError, calibStage,
		speedSource, output_speed, requested_speed,
		minPosition, maxPosition, target, *pPosition,
		(int32_t)(target-*pPosition));
}
