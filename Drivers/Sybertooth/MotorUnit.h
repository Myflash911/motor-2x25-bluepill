/*
 * MotorUnit.h
 *
 *  Created on: Jan 2, 2021
 *      Author: borpa
 */

#ifndef SYBERTOOTH_MOTORUNIT_H_
#define SYBERTOOTH_MOTORUNIT_H_

#include <inttypes.h>
#include "XPIDController.h"
#include "SabertoothSimplified.h"


#define HIT_BWD					0x01U
#define HIT_FWD 	  			0x02U
#define CALIBRATED				0x04U
#define STOPED					0x08U
#define FLAG5					0x10U
#define FLAG6					0x20U
#define FLAG7					0x40U
#define FLAG8					0x80U

#define DEFAULT_LIMIT_GAP		500


/*
 * statuses for MotorUnit
 */
typedef enum __SpeedSource_e
{
	Suspended,
	PID,
	LinearDriver,		// linear drive to targeted position
	Command,			// command to rotate with speed
	Calibration,
} SpeedSource_e;

typedef enum __CalibrationStages_e
{
	NotCalibrated,
	RequestedCalibration,
	AwaitForBWDRotaryDirection,
	VerifyEncoderBWDRotation,
	AwaitLimitBWD,
	HitLimitBWD,
	AwaitForStop,
	AwaitForFWDRotaryDirection,
	VerifyEncoderFWDRotation,
	AwaitLimitFWD,
	HitLimitFWD,
	AwaitCenter,

	Calibrated,
	ErrorAborted,
} CalibrationStages_e;


typedef enum __EventStatus_e
{
	OnEvent,
	OutEvent
} EventStatus_e;


typedef enum __ErrorCodes_e
{
	NoError,
	ErrorTimeout,
	ErrorEncoderRotation,
	ErrorEncoderDirection,
} ErrorCodes_e;



class MotorUnit {
private:
	XPIDController *pid;
	uint8_t			flags;
	uint32_t 		target, minTarget, maxTarget;

	CalibrationStages_e calibStage;

	uint32_t 			(*_getTick)(void);

	SabertoothSimplified *_ST;

	MotorID_e		id;
	SpeedSource_e 	speedSource;

	uint32_t start_tick = 0;
	uint32_t oldPosition = 0;

public:
	ErrorCodes_e	lastError;

	uint32_t 		minPosition, maxPosition;

	volatile uint32_t 		*pPosition; // pointer to TIMx->CNT

	uint16_t 		limit_gap;

	int8_t			requested_speed, output_speed;

public:
	MotorUnit(MotorID_e _id,
			XPIDController* _pid,
			SabertoothSimplified *ST,
			uint32_t (*getTick_function)(void));

	virtual ~MotorUnit();


	int GetStatus(char *p);

	void HappendEvent(uint8_t eventMSK, EventStatus_e st);

	void SetTarget(uint32_t _target);
	void SetTargetRange(uint32_t min, uint32_t max);
	void RotateWithSpeed(int8_t speed);
	// void TurnOnLinearMode(void) {speedSource = LinearDriver;}
	// void TurnOnPidMode(void) {speedSource = PID;}


	int8_t GetSpeed(void);

	void Drive(void);

	void CalcPID(uint16_t dt);

	void Stop(void);
	void Resume(void);
	void Suspend(void);
	uint8_t UsePID(void);
	uint8_t UseLinear(void);
	int8_t Rotate(int8_t speed);

	void RequestCalibration(void);

private:
	void Calibrate(void);
};

#endif /* SYBERTOOTH_MOTORUNIT_H_ */
