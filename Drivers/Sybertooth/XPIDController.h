/*
 * XPIDRegulator.h
 *
 *  Created on: 22 авг. 2020 г.
 *      Author: Boris Pavlenko <borpavlenko@ispovedn1k.com>
 *
 * 0				<=== TIMx->CNT == 0
 * LimitSwitch		<=== TIMx->CNT == ~~~~
 * SafeStart		<=== LimitSwitch + xxxx 	==	minPosition
 * SafeEnd			<=== LimitSwitch - xxxx 	==	maxPosition
 * LimitSwitch		<=== TIMx->CNT == ~~~~
 * 0xFFFF			<=== TIMx->CNT == 0xFFFF max value for 16-bit Timer
 * 0xFFFFFFFF
 */

#ifndef SYBERTOOTH_XPIDCONTROLLER_H_
#define SYBERTOOTH_XPIDCONTROLLER_H_

#include <inttypes.h>

typedef struct __PID_settings {
	float Ka;						// total mult. usually 1.0f
	float Kp; 						// proportional
	float Ki;						// integral
	float Kd;						// derivative
	uint16_t hitbox;				// target hitbox size
} PID_settings_t, *PID_settings_p;



class XPIDController: public PID_settings_t {
public:
	int32_t last_error;
	int32_t integrated_error;
	int8_t  output;

public:
	XPIDController();
	virtual ~XPIDController();

public:

	// TMRx->CNT 16bits only (STM32F103C8T6)
	// target is only 2 bytes got via command.
	/**
	 * @desc
	 * calculate PID using PID_settings,
	 * stores result as status.output
	 * @param
	 * position: current position.
	 * target: must be normalized to [minPosition : maxPosition].
	 * ms: dt interval. miliseconds.
	 * @return:
	 * [-127: 127] value
	 */
	int8_t CalcPID(uint32_t position, uint32_t target, uint16_t dt);

	void setPID(PID_settings_p src);
	void dumpPID(PID_settings_p dst);
	void setDefault(void);

	int sprint(char* ptrc);
};

#endif /* SYBERTOOTH_XPIDCONTROLLER_H_ */
