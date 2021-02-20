/*
 * SabertoothSimplified.h
 *
 *  Created on: 22 авг. 2020 г.
 *      Author: Boris Pavlenko <borpavlenko@ispovedn1k.com>
 */

#ifndef SYBERTOOTH_SABERTOOTHSIMPLIFIED_H_
#define SYBERTOOTH_SABERTOOTHSIMPLIFIED_H_

#include <inttypes.h>

#define SBT_ALL_STOP 		0U

#define SBT_M1_STOP			64U
#define SBT_M1_MIN_BWD 		SBT_M1_STOP -1
#define SBT_M1_MIN_FWD 		SBT_M1_STOP +1
#define SBT_M1_FULL_BWD 	SBT_M1_STOP -63
#define SBT_M1_FULL_FWD 	SBT_M1_STOP +63

#define SBT_M2_STOP			192U
#define SBT_M2_MIN_BWD 		SBT_M2_STOP -1
#define SBT_M2_MIN_FWD 		SBT_M2_STOP +1
#define SBT_M2_FULL_BWD 	SBT_M2_STOP -63
#define SBT_M2_FULL_FWD 	SBT_M2_STOP +63

#define SLOWLEST			40
//#define FASTEST				40



/*
 * describes wich one of two motor is.
 */
typedef enum __MotorID_e
{
	eLeftMotor	= 1,
	eRightMotor	= 2
} MotorID_e;



typedef enum __inversion
{
	eInversionOff = 1,
	eInvertesionOn = -1,
} inverstion_t;



typedef void (*port_write_function)(uint8_t);



class SabertoothSimplified {
public:
	SabertoothSimplified(port_write_function port_write);
	virtual ~SabertoothSimplified();

public:
	inverstion_t motor1_inversion, motor2_inversion;

	/**
	 * Any command with magnitude less then threshold will be ignored.
	 * Any greate then threshold and less then SLOWLEST will be forced to slowlest.
	 */
	int8_t threshold;
	/**
	 * Stop all action.
	 */
	void stop();

	/**
	 * Sets the driving power.
	 * @param power The power, between -127 and 127.
	 */
	void drive(MotorID_e motor, int8_t power);

private:


	port_write_function _port_write;
	//void _port_write(uint8_t cmd);

};

#endif /* SYBERTOOTH_SABERTOOTHSIMPLIFIED_H_ */
