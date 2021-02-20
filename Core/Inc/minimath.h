/*
 * minimath.h
 *
 *  Created on: 23 авг. 2020 г.
 *      Author: Boris Pavlenko <borpavlenko@ispovedn1k.com>
 */

#ifndef MINIMATH_H_
#define MINIMATH_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include <inttypes.h>

//uint8_t abs(int8_t a);

double constrain(double x, double min, double max);
long map(long x, long in_min, long in_max, long out_min, long out_max);
char lower(char c);



#ifdef __cplusplus
 }
#endif
#endif /* MINIMATH_H_ */
