/*
 * minimath.c
 *
 *  Created on: 23 авг. 2020 г.
 *      Author: Boris Pavlenko <borpavlenko@ispovedn1k.com>
 */
#include "minimath.h"

/*
inline uint8_t abs(int8_t a)
{
	return a < 0 ? -a : a;
}
//*/

double constrain(double x, double min, double max)
{
	if (x < min) return min;
	if (x > max) return max;
	return x;
}


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


inline char lower(char c)
{
    if (c >= 'A' && c <= 'Z')
        return c +'a'-'A';
    else
        return c;
}
