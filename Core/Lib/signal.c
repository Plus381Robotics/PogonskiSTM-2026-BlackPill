/*
 * signal.c
 *
 *  Created on: Dec 23, 2025
 *      Author: lazar
 */

#include <math.h>

void
wrap180 (volatile double *signal)
{
	if (*signal > 180.0)
		*signal -= 360.0;
	if (*signal < -180.0)
		*signal += 360.0;
}

void
wrap2Pi (volatile double *signal)
{
	if (*signal > M_PI)
		*signal -= 2*M_PI;
	if (*signal < -M_PI)
		*signal += 2 * M_PI;
}
