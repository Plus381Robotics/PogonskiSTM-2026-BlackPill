/*
 * signal.c
 *
 *  Created on: Dec 23, 2025
 *      Author: lazar
 */


void
wrap180 (volatile double *signal)
{
	if (*signal > 180.0)
		*signal -= 360.0;
	if (*signal < -180.0)
		*signal += 360.0;
}
