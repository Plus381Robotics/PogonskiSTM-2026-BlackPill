/*
 * lib.h
 *
 *  Created on: Aug 13, 2024
 *      Author: lazar
 */

#ifndef LIB_LIB_H_
#define LIB_LIB_H_

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "tim.h"
#include "gpio.h"
#include "usart.h"

// time.h
void
time_ISR();
void
time_start();
void
time_stop();

// encoder.h
int16_t
cnt_difference(TIM_HandleTypeDef *htim, volatile int16_t *cur);
void
enc_start(TIM_HandleTypeDef *htim);
void
enc_init();
double
enc_velocity(int16_t diff, double deltaT, uint16_t impulse_per_2Pi);

// pwm.h
void
pwm_set_dc(TIM_HandleTypeDef *htim, uint32_t channel, int16_t duty_cycle);
void
pwm_init();
void
pwm_left(float vel);
void
pwm_right(float vel);
void
set_motor_l_dir(int8_t dir);
void
set_motor_r_dir (int8_t dir);
// odometry.h
void
odometry_init();
void
update_odom();
double
get_x();
double
get_y();
double
get_phi();
double
get_v();
double
get_w();

//signal.h
void
wrap180(volatile double *signal);
void
wrap2Pi (volatile double *signal);

//motorControl.h
double calculate_control(double ref, double y);

// comm.h
void comm_init();
void update_tx_buffer();

// comm.h
void
update_tx_buffer ();
void
comm_init ();
void
process_rx_buffer ();

#endif /* LIB_LIB_H_ */
