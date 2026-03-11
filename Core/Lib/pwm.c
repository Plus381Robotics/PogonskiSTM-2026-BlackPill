/*
 * pwm.c
 *
 *  Created on: Nov 25, 2025
 *      Author: Dell
 */

#include "main.h"

volatile float diameter = 0.07, max_v = 2.0;

void pwm_set_dc(TIM_HandleTypeDef *htim, uint32_t channel, int16_t duty_cycle) {
	__HAL_TIM_SET_COMPARE(htim, channel, duty_cycle);
}

void pwm_init() {
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}

int8_t sign(float x) {
	if (x > 0.0)
		return 1;
	if (x < 0.0)
		return -1;
	return 0;
}

void pwm_left(float vel) {
	int16_t pwm;
	int8_t dir = sign(vel);
	pwm = dir * vel / max_v * 1680;
	pwm_set_dc(&htim9, TIM_CHANNEL_1, pwm);
}

void pwm_right(float vel) {
	int16_t pwm;
	int8_t dir = sign(vel);
	pwm = dir * vel / max_v * 1680;
	pwm_set_dc(&htim9, TIM_CHANNEL_2, pwm);

}
void set_motor_l_dir(int8_t dir) {
	switch (dir) {
	case 1:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
		break;
	case -1:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
		break;
	case 0:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
		break;
	}
}

void set_motor_r_dir(int8_t dir) {
	switch (dir) {
	case 1:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1);
		break;
	case -1:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
		break;
	case 0:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
		break;
	}
}
