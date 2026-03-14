/*
 * comm.c
 *
 *  Created on: Dec 9, 2025
 *      Author: lazar
 */
#define TXBUFFERSIZE 40
#define RXBUFFERSIZE 40

#include "main.h"
#include <string.h>

static uint16_t fletcher16(uint8_t *data, size_t len);
static void create_rxba();

volatile uint8_t tx_buffer[TXBUFFERSIZE], rx_buffer[RXBUFFERSIZE];
volatile uint8_t rxba[32];
volatile uint32_t rc = 0;
volatile int16_t x10k, y10k, phi2B;
volatile int16_t v_left_2B, v_right_2B;
volatile float v_left_comm, v_right_comm;
volatile uint16_t v_left_pwm, v_right_pwm;
volatile double phi_conversion;
volatile static uint16_t sync = 0;
volatile static uint8_t idx = 0;
volatile float x, y, phi;

volatile goal_type rx_goal;
double x_dbg = 1.0, y_dbg = 0.0, phi_dbg = 0.0;
/*
 * ROS2 -> STM32:
 * 40B:
 * 	 8 * uint8 255
 * 	 int8 type
 * 	 double x
 * 	 double y
 * 	 double phi
 * 	 int8 	direction
 * 	 uint8 	v_max_100				[0.00, 2.55]
 * 	 uint8 	w_max_10				[0.0, 25.5]
 * 	 uint8 	dis_tol_perc_16: 4	[0.0, 1.0 : 0.0625]
 * 	 		ang_tol_perc_16: 4	[0.0, 1.0 : 0.0625]
 * 	 int8	start_coeff_v_4: 2	[0.0, 1.0 : 0.25]
 * 	 		start_coeff_w_4: 2	[0.0, 1.0 : 0.25]
 * 	 		stop_coeff_v_4:  2	[0.0, 1.0 : 0.25]
 * 	 		stop_coeff_w_4:  2	[0.0, 1.0 : 0.25]
 * 	 uint16 checksum
 *
 * STM32 -> ROS2:
 *
 */

void process_rx_buffer() {
	// izvrsava cilj
	if (rx_goal.status > -1)
		return;

	create_rxba();
	uint16_t received_checksum = rxba[30] | (rxba[31] << 8);
	uint16_t calculated_checksum = fletcher16(rxba, 30);
	if (received_checksum != calculated_checksum) {
		// checksum failed
		return;
	}
	rx_goal.type = rxba[0];

	memcpy(&rx_goal.x, &rxba[1], sizeof(double));
	memcpy(&rx_goal.y, &rxba[9], sizeof(double));
	memcpy(&rx_goal.phi, &rxba[17], sizeof(double));

	rx_goal.direction = rxba[25];
	uint8_t v_max_100 = rxba[26];
	rx_goal.v_max = v_max_100 * 0.01;
	uint8_t w_max_10 = rxba[27];
	rx_goal.w_max = w_max_10 * 0.1;
	uint8_t tol_byte = rxba[28];
	rx_goal.distance_tolerance_percentage = (((tol_byte >> 4) & 0b1111) + 1) * 0.0625;
	rx_goal.angle_tolerance_percentage = (((tol_byte) & 0b1111) + 1) * 0.0625;
	uint8_t coeff_byte = rxba[29];
	rx_goal.start_coeff_v = (((coeff_byte >> 6) & 0b11) + 1) * 0.25;
	rx_goal.start_coeff_w = (((coeff_byte >> 4) & 0b11) + 1) * 0.25;
	rx_goal.stop_coeff_v = (((coeff_byte >> 2) & 0b11) + 1) * 0.25;
	rx_goal.stop_coeff_w = (((coeff_byte) & 0b11) + 1) * 0.25;
	return;
}

goal_type* get_rx_goal() {
	return &rx_goal;
}

void update_tx_buffer() {
	for (int i = 0; i < 8; i++)
		tx_buffer[i] = 0xFF;

	tx_buffer[8] = 1;
	memcpy(&tx_buffer[9], &x_dbg, sizeof(double));   // x
	memcpy(&tx_buffer[17], &y_dbg, sizeof(double));  // y
	memcpy(&tx_buffer[25], &phi_dbg, sizeof(double));  // phi
	tx_buffer[33] = 1;      // direction
	tx_buffer[34] = 150;    // v_max_100
	tx_buffer[35] = 94;     // w_max_10
	tx_buffer[36] = 0xff; 	// tol_perc_16
	tx_buffer[37] = 0xff; 	// start/stop coeffs = 1
	// checksum over payload bytes (8→37)
	uint16_t cksum = fletcher16(&tx_buffer[8], 30);
	tx_buffer[38] = cksum & 0xFF;
	tx_buffer[39] = cksum >> 8;

	huart1.gState = HAL_UART_STATE_READY;
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*) tx_buffer, TXBUFFERSIZE);
}

static uint16_t fletcher16(uint8_t *data, size_t len) {
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;

	for (size_t i = 0; i < len; i++) {
		sum1 = (sum1 + data[i]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}

	return (sum2 << 8) | sum1;
}

static void create_rxba() {
	int start_index = -1;

	for (int i = 0; i <= RXBUFFERSIZE - 8; i++) {
		if (rx_buffer[i] == 0xFF && rx_buffer[i + 1] == 0xFF
				&& rx_buffer[i + 2] == 0xFF && rx_buffer[i + 3] == 0xFF
				&& rx_buffer[i + 4] == 0xFF && rx_buffer[i + 5] == 0xFF
				&& rx_buffer[i + 6] == 0xFF && rx_buffer[i + 7] == 0xFF) {
			start_index = i + 8;
			break;
		}
	}
	if (start_index < 0)
		return;

	int first = 40 - start_index;
	if (first >= 32)
		memcpy(rxba, &rx_buffer[start_index], 32);
	else {
		memcpy(rxba, &rx_buffer[start_index], first);
		memcpy(rxba + first, &rx_buffer[0], 32 - first);
	}
}

void comm_init() {
	rx_goal.status = -1;

	if (HAL_UART_Receive_DMA(&huart1, (uint8_t*) rx_buffer,
	RXBUFFERSIZE) != HAL_OK) {
		Error_Handler();
	}
//  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tx_buffer, TXBUFFERSIZE)!= HAL_OK)
//  {
//    Error_Handler();
//  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    HAL_UART_Receive_DMA(&huart1, (uint8_t *)rx_buffer, RXBUFFERSIZE);
	process_rx_buffer();
}
