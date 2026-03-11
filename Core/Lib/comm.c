/*
 * comm.c
 *
 *  Created on: Dec 9, 2025
 *      Author: lazar
 */
#define TXBUFFERSIZE 8
#define RXBUFFERSIZE 8

#include "main.h"
volatile uint8_t tx_buffer[TXBUFFERSIZE], rx_buffer[RXBUFFERSIZE];
volatile uint8_t rxb_aligned[6];
volatile uint32_t rc = 0;
volatile int16_t x10k, y10k, phi2B;
volatile int16_t v_left_2B, v_right_2B;
volatile float v_left_comm, v_right_comm;
volatile uint16_t v_left_pwm, v_right_pwm;
volatile double phi_conversion;
volatile static uint16_t sync = 0;
volatile static uint8_t idx = 0;
volatile float x, y, phi;

void update_tx_buffer ()
{
			x10k = (int16_t) (get_x() * 10000);
			y10k = (int16_t) (get_y() * 10000);
			phi2B = (int16_t) (get_phi() * phi_conversion);

			tx_buffer[0] = 255;
			tx_buffer[1] = 255;
			tx_buffer[2] = x10k;
			tx_buffer[3] = x10k >> 8;
			tx_buffer[4] = y10k;
			tx_buffer[5] = y10k >> 8;
			tx_buffer[6] = phi2B;
			tx_buffer[7] = phi2B >> 8;

			huart1.gState = HAL_UART_STATE_READY;
			__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tx_buffer, TXBUFFERSIZE);
//	}
}



void rx_process_byte(uint8_t b)
{
    sync = (sync << 8) | b;

    if (idx == 0)
    {
        if (sync == 0xFFFF)
            idx = 1;
        return;
    }

    rxb_aligned[idx - 1] = b;
    idx++;

    if (idx == 7)
    {
        idx = 0;
    }
}


void process_rx_buffer()
{
	for (int i = 0; i < RXBUFFERSIZE; i++)
	    rx_process_byte(rx_buffer[i]);

	// TODO: mora ona obrada sa 255 255

	v_left_2B = ((int8_t)rxb_aligned[1] << 8 | rxb_aligned[0]);
	v_right_2B= ((int8_t)rxb_aligned[3] << 8 | rxb_aligned[2]);
	v_left_comm = (float)(v_left_2B >> 14);
	v_right_comm = (float)(v_right_2B >> 14);

//	phi = ((int8_t)rxb_aligned[5] << 8 | rxb_aligned[4])/phi_conversion;
	// TODO: iz rxba idu upravljacki signali za motore

}

void comm_init ()
{
	phi_conversion = pow(2, 14) / M_PI;

  if(HAL_UART_Receive_DMA(&huart1, (uint8_t *)rx_buffer, RXBUFFERSIZE) != HAL_OK)
  {
    Error_Handler();
  }
//  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tx_buffer, TXBUFFERSIZE)!= HAL_OK)
//  {
//    Error_Handler();
//  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//    HAL_UART_Receive_DMA(&huart1, (uint8_t *)rx_buffer, RXBUFFERSIZE);
	process_rx_buffer();
}
