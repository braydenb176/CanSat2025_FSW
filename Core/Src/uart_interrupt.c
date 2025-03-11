/*
 * uart_interrupt.c
 *
 *  Created on: Feb 25, 2025
 *      Author: bbevel0133
 */

#include "uart_interrupt.h"

uint8_t uart_rx_buffer3[UART_BUFFER_SIZE];
uint8_t uart_rx_buffer5[UART_BUFFER_SIZE];

volatile uint8_t uart3_data_ready = 0;
volatile uint8_t uart5_data_ready = 0;

void clear_uart_buffer(UART_HandleTypeDef *huart){
    uint8_t temp;
    while(HAL_UART_Receive(huart, &temp, 1, 0xFF) == HAL_OK);  // Read until no more data
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart3){ //xbee
		uart3_data_ready = 1;

		//reactivate uart4 interrupt, entering this callback disables it
		HAL_UART_Receive_IT(huart, uart_rx_buffer3, UART_BUFFER_SIZE);
	}else if(huart == &huart5){ //gps

	}
}


