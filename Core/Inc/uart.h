/*
 * uart.h
 *
 *  Created on: Feb 3, 2023
 *      Author: pusedu03
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
#include "string.h"

typedef struct {
	uint8_t 	cmd;
	uint16_t 	data;
} protocol_t;

// prototype 선언
void initUart(UART_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int16_t getChar();
void transmitPacket(protocol_t data);
protocol_t receivePacket();

#endif /* INC_UART_H_ */
