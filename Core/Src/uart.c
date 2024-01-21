/*
 * uart.c
 *
 *  Created on: Feb 3, 2023
 *      Author: pusedu03
 */

#include "uart.h"
#include <stdio.h>

#define STX	0x02
#define ETX 0x03

UART_HandleTypeDef *huart;

#define rxBufferMax	255
int rxBufferGp;					// get pointer (read)
int rxBufferPp;					// put pointer (write)
uint8_t rxBuffer[rxBufferMax];
uint8_t rxChar;

// uart 장치 초기화
void initUart(UART_HandleTypeDef *inHuart) {
	huart = inHuart;
	HAL_UART_Receive_IT(huart, &rxChar, 1);
}

// 문자수신 처리
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	rxBuffer[rxBufferPp++] = rxChar;
	rxBufferPp %= rxBufferMax;
	HAL_UART_Receive_IT(huart, &rxChar, 1);
}

// 버퍼에서 문자 꺼내오기
int16_t getChar() {
	int16_t result;
	if(rxBufferGp == rxBufferPp) return -1;
	result = rxBuffer[rxBufferGp++];
	rxBufferGp %= rxBufferMax;
	return result;
}

// packet 전송
void transmitPacket(protocol_t data) {
	// 사전준비, CRC, STX, ETX
	// STX, CMD, D1, D2, CRC, ETX
	// big endian
	uint8_t buffer[] = {STX, 0, 0, 0, 0, ETX};
	buffer[1] = data.cmd;
	buffer[2] = (data.data >> 7) | 0x80;
	buffer[3] = (data.data & 0x7f) | 0x80;
	buffer[4] = buffer[0] + buffer[1] + buffer[2] + buffer[3];
	// 데이터 전송
	HAL_UART_Transmit(huart, buffer, sizeof(buffer), 1);
	// 데이터 전송 완료 대기
	while (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX
			|| HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX_RX);
	//while(HAL_UART_GetState(huart) != HAL_UART_STATE_READY);
}
// packet 수신
protocol_t receivePacket() {
	protocol_t result;
	uint8_t buffer[6];
	uint8_t count = 0;
	uint32_t timeout;

	int16_t ch = getChar();
	memset(&result, 0, sizeof(protocol_t));
	if(ch == STX) {
		buffer[count++] = ch;
		timeout = HAL_GetTick();
		while(ch != ETX) {
			ch = getChar();
			if(ch != -1) {
				buffer[count++] = ch;
			}
			// 6byte 전송에 필요한 시간 522us이므로 2ms동안 기다려줌
			if(HAL_GetTick() - timeout >= 2) return result;
		}
		// crc 검사
		uint8_t crc = 0;
		for(int i = 0; i < 4; i++)
			crc += buffer[i];
		if(crc != buffer[4]) return result;
		// 정상 수신 완료 후 데이터 파싱 (parser, parsing)
		result.cmd = buffer[1];
		result.data = buffer[3] & 0x7f;
		result.data |= (buffer[2] & 0x7f) << 7;
	}
	return result;
}

int _write(int file, char *p, int len) {
	HAL_UART_Transmit(huart, p, len, len);
	return len;
}
