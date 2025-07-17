/*
 * utility.c
 *
 *  Created on: 13 февр. 2022 г.
 *      Author: Vitech-UA
 */

#include "utility.h"
#include <stdbool.h>

extern UART_HandleTypeDef huart2;
//extern I2C_HandleTypeDef hi2c1;

void print_binary(uint8_t size, void const *const ptr) {
	char UART_BUFFER[50];
	unsigned char *b = (unsigned char*) ptr;
	unsigned char byte;
	int i, j;
	sprintf(UART_BUFFER, "0b");
	HAL_UART_Transmit(&huart2, (uint8_t*) UART_BUFFER, strlen(UART_BUFFER),
	HAL_MAX_DELAY);

	for (i = size - 1; i >= 0; i--) {
		for (j = 7; j >= 0; j--) {
			byte = (b[i] >> j) & 1;
			sprintf(UART_BUFFER, "%u", byte);
			HAL_UART_Transmit(&huart2, (uint8_t*) UART_BUFFER,
					strlen(UART_BUFFER),
					HAL_MAX_DELAY);
		}
	}
	sprintf(UART_BUFFER, "\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) UART_BUFFER, strlen(UART_BUFFER),
	HAL_MAX_DELAY);
}

//void i2c_scan_bus(void) {
//	uint8_t i = 0;
//	char UART_BUFFER[20] = { };
//	bool search_result = false;
//	/* Scan only for 112 allowed addresses */
//	for (i = 0x07; i < 0x78; i++) {
//		if (HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 10, 100) == HAL_OK) {
//			search_result = true;
//			sprintf(UART_BUFFER, "Find: 0x%02X\r\n", i);
//			HAL_UART_Transmit(&huart2, (uint8_t*) UART_BUFFER,
//					strlen(UART_BUFFER), 100);
//		}
//
//	}
//	if (!search_result) {
//		sprintf(UART_BUFFER, "Devices not found\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*) UART_BUFFER, strlen(UART_BUFFER),
//				100);
//	}
//}

void write_one_int_value_to_uart(uint16_t value) {
	char uart_tx_buff[32];
	int len = snprintf(uart_tx_buff, sizeof(uart_tx_buff), "%u\n", value);
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buff, len, HAL_MAX_DELAY);
}

void write_adc_buffer_to_uart(uint16_t buffer_len, uint16_t *buffer) {
	static char uart_tx_buff[4096];
	int offset = 0;
	for (uint32_t i = 0; i < buffer_len; i++) {
		int len = snprintf(&uart_tx_buff[offset], sizeof(uart_tx_buff) - offset,
				"%i", buffer[i]);

		if (len < 0 || (offset + len >= sizeof(uart_tx_buff) - 2)) {
			break;
		}

		offset += len;

		if (i < buffer_len - 1) {
			uart_tx_buff[offset++] = ',';
		}
	}

	uart_tx_buff[offset++] = '\n';
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buff, offset, HAL_MAX_DELAY);
}

void UART_Printf(const char *fmt, ...) {
	static char buff[256];
	va_list args;
	va_start(args, fmt);
	int len = vsprintf(buff, fmt, args);
	va_end(args);

	if (len > 0) {
		HAL_UART_Transmit(&huart2, (uint8_t*) buff, len, HAL_MAX_DELAY);
	}
}

void write_q15_buffer_to_uart(uint16_t buffer_len, q15_t *buffer) {
	static char uart_tx_buff[4096];
	int offset = 0;

	for (uint32_t i = 0; i < buffer_len; i++) {
		int len = snprintf(&uart_tx_buff[offset], sizeof(uart_tx_buff) - offset,
				"%d", buffer[i]);  // q15_t — це int16_t

		if (len < 0 || (offset + len >= sizeof(uart_tx_buff) - 2)) {
			break;
		}

		offset += len;

		if (i < buffer_len - 1) {
			uart_tx_buff[offset++] = ',';
		}
	}

	uart_tx_buff[offset++] = '\n';
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buff, offset, HAL_MAX_DELAY);
}
