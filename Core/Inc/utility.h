/*
 * utility.h
 *
 *  Created on: 13 февр. 2022 г.
 *      Author: Vitech-UA
 */

#ifndef UTILITY_H_
#define UTILITY_H_
#include "main.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "arm_math.h"

void print_binary(uint8_t size, void const * const ptr);
void write_one_int_value_to_uart(uint16_t value);
void write_adc_buffer_to_uart(uint16_t buffer_len, uint16_t *buffer);
void i2c_scan_bus(void);
void UART_Printf(const char *fmt, ...);
void write_q15_buffer_to_uart(uint16_t buffer_len, q15_t *buffer);
#endif /* UTILITY_H_ */
