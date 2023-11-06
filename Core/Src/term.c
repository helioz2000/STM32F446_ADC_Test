/*
 * term.c
 *
 *  Created on: Nov 6, 2023
 *      Author: eb
 */

#include <stdio.h>
#include <stdarg.h>
#include "main.h"
#include "term.h"

extern UART_HandleTypeDef huart2;

#define TERM_BUF_SIZE 64
uint8_t term_buf[TERM_BUF_SIZE];

int term_init(void) {
	return 0;
}

HAL_StatusTypeDef term_print(const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	vsnprintf((char*)term_buf, TERM_BUF_SIZE, format, argptr);
	va_end(argptr);
	return HAL_UART_Transmit(&huart2, term_buf, strlen((char*)term_buf), 1000);
}

