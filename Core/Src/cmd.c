/*
 * cmd.c
 *
 *  Created on: Oct 27, 2023
 *      Author: eb
 *
 *  Command interpreter
 */

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "cmd.h"


#define CMD_MIN_LEN 1		// minimum command length


extern UART_HandleTypeDef huart2;

uint8_t cmd_len = 0;

void cmd_error(uint8_t* cmd_str) {
	printf("Error in command <%s>\r\n", cmd_str);
}

int cmd_L(uint8_t* cmd_str) {
	switch (cmd_str[1]) {
	case '0':
		printf("-LED OFF-\r\n");
		return 0;
		break;
	case '1':
		printf("-LED ON-\r\n");
		return 0;
		break;
	}
	return -1;
}

int cmd_help(void) {
	printf("\r\nCommand Help:\r\n");
	printf("L[0,1]: LED on / off\r\n");
	return 0;
}

int cmd_process(uint8_t* cmd_str) {
	int retval = -1;
	switch(cmd_str[0]) {
	case 'L':
	case 'l':
		retval = cmd_L(cmd_str);
		break;
	case 'H':
	case 'h':
		retval = cmd_help();
	}
	return retval;
}

/*
 * returns -1 if the command failed, 0 if processed OK
 */
int CMD_Handler(uint8_t* cmd_str)
{
	cmd_len = strlen(cmd_str);
	if (cmd_len < CMD_MIN_LEN) {
		cmd_error(cmd_str);
		return -1;
	}
	return cmd_process(cmd_str);
}
