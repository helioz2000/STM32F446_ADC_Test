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
#include "global.h"
#include "cmd.h"
#include "term.h"


#define CMD_MIN_LEN 1		// minimum command length


extern UART_HandleTypeDef huart2;
extern uint8_t adc_restart;
extern uint8_t show_buffer;
extern uint8_t cmd_display_buffer;
extern uint8_t csv_buffer;
extern uint8_t led_cmd;
extern uint8_t tft_display;
extern uint16_t new_time_period;

uint8_t cmd_len = 0;

void cmd_error(uint8_t* cmd_str) {
	term_print("Error in command <%s>\r\n", cmd_str);
}


int cmd_t(uint8_t* cmd_str) {
	switch (cmd_str[1]) {
	case '0':
		tft_display = 1;
		return 0;
		break;
	case '1':
		tft_display = 2;
		return 0;
		break;
	case 'T':
	case 't':
		tft_display = 9;
		return 0;
	}
	return -1;
}

// adjust timer period
int cmd_p(uint8_t* cmd_str) {
	unsigned int value;
	int result = sscanf((char*)cmd_str+1, "%u", &value);
	if (result != 1) return -1;
	new_time_period = value;
	return 0;
}


int cmd_help(void) {
	term_print("\r\nCommand Help:\r\n");
	term_print("C[1..4]: CSV output ADC channel 1 - 4 buffer content\r\n");
#ifdef USE_DISPLAY
	term_print("D[1..4]: Display ADC channel 1 - 4 on TFT display\r\n");
#endif
	term_print("L[0,1]: LED L2 OFF / ON\r\n");
	term_print("P[2000..2500]: adjust timer value for sample time\r\n");
	term_print("R: Restart ADC conversion\r\n");
	term_print("S[1..4]: Show ADC channel 1 - 4 buffer content in terminal\r\n");
	term_print("T[0|1|T]: TFT display OFF / ON / Performance test\r\n");
	return 0;
}

int cmd_process(uint8_t* cmd_str) {
	int retval = -1;
	switch(cmd_str[0]) {
	case 'C':
	case 'c':
		csv_buffer = cmd_str[1] - 0x30;
		retval = 0;
		break;
#ifdef USE_DISPLAY
	case 'D':
	case 'd':
		cmd_display_buffer = cmd_str[1] - 0x30;
		retval = 0;
		break;
#endif
	case 'L':
	case 'l':
		led_cmd = cmd_str[1] - 0x30 + 1;
		retval = 0;
		break;
	case 'P':
	case 'p':
		retval = cmd_p(cmd_str);
		break;
	case 'R':
	case 'r':
		adc_restart = 1;
		retval = 0;
		break;
	case 'S':
	case 's':
		show_buffer = cmd_str[1] - 0x30;
		retval = 0;
		break;
	case 'T':
	case 't':
		retval = cmd_t(cmd_str);
		break;
	case 'H':
	case 'h':
	case '?':
		retval = cmd_help();
	}
	return retval;
}

/*
 * returns -1 if the command failed, 0 if processed OK
 */
int CMD_Handler(uint8_t* cmd_str)
{
	cmd_len = strlen((char *) cmd_str);
	if (cmd_len < CMD_MIN_LEN) {
		cmd_error(cmd_str);
		return -1;
	}
	return cmd_process(cmd_str);
}
