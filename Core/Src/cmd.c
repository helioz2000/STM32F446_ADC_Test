/*
 * cmd.c
 *
 *  Created on: Oct 27, 2023
 *      Author: Erwin Bejsta
 *
 *  Command interpreter for terminal interface
 */

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "global.h"
#include "cmd.h"
#include "term.h"
#include "display.h"

#define CMD_MIN_LEN 1		// minimum command length

extern UART_HandleTypeDef huart2;
extern uint8_t adc_restart;
extern uint8_t cmd_display_buffer;
extern uint8_t tft_display;
extern uint8_t esp_mode;
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

int cmd_led(int cmd) {
	if (cmd > 1) {
		HAL_GPIO_WritePin (LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin (LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	return(0);
}


int cmd_help(void) {
	term_print("\r\nCommand Help:\r\n");
	term_print("C[1..4]: CSV output ADC channel 1 - 4 buffer content\r\n");
#ifdef USE_DISPLAY
	term_print("D[1..4]: Display ADC channel 1 - 4 on TFT display\r\n");
	term_print("D: Display all ADC channel on TFT display\r\n");
#endif
	term_print("E: Toggle ESP mode\r\n");
	term_print("L[0,1]: LED L2 OFF / ON\r\n");
	term_print("M show measurements using all channels\r\n");
	term_print("M[1..4]: Show measurements for ADC channel 1 - 4 buffer in terminal \r\n");
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
		term_csv_buffer(cmd_str[1] - 0x31);
		retval = 0;
		break;
#ifdef USE_DISPLAY
	case 'D':
	case 'd':
		if (strlen((char*)cmd_str) > 1) {
			display_show_curve(cmd_str[1] - 0x31);
		} else {
			//display_show_curves();
		}
		retval = 0;
		break;
#endif
	case 'E':
	case 'e':
		if (esp_mode) { esp_mode = 0; }
		else {
			esp_mode = 1;
			term_print("\r\nESP mode active, enter ~ to exit\r\n");
		}
		break;
	case 'L':
	case 'l':
		retval = cmd_led(cmd_str[1] - 0x30 + 1);
		break;
	case 'M':
	case 'm':
		if (strlen((char*)cmd_str) > 1) {
		    term_show_channel(cmd_str[1] - 0x31);
		} else {
			term_show_measurements();
		}
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
		term_show_buffer(cmd_str[1] - 0x31);
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
