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
extern uint16_t new_energy_time_period;
extern uint8_t modbus_addr_change;

extern double total_precision_vah[];
extern double total_precision_wh[];
extern uint32_t total_vah[];
extern uint32_t total_wh[];

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
	}
	return -1;
}


// adjust energy integration timer period
int cmd_z(uint8_t* cmd_str) {
	unsigned int value;
	int result = sscanf((char*)cmd_str+1, "%u", &value);
	if (result != 1) return -1;
	new_energy_time_period = value;
	return 0;
}

// adjust ADC timer period
int cmd_p(uint8_t* cmd_str) {
	unsigned int value;
	int result = sscanf((char*)cmd_str+1, "%u", &value);
	if (result != 1) return -1;
	new_time_period = value;
	return 0;
}


int cmd_a(uint8_t* cmd_str) {
	unsigned int value;
	int result = sscanf((char*)cmd_str+1, "%u", &value);
	if (result != 1) return -1;
	modbus_addr_change = value;
	return 0;
}

int cmd_led(int cmd) {
	if (cmd > 1) {
		HAL_GPIO_WritePin (LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin (LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	return 0;
}

int cmd_g() {
	for (int i=0; i<NUM_I_SENSORS; i++) {
		term_print("VAh[%d] = %u [1/10 VAh] %.2f\r\n", i, total_vah[i], total_precision_vah[i]);
		term_print(" Wh[%d] = %u [1/10 Wh] %.2f\r\n", i, total_wh[i],  total_precision_wh[i]);
	}
	return 0;
}

int cmd_help(void) {
	term_print("\r\nCommand Help:\r\n");
	term_print("A[1..255]: Set new Modbus Address (EEPROM)\r\n");
	term_print("C[1..4]: CSV output ADC channel 1 - 4 buffer content\r\n");
	term_print("E: Toggle ESP mode\r\n");
	term_print("G: Show Energy readings\r\n");
	term_print("L[0,1]: LED L2 OFF / ON\r\n");
	term_print("M show measurements using all channels\r\n");
	term_print("M[1..4]: Show measurements for ADC channel 1 - 4 buffer in terminal \r\n");
	term_print("P[2000..2500]: adjust timer value for sample time (EEPROM)\r\n");
	term_print("R: Restart ADC conversion\r\n");
	term_print("S[1..4]: Show ADC channel 1 - 4 buffer content in terminal\r\n");
	term_print("T[0|1]: TFT display OFF / ON\r\n");
	term_print("Y: Reset all energy totals (VA, W) to zero\r\n");
	term_print("Z[500..1500]: adjust energy integration timer (200ms) [EEPROM]\r\n");
	return 0;
}

int cmd_process(uint8_t* cmd_str) {
	int retval = -1;
	switch(cmd_str[0]) {
	case 'A':
	case 'a':
		retval = cmd_a(cmd_str);
		break;
	case 'C':
	case 'c':
		term_csv_buffer(cmd_str[1] - 0x31);
		retval = 0;
		break;
	case 'E':
	case 'e':
		if (esp_mode) { esp_mode = 0; }
		else {
			esp_mode = 1;
			term_print("\r\nESP mode active, enter ~ to exit\r\n");
		}
		break;
	case 'G':
	case 'g':
		retval = cmd_g();
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
	case 'Y':
	case 'y':
		retval = energy_totals_init(1);
		break;
	case 'Z':
	case 'z':
		retval = cmd_z(cmd_str);
		break;

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
