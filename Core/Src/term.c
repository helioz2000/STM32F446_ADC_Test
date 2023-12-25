/*
 * term.c
 *
 *  Created on: Nov 6, 2023
 *      Author: Erwin Bejsta
 *
 *  Handle terminal interface
 */

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <math.h>
#include "global.h"
#include "main.h"
#include "calc.h"
#include "term.h"

extern UART_HandleTypeDef huart2;
extern uint16_t adc_raw_buf[ADC_NUM*ADC_NUM_CHANNELS][ADC_NUM_DATA];	// buffer for channels of raw ADC data
extern uint16_t sample_buf[ADC_NUM_BUFFERS][SAMPLE_BUF_SIZE];			// buffer for channels of down-sampled data
extern struct sampleBufMeta sample_buf_meta[];
extern float metervalue_v, metervalue_i, metervalue_va, metervalue_w, metervalue_pf;

#define TERM_BUF_SIZE 1024
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

/*
 * Print buffer content in hexadecimal format
 * parameter buf: buffer
 * parameter len: buffer length
 * parameter per_line: number of values to print per line (0=default)
 */
void term_print_hex(uint8_t* buf, unsigned len, uint8_t per_line) {
	uint8_t count = 0;
	if (per_line < 4) { per_line = 16; }
	for (int i = 0; i < len; i++) {
		term_print("%02X ", buf[i]);
		if (++count > per_line) {
			term_print("\r\n");
			count = 0;
		}
	}
}

/*
 * Show the adc_raw_buf contents in terminal
 */
void term_show_buffer(uint8_t bufnum) {
	int count = 0;
	uint16_t address = 0;

	if (bufnum >= ADC_NUM_BUFFERS) { return; }
	term_print("Buffer %d\r\n", bufnum);
	term_print("%3d: ", 0);
	for (int i=0; i<SAMPLE_BUF_SIZE; i++) {
		if (count >= 20) {
			count =0;
			term_print("\r\n%3d: ", address);
		}
		term_print("%04u ", sample_buf[bufnum][i]);
		count++; address++;
	}
	term_print("\r\n");
	term_show_channel(bufnum);
}

void term_show_measurements() {
	if (sample_buf_meta[ADC_CH_V].measurements_valid != 1) {
		if (calc_measurements() != 0) {
			term_print(" invalid readings\r\n");
			return;
		}
	}
	term_print("%.1fV %.1fA %.1fVA %.1fW PF=%.2f (%.1fDeg)\r\n", metervalue_v, metervalue_i, metervalue_va,
			metervalue_w, metervalue_pf, acos(metervalue_pf) * (180.0 / 3.14159265) );
}

void term_show_channel(uint8_t bufnum) {
	if (bufnum >= ADC_NUM_BUFFERS) { return; }
	term_print("Measurements Buffer %d:\r\n", bufnum);
	if (sample_buf_meta[bufnum].measurements_valid != 1) {
		term_print("Buffer %d - invalid readings\r\n", bufnum);
		//term_print("Range: %dmV - %dmV\r\n", calc_adc_raw_to_mv_int(sample_buf_meta[bufnum].min), calc_adc_raw_to_mv_int(sample_buf_meta[bufnum].max) );
		//return;
	}

	int pp_reading = sample_buf_meta[bufnum].max - sample_buf_meta[bufnum].min;
	term_print("ADC raw: %d - %d (%d)\r\n", sample_buf_meta[bufnum].min, sample_buf_meta[bufnum].max, pp_reading );
	term_print("RMS: %dmV, P-P:%d mV, Zero: %dmV\r\n", calc_adc_raw_to_mv_int(sample_buf_meta[bufnum].rms_value) ,
			calc_adc_raw_to_mv_int(pp_reading),
			calc_adc_raw_to_mv_int(pp_reading/2 + sample_buf_meta[bufnum].min) );
	term_print("Range: %dmV - %dmV\r\n", calc_adc_raw_to_mv_int(sample_buf_meta[bufnum].min), calc_adc_raw_to_mv_int(sample_buf_meta[bufnum].max) );
	term_print("Zero crossing: pos=%d neg=%d\r\n", sample_buf_meta[bufnum].zero_cross_pos, sample_buf_meta[bufnum].zero_cross_neg);
}

/*
 * Output adc_raw_buf contents in CSV format to terminal
 */
void term_csv_buffer(uint8_t buf_num) {
	if (buf_num >= ADC_NUM_BUFFERS) { return; }
	term_print("Buffer %d\r\n", buf_num);
	for (int i=0; i<ADC_NUM_DATA; i++) {
		term_print("%d,%u\r\n", i, adc_raw_buf[buf_num][i]);
	}
	term_print("\r\n\r\n");
}

