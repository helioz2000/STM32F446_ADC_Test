/*
 * calc.c
 *
 *  Created on: Oct 29, 2023
 *      Author: eb
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "global.h"
#include "calc.h"

extern uint16_t adc_dma_buf[ADC_NUM][ADC_DMA_BUF_SIZE];
//extern uint16_t adc2_dma_buf[];

uint16_t adc_raw_buf[4][ADC_NUM_DATA];		// buffer for 4 channels of raw ADC data

/*
 * Process the DMA buffer
 * parameter second_half: > 0 to process 2nd half of buffer, 0 = 1st half of buffer
 * parameter adc_num: 0 = ADC1, 1 = ADC2 (use ADC1_IDX or ADC2_IDX)
 * returns: -1 on failure, 0 if OK
 */
int calc_process_dma_buffer(int second_half, int adc_num) {
	uint16_t i;
	uint16_t dma_buf_start, dma_buf_end;		// DMA buffer source
	uint16_t raw_buf_idx = 0;
	uint8_t raw_buf_first, raw_buf_second;		// destination index for raw readings
	// adc_num range check (has to be either ADC1 or ADC2
	if ( (adc_num != ADC1_IDX) && (adc_num != ADC2_IDX) ) {
		return -1;
	}
	// channel index to raw buffer array
	raw_buf_first = adc_num *2;			// destination index for first entry in DMA buffer
	raw_buf_second = raw_buf_first + 1; // destination index for second entry in DMA buffer
	// first or second half of DMA buffer?
	if (second_half) {
		dma_buf_start = ADC_DMA_BUF_SIZE / 2;
	} else {
		dma_buf_start = 0;	// first half
	}
	dma_buf_end = dma_buf_start + (ADC_DMA_BUF_SIZE / 2) -1;

	// split DMA buffer and copy into raw buffers
	// step of ADC_NUM_CHANNELS = 2
	for (i=dma_buf_start; i<=dma_buf_end; i+=ADC_NUM_CHANNELS) {
		adc_raw_buf[raw_buf_first][raw_buf_idx] = adc_dma_buf[adc_num][i];		// first entry in DMA buffer
		adc_raw_buf[raw_buf_second][raw_buf_idx++] = adc_dma_buf[adc_num][i+1]; // second entry in DMA buffer
	}
	return 0;
}

void calc_display_buffer(uint8_t buf_num) {
	int count = 0;
	uint16_t address = 0;
	uint64_t squared_acc = 0;
	uint16_t rms_value;
	if (buf_num > 3) { return; }
	printf("Buffer %d\r\n", buf_num);
	printf("%3d: ", 0);
	for (int i=0; i<ADC_NUM_DATA; i++) {
		if (count >= 20) {
			count =0;
			printf("\r\n%3d: ", address);
		}
		printf("%04u ", adc_raw_buf[buf_num][i]);
		squared_acc += adc_raw_buf[buf_num][i] * adc_raw_buf[buf_num][i];
		count++; address++;
	}
	rms_value = (uint16_t) sqrt((squared_acc / ADC_NUM_DATA));
	printf("\r\nRMS: %dmV [%u]\r\n", calc_adc_raw_to_mv_int(rms_value), rms_value);
}

/*
 * Convert ADC raw reading to mv
 * returns mv as int
 */
int calc_adc_raw_to_mv_int(uint16_t adc_raw) {
	return round(calc_adc_raw_to_mv_float(adc_raw));
}

float calc_adc_raw_to_mv_float(uint16_t adc_raw) {
	return ((float)adc_raw / (float)ADC_FS_RAW) * (float)ADC_FS_MV;
}
