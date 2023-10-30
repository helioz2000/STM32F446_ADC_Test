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

struct rawBufMeta {
	uint16_t min;
	uint16_t max;
	int zero_cross1;
	int zero_cross2;
};

uint16_t adc_raw_buf[4][ADC_NUM_DATA];		// buffer for 4 channels of raw ADC data
struct rawBufMeta adc_raw_meta[4];			// store meta data for associated buffer

//inline int16_t MAX(int16_t a, int16_t b) { return((a) > (b) ? a : b); }
//inline int16_t MIN(int16_t a, int16_t b) { return((a) < (b) ? a : b); }

/*
 * Process the DMA buffer
 * parameter second_half: > 0 to process 2nd half of buffer, 0 = 1st half of buffer
 * parameter adc_num: 0 = ADC1, 1 = ADC2 (use ADC1_IDX or ADC2_IDX)
 * returns: -1 on failure, 0 if OK
 */
int calc_process_dma_buffer(int second_half, int adc_num) {
	uint16_t i = 0;
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

	adc_raw_meta[raw_buf_first].min = adc_dma_buf[adc_num][0];
	adc_raw_meta[raw_buf_first].max = adc_dma_buf[adc_num][0];
	adc_raw_meta[raw_buf_second].min = adc_dma_buf[adc_num][1];
	adc_raw_meta[raw_buf_second].max = adc_dma_buf[adc_num][1];
	adc_raw_meta[raw_buf_first].zero_cross1 = -1;
	adc_raw_meta[raw_buf_first].zero_cross2 = -1;
	adc_raw_meta[raw_buf_second].zero_cross1 = -1;
	adc_raw_meta[raw_buf_second].zero_cross2 = -1;
	// split DMA buffer and copy into raw buffers
	// step of ADC_NUM_CHANNELS = 2
	for (i=dma_buf_start; i<=dma_buf_end; i+=ADC_NUM_CHANNELS) {
		adc_raw_buf[raw_buf_first][raw_buf_idx] = adc_dma_buf[adc_num][i];		// first entry in DMA buffer
		adc_raw_buf[raw_buf_second][raw_buf_idx++] = adc_dma_buf[adc_num][i+1]; // second entry in DMA buffer
		adc_raw_meta[raw_buf_first].min = MIN(adc_raw_meta[raw_buf_first].min, adc_dma_buf[adc_num][i]);
		adc_raw_meta[raw_buf_first].max = MAX(adc_raw_meta[raw_buf_first].max, adc_dma_buf[adc_num][i]);
		adc_raw_meta[raw_buf_second].min = MIN(adc_raw_meta[raw_buf_second].min, adc_dma_buf[adc_num][i+1]);
		adc_raw_meta[raw_buf_second].max = MAX(adc_raw_meta[raw_buf_second].max, adc_dma_buf[adc_num][i+1]);
	}
	return 0;
}

void calc_display_buffer(uint8_t buf_num) {
	int count = 0;
	uint16_t address = 0;
	uint64_t squared_acc = 0;
	uint16_t rms_value, adc_raw;
	uint8_t gt_zero_count = 0, lt_zero_count = 0;
	//uint16_t adc_raw_min = adc_raw_buf[buf_num][0];
	//uint16_t adc_raw_max = adc_raw_min;
	if (buf_num > 3) { return; }
	printf("Buffer %d\r\n", buf_num);
	printf("%3d: ", 0);
	for (int i=0; i<ADC_NUM_DATA; i++) {
		if (count >= 20) {
			count =0;
			printf("\r\n%3d: ", address);
		}
		adc_raw = adc_raw_buf[buf_num][i];
		printf("%04u ", adc_raw);

		squared_acc += adc_raw_buf[buf_num][i] * adc_raw_buf[buf_num][i];
		count++; address++;
	}
	rms_value = (uint16_t) sqrt((squared_acc / ADC_NUM_DATA));
	printf("\r\nMin: %dmV Max: %dmV ", calc_adc_raw_to_mv_int(adc_raw_meta[buf_num].min), calc_adc_raw_to_mv_int(adc_raw_meta[buf_num].max) );
	printf("RMS: %dmV [%u]\r\n", calc_adc_raw_to_mv_int(rms_value), rms_value);

}

void calc_csv_buffer(uint8_t buf_num) {
	if (buf_num > 3) { return; }
	printf("Buffer %d\r\n", buf_num);
	for (int i=0; i<ADC_NUM_DATA; i++) {
		printf("%d,%u\r\n", i, adc_raw_buf[buf_num][i]);
	}
	printf("\r\n\r\n");
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
