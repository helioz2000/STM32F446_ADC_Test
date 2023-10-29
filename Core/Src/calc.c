/*
 * calc.c
 *
 *  Created on: Oct 29, 2023
 *      Author: eb
 */

#include <stdint.h>
#include "global.h"
#include "calc.h"

extern uint16_t adc1_dma_buf[];
extern uint16_t adc2_dma_buf[];

uint16_t adc_buf[4][ADC_NUM_DATA];

/*
 * Process the DMA buffer
 * parameter high: > 0 to process 2nd half of buffer, 0 = 1st half of buffer
 * parameter adc_num: 1 = ADC1, 2 = ADC2
 */
void calc_process_dma_buffer(int high, int adc_num) {
	uint16_t i;
	uint16_t dma_bufsize = ADC_BUF_SIZE;
	uint16_t *dma_bufptr;
	uint16_t *ch0_bufptr;
	uint16_t *ch1_bufptr;
	if (adc_num == 1) {
		if (high == 0) {
			dma_bufptr = adc1_dma_buf;	// 1st half
		} else {
			dma_bufptr = adc1_dma_buf + (ADC_DMA_BUF_SIZE/2);	// 2nd half
		}
		ch0_bufptr = adc_buf[0];
		ch1_bufptr = adc_buf[1];
	} else {
		if (high == 0) {
			dma_bufptr = adc2_dma_buf;	// 1st half
		} else {
			dma_bufptr = adc2_dma_buf + (ADC_DMA_BUF_SIZE/2);	// 2nd half
		}
		ch0_bufptr = adc_buf[2];
		ch1_bufptr = adc_buf[3];
	}
	// split DMA buffer into 2 channels
	for (i=0; i<dma_bufsize; dma_bufsize+=2) {
		*ch0_bufptr++ = dma_bufptr[i];
		*ch1_bufptr++ = dma_bufptr[i+1];
	}
}

void calc_display_buffer(uint8_t buf_num) {
	int count = 0;
	if (buf_num > 3) { return; }
	for (int i=0; i<ADC_NUM_DATA; i++) {
		printf("%04u ", adc_buf[buf_num][i]);
		count++;
		if (count > 10) {
			count =0;
			printf("\r\n");
		}
	}


}
