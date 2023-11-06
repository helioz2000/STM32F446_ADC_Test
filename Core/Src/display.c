/*
 * display.c
 *
 *  Created on: Nov 6, 2023
 *      Author: eb
 *
 *  Handles all display related functions
 *
 *  Note: The ILI9xxx display Y axis (vertical) 0 pixel is at the top of the display
 *  so X:0 Y:0 describes the left had top corner of the display
 */

#include "main.h"
#include "global.h"
#include "math.h"

#define ADC_MAX 4095

extern uint16_t adc_raw_buf[][ADC_NUM_DATA];		// buffer for 4 channels of raw ADC data
extern struct rawBufMeta adc_raw_meta[];

void display_show_curve(uint8_t buf_num) {
	if ( (buf_num >= ADC_NUM*ADC_NUM_CHANNELS) || (buf_num < 0) ) return;	// buffer range check
	Displ_BackLight('1');
	Displ_CLS(BLACK);
	float fScale = (float)DISPLAY_Y / (float)adc_raw_meta[buf_num].max;
	// set multiplier and divider to ensure the function can handle a wide range of values
	int scale_mul = 1, scale_div = 1;
	if (fScale < 1) {
		scale_div = trunc(1/fScale)+1;
	} else {
		scale_mul = trunc(fScale);
	}
	int y_offset = 0;
	int value = 0, pos_y;
	int adc_index = 1;
	int max_x = ADC_NUM_DATA / 2;
	int pos_y_prev = DISPLAY_Y - ((((adc_raw_buf[buf_num][0] + adc_raw_buf[buf_num][1]) / 2) * scale_mul)/scale_div + y_offset) ;
	for (int pos_x=1; pos_x < max_x; pos_x++) {
		value = (adc_raw_buf[buf_num][adc_index] + adc_raw_buf[buf_num][adc_index-1] + adc_raw_buf[buf_num][adc_index+1]) / 3;
		pos_y = DISPLAY_Y - ((value * scale_mul)/scale_div + y_offset);
		adc_index+=2;
		Displ_Line(pos_x - 1, pos_y_prev, pos_x, pos_y, GREEN);
		pos_y_prev = pos_y;
	}



}
