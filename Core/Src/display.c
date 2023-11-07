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

uint16_t channel_colour[4] = { YELLOW, CYAN, GREEN, ORANGE};
uint16_t curve_y[DISPLAY_X];	// store the curve before drawing, enables overwrite on next curve
uint16_t curve_len = 0;

void draw_curve(uint16_t colour) {
	for (int x=1; x<curve_len; x++) {
		Displ_Line(x-1, curve_y[x-1], x, curve_y[x], colour);
	}
}

/*
 * Show a ADC channel curve on TFT display
 * parameter bufnum: adc_raw_buf index to ADC channel
 * Disp_CLS() takes a long time to run so we draw the previous curve in black to remove it
 */
void display_show_curve(uint8_t bufnum) {
	if ( (bufnum >= AADC_NUM_BUFFERS) || (bufnum < 0) ) return;	// buffer range check
	//Displ_BackLight('1');

	float fScale = (float)DISPLAY_Y / (float)adc_raw_meta[bufnum].max;
	int scale_factor = 1;

	int y_offset = 0;
	int value = 0;
	int buf_index = 1;

	if (curve_len) {
		// Delete previous curve
		draw_curve(BLACK);
	} else {
		Displ_CLS(BLACK);
		curve_len = 0;
	}

	// calculate the new curve
	// set multiplier and divider to ensure the function can handle a wide range of values
	if (fScale < 1) {
		scale_factor = trunc(1/fScale)+1; // divisor
		curve_y[0] = DISPLAY_Y - (((adc_raw_buf[bufnum][0] + adc_raw_buf[bufnum][1]) / 2) / scale_factor + y_offset) ;
		for (int pos_x=1; pos_x < ADC_NUM_DATA / 2; pos_x++) {
			// calculate reading value by averaging 3 readings (the one before and the one after)
			value = (adc_raw_buf[bufnum][buf_index] + adc_raw_buf[bufnum][buf_index-1] + adc_raw_buf[bufnum][buf_index+1]) / 3;
			// calculate reading pixel on display using the scale value
			curve_y[pos_x] = DISPLAY_Y - (value / scale_factor + y_offset);
			// advance readings buffer by twice
			buf_index+=2;
		}
	} else {
		scale_factor = trunc(fScale);	// multiplier
		// calculate start of first line
		curve_y[0] = DISPLAY_Y - (((adc_raw_buf[bufnum][0] + adc_raw_buf[bufnum][1]) / 2) * scale_factor + y_offset) ;
		for (int pos_x=1; pos_x < ADC_NUM_DATA / 2; pos_x++) {
			// calculate reading value by averaging 3 readings (the one before and the one after)
			value = (adc_raw_buf[bufnum][buf_index] + adc_raw_buf[bufnum][buf_index-1] + adc_raw_buf[bufnum][buf_index+1]) / 3;
			// calculate reading pixel on display using the scale value
			curve_y[pos_x] = DISPLAY_Y - value * scale_factor + y_offset;
			// advance readings buffer by twice
			buf_index+=2;
		}
	}

	curve_len = ADC_NUM_DATA / 2;
	draw_curve(channel_colour[bufnum]);
}
