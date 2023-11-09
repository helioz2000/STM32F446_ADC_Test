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
#include "calc.h"

#define ADC_MAX 4095

extern uint16_t adc_raw_buf[ADC_NUM_BUFFERS][ADC_NUM_DATA];		// buffer for 4 channels of raw ADC data
extern uint16_t sample_buf[ADC_NUM_BUFFERS][SAMPLE_BUF_SIZE];			// buffer for 4 channels of downsampled data
extern struct sampleBufMeta sample_buf_meta[];

uint16_t channel_colour[4] = { YELLOW, CYAN, GREEN, ORANGE};
uint16_t curve_y[DISPLAY_X];	// store the curve before drawing, enables overwrite on next curve
uint16_t curve_len = 0;
char str[32];
uint8_t lastbuf = 9;

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
	if ( (bufnum >= ADC_NUM_BUFFERS) || (bufnum < 0) ) return;	// buffer range check

	int y_offset = 0;
	int y_max = DISPLAY_Y -1;		// max Y pixel position
	int x_max = ADC_NUM_DATA / 2;	// we have twice as many readings as pixels on the display
	int value = 0;
	int buf_index = 1;
	int y_grid_100 = 0;
	int y_grid_50 = 0;
	int y_grid_25 = 0;

	int scale_factor = 1;
	float fScale = (float)DISPLAY_Y / (float)sample_buf_meta[bufnum].max;
	if (fScale < 1) {
		scale_factor = trunc(1/fScale)+1; // divisor
		value = sample_buf_meta[bufnum].max / scale_factor;
	} else {
		scale_factor = trunc(fScale);	// multiplier
		value = sample_buf_meta[bufnum].max * scale_factor;
	}
	y_grid_100 = y_max - value + y_offset;
	y_grid_50 = y_max - value/2 + y_offset;
	y_grid_25 = y_max - value/4 + y_offset;

	/*if ((curve_len) && (bufnum == lastbuf)) {
		// Delete previous curve
		draw_curve(BLACK);
	} else {
		Displ_CLS(BLACK);
		curve_len = 0;
	}*/

	Displ_CLS(BLACK);

	// Draw grid lines
	Displ_Line(0, y_grid_100, x_max, y_grid_100, WHITE);	// 100%
	Displ_Line(0, y_grid_50, x_max, y_grid_50, WHITE);	// 50%
	Displ_Line(0, DISPLAY_Y-1, x_max, DISPLAY_Y-1, WHITE);	// Zero

	// Display grid values
	snprintf(str,32,"%d",calc_adc_raw_to_mv_int(sample_buf_meta[bufnum].max));
	value = Font16.Width * strlen(str);
	Displ_WString(x_max, y_grid_100, str , Font16, 1, BLACK, WHITE);
	snprintf(str,32,"%d",calc_adc_raw_to_mv_int(sample_buf_meta[bufnum].max/2));
	value = Font16.Width * strlen(str);
	Displ_WString(x_max, y_grid_50-Font16.Height/2, str , Font16, 1, BLACK, WHITE);
	Displ_WString(x_max, DISPLAY_Y-Font16.Height, "0" , Font16, 1, BLACK, WHITE);

	Displ_WString(x_max, y_grid_25 - Font20.Height/2, "mV" , Font20, 1, WHITE, BLACK);

	// calculate the new curve
	// set multiplier and divider to ensure the function can handle a wide range of values
	if (fScale < 1) {
		// calculate start of first line
		curve_y[0] = y_max - (sample_buf[bufnum][0] / scale_factor + y_offset) ;
		for (int pos_x=1; pos_x < SAMPLE_BUF_SIZE; pos_x++) {
			// calculate reading pixel on display using the scale value
			curve_y[pos_x] = y_max - (sample_buf[bufnum][pos_x] / scale_factor + y_offset);
			// advance readings buffer by twice
			buf_index+=2;
		}
	} else {
		// calculate start of first line
		curve_y[0] = y_max - (sample_buf[bufnum][0] * scale_factor + y_offset) ;
		for (int pos_x=1; pos_x < SAMPLE_BUF_SIZE; pos_x++) {
			// calculate reading pixel on display using the scale value
			curve_y[pos_x] = y_max - sample_buf[bufnum][pos_x] * scale_factor + y_offset;
			// advance readings buffer by twice
			buf_index+=2;
		}
	}

	curve_len = SAMPLE_BUF_SIZE;
	draw_curve(channel_colour[bufnum]);
	lastbuf = bufnum;
}
