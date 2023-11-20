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
uint16_t curve_y[DISPLAY_X];		// store the curve before drawing, enables overwrite on next curve
uint16_t aligned_curve[ADC_NUM_BUFFERS][DISPLAY_X];	// raw samples reduced to one full cycle (around 400 samples)
uint16_t curve_len = 0;
char str[32];
uint8_t lastbuf = 9;

void display_start_screen() {
	// Draw initial TFT Display
	  Displ_CLS(BLACK);			// after initialization (above) and before turning on backlight (below), you can draw the initial display appearance.
	  Displ_Line(0, 0, 20, 0, BLUE);
	  Displ_Line(0, 0, 0, 20, BLUE);
	  //Displ_Line(0, 140, 240, 140, RED);
	  Displ_WString(10, 10, "10,10" , Font12, 1, BLUE, WHITE);
	  Displ_Line(DISPLAY_X-1, 0, DISPLAY_X-1, 20, RED);
	  Displ_Line(DISPLAY_X-1, 0, DISPLAY_X-1-20, 0, RED);
	  snprintf(str,32,"%d,%d",DISPLAY_X-10, 10);
	  Displ_WString(DISPLAY_X-1-10-strlen(str)*Font12.Width, 10, str, Font12, 1, RED, WHITE);

	  //Displ_Line(0, 0, 0, 20, BLUE);
	  Displ_Line(0, DISPLAY_Y-1, 0, DISPLAY_Y-1-20, GREEN);
	  Displ_Line(0, DISPLAY_Y-1, 20, DISPLAY_Y-1, GREEN);
	  snprintf(str,32,"%d,%d",10,DISPLAY_Y-10);
	  Displ_WString(10, DISPLAY_Y-10-Font12.Height, str, Font12, 1, BLACK, WHITE);

	  Displ_Line(DISPLAY_X-1, DISPLAY_Y-1, DISPLAY_X-1-20, DISPLAY_Y-1, YELLOW);
	  Displ_Line(DISPLAY_X-1, DISPLAY_Y-1, DISPLAY_X-1, DISPLAY_Y-1-20, YELLOW);
	  snprintf(str,32,"%d,%d",DISPLAY_X-10,DISPLAY_Y-10);
	  Displ_WString(DISPLAY_X-1-10-strlen(str)*Font12.Width, DISPLAY_Y-10-Font12.Height, str, Font12, 1, BLACK, WHITE);

	  //Displ_WString(380, 10, "380,10" , Font12, 1, RED, WHITE);
	  //Displ_WString(10, 300, "10,300" , Font12, 1, RED, WHITE);
	  Displ_BackLight('1');
}

void draw_curve(uint16_t colour) {
	for (int x=1; x<curve_len; x++) {
		Displ_Line(x-1, curve_y[x-1], x, curve_y[x], colour);
	}
}

/*
 * Aligns curve to start at positive zero crossing and reduces it to cuts one full cycle
 * parameter bufnum: buffer to use for curve display
 * returns: number of points in the aligned curve or -1 if alignment failed
 */
int display_align_curves() {
	int dest_idx = -1;
	int i, continue_at = -1;

	// if we don't have a zero crossing use the sample_buf curve entries
	if (sample_buf_meta[ADC_CH_V].zero_cross_pos < 0) {
		for (i=0; i<SAMPLE_BUF_SIZE; i++ ) {
			aligned_curve[ADC_CH_V][i] = sample_buf[ADC_CH_V][i];
			aligned_curve[ADC_CH_I1][i] = sample_buf[ADC_CH_I1][i];
			aligned_curve[ADC_CH_I2][i] = sample_buf[ADC_CH_I2][i];
			aligned_curve[ADC_CH_I3][i] = sample_buf[ADC_CH_I3][i];
		}
		return SAMPLE_BUF_SIZE;
	}

	// start at zero crossing and use every reading up to the end of the sample buffer
	for (i=sample_buf_meta[ADC_CH_V].zero_cross_pos; i<SAMPLE_BUF_SIZE; i++ ) {
		dest_idx++;
		aligned_curve[ADC_CH_V][dest_idx] = sample_buf[ADC_CH_V][i];
		aligned_curve[ADC_CH_I1][dest_idx] = sample_buf[ADC_CH_I1][i];
		aligned_curve[ADC_CH_I2][dest_idx] = sample_buf[ADC_CH_I2][i];
		aligned_curve[ADC_CH_I3][dest_idx] = sample_buf[ADC_CH_I3][i];
	}

	/*
	// search for the closest value the last point of the curve
	for (i=0; i<SAMPLE_BUF_SIZE; i++) {
		distance = abs(aligned_curve[dest_idx] - sample_buf[bufnum][i]);
		if (distance < last_distance) {
			last_distance = distance;
		} else {
			if (i>17 && i<23) {	// should be exactly 20
				continue_at = i;
				break;	// for loop
			}
		}
	}
	*/

	continue_at = SAMPLE_BUF_OVERLAP;

	for (i=continue_at; i<sample_buf_meta[ADC_CH_V].zero_cross_pos; i++) {
		dest_idx++;
		aligned_curve[ADC_CH_V][dest_idx] = sample_buf[ADC_CH_V][i];
		aligned_curve[ADC_CH_I1][dest_idx] = sample_buf[ADC_CH_I1][i];
		aligned_curve[ADC_CH_I2][dest_idx] = sample_buf[ADC_CH_I2][i];
		aligned_curve[ADC_CH_I3][dest_idx] = sample_buf[ADC_CH_I3][i];
	}
	return ++dest_idx;
}

void display_show_curves(void) {

	int y_offset = 0;
	int y_max = DISPLAY_Y -1;		// max Y pixel position
	int x_max = ADC_NUM_DATA / 2;	// we have twice as many readings as pixels on the display
	/*int value = 0;
	int y_grid_100 = 0;
	int y_grid_50 = 0;
	int y_grid_25 = 0;
*/
	int scale_factor = 1;
	float fScale;

	Displ_CLS(BLACK);
	// zero line
	Displ_Line(0, DISPLAY_Y/2, x_max, DISPLAY_Y/2, WHITE);

	/*
	y_grid_100 = y_max - value + y_offset;
	y_grid_50 = y_max - value/2 + y_offset;
	y_grid_25 = y_max - value/4 + y_offset;



	// Draw grid lines
	Displ_Line(0, y_grid_100, x_max, y_grid_100, WHITE);	// 100%
	Displ_Line(0, y_grid_50, x_max, y_grid_50, WHITE);	// 50%
	Displ_Line(0, DISPLAY_Y-1, x_max, DISPLAY_Y-1, WHITE);	// Zero

	// Display grid values
	snprintf(str,32,"%d",calc_adc_raw_to_mv_int(sample_buf_meta[ADC_CH_V].max));
	value = Font16.Width * strlen(str);
	Displ_WString(x_max, y_grid_100, str , Font16, 1, BLACK, WHITE);
	snprintf(str,32,"%d",calc_adc_raw_to_mv_int(sample_buf_meta[ADC_CH_V].max/2));
	value = Font16.Width * strlen(str);
	Displ_WString(x_max, y_grid_50-Font16.Height/2, str , Font16, 1, BLACK, WHITE);
	Displ_WString(x_max, DISPLAY_Y-Font16.Height, "0" , Font16, 1, BLACK, WHITE);

	Displ_WString(x_max, y_grid_25 - Font20.Height/2, "mV" , Font20, 1, WHITE, BLACK);
*/

	curve_len = display_align_curves();


	for (int i=0; i<4; i++) {
		fScale = (float)DISPLAY_Y / (float)sample_buf_meta[i].max;
		if (fScale < 1) {
			scale_factor = trunc(1/fScale)+1; // divisor
			//value = sample_buf_meta[i].max / scale_factor;
		} else {
			scale_factor = trunc(fScale);	// multiplier
			//value = sample_buf_meta[i].max * scale_factor;
		}

		// calculate the new curve
		// set multiplier and divider to ensure the function can handle a wide range of values
		if (fScale < 1) {
			// calculate start of first line
			curve_y[0] = y_max - (aligned_curve[i][0] / scale_factor + y_offset) ;
			for (int pos_x=1; pos_x < curve_len; pos_x++) {
				// calculate reading pixel on display using the scale value
				curve_y[pos_x] = y_max - (aligned_curve[i][pos_x] / scale_factor + y_offset);
			}
		} else {
			// calculate start of first line
			curve_y[0] = y_max - (aligned_curve[i][0] * scale_factor + y_offset) ;
			for (int pos_x=1; pos_x < curve_len; pos_x++) {
				// calculate reading pixel on display using the scale value
				curve_y[pos_x] = y_max - aligned_curve[i][pos_x] * scale_factor + y_offset;
			}
		}

		draw_curve(channel_colour[i]);
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
	//int buf_index = 1;
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

	curve_len = display_align_curves();

	// calculate the new curve
	// set multiplier and divider to ensure the function can handle a wide range of values
	if (fScale < 1) {
		// calculate start of first line
		curve_y[0] = y_max - (aligned_curve[bufnum][0] / scale_factor + y_offset) ;
		for (int pos_x=1; pos_x < curve_len; pos_x++) {
			// calculate reading pixel on display using the scale value
			curve_y[pos_x] = y_max - (aligned_curve[bufnum][pos_x] / scale_factor + y_offset);
		}
	} else {
		// calculate start of first line
		curve_y[0] = y_max - (aligned_curve[bufnum][0] * scale_factor + y_offset) ;
		for (int pos_x=1; pos_x < curve_len; pos_x++) {
			// calculate reading pixel on display using the scale value
			curve_y[pos_x] = y_max - aligned_curve[bufnum][pos_x] * scale_factor + y_offset;
		}
	}

	draw_curve(channel_colour[bufnum]);

	lastbuf = bufnum;
}
