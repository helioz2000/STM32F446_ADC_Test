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
#include "term.h"
#include "display.h"

extern uint16_t adc_raw_buf[ADC_NUM_BUFFERS][ADC_NUM_DATA];		// buffer for 4 channels of raw ADC data
extern uint16_t sample_buf[ADC_NUM_BUFFERS][SAMPLE_BUF_SIZE];			// buffer for 4 channels of downsampled data
extern struct sampleBufMeta sample_buf_meta[];
extern char product_msg[];
extern char copyright_msg[];
extern float metervalue_v, metervalue_i1, metervalue_va1, metervalue_w1, metervalue_pf1;
extern uint8_t meter_readings_invalid;

uint16_t channel_colour[4] = { GREEN, ORANGE, CYAN, BLUE};
int curve_y[ADC_NUM_DATA/4];		// store the curve before drawing, enables overwrite on next curve
uint16_t curve_y_zero;		// zero line of curve
uint16_t curve_y_size = 80;
uint16_t curve_x_zero = 15;
const uint16_t graph_border = 2;
uint16_t aligned_curve[ADC_NUM_BUFFERS][800];	// raw samples reduced to one full cycle (around 400 samples)
uint16_t curve_len = 0;
char str[32];
uint8_t lastbuf = 9;
const Displ_Orientat_e display_orientation = Displ_Orientat_180;
uint16_t display_x, display_y;


void display_init() {
	Displ_Init(display_orientation); // initialize the display and set the initial display orientation
	if ((display_orientation == Displ_Orientat_90) || (display_orientation == Displ_Orientat_270)) {	// Horizontal orientation
	  display_x = DISPL_HEIGHT;	// 480 or 320
	  display_y = DISPL_WIDTH;	// 320 or 240
	} else {		// vertical orientation
		display_y = DISPL_HEIGHT;	// 480 or 320
		display_x = DISPL_WIDTH;	// 320 or 240
	}
}

void display_splash_screen() {
	// Draw initial TFT Display
	Displ_CLS(BLACK);
	snprintf(str,32, "%s V%d.%02d",  product_msg ,VERSION_MAJOR, VERSION_MINOR);
	Displ_WString((display_x-(strlen(str)*Font24.Width))/2, (display_y-Font24.Height)/2, str, Font24, 1, YELLOW, BLACK);
	Displ_WString((display_x-(strlen(copyright_msg)*Font12.Width))/2, (display_y-Font24.Height)/2+25, copyright_msg, Font12, 1, WHITE, BLACK);

	Displ_BackLight('1');
}

void display_corners() {
	Displ_CLS(BLACK);
	Displ_Line(0, 0, 20, 0, BLUE);
	Displ_Line(0, 0, 0, 20, BLUE);
	Displ_WString(10, 10, "10,10" , Font12, 1, BLUE, WHITE);

	Displ_Line(display_x-1, 0, display_x-1, 20, RED);
	Displ_Line(display_x-1, 0, display_x-1-20, 0, RED);
	snprintf(str,32,"%d,%d",display_x-10, 10);
	Displ_WString(display_x-1-10-strlen(str)*Font12.Width, 10, str, Font12, 1, RED, WHITE);

	Displ_Line(0, display_y-1, 0, display_y-1-20, GREEN);
	Displ_Line(0, display_y-1, 20, display_y-1, GREEN);
	snprintf(str,32,"%d,%d",10,display_y-10);
	Displ_WString(10, display_y-10-Font12.Height, str, Font12, 1, BLACK, WHITE);

	Displ_Line(display_x-1, display_y-1, display_x-1-20, display_y-1, YELLOW);
	Displ_Line(display_x-1, display_y-1, display_x-1, display_y-1-20, YELLOW);
	snprintf(str,32,"%d,%d",display_x-10,display_y-10);
	Displ_WString(display_x-1-10-strlen(str)*Font12.Width, display_y-10-Font12.Height, str, Font12, 1, BLACK, WHITE);
}



void display_update_meter() {

	if (!meter_readings_invalid) {
		// V
		snprintf(str,sizeof(str),"%3.0f", metervalue_v);
		Displ_WString(9, 7, str , Font30, 1, GREEN, BLACK);
		// I
		snprintf(str,sizeof(str),"%4.1f", metervalue_i1);
		Displ_WString(120, 7, str , Font30, 1, ORANGE, BLACK);
		// VA
		snprintf(str,sizeof(str),"%7.1f", metervalue_va1 );
		Displ_WString(9, 48, str , Font30, 1, YELLOW, BLACK);
		// W
		snprintf(str,sizeof(str),"%7.1f", metervalue_w1 );
		Displ_WString(9, 89, str , Font30, 1, YELLOW, BLACK);
		// PF
		snprintf(str,sizeof(str),"%4.2f", metervalue_pf1 );
		Displ_WString(9, 130, str , Font30, 1, WHITE, BLACK);
		// Angle
		if (metervalue_pf1 < 0) {
			snprintf(str,sizeof(str),"%4.0f", acos(metervalue_pf1) * (180.0 / 3.14159265) );
		} else {
			snprintf(str,sizeof(str),"%4.1f", acos(metervalue_pf1) * (180.0 / 3.14159265) );
		}
		Displ_WString(138, 130, str , Font30, 1, WHITE, BLACK);
	} else {		// display for invalid measurements
		Displ_WString(9, 7, "---" , Font30, 1, GREEN, BLACK);
		Displ_WString(120, 7, "--.-" , Font30, 1, GREEN, BLACK);
		Displ_WString(9, 48, "-----.-" , Font30, 1, YELLOW, BLACK);
		Displ_WString(9, 89, "-----.-" , Font30, 1, YELLOW, BLACK);
		Displ_WString(9, 130, "-.--" , Font30, 1, WHITE, BLACK);
		Displ_WString(138, 130, "--.-" , Font30, 1, WHITE, BLACK);
	}
	display_show_curves();
}

/*
 * Draw the screen mask for meter main screen
 */
void display_meter_mask() {
	uint16_t ypos = 0;
	uint16_t box_height = 40;
	uint16_t border_col = GREEN;
	uint16_t font_col = GREEN;
	uint16_t back_col = BLACK;
	Displ_CLS(back_col);
	// V + A
	Displ_Line(0,ypos,display_x-1,ypos, border_col);
	Displ_Line(0,ypos,0,ypos+box_height, border_col);
	Displ_Line(0,ypos+box_height,display_x-1,ypos+box_height,border_col);
	Displ_Line(display_x-1, ypos+box_height, display_x-1, 0, border_col);
	Displ_WChar(75, ypos+9, 'V', Font24, 1, font_col, back_col);
	font_col = ORANGE;
	Displ_WChar(display_x-30, 9, 'A', Font24, 1, font_col, back_col);
	// VA
	ypos += box_height+1;
	border_col = font_col = YELLOW;
	Displ_Line(0,ypos,display_x-1,ypos, border_col);
	Displ_Line(0,ypos,0,ypos+box_height, border_col);
	Displ_Line(0,ypos+box_height,display_x-1,ypos+box_height,border_col);
	Displ_Line(display_x-1, ypos+box_height, display_x-1, ypos, border_col);
	Displ_WString(display_x-80, ypos+9, "VA", Font24, 1, font_col, back_col);
	// W
	ypos += box_height+1;
	border_col = font_col = YELLOW;
	Displ_Line(0,ypos,display_x-1,ypos, border_col);
	Displ_Line(0,ypos,0,ypos+box_height, border_col);
	Displ_Line(0,ypos+box_height,display_x-1,ypos+box_height,border_col);
	Displ_Line(display_x-1, ypos+box_height, display_x-1, ypos, border_col);
	Displ_WChar(display_x-80, ypos+9, 'W', Font24, 1, font_col, back_col);

	// PF
	ypos += box_height+1;
	border_col = font_col = WHITE;
	Displ_Line(0,ypos,display_x-1,ypos, border_col);
	Displ_Line(0,ypos,0,ypos+box_height, border_col);
	Displ_Line(0,ypos+box_height,display_x-1,ypos+box_height,border_col);
	Displ_Line(display_x-1, ypos+box_height, display_x-1, ypos, border_col);
	Displ_WString(95, ypos+9, "PF", Font24, 1, font_col, back_col);
	Displ_WChar(display_x-22, 130, 0x60, Font30, 1, font_col, back_col);

	// Graph Box
	ypos += box_height+1;
	border_col = BLUE;
	curve_y_size = display_y-ypos-graph_border*2;
	curve_y_zero = ypos + curve_y_size / 2;
	Displ_Border(0,ypos,display_x,display_y-ypos, graph_border, border_col);
	Displ_Line(curve_x_zero, curve_y_zero, curve_x_zero+210, curve_y_zero, WHITE);
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
#if I2_IN_USE
			aligned_curve[ADC_CH_I2][i] = sample_buf[ADC_CH_I2][i];
#endif
#if I3_IN_USE
			aligned_curve[ADC_CH_I3][i] = sample_buf[ADC_CH_I3][i];
#endif
		}
		return SAMPLE_BUF_SIZE;
	}

	// start at zero crossing and use every reading up to the end of the sample buffer
	for (i=sample_buf_meta[ADC_CH_V].zero_cross_pos; i<SAMPLE_BUF_SIZE; i++ ) {
		dest_idx++;
		aligned_curve[ADC_CH_V][dest_idx] = sample_buf[ADC_CH_V][i];
		aligned_curve[ADC_CH_I1][dest_idx] = sample_buf[ADC_CH_I1][i];
#if I2_IN_USE
		aligned_curve[ADC_CH_I2][dest_idx] = sample_buf[ADC_CH_I2][i];
#endif
#if I3_IN_USE
		aligned_curve[ADC_CH_I3][dest_idx] = sample_buf[ADC_CH_I3][i];
#endif
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
#if I2_IN_USE
		aligned_curve[ADC_CH_I2][dest_idx] = sample_buf[ADC_CH_I2][i];
#endif
#if I3_IN_USE
		aligned_curve[ADC_CH_I3][dest_idx] = sample_buf[ADC_CH_I3][i];
#endif
	}

	return ++dest_idx;
}

/*
 * Draw one curve
 * parameter colour: curve colour
 * parameter dont_clear: set to 1 to prevent clearing of the previous curve
 * The curve display area is cleared and the zero line is drawn.
 * Each point in the curve_y array is drawn as a line to the previous point
 */
void draw_curve(uint16_t colour, uint8_t dont_clear) {

	if (! dont_clear) {
		// first clear the curve area
		Displ_FillArea(2,display_y-curve_y_size-2,display_x-graph_border*2,curve_y_size, BLACK);
	}
	// draw curve border
	//Displ_Border(0,display_y-curve_y_size,display_x-1,curve_y_size, graph_border, BLUE);
	// draw zero line
	Displ_Line(curve_x_zero, curve_y_zero, curve_x_zero+curve_len, curve_y_zero, WHITE);
	// draw the curve
	for (int x=1; x<curve_len; x++) {
		Displ_Line(x + curve_x_zero-1, curve_y_zero - curve_y[x-1], x + curve_x_zero, curve_y_zero - curve_y[x], colour);
	}
}

/*
 * Makes the curve data by performing two steps
 * 1) Two adjacent data points are averaged to half the number of data points
 * 2) The data points are raw ADC values ranging between 0 and 4095. The curve data
 * represents plus/minus values centered around the half way point of the data range.
 * The (+/-)curve points are scaled to fit the vertical resolution of the graph.
 */
void make_curve(uint8_t bufnum) {
	int value;
	int scale_factor = 1;
	int src_idx = 0;
	int zero_value = (sample_buf_meta[bufnum].max - sample_buf_meta[bufnum].min) /2;

	//term_print("zero_value = %d\r\n", zero_value);

	float fScale = (float)curve_y_size / (float)sample_buf_meta[bufnum].max;
	if (fScale < 1) {
		scale_factor = trunc(1/fScale)+1; // divisor
		//value = sample_buf_meta[bufnum].max / scale_factor;
	} else {
		scale_factor = trunc(fScale);	// multiplier
		//value = sample_buf_meta[bufnum].max * scale_factor;
	}

	curve_len = display_align_curves() / 2;
	// calculate the new curve as +- values around the centre
	// set multiplier and divider to ensure the function can handle a wide range of values
	if (fScale < 1) {
		// calculate start of first line
		value = (aligned_curve[bufnum][src_idx] + aligned_curve[bufnum][src_idx+1]) / 2;
		curve_y[0] = (value  - zero_value) / scale_factor;
		for (int pos_x=1; pos_x < curve_len; pos_x++) {
			src_idx+=2;
			value = (aligned_curve[bufnum][src_idx] + aligned_curve[bufnum][src_idx+1]) / 2;
			// calculate reading pixel on display using the scale value
			curve_y[pos_x] = (value - zero_value) / scale_factor ;
			//curve_y[pos_x] = y_max - (value / scale_factor) + y_offset;
		}
	} else {
		// calculate start of first line
		value = (aligned_curve[bufnum][src_idx] + aligned_curve[bufnum][src_idx+1]) / 2;
		curve_y[0] = (value - zero_value) * scale_factor;
		for (int pos_x=1; pos_x < curve_len; pos_x++) {
			src_idx+=2;
			value = (aligned_curve[bufnum][src_idx] + aligned_curve[bufnum][src_idx+1]) / 2;
			// calculate reading pixel on display using the scale value
			curve_y[pos_x] = (value - zero_value) * scale_factor ;
			//curve_y[pos_x] = y_max - (value * scale_factor) + y_offset;
		}
	}
}

/*
 * Display curves for all configured channels on TFT display
 */
void display_show_curves(void) {
	uint8_t dont_clear = 0;
	for (int i=0; i<=NUM_I_SENSORS; i++) {
		make_curve(i);
		draw_curve(channel_colour[i], dont_clear);
		dont_clear = 1;
	}
}

/*
 * Show a ADC channel curve on TFT display
 * parameter bufnum: adc_raw_buf index to ADC channel
 * Disp_CLS() takes a long time to run so we draw the previous curve in black to remove it
 */
void display_show_curve(uint8_t bufnum) {
	if ( (bufnum >= ADC_NUM_BUFFERS) || (bufnum < 0) ) return;	// buffer range check

	make_curve(bufnum);
	draw_curve(channel_colour[bufnum], 0);

}
