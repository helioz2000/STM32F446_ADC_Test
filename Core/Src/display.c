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
extern float metervalue_v, metervalue_i, metervalue_va, metervalue_w, metervalue_pf;
extern uint8_t meter_readings_invalid;

uint8_t display_channel = 0;		// active display channel I1, I2, I3
uint16_t channel_colour[4] = { GREEN, BLUE, CYAN, ORANGE};
int curve_y[ADC_NUM_DATA/4];		// store the curve before drawing, enables overwrite on next curve
uint16_t curve_y_zero;		// zero line of curve
uint16_t curve_y_size = 60;
uint16_t curve_x_zero = 15;
uint16_t curve_x_size = 0;
const uint16_t graph_border = 2;
uint16_t aligned_curve[ADC_NUM_BUFFERS][800];	// raw samples reduced to one full cycle (around 400 samples)
char str[32];
uint8_t last_screen = 0;
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

// channel detail screen
void display_channel_detail() {
	if (!meter_readings_invalid) {
		// V
		snprintf(str,sizeof(str),"%3.0f", metervalue_v);
		Displ_WString(9, 7, str , Font30, 1, GREEN, BLACK);
		// I
		snprintf(str,sizeof(str),"%4.1f", metervalue_i);
		Displ_WString(120, 7, str , Font30, 1, ORANGE, BLACK);
		// VA
		snprintf(str,sizeof(str),"%7.1f", metervalue_va );
		Displ_WString(9, 48, str , Font30, 1, YELLOW, BLACK);
		// W
		snprintf(str,sizeof(str),"%7.1f", metervalue_w );
		Displ_WString(9, 89, str , Font30, 1, YELLOW, BLACK);
		// PF
		snprintf(str,sizeof(str),"%4.2f", fabs(metervalue_pf) );
		Displ_WString(9, 130, str , Font30, 1, WHITE, BLACK);

		// Angle
		/* if (metervalue_i >= I1_MIN_PF) {
			if (metervalue_pf < 0) {
				snprintf(str,sizeof(str),"%4.0f", acos(fabs(metervalue_pf)) * (180.0 / 3.14159265) );
			} else {
				snprintf(str,sizeof(str),"%4.1f", acos(fabs(metervalue_pf)) * (180.0 / 3.14159265) );
			}
		} else {
			snprintf(str,sizeof(str)," 0.0");
		}
		Displ_WString(138, 130, str , Font30, 1, WHITE, BLACK);
		*/
		display_show_curves();
	} else {		// display for invalid measurements
		Displ_WString(9, 7, "---" , Font30, 1, GREEN, BLACK);
		Displ_WString(120, 7, "--.-" , Font30, 1, ORANGE, BLACK);
		Displ_WString(9, 48, "-----.-" , Font30, 1, YELLOW, BLACK);
		Displ_WString(9, 89, "-----.-" , Font30, 1, YELLOW, BLACK);
		Displ_WString(9, 130, "-.--" , Font30, 1, WHITE, BLACK);
		//Displ_WString(138, 130, "--.-" , Font30, 1, WHITE, BLACK);
		}

}

void display_usage() {

}

void display_update_meter(uint8_t screen) {

	// detect screen number change
	if (screen != last_screen) {
		switch(screen) {
		case 1:
			meter_readings_invalid = 1;
			display_channel = I1;
			display_meter_mask();
			break;
		case 2:
			meter_readings_invalid = 1;
			display_channel = I2;
			display_meter_mask();
			break;
		case 3:
			meter_readings_invalid = 1;
			display_channel = I3;
			display_meter_mask();
			break;
		case 4:
			Displ_CLS(BLACK);
			Displ_WString(20, 20, "Usage" , Font24, 1, WHITE, BLACK);
			break;
		}
		last_screen = screen;
	}

	// update screen contents
	switch(screen) {
	case 1:
	case 2:
	case 3:
		display_channel_detail();
		break;
	case 4:
		display_usage();
		break;

	}
}

/*
 * force mask update on next meter update
 */
void display_update_mask(void) {
	last_screen = 0;
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
	// Displ_WChar(display_x-22, 130, 0x60, Font30, 1, font_col, back_col);		// Degree sign

	// Graph Box
	ypos += box_height+1;
	border_col = BLUE;
	curve_y_size = display_y-ypos-graph_border*2;
	curve_y_zero = ypos + curve_y_size / 2 + graph_border;
	Displ_Border(0,ypos,display_x,display_y-ypos, graph_border, border_col);
	Displ_Line(curve_x_zero, curve_y_zero, curve_x_zero+210, curve_y_zero, WHITE);

	// Channel
	snprintf(str,sizeof(str),"Ch%d", display_channel+1 );
	Displ_WString(150, 135, str , Font24, 1, WHITE, BLACK);
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
 * parameter centre_zero: set to 1 to draw curve around a centered zero line, otherwise from bottom
 * The curve display area is cleared and the zero line is drawn.
 * Each point in the curve_y array is drawn as a line to the previous point
 */
void draw_curve(uint16_t colour, uint8_t dont_clear, uint8_t centre_zero) {

	if ((! dont_clear) || (meter_readings_invalid)) {
		// first clear the curve area
		Displ_FillArea(2,display_y-curve_y_size-2,display_x-graph_border*2,curve_y_size, BLACK);
	}
	// draw curve border
	//Displ_Border(0,display_y-curve_y_size,display_x-1,curve_y_size, graph_border, BLUE);

	// Don't draw curve for invalid meter reading
	if (meter_readings_invalid) {
		//Displ_FillArea(2,display_y-curve_y_size-2,display_x-graph_border*2,curve_y_size, BLACK);
		Displ_WString(curve_x_zero+Font24.Width, curve_y_zero - Font24.Height / 2 , "Low Voltage", Font24, 1, YELLOW, RED);
		return;
	}

	if (centre_zero) {
		// draw zero line
		Displ_Line(curve_x_zero, curve_y_zero, curve_x_zero+curve_x_size, curve_y_zero, WHITE);
		// draw the curve
		for (int x=1; x<curve_x_size; x++) {
			Displ_Line(x + curve_x_zero-1, curve_y_zero - curve_y[x-1], x + curve_x_zero, curve_y_zero - curve_y[x], colour);
		}
	} else {	// display for non-AC wave forms
		for (int x=1; x<curve_x_size; x++) {
			Displ_Line(x + curve_x_zero-1, curve_y_zero - (curve_y[x-1]), x + curve_x_zero, curve_y_zero - (curve_y[x]), colour);
		}
	}
}

/*
 * @brief   Makes the curve data by performing two steps
 *          1) Two adjacent data points are averaged to half the number of data points
 *          2) The data points are raw ADC values ranging between 0 and 4095. The curve data
 *             represents plus/minus values centered around the half way point of the data range.
 *          The (+/-)curve points are scaled to fit the vertical resolution of the graph.
 * @ para bufnum      Sample buffer to use for curve
 * @ para auto_scale  1 = scale to fit available screen area, 0 = use ADC full scale
 */
void make_curve(uint8_t bufnum, uint8_t auto_scale) {
	int value;
	int scale_factor = 1;
	int src_idx = 0;
	int range = sample_buf_meta[bufnum].max - sample_buf_meta[bufnum].min;
	int zero_value = range / 2 + sample_buf_meta[bufnum].min;	// zero should be half way if DC-Bias is accurate
	int curve_y_min;
	float fScale;

	if (auto_scale) {
		fScale = (float)curve_y_size / (float)range;
	} else {
		fScale = (float)curve_y_size / ADC_FS_RAW;
	}
	if (fScale < 1) {
		scale_factor = trunc(1/fScale)+1; // divisor
	} else {
		scale_factor = trunc(fScale);	// multiplier
	}

	curve_x_size = display_align_curves() / 2;		// half the data points to fit screen size
	curve_y_min = 0 - (curve_y_size / 2 -1) ;	// limit y negative points to keep curve within area

	// calculate the new curve as +- values around the centre
	// set multiplier and divider to ensure the function can handle a wide range of values
	if (fScale < 1) {
		// calculate start of first line
		value = (aligned_curve[bufnum][src_idx] + aligned_curve[bufnum][src_idx+1]) / 2;
		//curve_y[0] = (value  - zero_value) / scale_factor;
		curve_y[0] = MAX((value - zero_value) / scale_factor, curve_y_min);
		for (int pos_x=1; pos_x < curve_x_size; pos_x++) {
			src_idx+=2;
			value = (aligned_curve[bufnum][src_idx] + aligned_curve[bufnum][src_idx+1]) / 2;
			// calculate reading pixel on display using the scale value
			curve_y[pos_x] = MAX((value - zero_value) / scale_factor, curve_y_min);
		}
	} else {
		// calculate start of first line
		value = (aligned_curve[bufnum][src_idx] + aligned_curve[bufnum][src_idx+1]) / 2;
		curve_y[0] = MAX((value - zero_value) * scale_factor, curve_y_min);
		for (int pos_x=1; pos_x < curve_x_size; pos_x++) {
			src_idx+=2;
			value = (aligned_curve[bufnum][src_idx] + aligned_curve[bufnum][src_idx+1]) / 2;
			// calculate reading pixel on display using the scale value
			curve_y[pos_x] = MAX((value - zero_value) * scale_factor, curve_y_min);
		}
	}
}

/*
 * @brief   Display curves for voltage + selected current channels on TFT display
 */
void display_show_curves(void) {

	make_curve(ADC_CH_V, 0);
	draw_curve(channel_colour[ADC_CH_V], 0, (sample_buf_meta[ADC_CH_V].zero_cross_pos >= 0));

	switch(display_channel) {
	case I1:
		if (!sample_buf_meta[ADC_CH_I1].value_is_zero) {
			make_curve(ADC_CH_I1, 1);
			draw_curve(channel_colour[ADC_CH_I1], 1, (sample_buf_meta[ADC_CH_I1].zero_cross_pos >= 0));
		}
		break;
	case I2:
		if ((NUM_I_SENSORS > 1)&&(!sample_buf_meta[ADC_CH_I2].value_is_zero)) {
			make_curve(ADC_CH_I2, 1);
			draw_curve(channel_colour[ADC_CH_I2], 1, (sample_buf_meta[ADC_CH_I2].zero_cross_pos >= 0));
		}
		break;
	case I3:
		if ((NUM_I_SENSORS > 2)&&(!sample_buf_meta[ADC_CH_I3].value_is_zero)) {
			make_curve(ADC_CH_I3, 1);
			draw_curve(channel_colour[ADC_CH_I3], 1, (sample_buf_meta[ADC_CH_I3].zero_cross_pos >= 0));
		}
		break;
	}
}

/*
 * Show a ADC channel curve on TFT display
 * parameter bufnum: adc_raw_buf index to ADC channel
 * Disp_CLS() takes a long time to run so we draw the previous curve in black to remove it
 */
void display_show_curve(uint8_t bufnum) {
	if ( (bufnum >= ADC_NUM_BUFFERS) || (bufnum < 0) ) return;	// buffer range check

	make_curve(bufnum, 1);
	draw_curve(channel_colour[bufnum], 0, (sample_buf_meta[bufnum].measurements_valid != 0));

}
