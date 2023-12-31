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
#include "fonts.h"
#ifdef USE_WIFI
#include "wifi.h"
#endif

extern uint16_t adc_raw_buf[ADC_NUM_BUFFERS][ADC_NUM_DATA];		// buffer for 4 channels of raw ADC data
extern uint16_t sample_buf[ADC_NUM_BUFFERS][SAMPLE_BUF_SIZE];			// buffer for 4 channels of downsampled data
extern struct sampleBufMeta sample_buf_meta[];
extern char product_msg[];
extern char copyright_msg[];
extern float v_filtered, i_filtered[], va_filtered[], w_filtered[], pf_filtered[];
extern double total_precision_vah[];
extern double total_precision_wh[];
// Below line controls energy display in Wh and VAh instead of kWh and kVAh
#define DISPLAY_ENERGY_K

#ifdef USE_WIFI
extern char ip_addr_str[];
#endif
#ifdef DEBUG
extern uint32_t calc_ticks, display_ticks;	// execution time measurement
#endif

extern uint8_t meter_readings_invalid;

uint8_t display_channel = 0;		// active display channel I1, I2, I3
uint16_t channel_colour[4] = { GREEN, CYAN, MAGENTA, ORANGE, };	// V, I1, I2, I3
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

void display_meter_mask(uint8_t clear);

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
	uint16_t font_col = channel_colour[display_channel+1];
	if (!meter_readings_invalid) {
		// V
		snprintf(str,sizeof(str),"%3.0f", v_filtered);
		Displ_WString(9, 7, str , Font30, 1, channel_colour[0], BLACK);
		// I
		snprintf(str,sizeof(str),"%4.1f", i_filtered[display_channel]);
		Displ_WString(120, 7, str , Font30, 1, font_col, BLACK);
		// VA
		snprintf(str,sizeof(str),"%7.1f", va_filtered[display_channel] );
		Displ_WString(9, 48, str , Font30, 1,  font_col, BLACK);
		// W
		snprintf(str,sizeof(str),"%7.1f", w_filtered[display_channel] );
		Displ_WString(9, 89, str , Font30, 1,  font_col, BLACK);
		// PF
		snprintf(str,sizeof(str),"%4.2f", fabs(pf_filtered[display_channel]) );
		Displ_WString(9, 130, str , Font30, 1,  font_col, BLACK);

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

/*
 * @brief      Display "Usage" page
 * @para mask  1=display Usage mask, 0=display values
 */
void display_usage(uint8_t mask) {
	int font_col = channel_colour[0];
	int ypos = 10;
	int xpos = 10;
	int line_space = -2;
	int section_space = 7;
	sFONT font = Font20;

	// Create page mask
	if (mask != 0) {
		//xpos += 8*font.Width;
		Displ_WString(xpos+6*font.Width, ypos, "V RMS", font, 1,  font_col, BLACK);
		ypos += section_space+line_space + font.Height;

		for (int i=0; i<NUM_I_SENSORS; i++) {
			font_col = channel_colour[i+1];
			Displ_WString(xpos+6*font.Width , ypos, "A" , font, 1,  font_col, BLACK);
			Displ_WString(xpos+12*font.Width, ypos, "PF" , font, 1,  font_col, BLACK);
			ypos += line_space + font.Height;
			Displ_WString(xpos+10*font.Width, ypos, "W" , font, 1,  font_col, BLACK);
			ypos += line_space + font.Height;
			Displ_WString(xpos+10*font.Width, ypos, "VA" , font, 1,  font_col, BLACK);
			ypos += line_space + font.Height;
#ifdef DISPLAY_ENERGY_K
			Displ_WString(xpos+10*font.Width, ypos, "kWh" , font, 1,  font_col, BLACK);
#else
			Displ_WString(xpos+10*font.Width, ypos, "Wh" , font, 1,  font_col, BLACK);
#endif
			ypos += line_space + font.Height;
#ifdef DISPLAY_ENERGY_K
			Displ_WString(xpos+10*font.Width, ypos, "kVAh" , font, 1,  font_col, BLACK);
#else
			Displ_WString(xpos+10*font.Width, ypos, "VAh" , font, 1,  font_col, BLACK);
#endif
			ypos += section_space+line_space + font.Height;
		}
		return;
	}

	snprintf(str,sizeof(str),"%3.0f", v_filtered);
	Displ_WString(xpos+3*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += section_space+line_space + font.Height;
	font_col = channel_colour[I1+1];
	snprintf(str,sizeof(str),"%4.1f", i_filtered[I1]);
	Displ_WString(xpos+2*font.Width , ypos, str , font, 1,  font_col, BLACK);
	snprintf(str,sizeof(str),"%4.2f", fabs(pf_filtered[I1]) );
	Displ_WString(xpos+8*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
	snprintf(str,sizeof(str),"%7.1f", w_filtered[I1]);
	Displ_WString(xpos+3*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
	snprintf(str,sizeof(str),"%7.1f", va_filtered[I1]);
	Displ_WString(xpos+3*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
#ifdef DISPLAY_ENERGY_K
	snprintf(str,sizeof(str),"%10.1f", total_precision_wh[I1]/1000.0);
#else
	snprintf(str,sizeof(str),"%10.1f", total_precision_wh[I1]);
#endif
	Displ_WString(xpos+0*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
#ifdef DISPLAY_ENERGY_K
	snprintf(str,sizeof(str),"%10.1f", total_precision_vah[I1]/1000.0);
#else
	snprintf(str,sizeof(str),"%10.1f", total_precision_vah[I1]);
#endif
	Displ_WString(xpos+0*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += section_space+line_space + font.Height;


	font_col = channel_colour[I2+1];
	snprintf(str,sizeof(str),"%4.1f", i_filtered[I2]);
	Displ_WString(xpos+2*font.Width , ypos, str , font, 1,  font_col, BLACK);
	snprintf(str,sizeof(str),"%4.2f", fabs(pf_filtered[I2]) );
	Displ_WString(xpos+8*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
	snprintf(str,sizeof(str),"%7.1f", w_filtered[I2]);
	Displ_WString(xpos+3*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
	snprintf(str,sizeof(str),"%7.1f", va_filtered[I2]);
	Displ_WString(xpos+3*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
#ifdef DISPLAY_ENERGY_K
	snprintf(str,sizeof(str),"%10.1f", total_precision_wh[I2]/1000.0);
#else
	snprintf(str,sizeof(str),"%10.1f", total_precision_wh[I2]);
#endif
	Displ_WString(xpos+0*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
#ifdef DISPLAY_ENERGY_K
	snprintf(str,sizeof(str),"%10.1f", total_precision_vah[I2]/1000.0);
#else
	snprintf(str,sizeof(str),"%10.1f", total_precision_vah[I2]);
#endif
	Displ_WString(xpos+0*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += section_space+line_space + font.Height;

	font_col = channel_colour[I3+1];
	snprintf(str,sizeof(str),"%4.1f", i_filtered[I3]);
	Displ_WString(xpos+2*font.Width , ypos, str , font, 1,  font_col, BLACK);
	snprintf(str,sizeof(str),"%4.2f", fabs(pf_filtered[I3]) );
	Displ_WString(xpos+8*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
	snprintf(str,sizeof(str),"%7.1f", w_filtered[I3]);
	Displ_WString(xpos+3*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
	snprintf(str,sizeof(str),"%7.1f", va_filtered[I3]);
	Displ_WString(xpos+3*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
#ifdef DISPLAY_ENERGY_K
	snprintf(str,sizeof(str),"%10.1f", total_precision_wh[I3]/1000.0);
#else
	snprintf(str,sizeof(str),"%10.1f", total_precision_wh[I3]);
#endif
	Displ_WString(xpos+0*font.Width, ypos, str , font, 1,  font_col, BLACK);
	ypos += line_space + font.Height;
#ifdef DISPLAY_ENERGY_K
	snprintf(str,sizeof(str),"%10.1f", total_precision_vah[I3]/1000.0);
#else
	snprintf(str,sizeof(str),"%10.1f", total_precision_vah[I3]);
#endif
	Displ_WString(xpos+0*font.Width, ypos, str , font, 1,  font_col, BLACK);

}

void display_debug() {
	int font_col = WHITE;
#ifdef DEBUG
	snprintf(str,sizeof(str),"Calc: %lums", calc_ticks);
	Displ_WString(20, 20, str , Font24, 1,  font_col, BLACK);
#ifdef USE_WIFI
	Displ_WString(20, 40, ip_addr_str, Font20, 1, font_col, BLACK);
#endif
#endif
}

void display_update_meter(uint8_t screen) {

	// detect screen number change
	if (screen != last_screen) {
		switch(screen) {
		case 1:
			//meter_readings_invalid = 1;
			display_channel = I1;
			display_meter_mask(1);
			break;
		case 2:
			//meter_readings_invalid = 1;
			display_channel = I2;
			display_meter_mask(0);
			break;
		case 3:
			//meter_readings_invalid = 1;
			display_channel = I3;
			display_meter_mask(0);
			break;
		case 4:
			Displ_CLS(BLACK);
			display_usage(1);
			break;
		case 5:
			Displ_CLS(BLACK);
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
		display_usage(0);
		break;
	case 5:
		display_debug();
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
 * @brief       Draw the screen mask for meter main screen
 * @para clear  0 = do not clear display
 */
void display_meter_mask(uint8_t clear) {
	uint16_t ypos = 0;
	uint16_t box_height = 40;
	uint16_t border_col = GREEN;
	uint16_t font_col = GREEN;
	uint16_t back_col = BLACK;
	if (clear) Displ_CLS(back_col);
	// V + A
	Displ_Line(0,ypos,display_x-1,ypos, border_col);
	Displ_Line(0,ypos,0,ypos+box_height, border_col);
	Displ_Line(0,ypos+box_height,display_x-1,ypos+box_height,border_col);
	Displ_Line(display_x-1, ypos+box_height, display_x-1, 0, border_col);
	Displ_WChar(75, ypos+9, 'V', Font24, 1, font_col, back_col);
	font_col = channel_colour[display_channel+1];
	Displ_WChar(display_x-30, 9, 'A', Font24, 1, font_col, back_col);
	// VA
	ypos += box_height+1;
	border_col = YELLOW;
	Displ_Line(0,ypos,display_x-1,ypos, border_col);
	Displ_Line(0,ypos,0,ypos+box_height, border_col);
	Displ_Line(0,ypos+box_height,display_x-1,ypos+box_height,border_col);
	Displ_Line(display_x-1, ypos+box_height, display_x-1, ypos, border_col);
	Displ_WString(display_x-80, ypos+9, "VA", Font24, 1, font_col, back_col);
	// W
	ypos += box_height+1;
	border_col = YELLOW;
	Displ_Line(0,ypos,display_x-1,ypos, border_col);
	Displ_Line(0,ypos,0,ypos+box_height, border_col);
	Displ_Line(0,ypos+box_height,display_x-1,ypos+box_height,border_col);
	Displ_Line(display_x-1, ypos+box_height, display_x-1, ypos, border_col);
	Displ_WChar(display_x-80, ypos+9, 'W', Font24, 1, font_col, back_col);

	// PF
	ypos += box_height+1;
	border_col = WHITE;
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
	snprintf(str,sizeof(str),"I%d", display_channel+1 );
	Displ_WString(190, 133, str , Font24, 1, font_col, back_col);
}

/*
 * @ brief       Aligns V + I curve to start at positive zero crossing and reduces it to one full cycle
 * @retval:      number of points in the aligned curve or -1 if alignment failed
 */
int display_align_curves() {
	int dest_idx = -1;
	int i, continue_at = -1;
	uint8_t adc_channel;

	switch(display_channel) {
	case I1:
		adc_channel = ADC_CH_I1;
		break;
	case I2:
		adc_channel = ADC_CH_I2;
		break;
	case I3:
		adc_channel = ADC_CH_I3;
		break;
	default:
		adc_channel = ADC_CH_I1;
	}

	// if we don't have a zero crossing use the sample_buf curve entries
	if (sample_buf_meta[ADC_CH_V].zero_cross_pos < 0) {
		for (i=0; i<SAMPLE_BUF_SIZE; i++ ) {
			aligned_curve[ADC_CH_V][i] = sample_buf[ADC_CH_V][i];
			aligned_curve[adc_channel][i] = sample_buf[adc_channel][i];
		}
		return SAMPLE_BUF_SIZE;
	}

	// start at zero crossing and use every reading up to the end of the sample buffer
	for (i=sample_buf_meta[ADC_CH_V].zero_cross_pos; i<SAMPLE_BUF_SIZE; i++ ) {
		dest_idx++;
		aligned_curve[ADC_CH_V][dest_idx] = sample_buf[ADC_CH_V][i];
		aligned_curve[adc_channel][dest_idx] = sample_buf[adc_channel][i];
	}

	continue_at = SAMPLE_BUF_OVERLAP;

	for (i=continue_at; i<sample_buf_meta[ADC_CH_V].zero_cross_pos; i++) {
		dest_idx++;
		aligned_curve[ADC_CH_V][dest_idx] = sample_buf[ADC_CH_V][i];
		aligned_curve[adc_channel][dest_idx] = sample_buf[adc_channel][i];
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
	uint8_t adc_channel;
	make_curve(ADC_CH_V, 0);
	draw_curve(channel_colour[ADC_CH_V], 0, (sample_buf_meta[ADC_CH_V].zero_cross_pos >= 0));

	switch(display_channel) {
	case I1:
		adc_channel = ADC_CH_I1;
		break;
	case I2:
		adc_channel = ADC_CH_I2;
		break;
	case I3:
		adc_channel = ADC_CH_I3;
		break;
	default:
		adc_channel = ADC_CH_I1;
	}
	if (!sample_buf_meta[adc_channel].value_is_zero) {
		make_curve(adc_channel, 1);
		draw_curve(channel_colour[display_channel+1], 1, (sample_buf_meta[adc_channel].zero_cross_pos >= 0));
	}
}

/*
 * Show a ADC channel curve on TFT display
 * parameter bufnum: adc_raw_buf index to ADC channel
 * Disp_CLS() takes a long time to run so we draw the previous curve in black to remove it
 */
/*void display_show_curve(uint8_t bufnum) {
	if ( (bufnum >= ADC_NUM_BUFFERS) || (bufnum < 0) ) return;	// buffer range check

	make_curve(bufnum, 1);
	draw_curve(channel_colour[bufnum], 0, (sample_buf_meta[bufnum].measurements_valid != 0));

}*/
