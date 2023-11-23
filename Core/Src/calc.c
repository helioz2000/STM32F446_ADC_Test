/*
 * calc.c
 *
 *  Created on: Oct 29, 2023
 *      Author: eb
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "global.h"
#include "calc.h"
#include "term.h"

extern uint16_t adc_dma_buf[ADC_NUM][ADC_DMA_BUF_SIZE];
//extern uint16_t adc2_dma_buf[];

extern float metervalue_v, metervalue_i1, metervalue_va1, metervalue_w1, metervalue_pf1;

extern uint16_t adc_raw_buf[ADC_NUM*ADC_NUM_CHANNELS][ADC_NUM_DATA];	// buffer for channels of raw ADC data
extern uint16_t sample_buf[ADC_NUM_BUFFERS][SAMPLE_BUF_SIZE];			// buffer for channels of down-sampled data

struct sampleBufMeta sample_buf_meta[ADC_NUM_BUFFERS];					// store meta data for associated buffer

uint8_t meter_readings_invalid = 0;

#define FILTER_NUM 10
float v_filter[FILTER_NUM] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float i1_filter[FILTER_NUM] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float va1_filter[FILTER_NUM] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float w1_filter[FILTER_NUM] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float pf1_filter[FILTER_NUM] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

float v_measured, i1_measured, va1_measured, w1_measured, pf1_measured;


//inline int16_t MAX(int16_t a, int16_t b) { return((a) > (b) ? a : b); }
//inline int16_t MIN(int16_t a, int16_t b) { return((a) < (b) ? a : b); }

/*
 * Process the DMA buffer and perform down-sample
 *
 * parameter second_half: > 0 to process 2nd half of buffer, 0 = 1st half of buffer
 * parameter adc_num: 0 = ADC1, 1 = ADC2 (use ADC1_IDX or ADC2_IDX)
 * returns: -1 on failure, 0 if OK
 * Split the multi channel readings from the DMA buffer into adc_raw_buf which is
 * structured to hold the readings for one channel per array element.
 * This function also establishes the min/max values for each channel
 * Note: Each ADC is assigned one DMA buffer.
 * One DMA buffer contains 2 data sets, one which is "completed" and
 * one which is currently in use by DMA. The parameter "second_half" indicates which
 * of the two halves is ready for processing (not in use by DMA).
 * The DMA buffer is made up of a sequence of alternate readings (CH0,CH1,CH0,CH1, ....)
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
	raw_buf_first = adc_num *2;			// destination index for first channel reading in DMA buffer
	raw_buf_second = raw_buf_first + 1; // destination index for second channel reading in DMA buffer
	// first or second half of DMA buffer?
	if (second_half) {
		dma_buf_start = ADC_DMA_BUF_SIZE / 2;
	} else {
		dma_buf_start = 0;	// first half
	}
	dma_buf_end = dma_buf_start + (ADC_DMA_BUF_SIZE / 2) -1;

	// clear meta data in sample buffer
	sample_buf_meta[raw_buf_first].min = ADC_FS_RAW;
	sample_buf_meta[raw_buf_first].max = 0;
	sample_buf_meta[raw_buf_second].min = ADC_FS_RAW;
	sample_buf_meta[raw_buf_second].max = 0;
	sample_buf_meta[raw_buf_first].zero_cross_pos = -1;
	sample_buf_meta[raw_buf_first].zero_cross_neg = -1;
	sample_buf_meta[raw_buf_second].zero_cross_pos = -1;
	sample_buf_meta[raw_buf_second].zero_cross_neg = -1;
	sample_buf_meta[raw_buf_first].measurements_valid = 0;
	sample_buf_meta[raw_buf_second].measurements_valid = 0;

	// split DMA buffer in to channels and copy readings into raw buffers
	// step of ADC_NUM_CHANNELS = 2
	for (i=dma_buf_start; i<=dma_buf_end; i+=ADC_NUM_CHANNELS) {
		adc_raw_buf[raw_buf_first][raw_buf_idx] = adc_dma_buf[adc_num][i];		// first entry in DMA buffer
		adc_raw_buf[raw_buf_second][raw_buf_idx++] = adc_dma_buf[adc_num][i+1]; // second entry in DMA buffer

	}
	// down-sample both channels
	calc_downsample(raw_buf_first);
	calc_downsample(raw_buf_second);
	return 0;
}

/*
 * Detect zero point crossings in a buffer and store index in buffer meta data
 * parameter bufnum: the sample buffer to examine
 * parameter zeropoint: the reading which represents the zero point
 * parameter window: zero point window height
 * The challenge with zero crossing is noise contained in the recordings.
 * Whilst some of the noise has been reduced by down-sampling, we have to
 * account for residual noise in the data, hence the somewhat extensive
 * detection method.
 * Detection method: iterate over all points and find the point
 * where one value is above zero threshold and a neighboring point is below
 * then check another point either side of those points to see if
 * they conform to the same slope. Out of those 3 detections at least
 * two being true will yield record a crossing.
 */
void calc_zero_detector(uint8_t bufnum, int zeropoint, int window) {
	if (bufnum >= ADC_NUM_BUFFERS) { return; }
	uint8_t detected = 0;
	uint8_t detect_count = 0;
	// set the detection window
	int window_h = zeropoint + (window/2);
	int window_l = zeropoint - (window/2);

	for (int i=1; i < SAMPLE_BUF_SIZE; i++) {
		// start looking for crossing if reading is within the window
		if ( (sample_buf[bufnum][i] >= window_l) && (sample_buf[bufnum][i] <= window_h) ) {
			// positive slope crossing
			if ( (sample_buf[bufnum][i] > zeropoint) && (sample_buf[bufnum][i-1] <= zeropoint) ) {
				detected++;
				if (i-2 >= 0) {	// check point on lower side
					if (sample_buf[bufnum][i-2] < zeropoint) {
						detected++;
					}
				}
				if (i+2 < SAMPLE_BUF_SIZE) {	// check point on higher side
					if (sample_buf[bufnum][i+2] > zeropoint) {
						detected++;
					}
				}
			}
			if (detected > 1) {	// if at least two of the above checks have a positive result
				sample_buf_meta[bufnum].zero_cross_pos = i;	// set positive crossing point
				detected = 0;
				detect_count++;
				continue;		// back to for loop
			}
			detected = 0;
			// negative slope crossing
			if ( (sample_buf[bufnum][i] < zeropoint) && (sample_buf[bufnum][i-1] >= zeropoint) ) {
				detected++;
				if (i-2 >= 0) {	// check point on lower side
					if (sample_buf[bufnum][i-2] >= zeropoint) {
						detected++;
					}
				}
				if (i+2 < SAMPLE_BUF_SIZE) {	// check point on higher side
					if (sample_buf[bufnum][i+2] < zeropoint) {
						detected++;
					}
				}
				if (detected > 1) {	// if at least two of the above checks have a positive result
					sample_buf_meta[bufnum].zero_cross_neg = i;	// set positive crossing point
					detect_count++;
				}
				detected = 0;
			}
		}	// if inside window
	}	// for in buffer

	// check how many crossings were detected, we could have up to 3
	if (detect_count > 3) {
		// if we have more than 2 crossings the crossing detections are marked invalid
		sample_buf_meta[bufnum].zero_cross_neg = -9;
		sample_buf_meta[bufnum].zero_cross_pos = -9;
	}
}

/*
 * Down-sample ADC raw readings into sample buffer
 * This function provides a filter for the raw ADC readings. It halves
 * the number of samples and averages adjoining samples to smooth out peaks.
 * It also establishes the meta data (min/max and zero crossing) for both channel
 */
void calc_downsample(uint8_t bufnum) {
	uint16_t range;
	uint16_t dest_idx=0;
	if (bufnum >= ADC_NUM_BUFFERS) { return; }
	for (int i=1; i < ADC_NUM_DATA-2; i+=2) {
		// calculate reading value by averaging 3 readings (the one before and the one after)
		sample_buf[bufnum][dest_idx] = (adc_raw_buf[bufnum][i] + adc_raw_buf[bufnum][i-1] + adc_raw_buf[bufnum][i+1]) / 3;
		// track min/max values
		sample_buf_meta[bufnum].min = MIN(sample_buf_meta[bufnum].min, sample_buf[bufnum][dest_idx]);
		sample_buf_meta[bufnum].max = MAX(sample_buf_meta[bufnum].max, sample_buf[bufnum][dest_idx]);
		dest_idx++;
	}
	// Last sample in the buffer (we only have 2 raw readings available for averaging)
	sample_buf[bufnum][dest_idx++] = (adc_raw_buf[bufnum][ADC_NUM_DATA-1] + adc_raw_buf[bufnum][ADC_NUM_DATA-2]) / 2;
	// test if the last value is not assigned
	if (dest_idx < SAMPLE_BUF_SIZE) {
		sample_buf[bufnum][dest_idx] = 0;
	}
	// range of readings
	range = sample_buf_meta[bufnum].max - sample_buf_meta[bufnum].min;
	// detect zero crossings
	calc_zero_detector(bufnum, range / 2 + sample_buf_meta[bufnum].min, range/5);
}

void calc_filter_measurements(void) {

	// shift filter values to make room for new measurement
	for (int i=0; i<FILTER_NUM-1; i++) {
		v_filter[i] = v_filter[i+1];
		i1_filter[i] = i1_filter[i+1];
		va1_filter[i] = va1_filter[i+1];
		w1_filter[i] = w1_filter[i+1];
		pf1_filter[i] = pf1_filter[i+1];
	}
	// add new measurements
	v_filter[FILTER_NUM-1] = v_measured;
	i1_filter[FILTER_NUM-1] = i1_measured;
	va1_filter[FILTER_NUM-1] = va1_measured;
	w1_filter[FILTER_NUM-1] = w1_measured;
	pf1_filter[FILTER_NUM-1] = pf1_measured;

	// zero readings
	metervalue_v = 0.0;
	metervalue_i1 = 0.0;
	metervalue_va1 = 0.0;
	metervalue_w1 = 0.0;
	metervalue_pf1 = 0.0;
	// add filter values
	for (int i=0; i<FILTER_NUM; i++) {
		metervalue_v += v_filter[i];
		metervalue_i1 += i1_filter[i];
		metervalue_va1 += va1_filter[i];
		metervalue_w1 += w1_filter[i];
		metervalue_pf1 += pf1_filter[i];
	}
	// calculate filtered valued
	metervalue_v /= FILTER_NUM;
	metervalue_i1 /= FILTER_NUM;
	metervalue_va1 /= FILTER_NUM;
	metervalue_w1 /= FILTER_NUM;
	metervalue_pf1 /= FILTER_NUM;
}

/*
 * Calculate all measurements
 * returns 0 if measurements are OK, -1 if zero crossing is not detected
 */
int calc_measurements(void) {
	int i;
	int64_t v_sq_acc = 0;		// accumulating the squared voltage values
	int64_t i1_sq_acc = 0;		// accumulating the squared I1 values
	double i1_va_acc = 0;
	double i1_w_acc = 0;			// accumulating I1 values where I > 0 (for W calculation)
	uint16_t num_readings = 0;		// number of squared readings for v, i and va
	int16_t v_reading;			// always positive, we are using the positive half wave
	int16_t i_reading;			// could be negative if current is leading or lagging
	double va_instant;			// instant VA value
	uint16_t v_zero = (sample_buf_meta[ADC_CH_V].max - sample_buf_meta[ADC_CH_V].min) / 2;
	uint16_t i1_zero = (sample_buf_meta[ADC_CH_I1].max - sample_buf_meta[ADC_CH_I1].min) / 2;
	float w=0, va=0;

	// Calculate values using the positive half of the sine wave

	if (sample_buf_meta[ADC_CH_V].zero_cross_pos < 0) { 	// do we have zero crossing?
		meter_readings_invalid = 1;
		//term_print("%s() - invalid measurements\r\n", __FUNCTION__);
		return -1;
	} else {
		meter_readings_invalid = 0;
	}

	//term_print("%s()\r\n", __FUNCTION__);

	// add up squared measurements
	if (sample_buf_meta[ADC_CH_V].zero_cross_pos < sample_buf_meta[ADC_CH_V].zero_cross_neg) {
		for (i=sample_buf_meta[ADC_CH_V].zero_cross_pos; i<sample_buf_meta[ADC_CH_V].zero_cross_neg; i++ ) {
			v_reading = sample_buf[ADC_CH_V][i] - v_zero;
			v_sq_acc += v_reading * v_reading;
			i_reading = sample_buf[ADC_CH_I1][i] - i1_zero;
			i1_sq_acc += i_reading * i_reading;
			num_readings++;
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i_reading);
			if (i_reading >= 0) {
				i1_va_acc += va_instant;
			} else {
				i1_w_acc += abs(va_instant);
			}
		}
	} else {
		for (i=sample_buf_meta[ADC_CH_V].zero_cross_pos; i<SAMPLE_BUF_SIZE; i++ ) {
			v_reading = sample_buf[ADC_CH_V][i] - v_zero;
			v_sq_acc += v_reading * v_reading;
			i_reading = sample_buf[ADC_CH_I1][i] - i1_zero;
			i1_sq_acc += i_reading * i_reading;
			num_readings++;
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i_reading);
			if (i_reading >= 0) {
				i1_va_acc += va_instant;
			} else {
				i1_w_acc += abs(va_instant);
			}
		}
		for (i=SAMPLE_BUF_OVERLAP; i<sample_buf_meta[ADC_CH_V].zero_cross_neg; i++ ) {
			v_reading = sample_buf[ADC_CH_V][i] - v_zero;
			v_sq_acc += v_reading * v_reading;
			i_reading = sample_buf[ADC_CH_I1][i] - i1_zero;
			i1_sq_acc += i_reading * i_reading;
			num_readings++;
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i_reading);

			if (i_reading >= 0) {
				i1_va_acc += va_instant;
			} else {
				i1_w_acc += abs(va_instant);
			}
		}
	}

	v_measured = calc_adc_raw_to_V (sqrt((v_sq_acc / num_readings)));		// RMS voltage
	i1_measured = calc_adc_raw_to_A (sqrt((i1_sq_acc / num_readings)));	// RMS current
	if (i1_va_acc > 0) { va = i1_va_acc / num_readings; }
	if (i1_w_acc > 0) { w = i1_w_acc / num_readings; }
	va1_measured = v_measured * i1_measured;
	if (w > 0) {
		w1_measured = va - w;
	} else {
		w1_measured = va1_measured;
	}
	pf1_measured = w1_measured / va1_measured;
	sample_buf_meta[ADC_CH_V].measurements_valid = 1;

	calc_filter_measurements();

	return 0;
}

/*
 * Calculate the measurements of single sample buffer
 * returns: 0 on success or -1 on failure
 * The RMS value is calculated from readings between the positive and negative zero crossing
 * that is, the positive half of the sine wave.
 * The RMS value is calculate by adding the square of each reading to an accumulator and then
 * diving the accumulator by the number of readings.
 */
int calc_channel(uint8_t bufnum) {
	int i;
	uint64_t squared_acc = 0;		// accumulating the squared values
	uint16_t num_readings = 0;		// number of squared readings
	uint16_t reading = 0;
	uint16_t zero_value = (sample_buf_meta[bufnum].max - sample_buf_meta[bufnum].min) / 2;

	if (bufnum >= ADC_NUM_BUFFERS) { return -1; }		// check valid buffer number

	// Calculate the RMS value using the positive half of the sinewave

	if (sample_buf_meta[bufnum].zero_cross_pos < 0) { return -1; }	// do we have zero crossing?
	if (sample_buf_meta[bufnum].zero_cross_pos < sample_buf_meta[bufnum].zero_cross_neg) {
		for (i=sample_buf_meta[bufnum].zero_cross_pos; i<sample_buf_meta[bufnum].zero_cross_neg; i++ ) {
			reading = sample_buf[bufnum][i] - zero_value;
			squared_acc += reading * reading;
			num_readings++;
		}
	} else {
		for (i=sample_buf_meta[bufnum].zero_cross_pos; i<SAMPLE_BUF_SIZE; i++ ) {
			reading = sample_buf[bufnum][i] - zero_value;
			squared_acc += reading * reading;
			num_readings++;
		}
		for (i=SAMPLE_BUF_OVERLAP; i<sample_buf_meta[bufnum].zero_cross_neg; i++ ) {
			reading = sample_buf[bufnum][i] - zero_value;
			squared_acc += reading * reading;
			num_readings++;
		}
	}
	sample_buf_meta[bufnum].rms_value = sqrt((squared_acc / num_readings));
	sample_buf_meta[bufnum].measurements_valid = 1;
	return 0;
}

/*
 * Convert ADC raw reading to mv
 * returns mv as int
 */
int calc_adc_raw_to_mv_int(int16_t adc_raw) {
	return round(calc_adc_raw_to_mv_float(adc_raw));
}

/*
 * Convert raw reading to mV
 */
float calc_adc_raw_to_mv_float(int16_t adc_raw) {
	return ((float)adc_raw / (float)ADC_FS_RAW) * (float)ADC_FS_MV;
}

float calc_adc_raw_to_V(int16_t adc_raw) {
	return ((float)adc_raw / (float)ADC_FS_RAW) * (float)ADC_FS_CH_V;
}

float calc_adc_raw_to_A(int16_t adc_raw) {
	return ((float)adc_raw / (float)ADC_FS_RAW) * (float)ADC_FS_CH_I;
}
