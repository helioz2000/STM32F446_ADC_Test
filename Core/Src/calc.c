/*
 * calc.c
 *
 *  Created on: Oct 29, 2023
 *      Author: Erwin Bejsta
 *
 *  This function handles raw ADC readings and calculates measurements.
 *
 *  Overview
 *  ========
 *  ADC readings are triggered by a time interrupt (TIM2) which occurs once every 25us. This interrupt is tuned for each
 *  individual board to get maximum precision and provides 800 samples per cycle (40kHz).
 *  ADC readings are stored interleaved in a DMA buffer which holds twice the number of required readings. Every time
 *  the buffer is half or full filled an interrupt is called and the ADC readings are processed.
 *  calc_process_dma_buffer() splits one DMA buffer (2Ch interleaved data) and stores each channel in adc_raw_buf[].
 *  To remove noise we down-sample the readings, by averaging adjacent values, into sample_buf[].
 *  adc_raw_buf[] holds 840 readings (21ms) to ensure capture of at least one full cycle.
 *  sample_buf[] holds 420 representing 21ms and is used for all measurement calculations.
 *  sample_buf_meta[] holds summary data of the corresponding buffer.
 *  A zero detector is run over each sample_buf[] to establish the relative location of the positive and negative zero crossing.
 *  If zero crossings can't be reliably established then sample_buf_meta[].measurements_valid will be zero.
 *  If the V sample_buf[] is invalid, the I samples are of no use.
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "global.h"
#include "calc.h"
#include "term.h"

extern uint16_t adc_dma_buf[ADC_NUM][ADC_DMA_BUF_SIZE];

extern float metervalue_v, metervalue_i, metervalue_va, metervalue_w, metervalue_pf;
extern uint8_t display_channel;

extern uint16_t adc_raw_buf[ADC_NUM*ADC_NUM_CHANNELS][ADC_NUM_DATA];	// buffer for channels of raw ADC data
extern uint16_t sample_buf[ADC_NUM_BUFFERS][SAMPLE_BUF_SIZE];			// buffer for channels of down-sampled data

extern double total_precision_vah[];
extern double total_precision_wh[];
extern uint32_t total_vah[];
extern uint32_t total_wh[];

struct sampleBufMeta sample_buf_meta[ADC_NUM_BUFFERS];					// store meta data for associated buffer

uint8_t meter_readings_invalid = 0;

#define FILTER_NUM 5
float v_filter[FILTER_NUM]; // = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float i_filter[NUM_I_SENSORS][FILTER_NUM]; // = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float va_filter[NUM_I_SENSORS][FILTER_NUM]; // = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float w_filter[NUM_I_SENSORS][FILTER_NUM]; // = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float pf_filter[NUM_I_SENSORS][FILTER_NUM]; // = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

float v_filtered;
float i_filtered[NUM_I_SENSORS], va_filtered[NUM_I_SENSORS], w_filtered[NUM_I_SENSORS], pf_filtered[NUM_I_SENSORS];

float v_measured, i_measured[NUM_I_SENSORS], va_measured[NUM_I_SENSORS], w_measured[NUM_I_SENSORS], pf_measured[NUM_I_SENSORS];
__IO double accumulator_vah[NUM_I_SENSORS] = { 0.0, 0.0, 0.0 };
__IO double accumulator_wh[NUM_I_SENSORS]  = { 0.0, 0.0, 0.0 };
__IO uint8_t accumulator_count = 0;
#define ENERGY_ACC_MAX 50			// * 100ms integrations before energy readings are updated. Must divide into 36000

/*
 * @brief   Process the DMA buffer and perform down-sample
 * @para    second_half: > 0 to process 2nd half of buffer, 0 = 1st half of buffer
 * @para    adc_num: 0 = ADC1, 1 = ADC2 (use ADC1_IDX or ADC2_IDX)
 * @retval: -1 on failure, 0 if OK
 * @note    Split the interleaved channel readings from the DMA buffer into adc_raw_buf which is
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
	sample_buf_meta[raw_buf_first].value_is_zero = 0;
	sample_buf_meta[raw_buf_second].value_is_zero = 0;

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
 * @brief           Detect zero point crossings in a buffer and store index in buffer meta data
 * @para bufnum:    the sample buffer to examine
 * @para zeropoint: the reading which represents the zero point
 * @para window:    zero point window height
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
 * @brief   Down-sample ADC raw readings into sample buffer
 * @note    This function provides a filter for the raw ADC readings. It halves
 * the number of samples and averages adjoining samples to smooth out peaks.
 * It also establishes the meta data (min/max and zero crossing, etc)
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
	if (range > ADC_NOISE_RAW) {
		calc_zero_detector(bufnum, range / 2 + sample_buf_meta[bufnum].min, range/5);
	} else {
		// mark zero crossings as invalid
		sample_buf_meta[bufnum].zero_cross_neg = -9;
		sample_buf_meta[bufnum].zero_cross_pos = -9;
		sample_buf_meta[bufnum].value_is_zero = 1;		// value is zero
	}
}

/*
 * @brief          Add new value to voltage filter
 * @para newValue: The new value to add to the filter
 */
void calc_filter_add_v(float newValue) {
	float v_total = 0;
	// shift filter values to make room for new measurement
	for (int i=0; i<FILTER_NUM-1; i++) {
		v_filter[i] = v_filter[i+1];
		v_total += v_filter[i];			// accumulate total
	}
	// add new measurements
	v_filter[FILTER_NUM-1] = newValue;
	v_total += newValue;
	v_filtered = v_total / FILTER_NUM;
}

/*
 * @brief              Add new values to current filters and calculate filtered values
 * @para channel:      Current channel I1=0, I2=1, I3=2
 * @para new_i_value:  The new current value to add to the filter
 * @para new_va_value: The new va value to add to the filter
 * @para new_w_value:  The new w value to add to the filter
 * @para new_pf_value: The new PF value to add to the filter
 */
void calc_filter_add_i(uint8_t channel, float new_i_value, float new_va_value, float new_w_value, float new_pf_value) {
	float i_total=0, va_total=0, w_total=0, pf_total=0;
	// shift filter values to make room for new measurement
	for (int i=0; i<FILTER_NUM-1; i++) {
		i_filter[channel][i] = i_filter[channel][i+1];
		i_total += i_filter[channel][i];
		va_filter[channel][i] = va_filter[channel][i+1];
		va_total += va_filter[channel][i];
		w_filter[channel][i] = w_filter[channel][i+1];
		w_total += w_filter[channel][i];
		pf_filter[channel][i] = pf_filter[channel][i+1];
		pf_total += pf_filter[channel][i];
	}
	// add new measurements
	i_filter[channel][FILTER_NUM-1] = new_i_value;
	i_total += new_i_value;
	va_filter[channel][FILTER_NUM-1] = new_va_value;
	va_total += new_va_value;
	w_filter[channel][FILTER_NUM-1] = new_w_value;
	w_total += new_w_value;
	pf_filter[channel][FILTER_NUM-1] = new_pf_value;
	pf_total += new_pf_value;

	i_filtered[channel] = i_total / FILTER_NUM;
	va_filtered[channel] = va_total / FILTER_NUM;
	w_filtered[channel] = w_total / FILTER_NUM;
	pf_filtered[channel] = pf_total / FILTER_NUM;
}


/*
 * @brief   Assign filtered values to meter values
 */
/*void calc_assign_meter_values(uint8_t channel) {
	// assign filtered valued
	if (channel >= NUM_I_SENSORS) {
		metervalue_v = 0.0;
		metervalue_i = 0.0;
		metervalue_va = 0.0;
		metervalue_w = 0.0;
		metervalue_pf = 0.0;
		meter_readings_invalid = 1;
		return;
	}
	metervalue_v = v_filtered;
	metervalue_i = i_filtered[channel];
	metervalue_va = va_filtered[channel];
	metervalue_w = w_filtered[channel];
	metervalue_pf = pf_filtered[channel];
}*/

/*
 * @brief   Calculate all measurements
 * @retval  0 if measurements are OK, -1 if zero crossing is not detected
 */
int calc_measurements(void) {
	int i;
	int64_t v_sq_acc = 0;		// accumulating the squared voltage values
	int64_t i1_sq_acc=0, i2_sq_acc=0, i3_sq_acc=0;		// accumulator for squared I values
	double i1_va_acc=0, i2_va_acc=0, i3_va_acc=0;		// accumulator for squared VA values
	double i1_w_acc=0, i2_w_acc=0, i3_w_acc=0;			// accumulating I values where I > 0 (for W calculation)
	uint16_t num_readings = 0;		// number of squared readings for v, i and va
	int16_t v_reading;			// always positive, we are using the positive half wave
	int16_t i1_reading, i2_reading, i3_reading;			// could be negative if current is leading or lagging
	double va_instant;			// instant VA value
	uint16_t v_zero;
	uint16_t v_pp;				// Voltage channel Peak-Peak
	uint16_t i1_zero, i2_zero, i3_zero;
	uint16_t i1_pp, i2_pp, i3_pp;			// Current channel P-P
	float w=0, va=0;

	// no zero crossing?
	if (sample_buf_meta[ADC_CH_V].zero_cross_pos < 0) {
		meter_readings_invalid = 1;
		//term_print("%s() - missing zero crossings in V\r\n", __FUNCTION__);
		return -1;
	}

	// low voltage?
	if ((sample_buf_meta[ADC_CH_V].max - sample_buf_meta[ADC_CH_V].min) < ADC_FS_RAW/4) {
		meter_readings_invalid = 1;
		//term_print("%s() - Voltage readings too low V (%d)\r\n", __FUNCTION__, (sample_buf_meta[ADC_CH_V].max - sample_buf_meta[ADC_CH_V].min));
		return -1;
	}

	meter_readings_invalid = 0;		// readings are valid

	v_pp = sample_buf_meta[ADC_CH_V].max - sample_buf_meta[ADC_CH_V].min;
	v_zero = v_pp / 2 + sample_buf_meta[ADC_CH_V].min;
	i1_pp = sample_buf_meta[ADC_CH_I1].max - sample_buf_meta[ADC_CH_I1].min;
	i2_pp = sample_buf_meta[ADC_CH_I2].max - sample_buf_meta[ADC_CH_I2].min;
	i3_pp = sample_buf_meta[ADC_CH_I3].max - sample_buf_meta[ADC_CH_I3].min;
	i1_zero = i1_pp / 2 + sample_buf_meta[ADC_CH_I1].min;
	i2_zero = i2_pp / 2 + sample_buf_meta[ADC_CH_I2].min;
	i3_zero = i3_pp / 2 + sample_buf_meta[ADC_CH_I3].min;

	// Calculate values using the positive half of the sine wave

	// Add up squared measurements
	// does the positive x-ing come before the negative?
	if (sample_buf_meta[ADC_CH_V].zero_cross_pos < sample_buf_meta[ADC_CH_V].zero_cross_neg) {
		// iterate from positive to negative crossing (positive half wave)
		for (i=sample_buf_meta[ADC_CH_V].zero_cross_pos; i<sample_buf_meta[ADC_CH_V].zero_cross_neg; i++ ) {
			v_reading = sample_buf[ADC_CH_V][i] - v_zero;
			v_sq_acc += v_reading * v_reading;
			i1_reading = sample_buf[ADC_CH_I1][i] - i1_zero;
			i1_sq_acc += i1_reading * i1_reading;
			i2_reading = sample_buf[ADC_CH_I2][i] - i2_zero;
			i2_sq_acc += i2_reading * i2_reading;
			i3_reading = sample_buf[ADC_CH_I3][i] - i3_zero;
			i3_sq_acc += i3_reading * i3_reading;
			num_readings++;
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i1_reading);
			if (i1_reading >= 0) { i1_va_acc += va_instant; }
			else { i1_w_acc += abs(va_instant); }
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i2_reading);
			if (i2_reading >= 0) { i2_va_acc += va_instant;}
			else { i2_w_acc += abs(va_instant); }
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i3_reading);
			if (i3_reading >= 0) { i3_va_acc += va_instant;}
			else { i3_w_acc += abs(va_instant); }
		}
	} else {	// negative crossing is first
		// iterate from positive x-ing to the end of the buffer ....
		for (i=sample_buf_meta[ADC_CH_V].zero_cross_pos; i<SAMPLE_BUF_SIZE; i++ ) {
			v_reading = sample_buf[ADC_CH_V][i] - v_zero;
			v_sq_acc += v_reading * v_reading;
			i1_reading = sample_buf[ADC_CH_I1][i] - i1_zero;
			i1_sq_acc += i1_reading * i1_reading;
			i2_reading = sample_buf[ADC_CH_I2][i] - i2_zero;
			i2_sq_acc += i2_reading * i2_reading;
			i3_reading = sample_buf[ADC_CH_I3][i] - i3_zero;
			i3_sq_acc += i3_reading * i3_reading;
			num_readings++;
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i1_reading);
			if (i1_reading >= 0) { i1_va_acc += va_instant; }
			else { i1_w_acc += abs(va_instant); }
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i2_reading);
			if (i2_reading >= 0) { i2_va_acc += va_instant;}
			else { i2_w_acc += abs(va_instant); }
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i3_reading);
			if (i3_reading >= 0) { i3_va_acc += va_instant;}
			else { i3_w_acc += abs(va_instant); }
		}
		// ..... the continue iterating from the start of the buffer to the negative crossing
		for (i=SAMPLE_BUF_OVERLAP; i<sample_buf_meta[ADC_CH_V].zero_cross_neg; i++ ) {
			v_reading = sample_buf[ADC_CH_V][i] - v_zero;
			v_sq_acc += v_reading * v_reading;
			i1_reading = sample_buf[ADC_CH_I1][i] - i1_zero;
			i1_sq_acc += i1_reading * i1_reading;
			i2_reading = sample_buf[ADC_CH_I2][i] - i2_zero;
			i2_sq_acc += i2_reading * i2_reading;
			i3_reading = sample_buf[ADC_CH_I3][i] - i3_zero;
			i3_sq_acc += i3_reading * i3_reading;
			num_readings++;
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i1_reading);
			if (i1_reading >= 0) { i1_va_acc += va_instant; }
			else { i1_w_acc += abs(va_instant); }
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i2_reading);
			if (i2_reading >= 0) { i2_va_acc += va_instant;}
			else { i2_w_acc += abs(va_instant); }
			va_instant = calc_adc_raw_to_V(v_reading) * calc_adc_raw_to_A(i3_reading);
			if (i3_reading >= 0) { i3_va_acc += va_instant;}
			else { i3_w_acc += abs(va_instant); }
		}
	}

	// Calculate measured RMS voltage
	v_measured = calc_adc_raw_to_V (sqrt((v_sq_acc / num_readings)));		// RMS voltage
	//v_measured = calc_adc_raw_to_V(v_pp) / 2 * 0.707;		// only works for a perfect sine wave (no distortion)
	sample_buf_meta[ADC_CH_V].measurements_valid = 1;
	pf_measured[I1] = pf_measured[I2] = pf_measured[I3] = 1.0;		// assumed PF

	// Process I1 values
	// do we have zero (below ADC noise) current reading?
	if (sample_buf_meta[ADC_CH_I1].value_is_zero) {	// set all measured values to zero
		i_measured[I1] = 0.0; va_measured[I1] = 0.0;w_measured[I1] = 0.0;
	} else {
		i_measured[I1] = calc_adc_raw_to_A (sqrt((i1_sq_acc / num_readings)));	// RMS current
		//term_print("%s() - I1 = %f (%d-%d)\r\n", __FUNCTION__, i_measured[I1], sample_buf_meta[ADC_CH_I1].min, sample_buf_meta[ADC_CH_I1].max);
		if (i_measured[I1] >= I1_MIN) {		// Reading above min current?
			if (i1_va_acc > 0) { va = i1_va_acc / num_readings; }
			if (i1_w_acc > 0) { w = i1_w_acc / num_readings; }
			va_measured[I1] = v_measured * i_measured[I1];
			if (w > 0) { w_measured[I1] = va - w;}
			else { w_measured[I1] = va_measured[I1];}
			if (w_measured[I1] > va_measured[I1]) w_measured[I1] = va_measured[I1];		// W must be =< than VA
			if (i_measured[I1] >= I1_MIN_PF) { pf_measured[I1] = w_measured[I1] / va_measured[I1]; }		// Calculate PF if we have sufficient current
			sample_buf_meta[ADC_CH_I1].measurements_valid = 1;
		} else {
			//term_print("%s() - I1 = %fA below minimum\r\n", __FUNCTION__, i_measured[I1]);
			i_measured[I1] = 0.0; va_measured[I1] = 0.0;w_measured[I1] = 0.0;
		}
	}

	// Process I2 values
	if (sample_buf_meta[ADC_CH_I2].value_is_zero) {	// set all measured values to zero
		i_measured[I2] = 0.0;va_measured[I2] = 0.0; w_measured[I2] = 0.0;
	} else {
		i_measured[I2] = calc_adc_raw_to_A (sqrt((i2_sq_acc / num_readings)));	// RMS current
		//term_print("%s() - I2 = %f (%d-%d)\r\n", __FUNCTION__, i_measured[I2], sample_buf_meta[ADC_CH_I2].min, sample_buf_meta[ADC_CH_I2].max);
		if (i_measured[I2] >= I2_MIN) {		// Reading above min current?
			if (i2_va_acc > 0) { va = i2_va_acc / num_readings; }
			if (i2_w_acc > 0) { w = i2_w_acc / num_readings; }
			va_measured[I2] = v_measured * i_measured[I2];
			if (w > 0) { w_measured[I2] = va - w;}
			else { w_measured[I2] = va_measured[I2];}
			if (w_measured[I2] > va_measured[I2]) w_measured[I2] = va_measured[I2];		// W must be =< than VA
			if (i_measured[I2] >= I2_MIN_PF) {pf_measured[I2] = w_measured[I2] / va_measured[I2];}	// Calculate PF if we have sufficient current
			sample_buf_meta[ADC_CH_I2].measurements_valid = 1;
		} else {
			//term_print("%s() - I2 = %fA below minimum\r\n", __FUNCTION__, i_measured[I2]);
			i_measured[I2] = 0.0;va_measured[I2] = 0.0; w_measured[I2] = 0.0;
		}
	}

	// Process I3 values
	if (sample_buf_meta[ADC_CH_I3].value_is_zero) {	// set all measured values to zero
		i_measured[I3] = 0.0; va_measured[I3] = 0.0; w_measured[I3] = 0.0;
	} else {
		i_measured[I3] = calc_adc_raw_to_A (sqrt((i3_sq_acc / num_readings)));	// RMS current
		//term_print("%s() - I3 = %f (%d-%d)\r\n", __FUNCTION__, i_measured[I3], sample_buf_meta[ADC_CH_I3].min, sample_buf_meta[ADC_CH_I3].max);
		if (i_measured[I3] >= I3_MIN) {		// Reading above min current?
			if (i3_va_acc > 0) { va = i3_va_acc / num_readings; }
			if (i3_w_acc > 0) { w = i3_w_acc / num_readings; }
			va_measured[I3] = v_measured * i_measured[I3];
			if (w > 0) { w_measured[I3] = va - w; }
			else { w_measured[I3] = va_measured[I3]; }
			if (w_measured[I3] > va_measured[I3]) w_measured[I3] = va_measured[I3];		// W must be =< than VA
			if (i_measured[I3] >= I3_MIN_PF) { pf_measured[I3] = w_measured[I3] / va_measured[I3]; }	// Calculate PF if we have sufficient current
			sample_buf_meta[ADC_CH_I3].measurements_valid = 1;
		} else {
			//term_print("%s() - I3 = %fA below minimum\r\n", __FUNCTION__, i_measured[I3]);
			i_measured[I3] = 0.0; va_measured[I3] = 0.0; w_measured[I3] = 0.0;
		}
	}

	// add measurements to filter
	calc_filter_add_v(v_measured);
	calc_filter_add_i(I1, i_measured[I1], va_measured[I1], w_measured[I1], pf_measured[I1]);
	calc_filter_add_i(I2, i_measured[I2], va_measured[I2], w_measured[I2], pf_measured[I2]);
	calc_filter_add_i(I3, i_measured[I3], va_measured[I3], w_measured[I3], pf_measured[I3]);
	//calc_assign_meter_values(display_channel);

	return 0;
}

/*
 * @brief    Integrate energy totals with the latest readings. To be called every 100ms via INT.
 * @retval   0 or accumulator count if target has been reached
 */
uint8_t calc_integrate_energy_totals() {
	uint8_t retval = 0;

	for (int i=0; i<NUM_I_SENSORS; i++) 	// every 100ms
	{
		accumulator_vah[i] += (double)va_filtered[i];
		accumulator_wh[i] += (double)w_filtered[i];
	}
	accumulator_count++;
	if (accumulator_count >= ENERGY_ACC_MAX) {		// once a second
		retval = accumulator_count;
		accumulator_count = 0;
	}
	return retval;
}

/*
 * @brief           Update energy totals. To be called every time the integration accumulator has reached it's target
 * @para acc_count  Number of values accumulated in accumulator_vah/wh
 */
void calc_update_energy_totals(uint8_t acc_count) {
	const double divisor = 36000.0 / (double) ENERGY_ACC_MAX;		// accumulator totals per hour
	for (int i=0; i<NUM_I_SENSORS; i++)
	{
		// update precision values
		total_precision_vah[i] += accumulator_vah[i] / (double)acc_count / divisor;
		total_precision_wh[i] += accumulator_wh[i] / (double)acc_count / divisor;
		accumulator_vah[i] = 0.0;
		accumulator_wh[i] = 0.0;
		// update integer values
		total_vah[i] = round(total_precision_vah[i] * 10);
		total_wh[i] = round(total_precision_wh[i] * 10);
	}
}

/*
 * @brief   Calculate the measurements of single sample buffer
 * @retval  0 on success or -1 on failure
 * The RMS value is calculated from readings between the positive and negative zero crossing
 * that is, the positive half of the sine wave.
 * The RMS value is calculate by adding the square of each reading to an accumulator and then
 * diving the accumulator by the number of readings.
 */
/*int calc_channel(uint8_t bufnum) {
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
}*/

/*
 * @brief   Convert ADC raw reading to mv
 * @retval  mv as int
 */
int calc_adc_raw_to_mv_int(int16_t adc_raw) {
	return round(calc_adc_raw_to_mv_float(adc_raw));
}

/*
 * @brief   Convert raw reading to mV
 */
float calc_adc_raw_to_mv_float(int16_t adc_raw) {
	return ((float)adc_raw / (float)ADC_FS_RAW) * (float)ADC_FS_MV;
}

/*
 * @brief   Convert ADC raw reading to V
 */
float calc_adc_raw_to_V(int16_t adc_raw) {
	return ((float)adc_raw / (float)ADC_FS_RAW) * (float)ADC_FS_CH_V;
}

/*
 * @brief   Convert ADC raw reading to A
 */
float calc_adc_raw_to_A(int16_t adc_raw) {
	return ((float)adc_raw / (float)ADC_FS_RAW) * (float)ADC_FS_CH_I;
}
