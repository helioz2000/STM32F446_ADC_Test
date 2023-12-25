/*
 * global.h
 *
 *  Created on: Oct 26, 2023
 *      Author: eb
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#define VERSION_MAJOR 0
#define VERSION_MINOR 2

struct sampleBufMeta {
	uint16_t min;			// minimum value in buffer
	uint16_t max;			// maximum value in buffer
	int zero_cross_pos;		// positive slope crossing, -1 = uninitialized, -9 = zero detection error
	int zero_cross_neg;		// negative slope crossing
	uint8_t measurements_valid;	// 0 - measurements are not valid (have not been calculated)
	uint8_t value_is_zero;	// 1 when the the analog reading is below the noise level.
	int rms_value;			// calculated RMS value

};

// Enable lines below to activate code for additional channels
//#define I2_IN_USE
//#define I3_IN_USE

#define NUM_I_SENSORS 3
#define I1 0		// index for I measurements
#define I2 1
#define I3 2

/*
#ifdef I2_IN_USE
#define NUM_I_SENSORS 2
#elif I3_IN_USE
#define NUM_I_SENSORS 3
#endif
*/

#define MEASUREMENT_INTERVAL 100	// [ms] calculate measurements from curves and add them to the filter

// WiFi module (ESP-01) definitions
//#define USE_WIFI		// comment out to disable WiFi code


// line below controls TFT display usage
#define USE_DISPLAY
#define DISPLAY_TIMEOUT 300000	// [ms] display timeout (LCD screen saver)
#define DISPLAY_UPDATE_TIME 350	// [ms] meter display update time
#define SPLASH_SCREEN_TIME 1500	// [ms] time to display splash screen

#define ADC_FS_RAW 4095		// full scale 12 bit ADC raw reading
#define ADC_FS_MV 3300		// millivolt at full scale
#define ADC_FS_CH_V	781		// P-P V at full scale (760V P-P = 380 * 0.707 = 268V RMS)
#define ADC_FS_CH_I 226		// P-P A at full scale (200A P-P = 100 * 0.707 = 70A RMS)
#define ADC_CENTER_RAW 1861	// centre of signal (1500mV = 1861, 1650mV = 2047)
#define ADC_NOISE_RAW 20	// readings below this level are considered to be noise are are replaced with zero

#define ADC_NUM 2		// number of ADCs in use
#define ADC1_IDX 0			// ADC1 index for arrays
#define ADC2_IDX 1			// ADC2 index for arrays

#define ADC_NUM_CHANNELS 2								// number of channels per ADC
#define ADC_NUM_BUFFERS	ADC_NUM * ADC_NUM_CHANNELS		// we need one buffer per channel

#define ADC_NUM_DATA 840U		   			// number of data points to record for each channel
#define SAMPLE_BUF_SIZE ADC_NUM_DATA / 2	// number of data points after downsampling
#define SAMPLE_BUF_OVERLAP 20				// half of raw buffer overlap

// 2 DMA buffers (one per ADC) contain both channels, need space for 2 lots
#define ADC_DMA_BUF_SIZE ADC_NUM_DATA * ADC_NUM_CHANNELS * 2	// size of each DMA buffer which contains 2 sets of data

// identify buffers by signal source
#define ADC_CH_V 0
#define ADC_CH_I1 3
#define ADC_CH_I2 2
#define ADC_CH_I3 1

// ADC scaling
#define ADC_V_MV_PER_BIT
#define ADC_I1_MA_PER_BIT

#define I1_MIN_PF 1.0		// [A] minimum current required for PF calculation
#define I2_MIN_PF 1.0		// [A] minimum current required for PF calculation
#define I3_MIN_PF 1.0		// [A] minimum current required for PF calculation

/*
 * DMA buffer description
 * The DMA buffer contains one int16 entry per ADC reading
 * The sequence order is: CH1, CH2, CH1, CH2, CH1, CH2, ....
 * 800 recordings for 2 channel require 1600 entries
 * The complete DMA buffer stores 2 sets of recordings
 * Our code receives a callback after the first half of the DMA
 * buffer is filled so it can process that data while the DMA fills the
 * second half.
 * Sampling at 40kHzA a full sine wave (20ms) contains 800 samples
 * one sample is taken every 25us or every 0.45 Degrees
 */


//#define ADC_BUF_SIZE ADC_NUM_DATA * ADC_NUM_CHANNELS // buffer size
//#define ADC_DMA_BUF_SIZE ADC_BUF_SIZE * 2	// DMA buffer contains 2 sets of data

#define MAX(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define MIN(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

#endif /* INC_GLOBAL_H_ */
