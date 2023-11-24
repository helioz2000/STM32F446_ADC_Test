/*
 * calc.h
 *
 *  Created on: Oct 29, 2023
 *      Author: eb
 */

#ifndef INC_CALC_H_
#define INC_CALC_H_

int calc_process_dma_buffer(int high, int adc_num);
void calc_downsample(uint8_t bufnum);
void calc_filter_measurements(void);
int calc_measurements(void);
int calc_adc_raw_to_mv_int(int16_t adc_raw);
float calc_adc_raw_to_mv_float(int16_t adc_raw);
float calc_adc_raw_to_V(int16_t adc_raw);
float calc_adc_raw_to_A(int16_t adc_raw);

#endif /* INC_CALC_H_ */
