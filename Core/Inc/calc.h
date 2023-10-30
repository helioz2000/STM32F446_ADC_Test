/*
 * calc.h
 *
 *  Created on: Oct 29, 2023
 *      Author: eb
 */

#ifndef INC_CALC_H_
#define INC_CALC_H_

int calc_process_dma_buffer(int high, int adc_num);
void calc_display_buffer(uint8_t buf_num);
void calc_csv_buffer(uint8_t buf_num);
int calc_adc_raw_to_mv_int(uint16_t adc_raw);
float calc_adc_raw_to_mv_float(uint16_t adc_raw);

#endif /* INC_CALC_H_ */
