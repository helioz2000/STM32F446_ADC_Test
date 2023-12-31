/*
 * term.h
 *
 *  Created on: Nov 6, 2023
 *      Author: eb
 */

#ifndef INC_TERM_H_
#define INC_TERM_H_

int term_init(void);
HAL_StatusTypeDef term_print(const char* format, ...);
void term_print_hex(uint8_t* buf, unsigned len, uint8_t per_line);
void term_show_buffer(uint8_t bufnum);
void term_csv_buffer(uint8_t buf_num);
void term_show_measurements(void);
void term_show_channel(uint8_t bufnum);

#endif /* INC_TERM_H_ */
