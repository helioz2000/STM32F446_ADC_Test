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

#endif /* INC_TERM_H_ */
