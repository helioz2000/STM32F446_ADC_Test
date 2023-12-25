/*
 * display.h
 *
 *  Created on: Nov 6, 2023
 *      Author: eb
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

void display_init(void);
void display_splash_screen(void);
void display_update_meter(uint8_t screen);
void display_update_mask(void);
void display_meter_mask(void);
void display_show_curves(void);

#endif /* INC_DISPLAY_H_ */
