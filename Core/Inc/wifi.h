/*
 * wifi.h
 *
 *  Created on: Dec 14, 2023
 *      Author: eb
 */

#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#define ESP_RX_BUF_SIZE 1024

void wifi_handle_esp_rx_data();
int wifi_send_esp_data(uint8_t* buf, unsigned len);

#endif /* INC_WIFI_H_ */
