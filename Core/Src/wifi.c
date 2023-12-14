/*
 * wifi.c
 *
 *  Created on: Dec 14, 2023
 *      Author: eb
 */

#include "main.h"
#include <stdbool.h>
#include <string.h>
#include "global.h"
#include "term.h"
#include "wifi.h"

__IO uint8_t esp_rx_buf[ESP_RX_BUF_SIZE];
#define ESP_TX_BUF_SIZE 128
__IO uint8_t esp_tx_buf[ESP_TX_BUF_SIZE];
__IO uint16_t esp_rx_count = 0;
__IO uint16_t esp_rx_error_count = 0;
__IO uint8_t esp_rx_buf[ESP_RX_BUF_SIZE];
__IO bool esp_rx_buffer_overflow = false;
char line_buf[ESP_RX_BUF_SIZE];

bool esp_first_rx = false;
bool esp_wifi_connected = false;
bool esp_wifi_got_ip = false;
uint8_t esp_init_step = 4;
//uint8_t esp_connection_count = 0;

int connection_id = -1;

extern UART_HandleTypeDef huart3;

#define ESP_UART huart3
#define MODBUS_SERVER_PORT 80

/*
 * Private function declarations
 */
void evaluate_esp_response(char* response, int len);
void init_sequence(void);

/*
 * @brief  Handle data received from ESP-01
 * @note   The first lot of data received from the ESP after reset is received with a different
 * 	       baudrate (76800) and is therefore unreadable. This function ignores the first RX.
 *         The ESP is init sequence is initiated as soon as the first lot of data is received.
 */
void wifi_handle_esp_rx_data() {

	if (esp_rx_error_count) {
		term_print("\r\nrx:%d error:%d\r\n", esp_rx_count, esp_rx_error_count);
		esp_rx_error_count = 0;
	}
	if (esp_rx_buffer_overflow) {
		term_print("\r\nESP RX buffer overflow\r\n");
		esp_rx_buffer_overflow = false;
	}
	if (!esp_first_rx) {
		term_print("\r\nESP first RX\r\n");
		esp_first_rx = true;
		HAL_Delay(50);
		init_sequence();	// kick off init sequence
	} else {
		esp_rx_buf[esp_rx_count] = 0;		// set EOS
		term_print("\r\n%s() - <%s>\r\n", __FUNCTION__, esp_rx_buf);
		evaluate_esp_response( (char*)esp_rx_buf, esp_rx_count);
	}
}

/*
 * @brief  Send data to connected client
 * @para   buf  data buffer
 * @para   len  length of buffer
 * @retval 0 on success
 */
int wifi_send_esp_data(uint8_t* buf, unsigned len) {
	uint8_t* data_start;
	int s_len;
	sprintf((char*)esp_tx_buf, "AT+CIPSEND=%d", len);	// send data
	s_len = strlen((char*)esp_tx_buf);
	data_start = (uint8_t*) esp_tx_buf + s_len;
	memcpy(data_start, buf, len);
	HAL_UART_Transmit(&ESP_UART, (uint8_t*)esp_tx_buf, s_len + len, 1000);
	return 0;
}

/*
 * Private functions
 */

/*
 * @brief  ESP-01 init sequence commands
 * @note   Each step decrements the init step number.
 *         Command sent = step--
 *         OK response = step--
 */
void init_sequence() {
	term_print( "%s() - step: %d\r\n", __FUNCTION__, esp_init_step );
	switch (esp_init_step) {
	case 4:
		sprintf((char*)esp_tx_buf, "ATE0\r\n");	// disable echo
		HAL_UART_Transmit(&ESP_UART, (uint8_t*)esp_tx_buf, strlen((char*)esp_tx_buf), 1000);
		esp_init_step --;
		break;
	case 2:
		sprintf((char*)esp_tx_buf, "AT+CIPMUX=1\r\n");	// start server
		HAL_UART_Transmit(&ESP_UART, (uint8_t*)esp_tx_buf, strlen((char*)esp_tx_buf), 1000);
		esp_init_step --;
		break;
	case 0:
		sprintf((char*)esp_tx_buf, "AT+CIPSERVER=1,%d\r\n", MODBUS_SERVER_PORT);	// start server
		HAL_UART_Transmit(&ESP_UART, (uint8_t*)esp_tx_buf, strlen((char*)esp_tx_buf), 1000);
		esp_init_step --;
		break;
	default:
	}

}

/*
 * @brief   Process the data received from client connection
 * @param   data  buffer containing received data
 * @param   len   length of data buffer
 * @retval  0 on success
 */
int process_incoming_data(uint8_t *data, unsigned len) {
	term_print_hex(data, len, 0);
	return 0;
}


/*
 * @brief  Process the ESP response containing data from a connected client
 * @param  response  ESP receive buffer
 * @param  len       length of receive buffer
 * @retval 0 on successful
 */
int process_esp_response_ipd(char* response, int len) {
	uint8_t *data_start;
	char* token;
	const char s[1] = {','};	// token separator
	int data_len = 0;

	token = strtok(response, s);	// +IPD
	if (token == NULL) { return -1; }
	token = strtok(NULL, s);	// connection number
	if (token == NULL) { return -1; }
	else { connection_id = token[0] - '0'; }
	token = strtok(NULL, s);	// data length
	if (token == NULL) { return -1; }
	data_len = atoi(token);
	if ((data_len <= 0) || (data_len >= len)) { return -1; }	// sanity check on data length
	//We have valid data
	data_start = (uint8_t*)token + strlen(token) + 1;			// data starts after end of token
	return (process_incoming_data(data_start, data_len));
}



/*
 * @brief   Process ESP responses starting with "WIFI ....."
 * @para    token      the token (word) which comes after "WIFI" in the ESP response
 * @para    token_num  the number if the token in the ESP response (WIFI=1)
 * @retval  -1 on failure, 0 or the number of tokens to be ignored
 */
int process_esp_response_wifi(char* token, uint8_t token_num) {
	//term_print( "%s() - token%d = <%s>\r\n", __FUNCTION__, token_num, token );
	int retval = -1;
	if (strncmp(token, "CONNECTED", 9)==0) {	// "WIFI CONNECTED"
		//term_print( "%s() - CON\r\n", __FUNCTION__);
		retval = 0;
		esp_wifi_connected = true;
	} else if(strncmp(token, "GOT", 3)==0) {		// "WIFI GOT IP"
		//term_print( "%s() - GOT\r\n", __FUNCTION__);
		retval = 1;		// ignore next token "IP"
		esp_wifi_got_ip = true;
	} else if(strncmp(token, "DISCONNECTED", 12)==0) { // "WIFI DISCONNECTED"
		retval = 0;
		esp_wifi_connected = false;
		esp_wifi_got_ip = false;
	}
	term_print( "%s() - CONNECTED=%d GOT IP=%d (retval=%d)\r\n", __FUNCTION__, esp_wifi_connected, esp_wifi_got_ip, retval );
	return retval;
}

/*
 * @brief   Process ESP status response "STATUS:2"
 * @para    token      the token (word) which starts with the word "STATUS"
 * @para    token_num  the number if the token in the ESP response, normally 1 for STATUS
 * @retval  -1 on failure, 0 or the number of tokens to be ignored
 */
int process_esp_response_status(char* token, uint8_t token_num) {
	int retval = 0;
	if (strlen(token) != 8) { return -1; }
	if (token[6] != ':') { return -1; }
	switch(token[7]) {
	case '2':		// GOT IP
		esp_wifi_got_ip = true;
		esp_wifi_connected = true;
		break;
	case '3':		// CONNECTED
		esp_wifi_connected = true;
		break;
	case '4':		// DISCONNECTED
		esp_wifi_got_ip = false;
		esp_wifi_connected = false;
		break;
	default:		// unknown status
		retval = -1;
	}

	return retval;
}

/*
 * @brief   Process ESP response line"
 * @para    line     the response line
 * @para    line_num the line number
 * @retval  -1 on failure, 0 or the number of lines to be ignored
 */
int process_esp_response_line(char* line, uint8_t line_num) {
	int retval = -1;
	char* token;
	uint8_t token_count = 0;
	const char s[1] = {' '};	// token separator
	int ignore_tokens = 0;

	//term_print( "%s() - %d:<%s>\r\n", __FUNCTION__, line_num, line);

	// evaluate all tokens
	token = strtok(line, s);
	while(token != NULL) {
		token_count++;
		if (ignore_tokens > 0) {
			ignore_tokens--;
		} else {
			if (strncmp(token,"WIFI",4)==0) {
				//term_print( "%s() - found WIFI\r\n", __FUNCTION__, line_num, line);
				token = strtok(NULL, s);
				if (token != NULL) {
					token_count++;
					ignore_tokens = process_esp_response_wifi(token, token_count);
					if (ignore_tokens >= 0) { retval = 0; }
				}
			} else if (strncmp(token,"STATUS",6)==0){
				ignore_tokens = process_esp_response_status(token, token_count);
				if (ignore_tokens >= 0) { retval = 1; }		// OK to follow
			} else if (strncmp(token,"OK",2)==0) {
				if (esp_init_step) {
					if (--esp_init_step) {
						init_sequence();
					}
				}
				if (token_count == 1) {
					retval = 0;
				}
			} else if (strncmp(token,"ATE0",4)==0) {		// echo of ATE0 command
				retval = 0;
				if (--esp_init_step) {
					init_sequence();
				}
			} else {
				term_print( "%s() - unknown token%d=%s\r\n", __FUNCTION__, token_count, token);
			}
		}
		token = strtok(NULL, s);
	}
	//term_print( "%s() - retval=%d\r\n", __FUNCTION__, retval);
	return retval;
}

void evaluate_esp_response(char* response, int len) {
	char *line;
	uint8_t line_count = 0;
	//const char s[2] = {0x0A, 0x0D};		// line separator
	int ignore_lines = 0;

	term_print("%s() - %d bytes: %s\r\n", __FUNCTION__, strlen(response), response);
	term_print_hex((uint8_t*)response, len, 0);
	term_print("\r\n");

	//return;

	// did we receive data from a connected client?
	if (strncmp(response,"+IPD",4)==0) {
		process_esp_response_ipd(response, len);
		return;
	}

	// ESP-01 response
	// process each line
	// get first line
	line = strtok(response, "\r\n");

	// iterate through remaining lines
	while( line != NULL ) {
		line_count++;
		if (ignore_lines > 0) {
			ignore_lines--;
			continue;
		}
		strcpy((char*)line_buf, line);
		ignore_lines = process_esp_response_line(line_buf, line_count);
		if (ignore_lines < 0) {
			term_print( "%s() - Error[%d] <<%s>>\r\n", __FUNCTION__, line_count, line_buf);
		} else {
			term_print( "%s() - %d:<<%s>> ignore %d\r\n", __FUNCTION__, line_count, line_buf, ignore_lines);
		}
		line = strtok(NULL, "\r\n");
	}

}

