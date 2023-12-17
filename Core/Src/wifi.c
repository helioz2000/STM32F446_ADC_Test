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
const char* empty_ip = "---.---.---.---";

bool esp_first_rx = false;
bool esp_wifi_connected = false;
bool esp_wifi_got_ip = false;
bool esp_con_is_up = false;
uint8_t esp_cmd_step = 0;
bool client_connection[10] = { false, false, false, false, false, false, false, false, false, false };
char ip_addr_str[16] = "000.000.000.000";
char mac_addr_str[18] = "00:00:00:00:00:00";

extern UART_HandleTypeDef huart3;

#define ESP_UART huart3
#define MODBUS_SERVER_PORT 80

/*
 * Private function declarations
 */
void evaluate_esp_response(char* response, int len);
void cmd_sequence(void);
void at_echo(bool on_off);
void set_connection_status(uint8_t new_status);

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
		//term_print("\r\nESP first RX\r\n");
		esp_first_rx = true;
		HAL_Delay(50);
		at_echo(false);		// turn echo off
	} else {
		esp_rx_buf[esp_rx_count] = 0;		// set EOS
		//term_print("\r\n%s() - <%s>\r\n", __FUNCTION__, esp_rx_buf);
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
void cmd_sequence() {
	//term_print( "%s() - step: %d\r\n", __FUNCTION__, esp_cmd_step );
	switch (esp_cmd_step) {
	case 6:
		sprintf((char*)esp_tx_buf, "AT+CIFSR\r\n");		// get IP address
		HAL_UART_Transmit(&ESP_UART, (uint8_t*)esp_tx_buf, strlen((char*)esp_tx_buf), 1000);
		esp_cmd_step --;
		break;
	case 4:
		sprintf((char*)esp_tx_buf, "AT+CIPMUX=1\r\n");	// start server
		HAL_UART_Transmit(&ESP_UART, (uint8_t*)esp_tx_buf, strlen((char*)esp_tx_buf), 1000);
		esp_cmd_step --;
		break;
	case 2:
		sprintf((char*)esp_tx_buf, "AT+CIPSERVER=1,%d\r\n", MODBUS_SERVER_PORT);	// start server
		HAL_UART_Transmit(&ESP_UART, (uint8_t*)esp_tx_buf, strlen((char*)esp_tx_buf), 1000);
		esp_cmd_step --;
		break;
	default:
	}
}

/*
 * @brief  disable echo
 * @para   on_off  true to enable echo, false to disable
 */
void at_echo(bool on_off) {
	if (on_off == true) {
		sprintf((char*)esp_tx_buf, "ATE1\r\n");	// enable echo
	} else {
		sprintf((char*)esp_tx_buf, "ATE0\r\n");	// disable echo
	}
	HAL_UART_Transmit(&ESP_UART, (uint8_t*)esp_tx_buf, strlen((char*)esp_tx_buf), 1000);
}

/*
 * @brief  perform functions on TCP Link Up/Down
 * @para   up_down  true = up, false = down
 */
void on_link(bool up_down) {
	if (up_down == true) {
		term_print("%s - LINK UP\r\n", __FUNCTION__);
		esp_cmd_step = 6;	// kick off commands
		cmd_sequence();
	} else {
		strcpy(ip_addr_str, empty_ip);
		term_print("%s - LINK DOWN\r\n", __FUNCTION__);
	}
}

/*
 * @brief  Update connection status
 * @para   new_status  new connection status
 */
void set_connection_status(uint8_t new_status) {
	bool new_con = false;
	if (new_status == 2) { new_con = true; }

	if (new_con != esp_con_is_up) {
		esp_con_is_up = new_con;
		on_link( new_con );
	}
}

/*
 * @brief   Client connection messsage
 * @para    token  0,CONNECT or 0,DISCONNECT
 * @retval  -1 on failure
 */
int esp_client_connection(char* token) {
	uint8_t client_num;
	if (token[1] != ',') { return -1; }
	client_num = token[0] - '0';
	if (token[2] !=  'C') { return -1; }
	if (token[3] ==  'O') {		// CONNECT
		client_connection[client_num] = true;
		term_print("%s() - Client %d Connected\r\n", __FUNCTION__, client_num);
	} else if (token[3] == 'L') {
		client_connection[client_num] = false;
		term_print("%s() - Client %d Disconnected\r\n", __FUNCTION__, client_num);
	} else {
		term_print("%s() - Error:<<%s>>\r\n", __FUNCTION__, token);
		return -1;
	}

	return 0;
}

int esp_client_disconnect(uint8_t connection) {
	// Disconnect client
	sprintf((char*)esp_tx_buf, "AT+CIPCLOSE=%d\r\n", connection);	// start server
	HAL_UART_Transmit(&ESP_UART, (uint8_t*)esp_tx_buf, strlen((char*)esp_tx_buf), 1000);
	return 0;
}

/*
 * @brief   Process the data received from client connection
 * @param   data        buffer containing received data
 * @param   len         length of data buffer
 * @param   connection  the origin connection
 * @retval  0 on success
 */
int process_incoming_data(uint8_t *data, unsigned len, uint8_t connection) {

	term_print("%s() - %d bytes from connection %d\r\n", __FUNCTION__, len, connection);
	term_print_hex(data, len, 0);
	term_print("\r\n");

	// Disconnect client
	esp_client_disconnect(connection);

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
	char* token_ptr;
	int data_len = 0;
	uint8_t con;

	token = strtok_r(response, ",", &token_ptr);	// +IPD
	if (token == NULL) { return -1; }
	token = strtok_r(NULL,",", &token_ptr);	// connection number
	if (token == NULL) { return -1; }
	else { con = token[0] - '0'; }
	token = strtok_r(NULL, ":", &token_ptr);	// data length
	if (token == NULL) { return -1; }
	data_len = atoi(token);
	if ((data_len <= 0) || (data_len >= len)) { return -1; }	// sanity check on data length
	//We have valid data
	data_start = (uint8_t*) token_ptr;
	return (process_incoming_data(data_start, data_len, con));
}

/*
 * @brief   Process ESP responses starting with "WIFI ....."
 * @para    token      the token (word) which comes after "WIFI" in the ESP response
 * @para    token_num  the number if the token in the ESP response (WIFI=1)
 * @retval  -1 on failure, 0 or the number of tokens to be ignored
 */
int process_esp_response_wifi(char* token, uint8_t token_num) {
	//term_print( "%s() - token%d = <%s>\r\n", __FUNCTION__, token_num, token );
	uint8_t con_status = 0;
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
		con_status = 4;
	}
	if (esp_wifi_connected == true) {
		if (esp_wifi_got_ip == true) {
			con_status = 2;
		} else {
			con_status = 3;
		}
	}
	if (con_status != 0) {
		set_connection_status(con_status);
	}
	//term_print( "%s() - CONNECTED=%d GOT IP=%d (retval=%d)\r\n", __FUNCTION__, esp_wifi_connected, esp_wifi_got_ip, retval );
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
	uint8_t con_status = 0;
	if (strlen(token) != 8) { return -1; }
	if (token[6] != ':') { return -1; }
	switch(token[7]) {
	case '2':		// GOT IP
		esp_wifi_got_ip = true;
		esp_wifi_connected = true;
		con_status = 2;
		break;
	case '3':		// CONNECTED
		esp_wifi_connected = true;
		con_status = 3;
		break;
	case '4':		// DISCONNECTED
		esp_wifi_got_ip = false;
		esp_wifi_connected = false;
		con_status = 4;
		break;
	default:		// unknown status
		retval = -1;
	}
	// advise connection status
	if (retval == 0) {
		set_connection_status(con_status);
	}
	return retval;
}

/*
 * @brief   Process line starting with '+'
 * @para    line     the response line
 * @retval  -1 on failure, 0 or the number of lines to be ignored
 */
int process_esp_repsonse_plus(char* line) {
	int retval = -1;
	char* token;
	char* token_ptr;
	int len;

	//term_print( "%s() - <%s>\r\n", __FUNCTION__, line);

	token = strtok_r(line, ",", &token_ptr);
	if (strncmp(token, "+CIFSR", 6)==0) {
		if (line[10] == 'I') { 			// +CIFSR:STAIP,"192.168.0.xxx"
			len = strlen(token_ptr);
			token_ptr[len-1] = 0;	// remove " at end of string
			token_ptr[0] = 0;		// remove " at start of string
			token_ptr++;			// advance ptr to start of IP string
			strcpy(ip_addr_str, token_ptr);
			//term_print( "%s() - IP=<%s>\r\n", __FUNCTION__, ip_addr_str);
			retval = 0;
		} else if (line[10] == 'M') {	// +CIFSR:STAMAC,"bc:dd:c2:a1:25:79"
			len = strlen(token_ptr);
			token_ptr[len-1] = 0;	// remove " at end of string
			token_ptr[0] = 0;		// remove " at start of string
			token_ptr++;			// advance ptr to start of MAC string
			strcpy(mac_addr_str, token_ptr);
			//term_print( "%s() - MAC=<%s>\r\n", __FUNCTION__, mac_addr_str);
			retval = 0;
		}
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
	char* token_ptr;
	uint8_t token_count = 0;
	const char s[1] = {' '};	// token separator
	int ignore_tokens = 0;

	//term_print( "%s() - %d:<%s>\r\n", __FUNCTION__, line_num, line);

	if (line[0] == '+') {	// IP related info
		return process_esp_repsonse_plus(line);
	}

	// evaluate all tokens
	token = strtok_r(line, s, &token_ptr);
	while(token != NULL) {
		token_count++;
		//term_print( "%s() - token %d = <%s>\r\n", __FUNCTION__, token_count, token);
		if (ignore_tokens > 0) {
			ignore_tokens--;
		} else {
			if (strncmp(token,"WIFI",4)==0) {
				//term_print( "%s() - found WIFI\r\n", __FUNCTION__);
				token = strtok_r(NULL, s, &token_ptr);
				if (token != NULL) {
					token_count++;
					ignore_tokens = process_esp_response_wifi(token, token_count);
					if (ignore_tokens >= 0) { retval = 0; }
				}
			} else if (strncmp(token,"STATUS",6)==0){
				ignore_tokens = process_esp_response_status(token, token_count);
				if (ignore_tokens >= 0) { retval = 1; }		// OK to follow
			} else if (strncmp(token,"OK",2)==0) {
				//term_print( "%s() - found OK\r\n", __FUNCTION__);
				if (esp_cmd_step) {
					if (--esp_cmd_step) {
						cmd_sequence();
					}
				}
				if (token_count == 1) {
					retval = 0;
				}
			} else if (strncmp(token,"ATE0",4)==0) {		// echo of ATE0 command
				retval = 0;		// init_squence() will be triggered by OK
				//if (--esp_init_step) {
				//	init_sequence();
				//}
			} else if ((token[0]>='0') && (token[0]<='9')) {	// 0,CONNECT
				retval = esp_client_connection(token);
			} else {
				term_print( "%s() - unknown token%d=%s\r\n", __FUNCTION__, token_count, token);
			}
		}
		token = strtok_r(NULL, s, &token_ptr);
	}
	//term_print( "%s() - retval=%d\r\n", __FUNCTION__, retval);
	return retval;
}

void evaluate_esp_response(char* response, int len) {
	char *line;
	char *resp;
	char *token_ptr;
	uint8_t line_count = 0;
	const char s[2] = {0x0D, 0x0A};		// line separator
	int ignore_lines = 0;

	// copy response to our own buffer
	resp = malloc(len+1);
	strncpy(resp, response, len);
	resp[len] = 0;		// EOS
	/*
	term_print("%s() - %d bytes: %s\r\n", __FUNCTION__, strlen(resp), resp);
	term_print_hex((uint8_t*)resp, len, 0);
	term_print("\r\n");
	*/
	// did we receive data from a connected client? (CR LF +IPD,0,5:xxxxx)
	if ((resp[2]=='+')&&(resp[3]=='I')&&(resp[4]=='P')) {
		process_esp_response_ipd(resp, len);
		return;
	}

	// get first line from ESP response buffer
	line = strtok_r(resp, s, &token_ptr);

	// iterate through lines
	while( line != NULL ) {
		line_count++;
		//term_print( "%s() - %d:<%s>\r\n", __FUNCTION__, line_count, line);
		if (ignore_lines > 0) {
			ignore_lines--;
		} else {
			strcpy((char*)line_buf, line);
			ignore_lines = process_esp_response_line(line_buf, line_count);
			if (ignore_lines < 0) {
				term_print( "%s() - Error[%d] <<%s>>\r\n", __FUNCTION__, line_count, line_buf);
			} else {
				//term_print( "%s() - Line%d ignore_lines=%d\r\n", __FUNCTION__, line_count, ignore_lines);
			}
		}
		line = strtok_r(NULL, s, &token_ptr);	// get next line
	}
	//term_print( "%s() - %d lines found\r\n", __FUNCTION__, line_count);
	free(resp);
}

