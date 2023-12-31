#ifndef	_EE24_H
#define	_EE24_H

#ifdef __cplusplus
extern "C" {
#endif
/*
  Author:     Nima Askari
  WebSite:    http://www.github.com/NimaLTD

  Version:    2.2.1
*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define EEPROM_ADDR_VERSION 0x00		// version number Major in low byte, minor in high byte
#define EEPROM_ADDR_TIM2ARR 0x02		// TIM2 value adjusted to give 25us pulse
#define EEPROM_ADDR_MODBUSADDR 0x04		// Modbus address
#define EEPROM_ADDR_TIM3ARR 0x06		// TIM3 value adjusted to give 100ms pulse

// Note: VAH array and WH array are each located inside a single page (16 bytes) so they can be written in a single command
#define EEPROM_ADDR_VAH 0xD0			// - 0xDB : 3 x 32bit values (12 bytes)
#define EEPROM_ADDR_WH 0xE0				// - 0xEB : 3 x 32bit values (12 bytes)

#define EEPROM_DELAY 4					// [ms] delay between consecutive EPPROM operations

bool ee24_isConnected(void);
bool ee24_write_byte(uint16_t address, uint8_t *data);
bool ee24_write_word(uint16_t address, uint16_t *data);
bool ee24_write(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);
bool ee24_read_byte(uint8_t address, uint8_t *data);
bool ee24_read_word(uint8_t address, uint16_t *data);
bool ee24_read(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif
