#ifndef	_EE24_H
#define	_EE24_H

#ifdef __cplusplus
extern "C" {
#endif
/*
  Author:     Nima Askari
  WebSite:    http://www.github.com/NimaLTD
  Instagram:  http://instagram.com/github.NimaLTD
  Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw

  Version:    2.2.1
*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
//#include "ee24Config.h"

bool ee24_isConnected(void);
bool ee24_write_byte(uint16_t address, uint8_t *data);
bool ee24_write(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);
bool ee24_read_byte(uint8_t address, uint8_t *data);
bool ee24_read(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif
