/*
 * https://github.com/sumancvb/at24cxx
 */
#include "main.h"
#include "ee24.h"
#include "term.h"

#define ee24_delay(x)   HAL_Delay(x)

extern I2C_HandleTypeDef hi2c1;

#define _EEPROM_PSIZE	8		// Page size
#define _EEPROM_ADDRESS 0xA0	// I2C device address
#define _EEPROM_I2C hi2c1		// I2C port


uint8_t ee24_lock = 0;

/*
 * @ brief  Check if EEPROM device is present
 */
bool ee24_isConnected(void)
{
  if (HAL_I2C_IsDeviceReady(&_EEPROM_I2C, _EEPROM_ADDRESS, 2, 200)!=HAL_OK)
    return false;

  return true;
}

/*
 * @ brief  Blocking call to write one byte to EEPROM
 */
bool ee24_write_byte(uint16_t address, uint8_t *data)
{
	if (ee24_lock == 1) return false;
	ee24_lock = 1;

	if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, 1, 100) != HAL_OK)
	//if (HAL_I2C_Mem_Write_DMA(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, 1) != HAL_OK)
	{
	/*	term_print("%s() HAL result = %u\r\n",__FUNCTION__, result);
		if (result == HAL_BUSY) { term_print("HAL_BUSY\r\n"); }
		if (result == HAL_ERROR) {
			switch(hi2c1.ErrorCode) {
			case HAL_I2C_ERROR_AF:
				term_print("HAL_I2C_ERROR_AF\r\n");
				break;
			default:
				term_print("HAL_ERROR %u\r\n", hi2c1.ErrorCode);
			}
		}*/
		ee24_lock = 0;
		return false;
	}
	ee24_lock = 0;
	return true;
}

/*
 * @ brief  Blocking call to write one word to EEPROM
 */
bool ee24_write_word(uint16_t address, uint16_t *data)
{
	if (ee24_lock == 1) return false;
	ee24_lock = 1;

	if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, 2, 100) != HAL_OK)
	//if (HAL_I2C_Mem_Write_DMA(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, 2) != HAL_OK)
	{
		ee24_lock = 0;
		return false;
	}
	ee24_lock = 0;
	return true;
}

/*
 * @ brief  Function Not Implemented ! (Blocking call to write bytes to EEPROM)
 */
bool ee24_write(uint16_t address, uint8_t *data, size_t len, uint32_t timeout)
{
	return false;
/* Not implemented
  if (ee24_lock == 1)
    return false;
  ee24_lock = 1;
  uint16_t w;
  uint32_t startTime = HAL_GetTick();

  while (1)
  {
    w = _EEPROM_PSIZE - (address  % _EEPROM_PSIZE);
    if (w > len)
      w = len;
    if (HAL_I2C_Mem_Write_DMA(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, w) == HAL_OK)
    {
      ee24_delay(10);
      len -= w;
      data += w;
      address += w;
      if (len == 0)
      {
        ee24_lock = 0;
        return true;
      }
      if (HAL_GetTick() - startTime >= timeout)
      {
        ee24_lock = 0;
        return false;
      }
    }
    else
    {
      ee24_lock = 0;
      return false;
    }
  }
 */
}


/*
 * @ brief  Blocking call to read one byte from EEPROM
 */
bool ee24_read_byte(uint8_t address, uint8_t *data)
{
	if (ee24_lock == 1) return false;
	ee24_lock = 1;


	if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, 1, 100) != HAL_OK )
	//if (HAL_I2C_Mem_Read_DMA(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, 1) != HAL_OK)
	{
		ee24_lock = 0;
		return false;
	}
	ee24_lock = 0;
	return true;
}

/*
 * @ brief  Blocking call to read one word from EEPROM
 */
bool ee24_read_word(uint8_t address, uint16_t *data)
{
	if (ee24_lock == 1) return false;
	ee24_lock = 1;

	if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, 2, 100) != HAL_OK )
	//if (HAL_I2C_Mem_Read_DMA(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, 2) != HAL_OK)
	{
		ee24_lock = 0;
		return false;
	}
	ee24_lock = 0;
	return true;
}

/*
 * @ brief  Blocking call to read bytes from EEPROM
 */
bool ee24_read(uint16_t address, uint8_t *data, size_t len, uint32_t timeout)
{
  if (ee24_lock == 1)
    return false;
  ee24_lock = 1;

  if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK )
  //if (HAL_I2C_Mem_Read_DMA(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, len) == HAL_OK)
  {
    ee24_lock = 0;
    return true;
  }
  else
  {
    ee24_lock = 0;
    return false;
  }
}

//################################################################################################################
