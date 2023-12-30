/*
 * https://github.com/sumancvb/at24cxx
 */
#include "main.h"
#include "ee24.h"
#include "term.h"

#define ee24_delay(x)   HAL_Delay(x)

extern I2C_HandleTypeDef hi2c1;

#define _EEPROM_PSIZE	16		// Page size
#define _EEPROM_ADDRESS 0xA0	// I2C device address
#define _EEPROM_I2C hi2c1		// I2C port


uint8_t ee24_lock = 0;
HAL_StatusTypeDef ee24_result;
uint32_t ee24_ErrorCode;

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

	ee24_result = HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
	//ee24_result = HAL_I2C_Mem_Write_DMA(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, 1);
	if (ee24_result != HAL_OK)
	{
		ee24_ErrorCode = _EEPROM_I2C.ErrorCode;
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

	ee24_result = HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, 2, 100);
	//ee24_result = HAL_I2C_Mem_Write_DMA(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, 2);
	if (ee24_result != HAL_OK)
	{
		ee24_ErrorCode = _EEPROM_I2C.ErrorCode;
		ee24_lock = 0;
		return false;
	}
	ee24_lock = 0;
	return true;
}

/*
 * @brief    Blocking call to write bytes to EEPROM
 * @retval   true on success
 */
bool ee24_write(uint16_t address, uint8_t *data, size_t len, uint32_t timeout)
{
	bool retval = false;
	uint16_t first = address, last=address+len;
	// check if all bytes are within page (16 byte) boundary
	if ( (first & 0xF0) != (last & 0xF0) ) {
		return false;		// fail if write goes outside page boundary
	}

	if (ee24_lock == 1) return false;
	ee24_lock = 1;

	if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK) {
		retval = true;
	}
	ee24_lock = 0;
	return retval;
}


/*
 * @brief    Blocking call to read one byte from EEPROM
 * @retval   true on success
 */
bool ee24_read_byte(uint8_t address, uint8_t *data)
{
	if (ee24_lock == 1) return false;
	ee24_lock = 1;

	ee24_result = HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
	if (ee24_result != HAL_OK )
	//if (HAL_I2C_Mem_Read_DMA(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, 1) != HAL_OK)
	{
		ee24_ErrorCode = _EEPROM_I2C.ErrorCode;
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

	ee24_result = HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, 2, 100);
	if (ee24_result != HAL_OK)
	//if (HAL_I2C_Mem_Read_DMA(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, 2) != HAL_OK)
	{
		ee24_ErrorCode = _EEPROM_I2C.ErrorCode;
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
	if (ee24_lock == 1) return false;
	ee24_lock = 1;

	ee24_result = HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, len, 100);
	//ee24_result = HAL_I2C_Mem_Read_DMA(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, len)
	if (ee24_result != HAL_OK )
	{
		ee24_ErrorCode = _EEPROM_I2C.ErrorCode;
		ee24_lock = 0;
		return false;
	}
	ee24_lock = 0;
	return true;
}

//################################################################################################################
