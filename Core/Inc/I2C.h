
#ifndef _I2C_
#define _I2C_

#include <stdint.h>

#include "main.h"

#define I2C_MAX_DELAY 10
#define I2C_U16toByte 2
#define I2C_U8toByte 1




typedef HAL_StatusTypeDef mystatus;

class I2C
{
public:
	mystatus writeRegister8(uint16_t regAddress, uint8_t data);
	mystatus writeRegister16(uint16_t regAddress, uint16_t pData);
	mystatus writeRegisterMulti(uint16_t regAddress, uint8_t* pData, uint16_t len);

	mystatus readRegister8(uint16_t regAddress, uint8_t* pBuffer);
	mystatus readRegister16(uint16_t regAddress, uint16_t* pBuffer);
	mystatus readRegisterMulti(uint16_t regAddress, uint8_t* pBuffer,uint16_t len);
protected:
	I2C_HandleTypeDef* _hi2c;
		uint8_t _devAddress;
private:

};



#endif

