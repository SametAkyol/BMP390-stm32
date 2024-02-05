#include "I2C.h"

static size_t calcAddrSize(uint16_t val) {
	if (val>255 && val< (1<<16))
	{
		return 2;
	}
	return 1;
}



mystatus I2C::writeRegister8(uint16_t regAddress, uint8_t data) {
	return (HAL_I2C_Mem_Write(_hi2c, _devAddress, regAddress, calcAddrSize(regAddress), &data, I2C_U8toByte,I2C_MAX_DELAY));
}

mystatus I2C::writeRegister16(uint16_t regAddress, uint16_t Data) {
	uint8_t buffer[2];
	buffer[0]= (uint8_t) Data >> 8;
	buffer[1]= (uint8_t) Data & 0xFF;
	
	return (HAL_I2C_Mem_Write(_hi2c, _devAddress, regAddress, calcAddrSize(regAddress), buffer, I2C_U16toByte, I2C_MAX_DELAY));
}

mystatus I2C::writeRegisterMulti(uint16_t regAddress, uint8_t* pData, uint16_t len) {
	return (HAL_I2C_Mem_Write(_hi2c, _devAddress, regAddress, calcAddrSize(regAddress), pData, len, I2C_MAX_DELAY));
}

mystatus I2C::readRegister8(uint16_t regAddress, uint8_t *pBuffer) {
	return (HAL_I2C_Mem_Read(_hi2c, _devAddress, regAddress, calcAddrSize(regAddress), pBuffer, I2C_U8toByte, I2C_MAX_DELAY));
}

mystatus I2C::readRegister16(uint16_t regAddress, uint16_t* pBuffer) {
	uint8_t temp[2];
	mystatus s=HAL_I2C_Mem_Read(_hi2c, _devAddress, regAddress, calcAddrSize(regAddress), temp, I2C_U16toByte, I2C_MAX_DELAY);
	if (s!=HAL_OK)
	{

	}
	*pBuffer= (uint16_t) temp[1]<<8 | temp[0];

}

mystatus I2C::readRegisterMulti(uint16_t regAddress, uint8_t* pBuffer, uint16_t len) {
	return (HAL_I2C_Mem_Read(_hi2c, _devAddress, regAddress, calcAddrSize(regAddress), pBuffer, len, I2C_MAX_DELAY));
}
