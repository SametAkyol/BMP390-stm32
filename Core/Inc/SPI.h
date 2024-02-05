/*
 * SPI.h
 *
 *  Created on: Dec 17, 2023
 *      Author: Samet
 */

#ifndef SRC_SPI_H_
#define SRC_SPI_H_
#include "main.h"


typedef HAL_StatusTypeDef mystatus;

class SPI
{
public:

	mystatus writeRegister8(uint8_t regAddress, uint8_t data);
	mystatus writeRegister16(uint8_t regAddress, uint16_t pData);
	mystatus writeRegisterMulti(uint8_t regAddress, uint8_t* pData, uint16_t len);

	mystatus readRegister8(uint8_t regAddress, uint8_t* pBuffer);
	mystatus readRegister16(uint8_t regAddress, uint16_t* pBuffer);
	mystatus readRegisterMulti(uint8_t regAddress, uint8_t* pBuffer,uint16_t len);
protected:
	SPI_HandleTypeDef *_hspi;
	GPIO_TypeDef *csGPIO;
	uint16_t _csPin;
	void csHigh();
	void csLow();
private:

};



#endif /* SRC_SPI_H_ */
