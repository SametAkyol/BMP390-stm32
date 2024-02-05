/*
 * SPI.cpp
 *
 *  Created on: Dec 17, 2023
 *      Author: Samet
 */

#include "SPI.h"

void SPI::csHigh() {
	HAL_GPIO_WritePin(csGPIO, _csPin, GPIO_PIN_SET);
}

void SPI::csLow() {
	HAL_GPIO_WritePin(csGPIO, _csPin, GPIO_PIN_RESET);
}

mystatus SPI::writeRegister8(uint8_t regAddress, uint8_t val) {
	HAL_StatusTypeDef status;
	uint8_t data[2];
	data[0] = regAddress;
	data[1] = val;

	csLow();
	status = HAL_SPI_Transmit(_hspi, data, 2, 10);
	csHigh();
	return status;
}

mystatus SPI::writeRegister16(uint8_t regAddress, uint16_t Data) {
	HAL_StatusTypeDef status;
	uint8_t buffer[3];
	buffer[0] = regAddress;
	buffer[1] = (uint8_t) Data >> 8;
	buffer[2] = (uint8_t) Data & 0xFF;
	csLow();
	status = HAL_SPI_Transmit(_hspi, buffer, 3, 10);
	csHigh();

	return status;
}

mystatus SPI::writeRegisterMulti(uint8_t regAddress, uint8_t *pData,
		uint16_t len) {
	HAL_StatusTypeDef status;
	csLow();
	HAL_SPI_Transmit(_hspi, &regAddress, 1, 10);
	status = HAL_SPI_Transmit(_hspi, pData, len, 10);

	csHigh();
	return status;
}

mystatus SPI::readRegister8(uint8_t regAddress, uint8_t *pBuffer) {
	HAL_StatusTypeDef status;
	regAddress |= 0x80;
	csLow();
	HAL_SPI_Transmit(_hspi, &regAddress, 1, 10);
	status = HAL_SPI_Receive(_hspi, pBuffer, 1, 10);
	csHigh();
	return status;
}

mystatus SPI::readRegister16(uint8_t regAddress, uint16_t *pBuffer) {
	HAL_StatusTypeDef status;
	uint8_t buffer[2];
	regAddress |= 0x80;
	csLow();
	HAL_SPI_Transmit(_hspi, &regAddress, 1, 10);
	status = HAL_SPI_Receive(_hspi, buffer, 2, 10);
	csHigh();
	*pBuffer = (buffer[1] << 8) | buffer[0];
	return status;
}

mystatus SPI::readRegisterMulti(uint8_t regAddress, uint8_t *pBuffer,
		uint16_t len) {
	HAL_StatusTypeDef status;
	regAddress |= 0x80;
	csLow();
	HAL_SPI_Transmit(_hspi, &regAddress, 1, 10);
	status = HAL_SPI_Receive(_hspi, pBuffer, len, 10);
	csHigh();
	return status;
}

