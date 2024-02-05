/*
 * BMP390.cpp
 *
 *  Created on: Dec 16, 2023
 *      Author: Samet
 */

#include "BMP390.h"
UART_HandleTypeDef huart2;
static void DEBUGPRINT(const char *str)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
}

BMP390::BMP390(I2C_HandleTypeDef *hi2c, uint8_t bmpAddr)
{
	_devAddress = bmpAddr;
	_hi2c = hi2c;

	BMP_init init;

	init.CTRL = PWR_PRESS_EN_Msk | PWR_TEMP_EN_Msk | Mode_Normal << PWR_Mode_Pos; //default all disable

	init.Filter = coef_127 << IIR_FILTER_Pos;

	init.Int = ((INT_LEVEL_Msk) | (INT_DRDY_EN_Msk)) & (~INT_LATCH_Msk) & (~INT_OD_Msk); // Push-Pull , Data ready int enable, Rising edge

	init.OSR = (OSR_X1 << OSR_T_Pos) | (OSR_X8 << OSR_P_Pos);

	init.ODR = ODR_200;

	init.FIFO_CFG1=(FIFO_MODE_Msk) | (FIFO_PRESS_EN_Msk) | (FIFO_TEMP_EN_Msk);

	init.FIFO_CFG2=(FILTERED<<DATA_SELECT_Pos) | 1<<FIFO_SUBSAMPLING_Pos;

	init.IF_CFG= (~SPI3W_Msk) & ((I2C_WDT_EN_Msk) | (I2C_WDT_SEL_Msk));

	Init(&init);
}

BMP390::BMP390(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
	_hspi=hspi;
	csGPIO =csPort;
	_csPin= csPin;

	BMP_init init;

	init.CTRL = PWR_PRESS_EN_Msk | PWR_TEMP_EN_Msk | Mode_Normal << PWR_Mode_Pos; //default all disable

	init.Filter = coef_127 << IIR_FILTER_Pos;

	init.Int = ((INT_LEVEL_Msk) | (INT_DRDY_EN_Msk)) | (INT_LATCH_Msk) | (INT_OD_Msk);

	init.OSR = (OSR_X1 << OSR_T_Pos) | (OSR_X8 << OSR_P_Pos);

	init.ODR = ODR_200;

	init.FIFO_CFG1=(FIFO_MODE_Msk) | (FIFO_PRESS_EN_Msk) | (FIFO_TEMP_EN_Msk) ;

	init.FIFO_CFG2=FILTERED<<DATA_SELECT_Pos | 1<<FIFO_SUBSAMPLING_Pos ;

	init.IF_CFG= SPI3W_Msk | I2C_WDT_EN_Msk ;

	Init(&init);
}

BMP390::~BMP390()
{

}

void BMP390::Error_Handler(const char *str)
{
	DEBUGPRINT(str);
}

void BMP390::Interrupt_Callback(void)
{
	uint8_t status = readIntStatus();
	if (status &FWM_INT_Msk)
	{
		//FIFO Watermark Interrupt
	}

	if (status & FFULL_INT_Msk)
	{
		//FIFO Full Interrupt
	}

	if (status & DRDY_INT_Msk)
	{
		//data ready
	}
}

int BMP390::readIntStatus(void) {
	uint8_t pBuffer;
	status s = PROTOCOL::readRegister8(INT_STATUS_REG_ADDR, &pBuffer);
	if (s!=HAL_OK)
	{
		return -1;
	}
	return pBuffer;
}

sensor_status_t BMP390::Init(BMP_init *init) {
	uint8_t data;
	PROTOCOL::writeRegister8( PWR_CTRL_REG_ADDR, init->CTRL);
	PROTOCOL::writeRegister8( OSR_REG_ADDR, init->OSR);
	PROTOCOL::writeRegister8( ODR_REG_ADDR, init->ODR);
	PROTOCOL::writeRegister8( CFG_REG_ADDR, init->Filter);
	PROTOCOL::writeRegister8( INT_CTRL_REG_ADDR, init->Int);


	HAL_Delay(10);
	PROTOCOL::readRegister8( ERR_REG_ADDR, &data);
	if (data & CONFIG_ERROR_MASK)
	{
		return Config_Error;
	}
	else
	{
		return No_Error;
	}

}

uint32_t BMP390::readSensorTime(void) {
	uint8_t buffer[3];
	status s = PROTOCOL::readRegisterMulti( SENSORTIME_O_REG_ADDR, buffer, 3);
	if (HAL_OK != s)
	{
		return Communication_Error;
	}
	return (static_cast<uint32_t>(((buffer[2] << 16) | (buffer[1] << 8)	| (buffer[0]))));
}

uint8_t BMP390::readChipID(void) {
	uint8_t data = 0;
	status s = PROTOCOL::readRegister8( CHIP_ID_REG_ADDR, &data);
	if (HAL_OK != s)
	{
		return Communication_Error;
	}
	return data;
}

float BMP390::readTemperature(void)
{
	uint8_t buffer[3];
	PROTOCOL::readRegisterMulti(DATA_3_REG_ADDR, buffer, 3);
	uint32_t temp_raw = (buffer[2] << 16) | (buffer[1] << 8) | (buffer[0]);
	float temp1, temp2;
	temp1 = (float) (temp_raw - calib.PAR_T1);
	temp2 = (float) (temp1 * calib.PAR_T2);
	calib.t = temp2 + (temp1 * temp1) * calib.PAR_T3;
	return calib.t;
}

float BMP390::readPressure(void)
{
	float data1, data2, data3, data4, out1, out2;
	uint8_t buffer[3];
	PROTOCOL::readRegisterMulti(DATA_0_REG_ADDR, buffer, 3);
	uint32_t press_raw = static_cast<uint32_t>((buffer[2] << 16)
			| (buffer[1] << 8) | (buffer[0]));
	data1 = calib.PAR_P6 * calib.t;
	data2 = calib.PAR_P7 * (calib.t * calib.t);
	data3 = calib.PAR_P8 * (calib.t * calib.t * calib.t);
	out1 = calib.PAR_P5 + data1 + data2 + data3;

	data1 = calib.PAR_P2 * calib.t;
	data2 = calib.PAR_P3 * (calib.t * calib.t);
	data3 = calib.PAR_P4 * (calib.t * calib.t * calib.t);

	out2 = (float) press_raw * (calib.PAR_P1 + data1 + data2 + data3);

	data1 = (float) press_raw * (float) press_raw;
	data2 = calib.PAR_P9 + calib.PAR_P10 * calib.t;
	data3 = data1 + data2;
	data4 = data3
			+ ((float) press_raw * (float) press_raw * (float) press_raw)
					* calib.PAR_P11;
	return (out1 + out2 + data4);
}

sensor_status_t BMP390::readCalibData(void)
{
	uint8_t buffer[CALIB_DATA_LEN];
	status s = PROTOCOL::readRegisterMulti(CALIB_REG_ADDR, buffer, CALIB_DATA_LEN);
	if (HAL_OK != s)
	{
		return Communication_Error;
	}

	calib.PAR_T1 = static_cast<uint16_t>((buffer[1] << 8) | buffer[0]);
	calib.PAR_T2 = static_cast<uint16_t>((buffer[3] << 8) | buffer[2]);
	calib.PAR_T3 = static_cast<int8_t>(buffer[4]);
	calib.PAR_P1 = static_cast<int16_t>((buffer[6] << 8) | buffer[5]);
	calib.PAR_P2 = static_cast<int16_t>((buffer[8] << 8) | buffer[7]);
	calib.PAR_P3 = static_cast<int8_t>(buffer[9]);
	calib.PAR_P4 = static_cast<int8_t>(buffer[10]);
	calib.PAR_P5 = static_cast<uint16_t>((buffer[12] << 8) | buffer[11]);
	calib.PAR_P6 = static_cast<uint16_t>((buffer[14] << 8) | buffer[13]);
	calib.PAR_P7 = static_cast<int8_t>(buffer[15]);
	calib.PAR_P8 = static_cast<int8_t>(buffer[16]);
	calib.PAR_P9 = static_cast<int16_t>((buffer[18] << 8) | buffer[17]);
	calib.PAR_P10 = static_cast<int8_t>(buffer[19]);
	calib.PAR_P11 = static_cast<int8_t>(buffer[20]);

	return No_Error;
}


sensor_status_t BMP390::SoftReset(void)
{
	uint8_t pBuffer;
	status s = PROTOCOL::readRegister8(STATUS_REG_ADDR, &pBuffer);
	if (HAL_OK != s)
	{
		return Communication_Error;
	}

	if (pBuffer & CMD_RDY_Msk)
	{
		s = PROTOCOL::writeRegister8( CMD_REG_ADDR, SOFT_RESET);
	}else
	{
		return BUSY;
	}

	s = PROTOCOL::readRegister8(ERR_REG_ADDR, &pBuffer);

	if (pBuffer & CMD_ERR_Msk)
	{
		return Command_Error;
	}
	return No_Error;
}

sensor_status_t BMP390::FIFO_Flush(void)
{
	uint8_t pBuffer;
	status s = PROTOCOL::readRegister8(STATUS_REG_ADDR, &pBuffer);
	if (HAL_OK != s)
	{
		return Communication_Error;
	}
	if (pBuffer & CMD_RDY_Msk)
	{
		s = PROTOCOL::writeRegister8( CMD_REG_ADDR, FIFO_FLUSH);
		return No_Error;
	}
	return BUSY;
}

float BMP390::getAltitude(float seaLevelhPa)
{
	float altitude;

		float pressure = readPressure();
		pressure /= 100;

		altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

		return altitude;
}

uint16_t BMP390::getFIFO_Size(void) {
	uint16_t size;
	status s = PROTOCOL::readRegister16(FIFO_WTM_0_REG_ADDR, &size);
	if (s != HAL_OK)
	{
		return 1000;
	}
	return size;
}

sensor_status_t BMP390::readFIFO(void)
{
	uint16_t sz=getFIFO_Size();
	uint8_t pBuffer[sz];
	mystatus s=PROTOCOL::readRegisterMulti(FIFO_DATA_REG_ADDR, pBuffer,sz);
	if (s!=HAL_OK) {
		return Communication_Error;
	}
	processFIFO_Data(pBuffer);
	return No_Error;
}


int BMP390::setFIFO_Watermark(uint16_t mrk)
{
	if (mrk>512) {
		return -1;
	}
	PROTOCOL::writeRegister16(FIFO_WTM_0_REG_ADDR, mrk);
	return 31;
}



