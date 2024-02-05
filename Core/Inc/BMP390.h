/*
 * BMP390.h
 *
 *  Created on: Dec 16, 2023
 *      Author: Samet
 */

#ifndef BMP390_H_
#define BMP390_H_
#include "BMP390Defs.h"
#include "main.h"
#include <string.h>
#include <math.h>
#include "Communication.h"
typedef HAL_StatusTypeDef status;


class BMP390 : public Communication {
public:
    BMP390(I2C_HandleTypeDef* hi2c, uint8_t bmpAddr);
    BMP390(SPI_HandleTypeDef* hspi, GPIO_TypeDef* GPIOx, uint16_t csPin);
    virtual ~BMP390();

    sensor_status_t Init(BMP_init* init);
    sensor_status_t readCalibData(void);
    sensor_status_t SoftReset(void);
    sensor_status_t FIFO_Flush(void);

    uint32_t readSensorTime(void);
    uint16_t getFIFO_Size(void);
    uint8_t readChipID(void);

    float readTemperature(void);
    float readPressure(void);
    float getAltitude(float);

    sensor_status_t readFIFO(void);
    void Interrupt_Callback(void);


    int readIntStatus(void);
    int setFIFO_Watermark(uint16_t VAL);

    void processFIFO_Data(uint8_t*);
private:
    Calib_Data calib = {0};
    void Error_Handler(const char*);

protected:
};

#endif /* BMP390_H_ */
