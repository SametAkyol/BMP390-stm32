/*
 * BMP390Defs.h
 *
 *  Created on: Dec 16, 2023
 *      Author: Samet
 */

#ifndef INC_BMP390DEFS_H_
#define INC_BMP390DEFS_H_
#include <stdint.h>
#include <stdbool.h>

#define IIR_FILTER_Pos 1
#define ODR_SEL_Pos 0
#define OSR_P_Pos 0
#define OSR_T_Pos 3
#define PWR_Mode_Pos 4
#define DATA_SELECT_Pos    3
#define FIFO_SUBSAMPLING_Pos 0

#define PWR_PRESS_EN_Msk   (1 << 0)
#define PWR_TEMP_EN_Msk    (1 << 1)

#define INT_OD_Msk         (1 << 0)
#define INT_LEVEL_Msk      (1 << 1)
#define INT_LATCH_Msk      (1 << 2)
#define INT_FWTM_EN_Msk    (1 << 3)
#define INT_FFULL_EN_Msk   (1 << 4)
#define INT_DS_Msk         (1 << 5)
#define INT_DRDY_EN_Msk    (1 << 6)
#define CMD_RDY_Msk 	   (1 << 4)
#define CMD_ERR_Msk  	   (1 << 1)
#define CONFIG_ERROR_MASK  (1 << 2)

#define FWM_INT_Msk		   (1 << 0)
#define FFULL_INT_Msk	   (1 << 1)
#define DRDY_INT_Msk  	   (1 << 2)



#define FIFO_MODE_Msk	   (1<<0)
#define FIFO_STOP_ON_FULL_Msk (1<<1)
#define FIFO_TIME_EN_Msk    (1<<2)
#define FIFO_PRESS_EN_Msk  (1<<3)
#define FIFO_TEMP_EN_Msk   (1<<4)
#define SPI3W_Msk           (1<<0)
#define I2C_WDT_EN_Msk		(1<<1)
#define I2C_WDT_SEL_Msk		(1<<2)


#define CHIP_ID_REG_ADDR    0x00

#define ERR_REG_ADDR        0x02
#define STATUS_REG_ADDR     0x03
#define DATA_0_REG_ADDR     0x04
#define DATA_3_REG_ADDR     0x07
#define SENSORTIME_O_REG_ADDR   0x0C
#define EVENT_REG_ADDR      0x10
#define INT_STATUS_REG_ADDR 0x11
#define FIFO_DATA_REG_ADDR 0x14
#define PWR_CTRL_REG_ADDR   0x1B
#define OSR_REG_ADDR        0x1C
#define ODR_REG_ADDR        0x1D
#define CFG_REG_ADDR        0x1F
#define FIFO_WTM_0_REG_ADDR	0x15
#define FIFO_WTM_1_REG_ADDR 0x16
#define FIFO_CONFIG_1       0x17
#define FIFO_CONFIG_2       0x18
#define INT_CTRL_REG_ADDR   0x19
#define CALIB_REG_ADDR      0x31
#define CMD_REG_ADDR        0x7E


#define CALIB_DATA_LEN 21
#define FILTERED 1
#define UNFILTERED 0


#define SOFT_RESET 0xB6
#define FIFO_FLUSH 0xB0

typedef enum {
	Disable, Enable
} State;

typedef enum {
	Fatal_Error,
	Command_Error,
	Config_Error,
	Communication_Error,
	No_Error,
	BUSY
} sensor_status_t;

typedef enum {
	Mode_Sleep, Mode_Forced, Mode_Normal = 3
} Mode;

typedef struct {
	float t;
	uint16_t PAR_T1;
	uint16_t PAR_T2;
	uint16_t PAR_P5;
	uint16_t PAR_P6;
	int16_t PAR_P1;
	int16_t PAR_P2;
	int16_t PAR_P9;
	int8_t PAR_T3;
	int8_t PAR_P3;
	int8_t PAR_P4;
	int8_t PAR_P7;
	int8_t PAR_P8;
	int8_t PAR_P10;
	int8_t PAR_P11;
} Calib_Data;

typedef enum {
	OSR_NO, OSR_X1, OSR_X2, OSR_X4, OSR_X8, OSR_X16, OSR_X32,
} BMP_OverSampling;

typedef enum {
	ODR_200,
	ODR_100,
	ODR_50,
	ODR_25,
	ODR_12p5,
	ODR_6p25,
	ODR_3p1,
	ODR_1p5,
	ODR_0p78,
} BMP_ODR;

typedef enum {
	coef_0, coef_1, coef_3, coef_7, coef_15, coef_31, coef_63, coef_127
} BMP_IIR;

typedef struct {
	uint8_t Filter;
	uint8_t Int;
	uint8_t ODR;
	uint8_t OSR;
	uint8_t CTRL;
	uint8_t FIFO_CFG1;
	uint8_t FIFO_CFG2;
	uint8_t IF_CFG;
} BMP_init;

#endif /* INC_BMP390DEFS_H_ */
