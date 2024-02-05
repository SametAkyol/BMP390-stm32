/*
 * Communication.h
 *
 *  Created on: 30 Ara 2023
 *      Author: Samet
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_
#include "I2C.h"
#include "SPI.h"

class Communication : public I2C, public SPI {
public:
	typedef I2C PROTOCOL;

private:
};



#endif /* INC_COMMUNICATION_H_ */
