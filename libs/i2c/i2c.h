/*
 * i2c.h
 *
 *  Created on: 7 Oct 2024
 *      Author: minht57
 */

#ifndef LIBS_I2C_I2C_H_
#define LIBS_I2C_I2C_H_

#include "inc/hw_memmap.h"
#include <stdint.h>

#define I2C0    I2C0_BASE
#define I2C1    I2C1_BASE
#define I2C2    I2C2_BASE
#define I2C3    I2C3_BASE

void I2CInit(uint32_t i2c);
void I2CWriteSingleByte(uint32_t ui32Id, uint8_t ui8Addr, uint8_t ui8RegAddr, uint8_t ui8Data);
void I2CWriteMultiByte(uint32_t ui32Id, uint8_t ui8Addr, uint8_t ui8RegAddr, uint8_t* pui8Data, uint8_t ui8Length);
uint8_t I2CReadSingleByte(uint32_t ui32Id, uint8_t ui8Addr, uint8_t ui8RegAddr);
void I2CReadMultiByte(uint32_t ui32Id, uint8_t ui8Addr, uint8_t ui8RegAddr, uint8_t* pui8Data, uint8_t ui8Length);


#endif /* LIBS_I2C_I2C_H_ */
