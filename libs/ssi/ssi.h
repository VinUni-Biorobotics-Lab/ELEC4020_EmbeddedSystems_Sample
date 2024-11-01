/*
 * spi.h
 *
 *  Created on: 9 Oct 2024
 *      Author: minht57
 */

#ifndef LIBS_SSI_SSI_H_
#define LIBS_SSI_SSI_H_

#include "inc/hw_memmap.h"
#include <stdint.h>

#define SSI0    SSI0_BASE
#define SSI1    SSI1_BASE
#define SSI2    SSI2_BASE
#define SSI3    SSI3_BASE

// Master mode only
void SSIInit(uint8_t ui8Id);
void SSIWriteSingleByte(uint8_t ui8Id, uint8_t ui8RegAddr, uint8_t ui8Data);
void SSIWriteMultiByte(uint8_t ui8Id, uint8_t ui8RegAddr, uint8_t* pui8Data, uint8_t ui8Length);
uint16_t SSIReadSingleByte(uint8_t ui8Id, uint8_t ui8RegAddr);
void SSIReadMultiByte(uint8_t ui8Id, uint8_t ui8RegAddr, uint8_t* pui8Data, uint8_t ui8Length);
uint16_t SSIReadWriteSingleByte(uint8_t ui8Id, uint8_t ui8Data);
void SSIReadWriteMultiByte(uint8_t ui8Id, uint8_t* pui8ReadData, uint8_t* pui8WriteData, uint8_t ui8Length);

#endif /* LIBS_SSI_SSI_H_ */
