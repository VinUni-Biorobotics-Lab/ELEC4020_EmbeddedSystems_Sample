/*
 * spi.c
 *
 *  Created on: 9 Oct 2024
 *      Author: minht57
 */

#include <stdbool.h>
#include <stdint.h>
#include "ssi.h"
#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

// Definition
#define DUMMY_BYTE                  0x0

// SSI Base
static const uint32_t u32SSIBase[] = {SSI0_BASE, SSI1_BASE, SSI2_BASE, SSI3_BASE};

// SSI peripherals
static const uint32_t u32SSIPer[] = {SYSCTL_PERIPH_SSI0, SYSCTL_PERIPH_SSI1, SYSCTL_PERIPH_SSI2, SYSCTL_PERIPH_SSI3};

// SSI Port peripherals
static const uint32_t u32SSIPortPer[] = {SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOF, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOD};

// SSI GPIO configurations
static const uint32_t u32SSIConfig[][4] = {
    {GPIO_PA2_SSI0CLK, GPIO_PA3_SSI0FSS, GPIO_PA4_SSI0RX, GPIO_PA5_SSI0TX},
    {GPIO_PF2_SSI1CLK, GPIO_PF3_SSI1FSS, GPIO_PF0_SSI1RX, GPIO_PF1_SSI1TX},
    {GPIO_PB4_SSI2CLK, GPIO_PB5_SSI2FSS, GPIO_PB6_SSI2RX, GPIO_PB7_SSI2TX},
    {GPIO_PD0_SSI3CLK, GPIO_PD1_SSI3FSS, GPIO_PD2_SSI3RX, GPIO_PD3_SSI3TX}
};

// SSI GPIO port bases
static const uint32_t u32SSIPort[] = {GPIO_PORTA_BASE, GPIO_PORTF_BASE, GPIO_PORTB_BASE, GPIO_PORTD_BASE};

// SSI GPIO port configurations
static const uint32_t u32SSIPins[] = {
    GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
    GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
};

void SSIInit(uint8_t ui8Id) {
    uint32_t ui32InitialData = 0;
    SysCtlPeripheralEnable(u32SSIPer[ui8Id]);
    SysCtlPeripheralEnable(u32SSIPortPer[ui8Id]);
    SSIDisable(u32SSIBase[ui8Id]);
    GPIOPinConfigure(u32SSIConfig[ui8Id][0]);
    GPIOPinConfigure(u32SSIConfig[ui8Id][1]);
    GPIOPinConfigure(u32SSIConfig[ui8Id][2]);
    GPIOPinConfigure(u32SSIConfig[ui8Id][3]);
    GPIOPinTypeSSI(u32SSIPort[ui8Id], u32SSIPins[ui8Id]);
    SSIConfigSetExpClk(u32SSIBase[ui8Id], SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 4000000 /* bitrate */, 8 /* data width */);
    SSIEnable(u32SSIBase[ui8Id]);
    // Read any residual data from the SSI port.
    while(SSIDataGetNonBlocking(u32SSIBase[ui8Id], &ui32InitialData));

}

void SSIWriteSingleByte(uint8_t ui8Id, uint8_t ui8RegAddr, uint8_t ui8Data) {
    SSIReadWriteSingleByte(ui8Id, ui8RegAddr);
    SSIReadWriteSingleByte(ui8Id, ui8Data);
}

void SSIWriteMultiByte(uint8_t ui8Id, uint8_t ui8RegAddr, uint8_t* pui8Data, uint8_t ui8Length) {
    SSIReadWriteSingleByte(ui8Id, ui8RegAddr);
    uint8_t ui8Idx = 0;
    for(ui8Idx = 0; ui8Idx < ui8Length; ui8Idx++) {
        SSIReadWriteSingleByte(ui8Id, pui8Data[ui8Idx]);
    }
}

uint16_t SSIReadSingleByte(uint8_t ui8Id, uint8_t ui8RegAddr) {
    SSIReadWriteSingleByte(ui8Id, ui8RegAddr);
    return SSIReadWriteSingleByte(ui8Id, DUMMY_BYTE);
}

void SSIReadMultiByte(uint8_t ui8Id, uint8_t ui8RegAddr, uint8_t* pui8Data, uint8_t ui8Length) {
    SSIReadWriteSingleByte(ui8Id, ui8RegAddr);
    uint8_t ui8Idx = 0;
    for(ui8Idx = 0; ui8Idx < ui8Length - 1; ui8Idx++) {
        pui8Data[ui8Idx] = SSIReadWriteSingleByte(ui8Id, ui8RegAddr++);
    }
    pui8Data[ui8Length - 1] = SSIReadWriteSingleByte(ui8Id, DUMMY_BYTE);
}

uint16_t SSIReadWriteSingleByte(uint8_t ui8Id, uint8_t ui8Data) {
    SSIDataPut(u32SSIBase[ui8Id], ui8Data);
    uint32_t ui32Data = 0;
    SSIDataGet(u32SSIBase[ui8Id], &ui32Data);
    return (uint16_t)(ui32Data & 0xFFFF);
}

void SSIReadWriteMultiByte(uint8_t ui8Id, uint8_t* pui8ReadData, uint8_t* pui8WriteData, uint8_t ui8Length) {
    SSIReadWriteSingleByte(ui8Id, pui8WriteData[0]);
    uint8_t ui8Idx = 0;
    for(ui8Idx = 0; ui8Idx < ui8Length - 1; ui8Idx++) {
        pui8ReadData[ui8Idx] = SSIReadWriteSingleByte(ui8Id, pui8WriteData[ui8Idx + 1]);
    }
    pui8ReadData[ui8Length - 1] = SSIReadWriteSingleByte(ui8Id, DUMMY_BYTE);
}

