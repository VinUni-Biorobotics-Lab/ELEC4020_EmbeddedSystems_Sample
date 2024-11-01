/*
 * i2c.c
 *
 *  Created on: 7 Oct 2024
 *      Author: minht57
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "i2c.h"

void I2CInit(uint32_t ui32Id) {
    // Enable I2C peripheral
    // Enable GPIO peripheral
    // Use pin with I2C peripheral
    if(I2C0 == ui32Id) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        GPIOPinConfigure(GPIO_PB2_I2C0SCL);
        GPIOPinConfigure(GPIO_PB3_I2C0SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
        GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
        I2CMasterInitExpClk(ui32Id, SysCtlClockGet(), false);
    }
    else if(I2C1 == ui32Id) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        GPIOPinConfigure(GPIO_PA6_I2C1SCL);
        GPIOPinConfigure(GPIO_PA7_I2C1SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
        GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
        I2CMasterInitExpClk(ui32Id, SysCtlClockGet(), false);
    }
    else if(I2C2 == ui32Id) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        GPIOPinConfigure(GPIO_PE4_I2C2SCL);
        GPIOPinConfigure(GPIO_PE5_I2C2SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
        GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);
        I2CMasterInitExpClk(ui32Id, SysCtlClockGet(), false);
    }
    else if(I2C3 == ui32Id) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        GPIOPinConfigure(GPIO_PD0_I2C3SCL);
        GPIOPinConfigure(GPIO_PD1_I2C3SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
        GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
        I2CMasterInitExpClk(ui32Id, SysCtlClockGet(), false);
    }
}

void I2CWriteSingleByte(uint32_t ui32Id, uint8_t ui8Addr, uint8_t ui8RegAddr, uint8_t ui8Data) {
    I2CWriteMultiByte(ui32Id, ui8Addr, ui8RegAddr, &ui8Data, 1);
}

void I2CWriteMultiByte(uint32_t ui32Id, uint8_t ui8Addr, uint8_t ui8RegAddr, uint8_t* pui8Data, uint8_t ui8Length) {
    // Set to write mode
    I2CMasterSlaveAddrSet(ui32Id, ui8Addr, false);
    // Place address into data register
    I2CMasterDataPut(ui32Id, ui8RegAddr);
    // Send start condition
    I2CMasterControl(ui32Id, I2C_MASTER_CMD_BURST_SEND_START);
    // Wait until transfer is done
    while (I2CMasterBusy(ui32Id));

    uint8_t i = 0;
    for (i = 0; i < ui8Length - 1; i++) {
        // Place data into data register
        I2CMasterDataPut(ui32Id, pui8Data[i]);
        // Send continues condition
        I2CMasterControl(ui32Id, I2C_MASTER_CMD_BURST_SEND_CONT);
        // Wait until transfer is done
        while (I2CMasterBusy(ui32Id));
    }

    // Place data into data register
    I2CMasterDataPut(ui32Id, pui8Data [ui8Length - 1]);
    // Send finish condition
    I2CMasterControl(ui32Id, I2C_MASTER_CMD_BURST_SEND_FINISH);
    // Wait until transfer is done
    while (I2CMasterBusy(ui32Id));
}

uint8_t I2CReadSingleByte(uint32_t ui32Id, uint8_t ui8Addr, uint8_t ui8RegAddr) {
    I2CMasterSlaveAddrSet(ui32Id, ui8Addr, false); // Set to write mode

    I2CMasterDataPut(ui32Id, ui8RegAddr); // Place address into data register
    I2CMasterControl(ui32Id, I2C_MASTER_CMD_SINGLE_SEND); // Send data
    while (I2CMasterBusy(ui32Id)); // Wait until transfer is done

    I2CMasterSlaveAddrSet(ui32Id, ui8Addr, true); // Set to read mode

    I2CMasterControl(ui32Id, I2C_MASTER_CMD_SINGLE_RECEIVE); // Tell master to read data
    while (I2CMasterBusy(ui32Id)); // Wait until transfer is done
    return I2CMasterDataGet(ui32Id); // Read data
}

void I2CReadMultiByte(uint32_t ui32Id, uint8_t ui8Addr, uint8_t ui8RegAddr, uint8_t* pui8Data, uint8_t ui8Length) {
    // Set to write mode
    I2CMasterSlaveAddrSet(ui32Id, ui8Addr, false);
    // Place address into data register
    I2CMasterDataPut(ui32Id, ui8RegAddr);
    // Send data
    I2CMasterControl(ui32Id, I2C_MASTER_CMD_SINGLE_SEND);
    // Wait until transfer is done
    while (I2CMasterBusy(ui32Id));
    // Set to read mode
    I2CMasterSlaveAddrSet(ui32Id, ui8Addr, true);
    // Send start condition
    I2CMasterControl(ui32Id, I2C_MASTER_CMD_BURST_RECEIVE_START);
    // Wait until transfer is done
    while (I2CMasterBusy(ui32Id));
    // Place data into data register
    pui8Data[0] = I2CMasterDataGet(ui32Id);
    uint8_t i = 1;
    for (i = 1; i < ui8Length - 1; i++) {
        // Send continues condition
        I2CMasterControl(ui32Id, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        // Wait until transfer is done
        while (I2CMasterBusy(ui32Id));
        // Place data into data register
        pui8Data[i] = I2CMasterDataGet(ui32Id);
    }
    // Send finish condition
    I2CMasterControl(ui32Id, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    // Wait until transfer is done
    while (I2CMasterBusy(ui32Id));
    // Place data into data register
    pui8Data[ui8Length - 1] = I2CMasterDataGet(ui32Id);
}
