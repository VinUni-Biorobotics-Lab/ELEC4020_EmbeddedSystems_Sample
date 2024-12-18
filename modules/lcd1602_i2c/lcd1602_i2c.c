/*
 * lcd1602_i2c.c
 *
 *  Created on: Dec 17, 2024
 *      Author: Hoang Tran
 *      Link: https://github.com/hoang-ttr/Tiva-C-Launchpad-TM4C123GH6PM/tree/master
 *      Date: May 2017
 *      Edited by: minht57
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/i2c.h"
#include "libs/i2c/i2c.h"
#include "lcd1602_i2c.h"
#include "lcd1602_i2c_regs.h"

// LCD1602 Write Commend to LCD module
static void CmdWrite(uint8_t command);
// LCD1602 Write DATA to LCD module
static void DataWrite(uint8_t data);

void LCD1602Init(void) {
    // Init I2C
    I2CInit(LCD1602_I2C);

    // Config LCD
    CmdWrite((LCD_FUNCTIONSET | LCD_4BIT) >> 4);
    CmdWrite(LCD_FUNCTIONSET | LCD_4BIT | LCD_2LINE | LCD_5x8);
    CmdWrite(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    CmdWrite(LCD_ENTRYMODE | LCD_NOSHIFT | LCD_INCREMENT);
}

void LCD1602Clear(void) {
    CmdWrite(LCD_CLEARDISPLAY);
}

void LCD1602SendCmd(char command) {
    CmdWrite(command);
}
void LCD1602SetString(char *pStr) {
    while (*pStr) {
        DataWrite(*pStr++);
    }
}

void LCD1602SetChar(char character) {
    DataWrite(character);
}

void LCD1602SetUDec(uint8_t number) {
    if (number >= 10) {
        LCD1602SetChar(number / 10);
        number = number % 10;
    }
    LCD1602SetChar(number + '0');
}

void LCD1602Goto(uint8_t row, uint8_t col) {
    if(row == 0) {
        CmdWrite(LINE1 + col);
    }
    else if(row == 1) {
        CmdWrite(LINE2 + col);
    }
}

static void SendData(uint32_t ui32Id, uint8_t ui8Addr, uint8_t ui8Data) {
    // Set to write mode
    I2CMasterSlaveAddrSet(ui32Id, ui8Addr, false);
    // Place address into data register
    I2CMasterDataPut(ui32Id, ui8Data);
    // Send start condition
    I2CMasterControl(ui32Id, I2C_MASTER_CMD_SINGLE_SEND);
    // Wait until transfer is done
    while (I2CMasterBusy(ui32Id));
}

static void CmdWrite(uint8_t command) {
    uint8_t cmdHi = ((command & 0xF0) | LCD_BACKLIGHT);
    uint8_t cmdLo = (((command<<4) & 0xF0) | LCD_BACKLIGHT);

    SendData(LCD1602_I2C, LCD_ADDR, cmdHi);
    SendData(LCD1602_I2C, LCD_ADDR, (cmdHi | E));
    SendData(LCD1602_I2C, LCD_ADDR, (cmdHi & ~E));

    SendData(LCD1602_I2C, LCD_ADDR, cmdLo);
    SendData(LCD1602_I2C, LCD_ADDR, (cmdLo | E));
    SendData(LCD1602_I2C, LCD_ADDR, (cmdLo & ~E));
}

static void DataWrite(uint8_t data) {
    uint8_t dataHi = ((data & 0xF0) | LCD_BACKLIGHT);
    uint8_t dataLo = (((data<<4) & 0xF0) | LCD_BACKLIGHT);

    SendData(LCD1602_I2C, LCD_ADDR, dataHi);
    SendData(LCD1602_I2C, LCD_ADDR, ((dataHi | RS) | E ));
    SendData(LCD1602_I2C, LCD_ADDR, ((dataHi | RS) & ~E));

    SendData(LCD1602_I2C, LCD_ADDR, dataLo);
    SendData(LCD1602_I2C, LCD_ADDR, ((dataLo | RS) | E ));
    SendData(LCD1602_I2C, LCD_ADDR, ((dataLo | RS) & ~E));
}
