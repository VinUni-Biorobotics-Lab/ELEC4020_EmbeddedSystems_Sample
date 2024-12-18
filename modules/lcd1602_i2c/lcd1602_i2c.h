/*
 * lcd1602.h
 *
 *  Created on: Dec 17, 2024
 *      Author: Hoang Tran
 *      Link: https://github.com/hoang-ttr/Tiva-C-Launchpad-TM4C123GH6PM/tree/master
 *      Date: May 2017
 *      Edited by: minht57
 */

#ifndef MODULES_LCD1602_I2C_LCD1602_I2C_H_
#define MODULES_LCD1602_I2C_LCD1602_I2C_H_

#include <stdint.h>
#include "libs/i2c/i2c.h"

//  GND --> GND
//  VCC --> 3v3
//  SCL --> PD0
//  SDA --> PD1
#define LCD1602_I2C        I2C3
#define LCD_ADDR            0x27    // LCD1602 address for I2C

void LCD1602Init(void);
void LCD1602SendCmd(char command);
void LCD1602Clear(void);
void LCD1602SetChar(char character);
void LCD1602SetString(char *pStr);
void LCD1602SetUDec(uint8_t number);
void LCD1602Goto(uint8_t row, uint8_t col);

#endif /* MODULES_LCD1602_I2C_LCD1602_I2C_H_ */
