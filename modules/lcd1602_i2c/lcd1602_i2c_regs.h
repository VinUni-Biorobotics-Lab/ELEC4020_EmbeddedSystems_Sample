/*
 * lcd1602_i2c_regs.h
 *
 *  Created on: Dec 18, 2024
 *      Author: minht57
 */

#ifndef MODULES_LCD1602_I2C_LCD1602_I2C_REGS_H_
#define MODULES_LCD1602_I2C_LCD1602_I2C_REGS_H_

/// These are Bit-Masks for the special signals and background light
#define RS      (1<<0)
#define RW      (1<<1)
#define E       (1<<2)

#define LINE1   0x80        // first Line of LCD1602
#define LINE2   0xC0        // second Line of LCD1602

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// commands for use with Cmd_Write()
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODE 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORDISPLAYSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for LCD_ENTRYMODE
#define LCD_NOSHIFT 0x00
#define LCD_SHIFT 0x01
#define LCD_DECREMENT 0x00
#define LCD_INCREMENT 0x02

// flags for LCD_DISPLAYCONTROL
#define LCD_DISPLAYOFF 0x00
#define LCD_DISPLAYON 0x04
#define LCD_CURSOROFF 0x00
#define LCD_CURSORON 0x02
#define LCD_BLINKOFF 0x00
#define LCD_BLINKON 0x01

// flags for LCD_CURSORDISPLAYSHIFT
#define LCD_CURSORMOVE 0x00
#define LCD_DISPLAYMOVE 0x08
#define LCD_MOVELEFT 0x00
#define LCD_MOVERIGHT 0x04

// flags for LCD_FUNCTIONSET
#define LCD_4BIT 0x00
#define LCD_8BIT 0x10
#define LCD_1LINE 0x00
#define LCD_2LINE 0x08
#define LCD_5x8 0x00
#define LCD_5x10 0x04

#endif /* MODULES_LCD1602_I2C_LCD1602_I2C_REGS_H_ */
