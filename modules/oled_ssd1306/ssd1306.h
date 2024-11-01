/*
 * ssd1306.h
 *
 *  Created on: 8 Oct 2024
 *      Author: minht57
 */

#ifndef MODULES_OLED_SSD1306_SSD1306_H_
#define MODULES_OLED_SSD1306_SSD1306_H_

//  GND --> GND
//  VCC --> 3v3
//  SCL --> PD0
//  SDA --> PD1
#define OLED_I2C        I2C3

void OLEDInit();
void OLEDDisplayOn();
void OLEDDisplayOff();
void OLEDResetDisplay();
void OLEDClearDisplay();
void OLEDSetXY(uint8_t ui8Row, uint8_t ui8Col);
void OLEDSetPixelXY(uint8_t ui8X, uint8_t ui8Y);
void OLEDSendCharXY(char cData, uint8_t ui8X, uint8_t ui8Y);
void OLEDSendStrXY(char *cString, uint8_t ui8X, uint8_t ui8Y);
void OLEDDisplayBitmap(const uint8_t *bitmap);

#endif /* MODULES_OLED_SSD1306_SSD1306_H_ */
