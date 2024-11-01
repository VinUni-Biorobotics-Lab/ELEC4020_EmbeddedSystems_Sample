/*
 * ssd1306.c
 *
 *  Created on: 8 Oct 2024
 *      Author: minht57, trungns, anpn
 *
 *  Original source: https://github.com/Bruno4l/SSD1306-EK-TM4C123GXL
 */

#include "libs/i2c/i2c.h"
#include "ssd1306.h"
#include "ssd1306_font.h"
#include "ssd1306_regs.h"

// Definition
#define SLAVE_ADDR          0x3C

// Prototype function
void OLEDCommand(uint8_t cmd);
void OLEDData(uint8_t data);

void OLEDInit() {
    // Init I2C
    I2CInit(OLED_I2C);

    // Should add a delay here, ~ 2us

    // Initialise sequence for 128x64 OLED module
    OLEDCommand(SSD1306_DISPLAYOFF);                    // 0xAE

    OLEDCommand(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    OLEDCommand(0x80);                                  // the suggested ratio 0x80

    OLEDCommand(SSD1306_SETMULTIPLEX);                  // 0xA8
    OLEDCommand(0x3F);                                   // 0 - 32

    OLEDCommand(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    OLEDCommand(0x0);                                   // no offset

    OLEDCommand(SSD1306_SETSTARTLINE);// | 0x0);        // line #0

    OLEDCommand(SSD1306_CHARGEPUMP);                    // 0x8D
    OLEDCommand(0x14);                                  // using internal VCC

    OLEDCommand(0x8D);                                  // Charge Pump Setting
    OLEDCommand(0x14);                                  // Enable charge pump (0x10 if external power supply is used)

    OLEDCommand(0xA1);                                  // Set Segment Re-map (A0 = normal, A1 = reversed)
    OLEDCommand(0xC8);                                  // Set COM Output Scan Direction (C0 = normal, C8 = reversed)

    OLEDCommand(SSD1306_SETCOMPINS);                    // 0xDA
//    OLEDCommand(0x02);
    OLEDCommand(0x12);

    OLEDCommand(SSD1306_SETCONTRAST);                   // 0x81
    OLEDCommand(0x7F);

    OLEDCommand(SSD1306_SETPRECHARGE);                  // 0xd9
//    OLEDCommand(0x22);
    OLEDCommand(0xF1);

    OLEDCommand(SSD1306_SETVCOMDETECT);                 // 0xDB
    OLEDCommand(0x40);

    OLEDCommand(SSD1306_DISPLAYALLON_RESUME);           // 0xA4 hello fix here

    OLEDCommand(SSD1306_MEMORYMODE);
    OLEDCommand(0x00);

    OLEDCommand(SSD1306_NORMALDISPLAY);                 // 0xA6

//    OLEDCommand(0x2E);                                    // Deactivate scroll (just in case)

    OLEDCommand(SSD1306_DISPLAYON);                     //switch on OLED
}

void OLEDDisplayOn() {
    OLEDCommand(SSD1306_DISPLAYON);
}

void OLEDDisplayOff(){
    OLEDCommand(SSD1306_DISPLAYOFF);
}

void OLEDResetDisplay() {
    OLEDDisplayOff();
    OLEDClearDisplay();
    OLEDDisplayOn();
}

void OLEDClearDisplay() {
    uint8_t i, k;
    for(k=0; k < 8; k++) {
      OLEDSetXY(k,0);
      {
          for(i=0; i<128; i++) {
              OLEDData(0);
          }
      }
    }
}

void OLEDSetXY(uint8_t ui8Row, uint8_t ui8Col) {
    // Set page address
    OLEDCommand(0xb0 + ui8Row);
    // Set low col address
    OLEDCommand(0x00 + (8 * ui8Col & 0x0F));
    // Set high col address
    OLEDCommand(0x10 + ((8 * ui8Col >> 4) & 0x0F));
}

void OLEDSetPixelXY(uint8_t ui8X, uint8_t ui8Y) {
    OLEDCommand(0xb0 + ui8X);
    OLEDCommand(0x00 + ui8Y & 0x0F);
    OLEDCommand(0x10 + ((ui8Y >> 4) & 0x0F));
}

void OLEDSendCharXY(char cData, uint8_t ui8X, uint8_t ui8Y) {
    uint8_t cArr[5];
    uint8_t i;
    OLEDSetXY(ui8X, ui8Y);

    for(i = 0; i < 5; i++) {
        cArr[i] = ASCIIFont5x8[cData - 0x20][i];
    }

    I2CWriteMultiByte(OLED_I2C, SLAVE_ADDR, 0x40, cArr, 5);
}

void OLEDSendStrXY(char *cString, uint8_t ui8X, uint8_t ui8Y) {
    uint8_t i;
    for(i = 0; cString[i] != '\0'; i++){
        OLEDSendCharXY(cString[i], ui8X, ui8Y++);
    }
}

void OLEDDisplayBitmap(const uint8_t *bitmap) {
    uint16_t i;

    // Set the column and page address ranges
    // Set column address
    OLEDCommand(SSD1306_COLUMNADDR);
    // Start column: 0
    OLEDCommand(0x00);
    // End column: 127
    OLEDCommand(0x7F);

    // Set page address
    OLEDCommand(SSD1306_PAGEADDR);
    // Start page: 0
    OLEDCommand(0x00);
    // End page: 7 (for 128x64 display, 8 pages)
    OLEDCommand(0x07);

    // Write the bitmap data (1024 bytes)
    for (i = 0; i < 1024; i++) {
        OLEDData(bitmap[i]);
    }
}

void OLEDCommand(uint8_t cmd) {
    I2CWriteSingleByte(OLED_I2C, SLAVE_ADDR, 0x00, cmd);
}

void OLEDData(uint8_t data) {
    I2CWriteSingleByte(OLED_I2C, SLAVE_ADDR, 0x40, data);
}
