#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "driverlib/i2c.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "modules/oled_ssd1306/ssd1306.h"
#include "modules/oled_ssd1306/ssd1306_images.h"
#include "libs/adc/adc.h"
#include "libs/uart/uart.h"
#include "modules/dc_motor/motor.h"

uint32_t gui32Buffer[8];
char cBuffer[50];

void ADCCallback(uint32_t * ui32Buffer) {
    uint8_t ui32Idx = 0;
    for(ui32Idx = 0; ui32Idx < 8; ui32Idx++) {
        gui32Buffer[ui32Idx] = ui32Buffer[ui32Idx];
    }
}

/**
 * main.c
 */


void main() {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); // 80 MHz

    /* I2C Demo */
    OLEDInit();
    OLEDClearDisplay();
    OLEDDisplayBitmap(aui8Prof);

    /* ADC Demo */
//    gui32Buffer[0] = ADCSingleRead(AIN7);
//    gui32Buffer[1] = ADCSingleRead(AIN6);

    uint8_t ui8Pins[] = {AIN7, AIN6};
    ADCInit(ui8Pins, 2);
    ADCRegisterCallback(&ADCCallback);

    /* UART Demo */
    UARTInit();

//    /* PWM Demo */
//    pwm_mod_t motorList = PWM_MOD1_GEN3;
//    MotorsInit(&motorList, 1);
//
//    uint8_t ui8Speed = 50;
//    uint8_t ui8Increase = 1;
//    uint32_t ui32Position = 0;

    // Enable processor interrupt
    IntMasterEnable();

    while(1) {
//        MotorsSetSpeed(0, ui8Speed);
//
//        sprintf(cBuffer, "%d\n", ui32Position);
//        UARTWriteCMD(cBuffer);
//
//        if(ui8Speed <= 5) {
//            ui8Increase = 1;
//        }
//        else if(ui8Speed >= 90) {
//            ui8Increase = 0;
//        }
//        if(ui8Increase) {
//            ui8Speed += 5;
//        }
//        else {
//            ui8Speed -= 5;
//            ui8Speed = ui8Speed == 0 ? 1 : ui8Speed;
//        }

        SysCtlDelay(SysCtlClockGet()/30);
    }
}

