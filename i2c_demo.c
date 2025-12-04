#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "driverlib/i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "modules/oled_ssd1306/ssd1306.h"
#include "modules/oled_ssd1306/ssd1306_images.h"
#include "libs/adc/adc.h"
#include "libs/uart/uart.h"

#define NO_MAX_IMAGES       4

uint32_t gui32Buffer[2];
char cBuffer[50];
uint8_t gui8Mode = 0;
uint8_t gui8ImageIdx = 1;
uint8_t * gaImages[] = {aui8Class, aui8VinUni, aui8Prof, aui8BuildingI};

void ADCCallback(uint32_t * ui32Buffer) {
    uint8_t ui32Idx = 0;
    for(ui32Idx = 0; ui32Idx < 1; ui32Idx++) {
        gui32Buffer[ui32Idx] = ui32Buffer[ui32Idx];
    }
}

void GPIOPortFIntHandler() {
    uint32_t mask = GPIOIntStatus(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
    // Clear the interrupt
    GPIOIntClear(GPIO_PORTF_BASE, mask);
    SysCtlDelay(SysCtlClockGet() / 1000);

    // SW2 - decrease
    if((mask & GPIO_INT_PIN_0) && (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0) == 0)) {
        gui8Mode = 1;
        OLEDClearDisplay();
        OLEDSendStrXY("ELEC4020", 0, 2);

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) ^ GPIO_PIN_3);
    }
    // SW1 - increase
    if((mask & GPIO_INT_PIN_4) && (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) == 0)) {
        if(gui8Mode != 0) {
            gui8Mode = 0;
            OLEDClearDisplay();
        }

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2) ^ GPIO_PIN_2);

        OLEDDisplayBitmap(gaImages[gui8ImageIdx++]);

        if(gui8ImageIdx >= NO_MAX_IMAGES) {
            gui8ImageIdx = 0;
        }
    }
}

void GPIOInit() {
    // Enable Clock for PORTF
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    // GPIO Configuration
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Set up the interrupt
    IntEnable(INT_GPIOF);
    GPIOIntRegister(GPIO_PORTF_BASE,GPIOPortFIntHandler);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_FALLING_EDGE);

    // Enable the GPIO interrupt
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
}

/**
 * main.c
 */


void main() {
    SysCtlClockSet(SYSCTL_SYSDIV_20|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); // 80 MHz

    /* GPIO Init */
    GPIOInit();

    /* I2C Demo */
    OLEDInit();
    OLEDClearDisplay();
    OLEDDisplayBitmap(gaImages[0]);

    /* ADC Demo */
//    gui32Buffer[0] = ADCSingleRead(AIN7);
//    gui32Buffer[1] = ADCSingleRead(AIN6);

    uint8_t ui8Pins[] = {AIN0};
    ADCInit(ui8Pins, 1);
    ADCRegisterCallback(&ADCCallback);

    /* UART Demo */
    UARTInit();

    // Enable processor interrupt
    IntMasterEnable();

    while(1) {
        if(gui8Mode == 1) {
            ADCTriggerConversion();
            SysCtlDelay(SysCtlClockGet()/100);
            uint16_t noBytes = sprintf(cBuffer,"ADC: %4d", gui32Buffer[0]);
//            OLEDClearDisplay();
            OLEDSendStrXY(cBuffer, 2, 2);
            noBytes = sprintf(cBuffer,"V: %.2f V", (float)(gui32Buffer[0]) / 1241);
            OLEDSendStrXY(cBuffer, 4, 2);
//            noBytes = sprintf(cBuffer,"Pressed: %d", (bool)!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));
//            OLEDSendStrXY(cBuffer, 6, 2);

            SysCtlDelay(SysCtlClockGet()/30);
        }
    }
}

