/*
 * adc.c
 *
 *  Created on: 25 Oct 2024
 *      Author: minht57
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "adc.h"

// ADC Input channel
static const uint32_t u32ADCChannel[] = { ADC_CTL_CH0, ADC_CTL_CH1, ADC_CTL_CH2, ADC_CTL_CH3,
                                          ADC_CTL_CH4, ADC_CTL_CH5, ADC_CTL_CH6, ADC_CTL_CH7,
                                          ADC_CTL_CH8, ADC_CTL_CH9, ADC_CTL_CH10, ADC_CTL_CH11,
                                          ADC_CTL_TS};

// ADC Port peripherals
static const uint32_t u32ADCPortPer[] = {SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOE,
                                         SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOD,
                                         SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOB,
                                         0x0};

// SSI GPIO port bases
static const uint32_t u32ADCPort[] = {GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTE_BASE,
                                      GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE,
                                      GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTB_BASE, GPIO_PORTB_BASE,
                                      0x0};

// SSI GPIO port configurations
static const uint32_t u32ADCPins[] = {GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0,
                                      GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0,
                                      GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_4, GPIO_PIN_5,
                                      0x0};

// Prototype function
void ADCIntHandler();

// Local variables
uint8_t lui8ADCInitialized = 0;
uint32_t lui32Seq = 3;
uint32_t pui32Buffer[8];
adc_callback_func_t ladc_callback_func = 0;

uint32_t ADCSingleRead(uint8_t ui8PinName) {
    if(ui8PinName > 12) {
        return 0;
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    if(ui8PinName != AINTS) {
        SysCtlPeripheralEnable(u32ADCPortPer[ui8PinName]);
        GPIOPinTypeADC(u32ADCPort[ui8PinName],u32ADCPins[ui8PinName]);
    }

    ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC1_BASE, 3, 0, u32ADCChannel[ui8PinName] | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 3);

    ADCIntClear(ADC1_BASE, 3);
    ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false)) {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC1_BASE, 3, pui32Buffer);

    return pui32Buffer[0];
}

void ADCInit(uint8_t * pui8Pins, uint8_t ui8NoPins) {
    if(ui8NoPins > 8) {
        return ;
    }

    uint32_t ui32Seq = 3;
    uint8_t ui8Idx = 0;
    if(ui8NoPins > 4) {
        ui32Seq = 0;
    }
    else if(ui8NoPins > 1) {
        ui32Seq = 1;
    }


    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Disable the sequencer before configuration
    ADCSequenceDisable(ADC0_BASE, ui32Seq);

    // NOTE: This for loop is not a optimal solution
    for(ui8Idx = 0; ui8Idx < ui8NoPins; ui8Idx++) {
        uint8_t ui8PinName = pui8Pins[ui8Idx];

        if(ui8PinName != AINTS) {
            SysCtlPeripheralEnable(u32ADCPortPer[ui8PinName]);
            GPIOPinTypeADC(u32ADCPort[ui8PinName],u32ADCPins[ui8PinName]);
        }

        if(ui8Idx == (ui8NoPins - 1)) {
            ADCSequenceStepConfigure(ADC0_BASE, ui32Seq, ui8Idx, u32ADCChannel[ui8PinName] | ADC_CTL_IE | ADC_CTL_END);
        }
        else {
            ADCSequenceStepConfigure(ADC0_BASE, ui32Seq, ui8Idx, u32ADCChannel[ui8PinName]);
        }
    }

    ADCSequenceEnable(ADC0_BASE, ui32Seq);

//    // Clear ADC interrupt status flag
    ADCIntClear(ADC0_BASE, ui32Seq);
    ADCIntRegister(ADC0_BASE, ui32Seq, ADCIntHandler);
    ADCIntEnable(ADC0_BASE, ui32Seq);

    // Store for local use
    lui32Seq = ui32Seq;
}

void ADCTriggerConversion() {
    ADCProcessorTrigger(ADC0_BASE, lui32Seq);
}

void ADCRegisterCallback(adc_callback_func_t pcallback) {
    if(pcallback != 0) {
        ladc_callback_func = pcallback;
    }
}

void ADCIntHandler() {
    ADCIntClear(ADC0_BASE, lui32Seq);
    ADCSequenceDataGet(ADC0_BASE, lui32Seq, pui32Buffer);

    if(ladc_callback_func != 0) {
        ladc_callback_func(pui32Buffer);
    }
}
