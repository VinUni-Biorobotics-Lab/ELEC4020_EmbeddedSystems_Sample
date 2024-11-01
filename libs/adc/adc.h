/*
 * adc.h
 *
 *  Created on: 25 Oct 2024
 *      Author: minht57
 */

#ifndef LIBS_ADC_ADC_H_
#define LIBS_ADC_ADC_H_

#include <stdint.h>

#define AIN0        0       // PE3
#define AIN1        1       // PE2
#define AIN2        2       // PE1
#define AIN3        3       // PE0
#define AIN4        4       // PD3
#define AIN5        5       // PD2
#define AIN6        6       // PD1
#define AIN7        7       // PD0
#define AIN8        8       // PE5
#define AIN9        9       // PE4
#define AI10        10      // PB4
#define AIN11       11      // PB5
#define AINTS       12

typedef void (*adc_callback_func_t)(uint32_t *);

// For single pin without configuration
uint32_t ADCSingleRead(uint8_t ui8PinName);

// For multiple pins
void ADCInit(uint8_t * pui8Pins, uint8_t ui8NoPins);
// NOTE: This callback function is called in the ADC interrupt
void ADCRegisterCallback(adc_callback_func_t pcallback);
void ADCTriggerConversion();

#endif /* LIBS_ADC_ADC_H_ */
