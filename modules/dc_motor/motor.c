/*
 * motor.c
 *
 *  Created on: 30 Oct 2024
 *      Author: minht57
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_pwm.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "motor.h"

//static const uint32_t u32PWMModule[] = {SYSCTL_PERIPH_PWM0, SYSCTL_PERIPH_PWM1};

static const uint32_t u32PWMPerPort[] = {SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOB,
                                         SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOC,
                                         SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOA,
                                         SYSCTL_PERIPH_GPIOF, SYSCTL_PERIPH_GPIOF};

static const uint32_t u32PWMPin[][2] = {{GPIO_PB6_M0PWM0, GPIO_PB7_M0PWM1},
                                        {GPIO_PB4_M0PWM2, GPIO_PB5_M0PWM3},
                                        {GPIO_PE4_M0PWM4, GPIO_PE5_M0PWM5},
                                        {GPIO_PC4_M0PWM6, GPIO_PC5_M0PWM7},
                                        {GPIO_PD0_M1PWM0, GPIO_PD1_M1PWM1},
                                        {GPIO_PA6_M1PWM2, GPIO_PA7_M1PWM3},
                                        {GPIO_PF0_M1PWM4, GPIO_PF1_M1PWM5},
                                        {GPIO_PF2_M1PWM6, GPIO_PF3_M1PWM7}};

static const uint32_t u32PWMPortBase[] = {GPIO_PORTB_BASE, GPIO_PORTB_BASE,
                                          GPIO_PORTE_BASE, GPIO_PORTC_BASE,
                                          GPIO_PORTD_BASE, GPIO_PORTA_BASE,
                                          GPIO_PORTF_BASE, GPIO_PORTF_BASE};

static const uint8_t u8PWMGPIOPin[] = {GPIO_PIN_6| GPIO_PIN_7,
                                            GPIO_PIN_4| GPIO_PIN_5,
                                            GPIO_PIN_4| GPIO_PIN_5,
                                            GPIO_PIN_4| GPIO_PIN_5,
                                            GPIO_PIN_0| GPIO_PIN_1,
                                            GPIO_PIN_6| GPIO_PIN_7,
                                            GPIO_PIN_0| GPIO_PIN_1,
                                            GPIO_PIN_2| GPIO_PIN_3};

//static const uint32_t u32PWMBase[] = {PWM0_BASE, PWM1_BASE};
static const uint32_t u32PWMGen[] = {PWM_GEN_0, PWM_GEN_1, PWM_GEN_2, PWM_GEN_3};

static const uint32_t u32PWMOutBit[] = {PWM_OUT_0_BIT| PWM_OUT_1_BIT,
                                        PWM_OUT_2_BIT| PWM_OUT_3_BIT,
                                        PWM_OUT_4_BIT| PWM_OUT_5_BIT,
                                        (PWM_OUT_6_BIT| PWM_OUT_7_BIT)};

static const uint32_t u32PWMOut[] = {PWM_OUT_0,
                                     PWM_OUT_2,
                                     PWM_OUT_4,
                                     PWM_OUT_6};

// Variable
uint32_t ui32Load = 1;
uint8_t aui8MotorList[8];

void MotorsInit(pwm_mod_t * pConfig, uint8_t ui8NoMotors) {
    if(pConfig == 0) {
        return;
    }
    if(ui8NoMotors > 8) {
        return;
    }

    // Set PWM clock divider (PWM clock = System clock / 8)
    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);

    uint8_t ui8Idx = 0;
    uint32_t ui32PWMBase = PWM0_BASE;
    for(ui8Idx = 0; ui8Idx < ui8NoMotors; ui8Idx++) {
        uint8_t ui8PWMMod = (uint8_t)pConfig[ui8Idx];
        uint8_t ui8Mod = ui8PWMMod % 4;
        uint32_t ui32PWMGen = u32PWMGen[ui8Mod];
        aui8MotorList[ui8Idx] = ui8PWMMod;

        if(ui8PWMMod < PWM_MOD1_GEN0) {
            SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
        }
        else {
            SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
            ui32PWMBase = PWM1_BASE;
        }


        //  Enable GPIOF peripherals
        SysCtlPeripheralEnable(u32PWMPerPort[ui8PWMMod]);

        // Configure pins for PWM output
        GPIOPinConfigure(u32PWMPin[ui8PWMMod][0]);
        GPIOPinConfigure(u32PWMPin[ui8PWMMod][1]);
        GPIOPinTypePWM(u32PWMPortBase[ui8PWMMod], u8PWMGPIOPin[ui8PWMMod]);

        ui32Load = (SysCtlClockGet() / 8 / PWM_FREQUENCY) - 1;

        // Configure PWM Generator 3 for count-down mode
        PWMGenConfigure(ui32PWMBase, ui32PWMGen, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        PWMGenPeriodSet(ui32PWMBase, ui32PWMGen, ui32Load);

        PWMPulseWidthSet(ui32PWMBase, u32PWMOut[ui8Mod], ui32Load / 2);

        // Enable dead-band with specific delay
        PWMDeadBandEnable(ui32PWMBase, ui32PWMGen, 100, 100);

        // Enable PWM outputs
        PWMOutputState(ui32PWMBase, u32PWMOutBit[ui8Mod], true);

        // Enable PWM generator
        PWMGenEnable(ui32PWMBase, ui32PWMGen);
    }
}

void MotorsSetSpeed(uint8_t ui8Id, uint8_t ui8Speed) {
    if(ui8Speed > 100) {
        ui8Speed = 100;
    }
    if(ui8Id > 8) {
        return;
    }

    uint32_t ui32PulseWidth = ui8Speed * ui32Load / 100;

    uint32_t ui32PWMBase = aui8MotorList[ui8Id] < PWM_MOD1_GEN0 ? PWM0_BASE : PWM1_BASE;
    PWMPulseWidthSet(ui32PWMBase, u32PWMOut[aui8MotorList[ui8Id] % 4], ui32PulseWidth);
}

