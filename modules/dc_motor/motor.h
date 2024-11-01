/*
 * motor.h
 *
 *  Created on: 30 Oct 2024
 *      Author: minht57
 */

#ifndef MODULES_DC_MOTOR_MOTOR_H_
#define MODULES_DC_MOTOR_MOTOR_H_

#include <stdint.h>

#define PWM_FREQUENCY       1000        // 1kHz

typedef enum {
    PWM_MOD0_GEN0 = 0,   // Module 0, generator 0
    PWM_MOD0_GEN1,
    PWM_MOD0_GEN2,
    PWM_MOD0_GEN3,
    PWM_MOD1_GEN0,
    PWM_MOD1_GEN1,
    PWM_MOD1_GEN2,
    PWM_MOD1_GEN3
} pwm_mod_t;

void MotorsInit(pwm_mod_t * pConfig, uint8_t ui8NoMotors);
void MotorsSetSpeed(uint8_t ui8Id, uint8_t ui8Speed);

#endif /* MODULES_DC_MOTOR_MOTOR_H_ */
