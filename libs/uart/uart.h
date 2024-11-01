/*
 * uart.h
 *
 *  Created on: 14 Sep 2024
 *      Author: minht57
 */

#ifndef INCLUDE_BLUETOOTH_H_
#define INCLUDE_BLUETOOTH_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#define DEBUG

#ifdef DEBUG

#define UART_PER          SYSCTL_PERIPH_UART0
#define UART_GPIO          SYSCTL_PERIPH_GPIOA
#define UART_RX            GPIO_PA0_U0RX
#define UART_TX            GPIO_PA1_U0TX
#define UART_GPIO_BASE     GPIO_PORTA_BASE
#define UART_GPIO_PIN      (GPIO_PIN_0 | GPIO_PIN_1)
#define UART_BASE          UART0_BASE
#define UART_BAUDRATE      9600
#define UART_CONFIG        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)
#define UART_INT           INT_UART0
#define UART_INT_CONFIG    (UART_INT_RX | UART_INT_RT)

#else

#define UART_PER           SYSCTL_PERIPH_UART1
#define UART_GPIO          SYSCTL_PERIPH_GPIOB
#define UART_RX            GPIO_PB0_U1RX
#define UART_TX            GPIO_PB1_U1TX
#define UART_GPIO_BASE     GPIO_PORTB_BASE
#define UART_GPIO_PIN      (GPIO_PIN_0 | GPIO_PIN_1)
#define UART_BASE          UART1_BASE
#define UART_BAUDRATE      9600
#define UART_CONFIG        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)
#define UART_INT           INT_UART1
#define UART_INT_CONFIG    (UART_INT_RX | UART_INT_RT)

#endif

void UARTInit();
void UARTRead(uint8_t * buf, uint16_t len);
void UARTWrite(uint8_t* cBuff, uint16_t ui16Len);
uint16_t UARTQueryData();
void UARTWriteCMD(char* cBuff);

#endif /* INCLUDE_BLUETOOTH_H_ */
