/*
 * uart.c
 *
 *  Created on: 14 Sep 2024
 *      Author: minht57
 */

#include "uart.h"

#define TIME_WAIT_FOR_CMD       10
#define MAX_BUF_DATA_RECV       50

uint8_t UARTBuf[MAX_BUF_DATA_RECV];
uint16_t UARTReadIdx = 0;
uint16_t UARTWriteIdx = 0;
uint16_t u16AvailByte = 0;

void UARTIntHandler();

void UARTInit() {
    SysCtlPeripheralEnable(UART_PER);
    SysCtlPeripheralEnable(UART_GPIO);

    GPIOPinConfigure(UART_RX);
    GPIOPinConfigure(UART_TX);
    GPIOPinTypeUART(UART_GPIO_BASE, UART_GPIO_PIN);

    UARTConfigSetExpClk(UART_BASE, SysCtlClockGet(), UART_BAUDRATE, UART_CONFIG);

    UARTIntRegister(UART_BASE, &UARTIntHandler);

    // Only enable RX and TX interrupts
    UARTIntEnable(UART_BASE, UART_INT_CONFIG);

    // Enable the UART interrupt
    IntEnable(UART_INT);
}

void UARTIntHandler(void) {
    uint32_t ui32Status;

    // Get interrupt status
    ui32Status = UARTIntStatus(UART_BASE, true);
    // Clear the asserted interrupts
    UARTIntClear(UART_BASE, ui32Status);

    while(UARTCharsAvail(UART_BASE)) {
        if(((UARTWriteIdx == (MAX_BUF_DATA_RECV -1))&&(UARTReadIdx != 0))||
           ((UARTWriteIdx != (MAX_BUF_DATA_RECV -1))&&((UARTWriteIdx+1) != UARTReadIdx))) {
            UARTBuf[UARTWriteIdx++] = UARTCharGetNonBlocking(UART_BASE);
            u16AvailByte++;
            UARTWriteIdx %= MAX_BUF_DATA_RECV;
        }
    }
}

uint16_t UARTQueryData(void) {
  return u16AvailByte;
}


void UARTRead(uint8_t * buf, uint16_t len) {
    uint16_t idx;
    if(UARTQueryData() >= len){
        for(idx = 0; idx < len; idx++) {
            if (UARTReadIdx != UARTWriteIdx) {
              *(buf + idx) = UARTBuf[UARTReadIdx++];
              if (u16AvailByte) {
                  u16AvailByte--;
              }
            }
            if(UARTReadIdx >= MAX_BUF_DATA_RECV) {
                UARTReadIdx = 0;
            }
        }
    }
}

void UARTWrite(uint8_t* cBuff, uint16_t ui16Len) {
    uint16_t ui16Idx;
    for(ui16Idx = 0; ui16Idx < ui16Len; ui16Idx++) {
        UARTCharPut(UART_BASE, (char)cBuff[ui16Idx]);
    }
}

void UARTWriteCMD(char* cBuff) {
    while(*cBuff != 0x00) {
        UARTCharPut(UART_BASE, (char)(*cBuff++));
    }
}
