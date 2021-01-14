/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: EEPROM routines header

Maintainer: Gregory Cristian, Gilbert Menth & Matthieu Verdy
*/

#include "mbed.h"

#define USB_SERIAL_MAX_LENGTH 30
#define USB_SERIAL_TIMEOUT 1000
extern int slave_index;
extern Serial pc;
extern uint8_t serialBuffer[USB_SERIAL_MAX_LENGTH];
extern UART_HandleTypeDef s_UARTHandle;

void FactoryReset( void );
void usbReceive();
