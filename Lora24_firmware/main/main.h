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
#include "math.h"

#define USB_SERIAL_MAX_LENGTH 30
#define USB_SERIAL_TIMEOUT 1000

 /*!
 * \brief maximum digits to display for a float in a printf
 */
#define FLOAT_DECIMALS 4
#define FLOAT_MAX_DIGITS 20
#define FLOAT_EXPONENT pow(10, FLOAT_DECIMALS)
#define _F(_FLOAT) (printFloat(_FLOAT))
#define INT(_FLOAT) (trunc(_FLOAT))
#define DEC(_FLOAT) (FLOAT_EXPONENT * fabs(_FLOAT - INT(_FLOAT)) )

#if (!defined(NUCLEO_L432KC) && !defined(NUCLEO_L476RG))
  #define NUCLEO_L432KC
#endif


extern int slave_index;
extern BufferedSerial pc;

extern const int RNG_ADDR_LIST[];
extern uint8_t serialBuffer[USB_SERIAL_MAX_LENGTH];


void FactoryReset( void );
void usbReceive();
char *printFloat(float f);
