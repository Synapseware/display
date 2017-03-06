#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/delay.h>

//#define BAUD 9600
//#include <util/setbaud.h>

//-----------------------------------------------------------------------------
// Pin definitions
#define SPI_PORT		PORTB
#define SPI_DDR			DDRB
#define SPI_PIN			PINB

#define SCK				PB5
#define MISO			PB4
#define MOSI			PB3
#define SS				PB2

#define ADC_CHANNEL		0

#define DBG_LED			PB0
#define DBG_PORT		PORTB
#define DBG_PIN			PINB
#define DBG_DDR			DDRB


#define UART_PORT		PORTD
#define UART_DDR		DDRD
#define UART_RX			PD0
#define UART_TX			PD1


#endif
