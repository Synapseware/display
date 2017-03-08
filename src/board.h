#ifndef __BOARD_H__
#define __BOARD_H__

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

#define DBG_LED_ALT		PB1
#define DBG_LED			PB0
#define DBG_PORT		PORTB
#define DBG_PIN			PINB
#define DBG_DDR			DDRB


#define UART_PORT		PORTD
#define UART_DDR		DDRD
#define UART_RX			PD0
#define UART_TX			PD1

#endif
