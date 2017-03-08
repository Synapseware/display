#include "display.h"


volatile uint16_t	lastADC = 0;
volatile uint8_t	tick	= 0;



//-----------------------------------------------------------------------------
// Configure the SPI device
void ConfigureSPI(void)
{
	power_spi_disable();
	/*
	SPCR	=	(0<<SPIE)	|	// 
				(1<<SPE)	|	// Bit 6 – SPE: SPI Enable
				(0<<DORD)	|	// MSB first
				(1<<MSTR)	|	// Master mode
				(0<<CPOL)	|	// Mode 0
				(0<<CPHA)	|	// Mode 0
				(1<<SPR1)	|	// Fosc = Fcpu / 64
				(0<<SPR0);		// ...

	SPSR	=	(1<<SPIF)	|	// Clear flags
				(1<<WCOL)	|	// Clear flags
				(0<<SPI2X);		// No double speed

	// configure SPI pins
	SPI_DDR		|= (1<<SS) | (1<<SCK) | (1<<MOSI);
	SPI_DDR		&= ~(1<<MISO);

	SPI_PORT	|= (1<<SS) | (1<<SCK) | (1<<MOSI) | (1<<MISO);
	*/
}


//----------------------------------------------------------------
// Configures the UART for 9600-N-8-1
void ConfigureUART(void)
{
	UCSR0A	=	(0<<U2X0)	|	// No double speed
				(0<<MPCM0);		// No multi-proc mode

	UCSR0B	=	(0<<RXCIE0)	|	// RX Complete Interrupt Enable
				(0<<TXCIE0)	|	// TX Complete Interrupt Enable
				(0<<UDRIE0)	|	// USART Data Register Empty Interrupt Enable
				(0<<RXEN0)	|	// Receiver Enable
				(1<<TXEN0)	|	// Transmitter Enable
				(0<<UCSZ02);	// 8 bit characters

	UCSR0C	=	(0<<UMSEL01)|	// Async mode
				(0<<UMSEL00)|	// ...
				(0<<UPM01)	|	// No parity
				(0<<UPM00)	|	// ...
				(0<<USBS0)	|	// 1 stop bit
				(1<<UCSZ01)	|	// 8 bit characters
				(1<<UCSZ00)	|	// ..
				(0<<UCPOL0);	// Clock Polarity

	UBRR0	=	95;				// 9600 baud

	// configure UART port pins
	UART_DDR	|= (1<<UART_TX);
	UART_DDR	&= ~(1<<UART_RX);

	UART_PORT	|= (1<<UART_TX) | (1<<UART_RX);
}


//-----------------------------------------------------------------------------
// Configure Timer 0
void ConfigureTimer0(void)
{
	// Fcpu / 1024 / 144
	// 14745600 / 1024 / 144 = 100

	power_timer0_enable();

	TCCR0A	=	(1<<WGM01)	|	// CTC
				(0<<WGM00);		// ...

	TCCR0B	=	(0<<WGM02)	|	// ...
				(1<<CS02)	|	// Fcpu / 1024
				(0<<CS01)	|	// ...
				(1<<CS00);		// ...

	TIMSK0	=	(0<<OCIE0B)	|
				(1<<OCIE0A)	|
				(0<<TOIE0);

	OCR0A	=	144-1;
}


//-----------------------------------------------------------------------------
// Configure Timer 1
void ConfigureTimer1(void)
{
	power_timer1_enable();

	// FCPU / 1024 / 14400 = 1Hz
	// 14745600 / 1024 / 14400 = 1

	TCCR1A	=	(0<<COM1A1)	|
				(0<<COM1A0)	|
				(0<<COM1B1)	|
				(0<<COM1B0)	|
				(0<<WGM11)	|	// CTC
				(0<<WGM10);		// CTC

	TCCR1B	=	(0<<ICNC1)	|
				(0<<ICES1)	|
				(0<<WGM13)	|	// CTC
				(1<<WGM12)	|	// CTC
				(1<<CS12)	|	// FCPU/1024
				(0<<CS11)	|	// ...
				(1<<CS10);		// ...

	TIMSK1	=	(0<<ICIE1)	|
				(1<<OCIE1B)	|
				(1<<OCIE1A)	|	// Compare A interrupt enable
				(0<<TOIE1);

	OCR1A	=	14400-1;
	OCR1B	=	360;
}


//----------------------------------------------------------------
// Gets the MUX configuration bits for the specified channel
static void SelectChannel(uint8_t channel)
{
	channel		&=	0x0F;
	uint8_t mux =	(ADMUX & 0xF0) |	// mask out the channel bits
					(channel);			// set the channel

	if (channel < 8)
	{
		// disable digital input on the selected channel
		DIDR0 = (1<<channel);

		// set the pin as input
		DDRC &= ~(1<<channel);
	}
	else if (0x08 == channel) // 8
	{
		// internal temperature sensor
	}
	else if (0x0E == channel) // 14
	{
		// internal 1.1v band gap reference
	}
	else if (0x0F == channel) // 15
	{
		// ground
	}
	else
	{
		// invalid channel selection
		return;
	}

	// set the MUX register
	ADMUX = mux;
}


//-----------------------------------------------------------------------------
// 
void ConfigureADC(void)
{
	power_adc_enable();

	ADMUX	|=	(0<<ADLAR);		// Right adjust result

	SelectChannel(ADC_CHANNEL);

	ADCSRA	=	(1<<ADEN)	|	// ADC Enable
				(0<<ADSC)	|	// 
				(0<<ADATE)	|	// 
				(0<<ADIF)	|	// 
				(1<<ADIE)	|	// Enable interrupts
				(1<<ADPS2)	|	// Prescaler of 64
				(1<<ADPS1)	|	// 8MHz / 64 = 125kHz ADC clock
				(0<<ADPS0);		// ...

	ADCSRB	=	(0<<ACME)	|	// Disable the analog comparator
				(0<<ADTS2)	|	// Free running mode
				(0<<ADTS1)	|	// ...
				(0<<ADTS0);		// ...
}


//-----------------------------------------------------------------------------
// 
uint8_t WriteByte(uint8_t data)
{
	// clear transmit flag
	UCSR0A |= (1<<TXC0);

	// write the data to the USART register
	UDR0 = data;

	// wait for the data to be sent
	while (!(UCSR0A & (1<<TXC0)))
		;

	return data;

	/*
	// write to the data register
	SPDR = data;

	// wait for data to be transfered
	while(!(SPSR & (1<<SPIF)))
		;

	// return any data.  this also clears the SPIF in SPI status register
	return SPDR;
	*/
}


//-----------------------------------------------------------------------------
// Sets up the display
void InitializeDisplay(void)
{
	SPI_PORT &= ~(1<<SS);
	WriteByte(0x7F);		// BAUD rate
	WriteByte(2);			// 9600
	SPI_PORT |= (1<<SS);

	_delay_ms(10);

	SPI_PORT &= ~(1<<SS);
	WriteByte(0x7A);		// brightness control
	WriteByte(1);			// max
	SPI_PORT |= (1<<SS);

	_delay_ms(10);

	SPI_PORT &= ~(1<<SS);
	WriteByte('x');
	WriteByte('x');
	WriteByte('x');
	WriteByte('x');
	SPI_PORT |= (1<<SS);
}


//-----------------------------------------------------------------------------
// Updates the display with the given value
void UpdateDisplay(int data)
{
	char buff[4] = "xxxx";
	uint8_t idx = 0;

	if (data < 10)
		idx = 3;
	else if (data < 100)
		idx = 2;
	else if (data < 1000)
		idx = 1;

	utoa(data, &buff[idx], 10);

	// select the SS line
	SPI_PORT &= ~(1<<SS);

	WriteByte(buff[0]);
	WriteByte(buff[1]);
	WriteByte(buff[2]);
	WriteByte(buff[3]);

	// release the SS line
	SPI_PORT |= (1<<SS);
}


//-----------------------------------------------------------------------------
// 
void init(void)
{
	cli();

	ConfigureSPI();
	ConfigureUART();
	ConfigureTimer0();
	ConfigureTimer1();
	ConfigureADC();

	sei();

	// setup debug LEDs
	DBG_DDR |= (1<<DBG_LED) | (1<<DBG_LED_ALT);

	InitializeDisplay();
}


//-----------------------------------------------------------------------------
// main
int main(void)
{
	init();

	// main loop
	while(1)
	{
		if (tick)
		{
			UpdateDisplay(lastADC);

			tick = 0;
		}
	}
}


//-----------------------------------------------------------------------------
// Timer0 compare A @ 100Hz
/*
ISR(TIMER0_COMPA_vect)
{
	static uint8_t delay = 0;
	if (++delay < 25)
		return;
	delay = 0;
	tick = 1;
	ADCSRA |= (1<<ADSC);
}
*/
