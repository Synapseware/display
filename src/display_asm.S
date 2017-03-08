; Display assembly routines
#include <avr/io.h>
#include "board.h"


.data

;			could use .byte, but .zero init's to 0
delay:		.zero 1



.text

;------------------------------------------------------------------------------
; extern's from the rest of the program
.extern lastADC								; 16 bit value
.extern tick								; 8 bit value


;------------------------------------------------------------------------------
;ADC conversion complete
.global ADC_vect

	ADC_vect:
		in		r24, _SFR_IO_ADDR(SREG)
		push	r24

		lds		r24, ADCL					; read ADCL first
		lds		r25, ADCH					; read ADCH last
		sts		lastADC+1, r25
		sts		lastADC+0, r24

		; turn off the alternate LED
		cbi		_SFR_IO_ADDR(DBG_PORT), DBG_LED_ALT

		pop		r24
		out		_SFR_IO_ADDR(SREG), r24

		reti


;------------------------------------------------------------------------------
;Timer0 compare A @ 100Hz
.global TIMER0_COMPA_vect

	TIMER0_COMPA_vect:
		push	r24
		in		r24, _SFR_IO_ADDR(SREG)
		push	r24

		lds		r24, delay					; load the global delay value
		inc		r24							; increment
		sts		delay, r24					; store the updated global

		CPI		r24, 5						; introduce a delay
		brlo	TIMER0_COMPA_vect_01		; branch if delay not met

		sts		delay, r1					; reset the delay
		ldi		r24, 1						; toggle the "tick" global
		sts		tick, r24

		lds		r24, ADCSRA					; start an ADC conversion
		ori		r24, (1<<ADSC)				; set ADSC (bit 6)
		sts		ADCSRA, r24					; store the updated flag

		; turn on the alternate LED
		sbi		_SFR_IO_ADDR(DBG_PORT), DBG_LED_ALT

	TIMER0_COMPA_vect_01:
		pop		r24
		out		_SFR_IO_ADDR(SREG), r24
		pop		r24
		reti


;------------------------------------------------------------------------------
;Timer1 compare A
.global TIMER1_COMPA_vect

	TIMER1_COMPA_vect:
		sbi		_SFR_IO_ADDR(DBG_PORT), DBG_LED
		reti


;------------------------------------------------------------------------------
;Timer1 compare B
.global TIMER1_COMPB_vect

	TIMER1_COMPB_vect:
		cbi		_SFR_IO_ADDR(DBG_PORT), DBG_LED
		reti
