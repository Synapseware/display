; Display assembly routines
#include <avr/io.h>
#include "board.h"


.data

;			could use .byte, but .zero init's to 0
delay:		.zero 1

.text

;------------------------------------------------------------------------------
; extern's from the rest of the program
.extern lastADC
.extern tick


;------------------------------------------------------------------------------
;ADC conversion complete
.global ADC_vect

	ADC_vect:
		IN		r25,_SFR_IO_ADDR(SREG)
		PUSH	r25

		LDS		r24,_SFR_IO_ADDR(ADCL)		; read ADCL first
		LDS		r25,_SFR_IO_ADDR(ADCH)		; read ADCH last
		STS		lastADC+1,r25
		STS		lastADC,r24

		POP		r25
		OUT		_SFR_IO_ADDR(SREG),r25
		reti


;------------------------------------------------------------------------------
;Timer0 compare A @ 100Hz
.global TIMER0_COMPA_vect

	TIMER0_COMPA_vect:
		IN		r24, _SFR_IO_ADDR(SREG)
		PUSH	r24

		LDS		r24, delay					; load the global delay value
		INC		r24							; increment
		STS		delay, r24					; store the updated global

		CPI		r24, 25						; require 25 interrupts...
		BRLO	TIMER0_COMPA_vect_01		; skip out if no work

		STS		delay, r1					; reset the delay
		LDI		r24, 1						; toggle the "tick" global
		STS		tick, r24


		LDS		r24, _SFR_IO_ADDR(ADCSRA)	; start an ADC conversion
		ORI		r24, (1<<ADSC)
		STS		_SFR_IO_ADDR(ADCSRA), r24

	TIMER0_COMPA_vect_01:
		POP		r24
		OUT		_SFR_IO_ADDR(SREG), r24
		reti


;------------------------------------------------------------------------------
;Timer1 compare A
.global TIMER1_COMPA_vect

	TIMER1_COMPA_vect:
		SBI		_SFR_IO_ADDR(DBG_PORT),0
		reti


;------------------------------------------------------------------------------
;Timer1 compare B
.global TIMER1_COMPB_vect

	TIMER1_COMPB_vect:
		CBI		_SFR_IO_ADDR(DBG_PORT),0
		reti
