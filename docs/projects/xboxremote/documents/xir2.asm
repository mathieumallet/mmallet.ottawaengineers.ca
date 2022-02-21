	list      p=12F675	; list directive to define processor
	#include <p12f675.inc>	; processor specific variable definitions

	__CONFIG  _CP_OFF & _WDT_ON & _BODEN_ON & _PWRTE_ON & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _CPD_OFF

;******************************************************************************
; Macros
;******************************************************************************
#define BANK0Selected
BANK0 macro		; this macro switches bank # to 0
	ERRORLEVEL +302
	bcf STATUS, RP0
	endm
BANK1 macro		; this macro switches bank # to 1, and disables those damn warning msg
	ERRORLEVEL -302
 	bsf STATUS, RP0
	endm


;******************************************************************************
; Defines
;******************************************************************************

; IR codes
#define IRCODE_ON_BYTE1		b'01001010'		; bits 7-0 of IR code
#define IRCODE_ON_BYTE2	    b'01010101'		; bits 15-8 of IR code
#define IRCODE_ON_BYTE3		b'10101011'		; bits 23-16 of IR code
#define IRCODE_RESET_BYTE1	b'11001010'		; bits 7-0 of IR code
#define IRCODE_RESET_BYTE2	b'01010000'		; bits 15-8 of IR code
#define IRCODE_RESET_BYTE3	b'11110011'		; bits 23-16 of IR code

; How many Timer1 'overflows' to count before considering that a button has been
; held long enough for the second button function (e.g. reset/off)
#define REQ_HOLD_COUNTS		.4	; actually .10, but we check if result of substraction is negative so we need one less

; How many Timer1 'overflows' to count before the device should go in sleep mode
#define IDLE_TIME_BEFORE_SLEEP	.240

; Waveform timings
; Exact Header High length: 4172 탎, 149 loops
;#define HEADER_HI_MIN_LENGTH	.120		; min lenght of header 'high' time, in number of measuring loops (1탎 * 28 instructions)
;#define HEADER_HI_MAX_LENGTH	.140		; max lenght of header 'high' time, in number of measuring loops (1탎 * 28 instructions)

; Exact Header Low length: 3821 탎, 136.5 loops
;#define HEADER_LO_MIN_LENGTH	.116	; min length of header 'low' time, in number of measuring loops (1탎 * 28 instructions)
;#define HEADER_LO_MAX_LENGTH	.156	; max length of header 'low' time, in number of measuring loops (1탎 * 28 instructions)

; Exact High part of encoded bit length: 653 탎, 54.4 loops
;#define HIGH_BIT_MAX_LENGTH		.64		; max length of the 'high' part of an encoded bit, in # of measuring loops (1탎 * 12 instructions)

;#define LOW_BIT_ZERO_MIN_LENGTH		.1		; min lenght of 'low' bit that defines a zero, in number of measuring loops (1탎 * 12 instructions)
;#define LOW_BIT_ZERO_MAX_LENGTH		.137	; max lenght of 'low' bit that defines a zero, in number of measuring loops (1탎 * 12 instructions)
;#define LOW_BIT_ONE_MAX_LENGTH		.250	; max lenght of 'low' bit that defines a one, in number of measuring loops (1탎 * 12 instructions)

;
; We use more generous timing margins because the internal RC oscillator isn't as precise as we wish it was :)
;

; Exact Header High length: 4172 탎, 149 loops
#define HEADER_HI_MIN_LENGTH	.100		; min lenght of header 'high' time, in number of measuring loops (1탎 * 28 instructions)
#define HEADER_HI_MAX_LENGTH	.160		; max lenght of header 'high' time, in number of measuring loops (1탎 * 28 instructions)

; Exact Header Low length: 3821 탎, 136.5 loops
#define HEADER_LO_MIN_LENGTH	.100	; min length of header 'low' time, in number of measuring loops (1탎 * 28 instructions)
#define HEADER_LO_MAX_LENGTH	.176	; max length of header 'low' time, in number of measuring loops (1탎 * 28 instructions)

; Exact High part of encoded bit length: 653 탎, 54.4 loops
#define HIGH_BIT_MAX_LENGTH		.64		; max length of the 'high' part of an encoded bit, in # of measuring loops (1탎 * 12 instructions)

#define LOW_BIT_ZERO_MIN_LENGTH		.1		; min lenght of 'low' bit that defines a zero, in number of measuring loops (1탎 * 12 instructions)
#define LOW_BIT_ZERO_MAX_LENGTH		.137	; max lenght of 'low' bit that defines a zero, in number of measuring loops (1탎 * 12 instructions)
#define LOW_BIT_ONE_MAX_LENGTH		.250	; max lenght of 'low' bit that defines a one, in number of measuring loops (1탎 * 12 instructions)


; Pins
#define SIGNAL_PIN			5			; GP5 (Pin 2 on chip)
#define STATUS_PIN			4			; GP4 (Pin 3 on chip)
#define XBOX_ON_PIN			3			; GP3 (Pin 4 on chip)
#define XBOX_PWR_SWITCH_PIN	2			; GP2 (Pin 5 on chip)

#define DIP0_PIN			0			; GP0 (Pin 7 on chip)
#define DIP1_PIN			1			; GP1 (Pin 6 on chip)
#define HOLD_COMMANDS_ENABLED_PIN	DIP0_PIN
#define	STATUS_LED_ENABLED_PIN		DIP1_PIN

; Flags in MYSTATUS register
#define	ABORT_FLAG			0

;******************************************************************************
; General Purpose Registers (GPR's) 
;******************************************************************************

	cblock	0x20

	; State registers
	ACTIVE_CODE_ID	; contains 1 if ON code is active, 2 if RESET code is active	
	
	; Timing registers
	CODE_ON_HELD_TIME
	CODE_RESET_HELD_TIME	
	IDLE_TIME
	LAST_TIMER1_VALUE
	CURRENT_OVERFLOW_NUM	; 0 if this is the second overflow, 1 if it is the first

	; Registers used in delay functions
	TIMER1_INTERRUPT_OCCURED
	TEMP
	TEMP2
	TEMP3

	; Storage for interupt handling
	INTERRUPT_STATUS
	INTERRUPT_W

	; Misc Registers
	RETURN_VALUE

	; For debugging
	STATUS_LED
	STATUS_LED2
	MYDATA


; Registers used by the ReceiveIRCode subroutine:
	; Received code registers
	CODE_BYTE1
	CODE_BYTE2
	CODE_BYTE3

	; Misc registers
	LOOPS_COUNTER
	CURRENT_BYTE
	CURRENT_BYTE_MEM_OFFSET
	CURRENT_BIT

	endc

;******************************************************************************
; Reset Vector 
;******************************************************************************
	ORG     0x000		; processor reset vector
	nop					; required by in circuit debugger
	goto    Init		; go to beginning of program

;******************************************************************************
; Interrupt Vector  
;  The Interrupt Vector is only used when there is a change on the signal pin
;  while the device is in sleep.   
;******************************************************************************
	ORG     0x004
	movf	GPIO, W			; Read GPIO to clear interrupt-on-change mismatch condition
	bcf		INTCON, GPIF	; Clear interrupt flag
	retfie
	;goto	Interupt_Handler	; Set interupt vector

;******************************************************************************
; Initialization
;******************************************************************************
Init:

; Init inputs/outpus

	BANK0
	clrf 	GPIO		; Init GPIO -- all pins are input by default

	movlw 	07h			; Disable comparator (thus enabling digital I/O ports)
	movwf	CMCON		;

; Init Watchdog Timer
	BANK1
	movlw	b'0001111'	; Sets prescaler assignment to watchdog timer,
	movwf	OPTION_REG	; sets prescaler to 111 (1:128)
						; and enables individual pull-ups on GPIO

; Init rest of input/ouputs
	;BANK1
	clrf	ANSEL				; Set pins 0-3 to be digital I/O ports
	clrf	TRISIO				; Clear TRISO register (set all pins as outputs)
	bsf		TRISIO, SIGNAL_PIN	; Set signal pin to be an input (duh)
	bsf		TRISIO, DIP0_PIN	; Set first dip switch pin as input
	bsf		TRISIO, DIP1_PIN	; Set second dip switch pin as input
	bsf		TRISIO, XBOX_ON_PIN	; Set xbox status pin as input
	bsf		WPU, DIP0_PIN		; Enabled weak pull-up on DIP0
	bsf		WPU, DIP1_PIN		; Enabled weak pull-up on DIP1
	
; Set state of GPIO for xbox power switch to high (switch is active low)
	BANK0
	bsf		GPIO, XBOX_PWR_SWITCH_PIN

; Init TIMER1
	;BANK0
	movlw	b'00110101'	; Use prescaler of 8 -- this will give a period to overflow
	movwf	T1CON		; of 0.524288 seconds

; Setup interrupts
	;BANK0
	bsf		INTCON, GIE		; enable maskable interrupts
	bsf		INTCON, GPIE	; enable interrupts on change on GPIO

; Flash init done sequence
	call	FlashLed
	call	FlashLed
	call	FlashLed
	

;******************************************************************************
; Main program
;******************************************************************************

; Wait for header
Beginning:
	;BANK0
	call	ResetTimers
	clrf	IDLE_TIME

PreSeeking:
	;BANK0
	clrwdt
	call	HandleTimer1			; Handle Timer1 'overflows'

	;BANK0
	btfsc	GPIO, SIGNAL_PIN		; Wait till we receive a signal (0 on pin)		
	goto	PreSeeking

; Call function to receive and store the IR signal
	call	ReceiveIRCode
	movwf	RETURN_VALUE

; Check if code was received successfully
;	movf	RETURN_VALUE, F
;	btfsc	STATUS, Z
;	call	ToggleLed

; Ignore 'header too short' errors -- those are due to fact that we're not reading
; all of the signal yet
;	movfw	RETURN_VALUE
;	sublw	.1
;	btfsc	STATUS, Z
;	goto	PreSeeking

; Check if we got an error code
	movf	RETURN_VALUE, F
	btfss	STATUS, Z
	goto	PreSeeking	; No point to check for proper code if we got an error

; Check if we received correct code for '0'
	movfw	CODE_BYTE1
	xorlw	IRCODE_RESET_BYTE1
	btfss	STATUS, Z
	goto	CheckNextCode

	movfw	CODE_BYTE2
	xorlw	IRCODE_RESET_BYTE2
	btfss	STATUS, Z
	goto	CheckNextCode

	movfw	CODE_BYTE3
	xorlw	IRCODE_RESET_BYTE3
	btfss	STATUS, Z
	goto	CheckNextCode
	
	call	Handle0Detected

; Check if we received correct code for 'DISPLAY'
CheckNextCode:
	movfw	CODE_BYTE1
	xorlw	IRCODE_ON_BYTE1
	btfss	STATUS, Z
	goto	PreSeeking

	movfw	CODE_BYTE2
	xorlw	IRCODE_ON_BYTE2
	btfss	STATUS, Z
	goto	PreSeeking

	movfw	CODE_BYTE3
	xorlw	IRCODE_ON_BYTE3
	btfss	STATUS, Z
	goto	PreSeeking

	call	HandleDisplayDetected

; Continue seeking
	goto	PreSeeking


;******************************************************************************
; ReceiveIRCode
; -----------------------------------------------------------------------------
; Function that receives an IR code and stores it in registers. The subroutine
; assumes that the beginning of a signal has already been received (e.g. high
; signal received, 0 on pin)
;
; The following values are returned in W to indicate the status of
; the reception:
; 0 - Code received OK
; 1 - High part of header too short
; 2 - High part of header too long
; 3 - Low part of header too short
; 4 - Low part of header too long
; 5 - High part of data bit too short
; 6 - High part of data bit too long
; 7 - Low part of data bit too short
; 8 - Low part of data bit too long
; 9 - Overflow on loop counter
;******************************************************************************
ReceiveIRCode:
	clrf	LOOPS_COUNTER

; Loop till signal changes to low (1 on pin)
Seeking:	; loop is 28 cycles long
	btfsc	GPIO, SIGNAL_PIN
	goto	Continue

	incf	LOOPS_COUNTER, F
	call	ReturnLabel		; 4 instructions delay
	call	ReturnLabel		; 4 instructions delay
	call	ReturnLabel		; 4 instructions delay
	call	ReturnLabel		; 4 instructions delay
	nop
	
	btfsc	STATUS, Z	; Check if counter overflowed (should never happen)
	retlw	9	; Counter overflowed

	movfw	LOOPS_COUNTER
	sublw	HEADER_HI_MAX_LENGTH
	btfss	STATUS, C	; Check if high part of header is too long (carry of 0 means that result was negative)
	retlw	2	; High part of header too long	

	goto	Seeking

Continue:
	movfw	LOOPS_COUNTER
	sublw	HEADER_HI_MIN_LENGTH
	btfsc	STATUS, C	; Check if high part of header is too short (carry of 0 means that result was negative)
	retlw	1	; High part of header too short

	clrf	LOOPS_COUNTER

; Loop till signal changes to high (0 on pin, which represents the beginning of the signal data)
Seeking2:	; loop is 28 cycles long
	btfss	GPIO, SIGNAL_PIN
	goto	Continue2	

	incf	LOOPS_COUNTER, F
	call	ReturnLabel		; 4 instructions delay
	call	ReturnLabel		; 4 instructions delay
	call	ReturnLabel		; 4 instructions delay
	call	ReturnLabel		; 4 instructions delay
	nop
	
	btfsc	STATUS, Z	; Check if counter overflowed (should never happen)
	retlw	9	; Counter overflowed

	movfw	LOOPS_COUNTER
	sublw	HEADER_LO_MAX_LENGTH
	btfss	STATUS, C	; Check if low part of header is too long (carry of 0 means that result was negative)
	retlw	4	; Low part of header too long	

	goto	Seeking2


Continue2:
	movfw	LOOPS_COUNTER
	sublw	HEADER_LO_MIN_LENGTH
	btfsc	STATUS, C	; Check if low part of header is too short (carry of 0 means that result was negative)
	retlw	3	; Low part of header too short

	clrf	LOOPS_COUNTER

; Loop till signal changes to low (1 on pin, what follows will be a bit encoded in the duration of the low signal)
Seeking3:	; loop is 12 instructions long
	btfsc	GPIO, SIGNAL_PIN
	goto	Continue3	

	incf	LOOPS_COUNTER, F
	nop
	
	btfsc	STATUS, Z	; Check if counter overflowed (should never happen)
	retlw	9	; Counter overflowed

	movfw	LOOPS_COUNTER
	sublw	HIGH_BIT_MAX_LENGTH
	btfss	STATUS, C	; Check if high part of separator bit is too long (carry of 0 means that result was negative)
	retlw	6	; Low part of header too long	

	goto	Seeking3


Continue3:
	; We don't check if high part of encoded bit is too short -- not really any point

	; Setup sampling registers
	;clrf	CURRENT_BYTE	; not needed since we'll be rotating through carry 8 times
	movlw	CODE_BYTE1
	movwf	CURRENT_BYTE_MEM_OFFSET
	movlw	.8
	movwf	CURRENT_BIT


SamplingLoop:
	clrf	LOOPS_COUNTER

; Loop till signal changes to high (0 on pin, meaning whatever is in LOOPS_COUNTER is the lenght of the space)
Seeking4:	; loop is 12 instructions long
	btfss	GPIO, SIGNAL_PIN
	goto	Continue4

	incf	LOOPS_COUNTER, F
	nop
		
	btfsc	STATUS, Z	; Check if counter overflowed (should never happen)
	retlw	9	; Counter overflowed

	movfw	LOOPS_COUNTER
	sublw	LOW_BIT_ONE_MAX_LENGTH
	btfss	STATUS, C	; Check if low part of encoded bit is too long (carry of 0 means that result was negative)
	retlw	8	; Low part of encoded bit too long

	goto	Seeking4

Continue4:
	movfw	LOOPS_COUNTER
	sublw	LOW_BIT_ZERO_MIN_LENGTH
	btfsc	STATUS, C	; Check if low part of encoded bit is too short (carry of 0 means that result was negative)
	retlw	7	; Low part of encoded bit too short

	; Check if we received a 0 or a 1
	movfw	LOOPS_COUNTER
	sublw	LOW_BIT_ZERO_MAX_LENGTH

	; The last operation puts 1 in the Carry if result is negative -- that is it puts 1 in the carry
	; if the pulse is 'short', e.g. if the pulse represents a '0' of data. Since we'll use the carry
	; to rotate the bytes, we just leave it like that and go ahead with the byte rotation. Note
	; that we'll have to invert the result once a whole byte has been received.
	rrf		CURRENT_BYTE, F

	clrf	LOOPS_COUNTER

	decf	CURRENT_BIT, F
	btfsc	STATUS, Z
	goto	SaveCurrentByte

; loop till we get a low signal (1 on pin)
Seeking5:	; loop is 12 instructions long
	btfsc	GPIO, SIGNAL_PIN
	goto	SamplingLoop

	incf	LOOPS_COUNTER, F
	nop
		
	btfsc	STATUS, Z	; Check if counter overflowed (should never happen)
	retlw	9	; Counter overflowed

	movfw	LOOPS_COUNTER
	sublw	HIGH_BIT_MAX_LENGTH
	btfss	STATUS, C	; Check if high part of encoded bit is too long (carry of 0 means that result was negative)
	retlw	6	; Low part of encoded bit too long

	goto	Seeking5

; Save the current byte in the proper register and stop sampling if we're done
SaveCurrentByte:
	movfw	CURRENT_BYTE_MEM_OFFSET		; Set indirect address to write to
	movwf	FSR							;

	movfw	CURRENT_BYTE				; Store received byte in memory
	xorlw	0xFF						; 
	movwf	INDF						;	

	movlw	.8							; Reset bit count
	movwf	CURRENT_BIT					;

	incf	CURRENT_BYTE_MEM_OFFSET, F	; Increase byte offset
	movfw	CURRENT_BYTE_MEM_OFFSET		; Check if we received 3 bytes already
	sublw	(CODE_BYTE1  +  3)
	btfss	STATUS, Z
	goto 	Seeking5					; If not, continue sampling.
	
; We successfully received all 24 bits -- seek till the end of the stop bit just to be safe
; loop till we get a low signal (1 on pin)
Seeking6:	; loop is 12 instructions long
	btfsc	GPIO, SIGNAL_PIN
	goto	SamplingCompleted

	incf	LOOPS_COUNTER, F
	nop
		
	btfsc	STATUS, Z	; Check if counter overflowed (should never happen)
	retlw	9	; Counter overflowed

	movfw	LOOPS_COUNTER
	sublw	HIGH_BIT_MAX_LENGTH
	btfss	STATUS, C	; Check if high part of encoded bit is too long (carry of 0 means that result was negative)
	retlw	6	; Low part of encoded bit too long

	goto	Seeking6

SamplingCompleted:
	retlw	0





;******************************************************************************
; Interrupt Handler
;  The interrupt handler is only called by a change on the data I/O while in
;  sleep mode.
;******************************************************************************

Interupt_Handler:
	; Save current status and such
	movwf	INTERRUPT_W	
	movfw	STATUS
	movwf	INTERRUPT_STATUS

	;call	ToggleLed2	 ; debug: toggle the red led

; Check if we got an interrupt from TIMER0
	btfsc	INTCON, T0IF
	call 	Timer0_Int_Handler

	;goto	Return_From_Interrupt	; Not needed since we just fall down onto the return code

Return_From_Interrupt:
	; Restore status from before this interrupt handler was called
	movfw	INTERRUPT_STATUS
	movwf	STATUS
	movfw	INTERRUPT_W	

	retfie

; Timer0 handler -- sets the abort flag
Timer0_Int_Handler:
	bcf		INTCON, T0IF			; Clear timer0 overflow flag	
	;bsf		MYSTATUS, ABORT_FLAG	; Set abort flag
	return	


;******************************************************************************
; ToggleLed
;   Toggles the value of the status led. For debugging only.
;******************************************************************************

ToggleLed:
	movfw STATUS_LED		; Get status of led from memory
	xorlw b'00000001'		; Toggle led status
	movwf STATUS_LED		; Store new value
	bcf GPIO, STATUS_PIN 	; Set led to off
	btfsc STATUS_LED, 0		; Check if STATUS_LED == 1
	bsf GPIO, STATUS_PIN 	; If so, set led to on

	return

;******************************************************************************
; Flash 8 bit code
;   Displays 8 bit of data on the 2 leds: green = 1, red = 0
;   Bits are flashed most-significative-bit first (so they can be written down
;   easier)
;******************************************************************************
;Flash8Bit:
;	movwf	MYDATA ; store 
;	movlw	.8
;	movwf	TEMP3
;	bsf		GPIO, STATUS_PIN
;	bsf		GPIO, STATUS_PIN2
;	call	WaitHalfSecond
;	call	WaitHalfSecond
;
;	; Quick off time for both leds so we can see bit separation
;	bcf		GPIO, STATUS_PIN
;	bcf		GPIO, STATUS_PIN2
;	call	Count65536
;
;FlashLoop:
;	; Turn bits on for half a second
;	bcf		GPIO, STATUS_PIN
;	bcf		GPIO, STATUS_PIN2
;	rlf		MYDATA, F
;	btfsc	STATUS, C
;	bsf		GPIO, STATUS_PIN
;	btfss	STATUS, C
;	bsf		GPIO, STATUS_PIN2
;	call	WaitHalfSecond
;
;	; Quick off time for both leds so we can see bit separation
;	bcf		GPIO, STATUS_PIN
;	bcf		GPIO, STATUS_PIN2
;	call	Count65536
;
;	decfsz	TEMP3, F
;	goto	FlashLoop	
;
;	bcf		GPIO, STATUS_PIN
;	bcf		GPIO, STATUS_PIN2
;	call	WaitHalfSecond
;	
;	return

;******************************************************************************
; Wait 5 seconds
;   Waits roughly 5 second before returning (4.9s)
;******************************************************************************
Wait5Seconds: ; (waits for 4921704 cycles)
	movlw	.25
	movwf	TEMP3
Wait5SecondsLoop:
	clrwdt
	call 	Count65536
	decfsz	TEMP3, F
	goto	Wait5SecondsLoop
	return

;******************************************************************************
; Wait Half Second
;   Waits roughly half a second before returning (0.59s)
;******************************************************************************
WaitHalfSecond: ; (waits for 590587 cycles)
	clrwdt
	call 	Count65536
	call 	Count65536
	call 	Count65536
	return

;******************************************************************************
; Wait 200 ms
;   Waits roughly 200 ms before returning (196 ms)
;******************************************************************************
Wait200ms: ; (waits for 196863 cycles)
	clrwdt

Count65536: ; 5 cyles * 255 (waits for 196862 cycles)
	movlw	0xFF
	movwf	TEMP
Loop65536
	call	Count255
	decfsz	TEMP, F
	goto Loop65536
	return

Count255:	; 3 cycles * 255 (waits for 767 cycles)
	movlw	0xFF
	movwf	TEMP2
Loop255
	decfsz	TEMP2, F
	goto Loop255
	return
	

;******************************************************************************
; HandleTimer1
;   This subroutine constantly checks the status of the Timer1 and will
;   execute extra code if the timer overflowed.
;******************************************************************************
HandleTimer1:
	; Substract current timer value from last timer value
	movfw	TMR1H
	movwf	TEMP
	movfw	LAST_TIMER1_VALUE
	subwf	TEMP, W	; W = LAST_TIMER1_VALUE - W (positive if overflowed)

	; Check if timer value changed
	btfsc	STATUS, Z
	return	; If not, return

	; Save current timer value (next two ops don't affect carry of last operation)
	movfw	TEMP
	movwf	LAST_TIMER1_VALUE

	; Check if timer overflowed
	btfsc	STATUS, C	; If C is 1 then result is positive and timer overflowed
	return

	; Timer1 overflowed!!

	; Check if this is the first overflow or the second (so we have kinda of like a timer
	; with a 1 second period)
	incf	CURRENT_OVERFLOW_NUM, F		; Since we only care about the first bit, incrementing
										; the register is effectively toggling the first bit
	btfsc	CURRENT_OVERFLOW_NUM, 0
	return

	; Check if the hold commands are enabled
	btfsc	GPIO, HOLD_COMMANDS_ENABLED_PIN
	goto 	IdleHandleTimer		; If not skip to idle handling

	; Check if we're holding anything
	movf	ACTIVE_CODE_ID, F
	btfsc	STATUS, Z
	goto	IdleHandleTimer

	; Since we're holding something, it means we're not idling anymore
	clrf	IDLE_TIME

	; No point in handling held buttons if xbox is off...
	btfss	GPIO, XBOX_ON_PIN
	goto	IdleHandleTimer

	; Check if active code is 'ON'
	btfsc	ACTIVE_CODE_ID, 0
	goto	OnHandleTimer

	; Check if active code is 'Reset'
	btfsc	ACTIVE_CODE_ID, 1
	goto	ResetHandleTimer

	; Nothing is active! Fall back to idling (should never be needed)
	goto	IdleHandleTimer

OnHandleTimer:
	; If we've held the button for at least 1 second, then turn on the status led
	btfsc	CODE_ON_HELD_TIME, 0
	call	SetLedOn

	clrf	ACTIVE_CODE_ID	; Clear current code (needs to be received again to be considered held down)
	clrf 	CODE_RESET_HELD_TIME
	clrf	IDLE_TIME
	incf	CODE_ON_HELD_TIME, F
	movf	CODE_ON_HELD_TIME, W
	sublw	REQ_HOLD_COUNTS

	; Check if we exceeded the required count
	btfsc	STATUS, C		; 0 in carry = negative result
	return

	; Check if xbox is already off (technically this should never happen)
	btfss	GPIO, XBOX_ON_PIN
	goto	ResetTimersHandleTimer	; Xbox is off -- do nothing and return	

	call	PowerOff
	return

ResetHandleTimer:
	; If we've held the button for at least 1 second, then turn on the status led
	btfsc	CODE_RESET_HELD_TIME, 0
	call	SetLedOn

	clrf	ACTIVE_CODE_ID	; Clear current code (needs to be received again to be considered held down)
	clrf 	CODE_ON_HELD_TIME
	clrf	IDLE_TIME
	incf	CODE_RESET_HELD_TIME, F
	movf	CODE_RESET_HELD_TIME, W
	sublw	REQ_HOLD_COUNTS

	; Check if we exceeded the required count
	btfsc	STATUS, C		; 0 in carry = negative result
	return

	; Check if xbox is already off (technically this should never happen)
	btfss	GPIO, XBOX_ON_PIN
	goto	ResetTimersHandleTimer	; Xbox is off -- do nothing and return	

	call	ResetXbox
	return

IdleHandleTimer:
	call	SetLedOff
	incf	IDLE_TIME, F
	;clrf	ACTIVE_CODE_ID	; Active code is reset in ResetTimers subroutine

	movlw	IDLE_TIME_BEFORE_SLEEP
	subwf	IDLE_TIME, W
	
	btfsc	STATUS, Z	; If result of substraction is negative, then we need to go to sleep.
	call	HandleSleep

ResetTimersHandleTimer:
	call	ResetTimers
	return

;******************************************************************************
; Handle0Detected
;   This subroutine handles the detection of a single '0' press.
;******************************************************************************
Handle0Detected:
	; Save active code
	movlw	.2	; 2 = code for 'reset'
	movwf	ACTIVE_CODE_ID

	; The '0' key doesn't do anything when it's pressed only once
	return


;******************************************************************************
; HandleDisplayDetected
;   This subroutine handles the detection of a single 'Display' press.
;******************************************************************************
HandleDisplayDetected:
	; Save active code
	movlw	.1	; 1 = code for 'on'
	movwf	ACTIVE_CODE_ID

	; Check if xbox is already on
	btfsc	GPIO, XBOX_ON_PIN
	return	; Xbox is on -- do nothing and return

	; Call function to powerup the xbox
	call	PowerUp

	call	ResetTimers

	return


;******************************************************************************
; PowerUp
;   This subroutine will powerup the xbox and wait 5 seconds before returning.
; PowerOff
;   Hm seems code for powering off is pretty much the same as code for powering
;   on :)
;******************************************************************************
PowerUp:
PowerOff:
	bcf		GPIO, XBOX_PWR_SWITCH_PIN	; Pull power switch down
	
	; Wait 600 ms before stopping
	call	FlashOK	; Takes ~400ms
	call	Wait200ms

	bsf		GPIO, XBOX_PWR_SWITCH_PIN
	
	; Wait 5 seconds before doing anything else
	call	Wait5Seconds

	; Reset hold timers
	call	ResetTimers

	return

;******************************************************************************
; ResetXbox
;   This subroutine will poweroff the xbox, wait 1 second and power it on
;   again.
;******************************************************************************
ResetXbox:
	bcf		GPIO, XBOX_PWR_SWITCH_PIN	; Pull power switch down
	
	; Wait 600 ms before stopping
	call	FlashOK	; Takes ~400ms
	call	Wait200ms

	bsf		GPIO, XBOX_PWR_SWITCH_PIN

	; Wait a bit before turning back on
	call	Wait200ms
	call	Wait200ms
	call	Wait200ms

	call	PowerUp

	; No need to call ResetTimers since it's called in the PowerUp subroutine

	return


;******************************************************************************
; ResetTimers
;   This subroutine will reset all timers (such as hold time and idle time)
;   as well as the current active code
;******************************************************************************
ResetTimers:
	; Clear hold & idle timers
	clrf	ACTIVE_CODE_ID
	clrf	CODE_ON_HELD_TIME
	clrf	CODE_RESET_HELD_TIME	
	;clrf	IDLE_TIME
	clrf	CURRENT_OVERFLOW_NUM

	; Save current timer1 value
	movfw	TMR1H
	movwf	LAST_TIMER1_VALUE

	return

;******************************************************************************
; HandleSleep
;   This subroutine sends the device into sleep mode until it is waken-up
;   by either the WDT or a change on the IR receiver. A wakeup by the WDT
;   makes the device go back into sleep.
;******************************************************************************
HandleSleep:
	; Show user that we're going into sleep mode
	clrwdt
	call	FlashLed

	; Init registers & interrupts
	clrf	IDLE_TIME
	BANK1
	bsf		IOC, SIGNAL_PIN		; Enable interrupt on change on the signal port
	BANK0

SleepLoop:
	sleep

	; Check if we woke up due to a WDT timeout
	btfss	STATUS, 4
	goto	SleepLoop ; if so, go back to sleep

	BANK1
	bcf		IOC, SIGNAL_PIN		; Disable interrupt on change on the signal port
	BANK0

	return


;******************************************************************************
; SetLedOn
;   This subroutine will turn on the status led.
;******************************************************************************
SetLedOn:
	btfss	GPIO, STATUS_LED_ENABLED_PIN
	bsf		GPIO, STATUS_PIN
	return;

;******************************************************************************
; SetLedOff
;   This subroutine will turn off the status led.
;******************************************************************************
SetLedOff:
	bcf		GPIO, STATUS_PIN
	return;

;******************************************************************************
; FlashLed
;   This subroutine will turn the led on, wait for 200ms, turn the led off,
;   wait for 200ms and return.
;******************************************************************************
FlashLed:
	call	SetLedOn
	call	Wait200ms
	call	SetLedOff
	call	Wait200ms
	return

;******************************************************************************
; FlashError
;   This subroutine will flash the led once.
;******************************************************************************
FlashError:
	call	FlashLed
	return

;******************************************************************************
; FlashError
;   This subroutine will flash the led twice (takes ~400ms)
;******************************************************************************
FlashOK:
	call	FlashLed
	call	FlashLed
	return

;******************************************************************************
; ReturnLabel
;   Calling this subroutine induces a delay of 3 instructions (using only one!)
;******************************************************************************
ReturnLabel:
	return

;******************************************************************************
; WDTTrap
;   The following code fills the rest of the memory with the 'goto WDTTrap'
;   instruction. In the unlikely case that the program starts executing
;   empty memory, then the program will loop and the WDT will reset the
;   device.
;******************************************************************************
WDTTrap:
	fill (goto WDTTrap), 0x03FF-$

	END
