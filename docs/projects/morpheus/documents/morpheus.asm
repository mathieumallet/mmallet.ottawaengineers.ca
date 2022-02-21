	list      p=16F676	; list directive to define processor
	#include <p16f676.inc>	; processor specific variable definitions

	__CONFIG  _CP_OFF & _WDT_ON & _BODEN & _PWRTE_ON & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _CPD_OFF
	__idlocs H'268D'

;#define TESTING
;#define TEST_SETTINGS
;#define DETAILLED_SCAN_LOGGING	; only when TESTING is defined
#define USE_PRECISE_AD
;#define TIMER_CALIBRATION		; uncommenting this will make leds blink every 150 seconds (2.5 minutes)

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

SWAPWF	macro	REG		; This macro performs a switch-a-roo between W and REG
	xorwf	REG, F
	xorwf	REG, W
	xorwf	REG, F
	endm


;******************************************************************************
; Defines
;******************************************************************************

; PORTA mappings
#define	ENTER_BTN_BIT				0	; RA0, pin # 13
#define SELECT_BTN_BIT				1	; RA1, pin # 12
#define ACTIVATE_BTN_BIT			3	; RA3, pin # 4
#define RIGHT_LED_BIT				5	; RA5, pin # 2
#define IR_TRANSISTOR_BIT			4	; RA4, pin # 3
#define BATTERY_TEST_PROBE_BIT		2	; RA2, pin # 11
#define INSTANT_SCAN_MASK			b'00001001'

; PORTC mappings
#define LEFT_LED_BIT				3	; RC3, pin # 7
#define BUZZER_BIT					2	; RC2, pin # 8
#define IR_LED_BIT					5	; RC5, pin # 5
#define BATTERY_TEST_ENABLE_BIT		4	; RC4, pin # 6
#define SWITCHED_GROUND_ENABLE_BIT	BATTERY_TEST_ENABLE_BIT
#define RS232_RX					0	; RC0, pin # 10
#define RS232_TX					1	; RC1, pin # 9

; A/D Modes
#define AD_BATTERY_PROBE_MODE		b'00001001'
#ifndef USE_PRECISE_AD
  #define AD_REM_SCAN_MODE			b'00001101'
  #define AD_JITTER_THRESHOLD		.2			; Consider difference of 0x01 on A/D result as being same as last result
#else
  #define AD_REM_SCAN_MODE			b'10001101'
  #define AD_JITTER_THRESHOLD		.3			; Consider difference of 0x03 on A/D result as being same as last result
#endif
#define AD_INACTIVE_MODE			b'00001100'
#define AD_PORT_SELECTION			b'00001100'

; Misc defines
#define PWM_PULSE_DURATION	.3					; 1 = 1 loop, 255 = 255 loops, 0 = 256 loops
#define TIMER1_OVERFLOWS_PER_150_SECS .29		; Use timer period of 30 seconds instead of 2.5 minutes
;#define TIMER1_OVERFLOWS_PER_150_SECS .147		; Number of TIMER1 overflows to count
												; before incrementing the 2.5 minutes count
#define TIME_INCREMENT_ON_REALITY_CHECK	.30		; 15 minutes

#ifdef TESTING
	#define INITIAL_MENU_SELECTION		.5
#else
	#define INITIAL_MENU_SELECTION		.0
#endif

; Comparator
#define	COMPARATOR_OFF_MODE			b'00000111'		; Comparator off to preserve power

; Calibration settings
#define CALIB_LOWER_BOUND			0x60
#define CALIB_UPPER_BOUND			0x80

; REM scan defines
#define NUM_REM_SCANS							.40		; Scan for REM activity 40 times
#define TIME_BETWEEN_AD_SAMPLES					.130	; Multiply by two to get actual time
#define MIN_AD_CHANGES_FOR_REM_DETECT			.4		; How many A/D output changes are needed for the
														; user to be considered in REM sleep
#define TIME_UNTIL_NEXT_SCAN					.2		; 1 minute till next REM scan
#define TIME_UNTIL_ALARM_SOUNDS					.10		; 5 minutes

; RS232 defines
#define RS232_BIT_LENGTH		.22		; Delay needed for a 9600 baud rate
										; To calculate for a difference rate, use the formula
										; delay = (baud^-1 * 1000^2 - 16) / 4

; Battery voltages
#define BATTERY_EXCELLENT		0xE5	; > 0xE5	--	> 8.98 V
#define BATTERY_GOOD			0xCC	; > 0xCC	--	> 8 V
#define BATTERY_NORMAL			0xB2	; > 0xB2	--	> 6.98 V
#define BATTERY_BAD				0x99	; > 0x99	--	> 6 V
#define BATTERY_INSUFFICIENT	0x40	; > 0x40	--	> 2.5 V
#define BATTERY_NOT_PRESENT		0x00	; > 0x00	--	> 0 V (running on pc power)

; Configuration addresses
#define ALARM_ACTIVE_ADDR		0x00
#define INITIAL_DELAY_ADDR		0x01
#define	USE_SOUND_SIGNALS_ADDR	0x02
#define NUMBER_OF_SIGNALS_ADDR	0x03
#define	SIGNAL_INTENSITY_ADDR	0x04
#define MENU_ENTRY6_ADDR		0x05
#define MENU_ENTRY7_ADDR		0x06
#define NUMBER_OF_MENU_ENTRIES	0x07
#define INITIAL_LOG_OFFSET		0x08

; Number of settings for each configurable setting
#define ALARM_NUM_OPTIONS				2		; 0, 1 -- off, on
#define	INITIAL_DELAY_NUM_OPTIONS		5		; 0, 1, 2, 3, 4 -- 15, 30, 45, 60, 75
#define USE_SOUND_SIGNALS_NUM_OPTIONS	3		; 0, 1, 2  -- off, only on first signal, on
#define NUMBER_OF_SIGNALS_NUM_OPTIONS	3		; 0, 1, 2 -- 3, 5, 10
#define SIGNAL_INTENSITY_NUM_OPTIONS	3		; 0, 1, 2 -- minimum, normal, maximum

; Log messages
#define MSG_PROGRAM_ACTIVATED		0xA1	; Following this message is a byte saying how much juice is left in battery
#define	MSG_REALITY_CHECKED			0xA2	; Instead of using this message, the amount of time till next scan is recorded in log
#define MSG_REM_DETECTED			0xA3
#define	MSG_CUES_GIVEN				0xA4
#define	MSG_ALARM_GIVEN				0xA5
#define MSG_REM_SCAN_STARTED		0xA6
#define MSG_PROGRAM_STOPPED			0xA7	; Following this message is a byte saying how much juice is left in battery
#define	MSG_300_SECS_DELAY_STARTED	0xA8
#define	MSG_PRE_ALARM_DELAY_STARTED	0xA9
#define	MSG_DELAY_OVER				0xAA
#define	MSG_NO_REM_DETECTED			0xAB
#define MSG_INSTANT_SCAN_REQUESTED	0xAC	; Following this message is the amount of time till next scan
#define	MSG_NUMBER_OF_CHANGES		0xB0	; the number of changes is added to this value then
											; logged (values in log can be between 0xB0 and 0xC4)
#define MSG_REPEAT_PREVIOUS			0xD0	; Means 'previous eeprom value should be repeated x times'
											; where x is (Value - 0xD0). Values of 0xD0 to 0xEE
											; are allowed.


;******************************************************************************
; General Purpose Registers (GPR's) 
;******************************************************************************

	cblock	0x20

	; Configuration variables
	; Those variables are placed at the beginning so that they can easily be
	; indirectly accessed from addres 0x20 onward.
	ALARM_ACTIVE
	INITIAL_DELAY
	USE_SOUND_SIGNALS
	NUMBER_OF_SIGNALS
	SIGNAL_INTENSITY
	MENU_ENTRY6
	MENU_ENTRY7

	; Interrupt Service Routine temporary registers
	STATUS_ISR	; contains content of status register when ISR was called
	W_ISR		; contains contetn of W register when ISR was called

	; Delay-making temporary registers
	DELAY_INNER	; inner-loop variable
	DELAY_OUTER	; outer-loop variable

	; Looping temp
	LOOP_TEMP
	LOOP_TEMP2

	; PWM generation variables
	PWM_DUTY
	PWM_LOOPS
	PWM_CURRENT_LOOP

	; EEPROM convenience registers
	EEPROM_DATA
	EEPROM_ADDR		; Only used to check if address is within range (0x00-0x7F)

	; EEPROM compression registers
	EEPROM_LAST_WAS_COMPRESSED		; 0 = last value was not compressed
	EEPROM_LAST_WRITTEN_VALUE		; Last non-compressed value that was written
	EEPROM_COMPRESSED_COUNT			; Number of times that the compressed value was saved

	; Offset of the current LOG entry
	LOG_OFFSET

	; Timer1 data
	TIMER1_MARKER			; 0 if this is the first TIMER1 overflow, 1 otherwize
	TIMER1_OVERFLOWS		; Counts backwards from TIMER1_OVERFLOWS_PER_150_SECS to 0
	TIME_SINCE_BOOT			; Number of 150-seconds units (2.5 minutes) elapsed since boot
	;TIME_150_SECS_ELAPSED	; Set to 1 when 2.5 minutes have passed (cleared by whatever function uses it)
	TIME_REMAINING			; Time remaining until a certain event occurs (e.g. REM scans to begin)

	; Buttons' state
	BUTTONS_STATE			; Contains status of the three buttons (uses same bits as from PORTA)

	; Menu state
	MENU_CURRENT_SELECTION

	; Program state
	HAS_BEEN_IN_ACTIVE_MODE		; Set to 0 on boot, set to 1 when Active Mode is entered (so that log is only wiped once)

	; REM scan registers
	AD_CHANGES
	CURRENT_AD_RESULT
	LAST_AD_RESULT
	NUM_REMAINING_REM_SCANS

	; RS232 registers
	RS232_CHECKSUM
	RS232_DELAYVAR
	
	; Misc temps	
	TEMP, TEMP2, TEMP3, CUES_TEMP, CUES_TEMP2

	endc

;******************************************************************************
; EEPROM initial values
;******************************************************************************
	; 1 - Alarm --  0, 1 -- off, on
	; 2 - Initial Delay --  0, 1, 2, 3, 4 -- 15, 30, 45, 60, 75
	; 3 - Use sounds signals -- 0, 1, 2 -- off, only on first signal, on
	; 4 - Number of signals -- 0, 1, 2 -- 3, 5, 10
	; 5 - Signal intensity -- 0, 1, 2 -- minimum, normal, maximum
	ORG 0x2100						; Initialize EEPROM Data 
#ifdef TESTING
	de .1, .0, .1, .1, .1			; Initialize variables default values
#else
	de .0, .4, .1, .1, .1			; Initialize variables default values
#endif
	de .0, .0, 0xFF					; Initialize 'menu' values and separator
	fill 0xEE, 0x2180-$				; Initialize rest of EEPROM to 'EE'


;******************************************************************************
; Reset Vector 
;******************************************************************************
	ORG     0x000		; processor reset vector
	nop					; required by in circuit debugger
	goto    Init		; go to beginning of program


;******************************************************************************
; Interrupt Vector  
;******************************************************************************
	ORG     0x004
	goto	Interrupt_Handler	; Set interupt vector


;****************************************************************************
; GetSettingMaximumValue(MENU_CURRENT_SELECTION)
;   Returns the maximum value of the setting referenced
;   by MENU_CURRENT_SELECTION. MENU_CURRENT_SELECTION *must* be bound between
;   the number of setting entries.
;   This sub is placed at the beginning of the program to avoid it being
;   split along more than 1 256 byte blocks.
;****************************************************************************
GetSettingMaximumValue:
	movfw	MENU_CURRENT_SELECTION
	addwf	PCL, F
	retlw	ALARM_NUM_OPTIONS
	retlw	INITIAL_DELAY_NUM_OPTIONS
	retlw	USE_SOUND_SIGNALS_NUM_OPTIONS
	retlw	NUMBER_OF_SIGNALS_NUM_OPTIONS
	retlw	SIGNAL_INTENSITY_NUM_OPTIONS


;******************************************************************************
; Initialization
;******************************************************************************
Init:

	BANK1
	movlw	b'1001111'	; Sets prescaler assignment to watchdog timer,
	movwf	OPTION_REG	; sets prescaler to 111 (1:128)
						; and disable individual pull-ups on PORTA

	clrf	ANSEL				; Set all PORTA & PORTC pins as digital I/O

	movlw	0xFF
	movwf 	TRISA					; Set all PORTA pins as digital inputs
	bcf		TRISA, RIGHT_LED_BIT	; Set right led pin as output	

	clrf 	TRISC					; Set all PORTC pins as digital outputs
	bsf		TRISC, RS232_TX			; Set RS232 TX pin as input to prevent MAX232 from draining power through it
	bsf		TRISC, RS232_RX			; Set RS232 RX pin as input
	

	; Enable interrupt-on-change on button pins
	bsf		IOCA, ACTIVATE_BTN_BIT
	bsf		IOCA, SELECT_BTN_BIT
	bsf		IOCA, ENTER_BTN_BIT

	; Calibrate clock using OSCCAL value
	BANK1
	call	03FFh
	movwf	OSCCAL

	BANK0
	; Setup comparator mode
	movlw 	COMPARATOR_OFF_MODE		
	movwf	CMCON

	; Setup A/D mode
	movlw	AD_INACTIVE_MODE		; Setup voltage reference, channel, etc
	movwf	ADCON0
	movlw	b'01010000'				; Setup A/D clock
	BANK1
	movwf	ADCON1
	movlw	AD_PORT_SELECTION		; Setup ports to be used by A/D
	movfw	ANSEL
	BANK0

	; Setup TIMER1
	movlw	b'00110101'			; Use prescaler of 8 -- this will give a period to overflow
	movwf	T1CON				; of 0.524288 seconds
	clrf	TMR1L				; Clear lower timer byte value
	clrf	TMR1H				; Clear upper timer byte value
	BANK1
	bsf		PIE1, TMR1IE		; Enable interrupt on overflow
	

	BANK0

	; Initialize variables
	movlw	INITIAL_LOG_OFFSET
	movwf	LOG_OFFSET
	clrf	HAS_BEEN_IN_ACTIVE_MODE

	;movlw	TIMER1_OVERFLOWS_PER_150_SECS
	movlw	.3
	movwf	TIMER1_OVERFLOWS

	clrf	TIME_SINCE_BOOT
	call	LoadSettings
	clrf	BUTTONS_STATE

	; Enable general and peripherial interrupts
	bsf		INTCON, GIE
	bsf		INTCON, PEIE




; Initialisation complete -- check if we should go in RS232 mode
	btfss	PORTA, ACTIVATE_BTN_BIT
	goto	RS232_Mode

; Flash boot sequence
	call	Flash_Both_Leds
	call	Flash_Both_Leds

; Flash how much juice is left in battery
	call	Battery_Test
	call	Flash_Battery_Amount






MainLoop:

	; Wait for button press to figure what to do
MainLoop_Button_Wait:
	clrwdt
	movf	BUTTONS_STATE, F
	btfsc	STATUS, Z
	goto	MainLoop_Button_Wait

	; If we got an 'Activate' button press, skip the config and go directly to activation
	btfsc	BUTTONS_STATE, ACTIVATE_BTN_BIT
	goto	MainLoop_Activate

MainLoop_Config:
	clrf	BUTTONS_STATE
	; Go to config mode
	call	Flash_Both_Leds
	call	MainMenu

MainLoop_Activate:
	clrf	BUTTONS_STATE
	; Go to active mode
	call	Flash_Both_Leds
	call	Flash_Both_Leds
	call	ActiveMode

	; Exited from Active mode -- go back to config mode.
	goto	MainLoop_Config


;****************************************************************************
; MainMenu()
;   Handles selections and option changes.
;   Returns when the 'Activate' button is pressed.
;****************************************************************************
MainMenu:
	movlw	INITIAL_MENU_SELECTION
	movwf	MENU_CURRENT_SELECTION

MainMenu_Loop:
	; Flash current selection entry on left led
	incf	MENU_CURRENT_SELECTION, W	; Since MENU_CURRENT_SELECTION is 0-based
										; and we want at least one flash, increment
	movwf	TEMP						; it before putting it in temp register
MainMenu_SelFlashingLoop:
	call	Flash_Left_Led
	decfsz	TEMP, F
	goto	MainMenu_SelFlashingLoop

MainMenu_ValFlashing
	; Flash current selection value on right led
	movlw	0x20						; Load address of first 'value' register
	addwf	MENU_CURRENT_SELECTION, W	; Offset by selection #
	movwf	FSR							; Load value in indirect address register
	incf	INDF, W						; Read register value, increment it..
	movwf	TEMP						; .. and store it in temp register 
MainMenu_ValFlashingLoop:
	call	Flash_Right_Led
	decfsz	TEMP, F
	goto	MainMenu_ValFlashingLoop


	; Wait until we get a button press
MainMenu_Button_Wait:
	clrwdt
	movf	BUTTONS_STATE, F
	btfsc	STATUS, Z
	goto	MainMenu_Button_Wait

	; Check if we received an 'Activate' signal
	btfss	BUTTONS_STATE, ACTIVATE_BTN_BIT
	goto	MainMenu_Button_Check1
	clrf	BUTTONS_STATE
	; Since we did receive an 'Activate' signal, return from this subroutine.
	return


MainMenu_Button_Check1:
	; Check if we received a 'Select' signal
	btfss	BUTTONS_STATE, SELECT_BTN_BIT
	goto	MainMenu_Button_Check2
	clrf	BUTTONS_STATE

	incf	MENU_CURRENT_SELECTION, F	; Increment the current selection
	movfw	MENU_CURRENT_SELECTION
	sublw	NUMBER_OF_MENU_ENTRIES		; Check if we should loop back to selection #1
	btfsc	STATUS, Z
	clrf	MENU_CURRENT_SELECTION

	goto 	MainMenu_Loop				; Show selection entry and value again


MainMenu_Button_Check2:
	; Check if we received an 'Enter' signal
	btfss	BUTTONS_STATE, ENTER_BTN_BIT
	goto	MainMenu_Button_Check3
	clrf	BUTTONS_STATE

	; Check if current menu entry is one of the non-config entries
	movfw	MENU_CURRENT_SELECTION
	sublw	MENU_ENTRY6_ADDR
	btfsc	STATUS, Z
	goto	MainMenu_Entry6_Activated
	movfw	MENU_CURRENT_SELECTION
	sublw	MENU_ENTRY7_ADDR
	btfsc	STATUS, Z
	goto	MainMenu_Entry7_Activated

	; Read current setting from memory using indirect addressing
	movlw	0x20						; Load address of first 'value' register
	addwf	MENU_CURRENT_SELECTION, W	; Offset by selection #
	movwf	FSR							; Load value in indirect address register
	movfw	INDF						; Read register value
	movwf	TEMP						; Place value in TEMP register

	incf	TEMP, F						; Increment value
	call	GetSettingMaximumValue		; Get the maximum value for the current
										; setting and place it in W	
	subwf	TEMP, W						; Check if new value exceeds maximum
	btfsc	STATUS, Z					; value...
	clrf	TEMP						; If so, set the new value to 0.

	; Write current setting to memory using indirect addressing
	; FSR should still contain same address as it did a few lines ago
	movlw	0x20						; Load address of first 'value' register
	addwf	MENU_CURRENT_SELECTION, W	; Offset by selection #
	movwf	FSR							; Load value in indirect address register
	movfw	TEMP
	movwf	INDF

	; Save new settings to EEPROM
	call	SaveSettings

	; Flash new value
	goto	MainMenu_ValFlashing


MainMenu_Button_Check3:
	; 'k.. so what button was pressed?? o_O
	clrf	BUTTONS_STATE
	goto	MainMenu_Button_Wait


MainMenu_Entry6_Activated:
	; Led Calibration Mode
	call	Calibrate_Mode
	goto	MainMenu_Loop


MainMenu_Entry7_Activated:
	; Display number of given REM signals
	; TO BE IMPLEMENTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	;call	CountAndShowRemSignals
	goto	MainMenu_Button_Wait


;****************************************************************************
; Calibrate_Mode()
;   Calibrates the power output to the IR led. Flashes the left led
;   if the calibration is correct, flashes the right led if it is not.
;****************************************************************************
Calibrate_Mode:

	; Turn on A/D
	movlw	AD_REM_SCAN_MODE
	movwf	ADCON0

	; Output maximum power to IR led
	bsf		PORTC, IR_LED_BIT	

Calibrate_Loop:

	; Init A/D conversion
	bsf		ADCON0, GO
Calibrate_ADLoop:
	clrwdt
	btfsc	ADCON0, GO
	goto	Calibrate_ADLoop

	; Store result in register
#ifndef USE_PRECISE_AD
	movfw	ADRESH
#else
	BANK1
	movfw	ADRESL
	BANK0
#endif
	movwf	CURRENT_AD_RESULT

#ifdef TESTING
	movfw	CURRENT_AD_RESULT
	call	LogNoSeparator
#endif


;	; Check if result is smaller than 0x80
;	movfw	CURRENT_AD_RESULT
;	sublw	0x80
;	btfsc	STATUS, C
;	goto	Calibrate_result_out_of_range
;	call	Flash_Left_Led
;	goto	Calibrate_Cont
;


	; Check if result is within range
	movfw	CURRENT_AD_RESULT
	sublw	CALIB_LOWER_BOUND
	btfsc	STATUS, C		; LOWER - W -> W ; If C is 0, then we got a negative number.. in range
	goto	Calibrate_result_out_of_range

	movfw	CURRENT_AD_RESULT
	sublw	CALIB_UPPER_BOUND
	btfss	STATUS, C		; UPPER - W -> W ; If C is 1, then we got a positive number.. in range
	goto	Calibrate_result_out_of_range

	call	Flash_Left_Led
	goto	Calibrate_Cont

Calibrate_result_out_of_range:
	call	Flash_Right_Led

Calibrate_Cont:
	clrw
	call	Delay

	; End calibration on button press
	movf	BUTTONS_STATE, F
	btfss	STATUS, Z
	goto	Calibrata_Complete

	goto	Calibrate_Loop


Calibrata_Complete:

	; Calibration complete, disable IR led and A/D
	bcf		PORTC, IR_LED_BIT
	movlw	AD_INACTIVE_MODE
	movwf	ADCON0

	; Clear buttons state
	clrf	BUTTONS_STATE

	return




;****************************************************************************
; ActiveMode()
;   Main program loop. Handles REM scans and signals and such.
;****************************************************************************
ActiveMode:
	; Check if this is the first time we go in active mode
	btfsc	HAS_BEEN_IN_ACTIVE_MODE, 0
	goto	ActiveMode_Init

	; If not...
	bsf		HAS_BEEN_IN_ACTIVE_MODE, 0

	; Wipe the log
	call	WipeLog

ActiveMode_Init:
	; Log that the program has been started
	movlw	MSG_PROGRAM_ACTIVATED
	call	LogNoSeparator
	
	; Check and log battery power
	call	Battery_Test
	call	LogNoSeparator

	; Initialize 'time till scan' register
	call	GetInitialTimeTillScan
	movwf	TIME_REMAINING

ActiveMode_Wait_Till_Scan:
	clrwdt

	; Check if it's time for a scan
	movf	TIME_REMAINING, F
	btfsc	STATUS, Z
	goto	ActiveMode_DoScan

	; Check if we got a key press
	movf	BUTTONS_STATE, F
	btfss	STATUS, Z
	goto	ActiveMode_ReceivedKeyPress
	
	goto	ActiveMode_Wait_Till_Scan


ActiveMode_ReceivedKeyPress:
	; Check if we got an 'Activate' + 'Enter' combo
	movfw	BUTTONS_STATE
	xorlw	INSTANT_SCAN_MASK
	btfss	STATUS, Z
	goto	ActiveMode_ReceivedKeyPress1

	; Combo detected -- we got an instant scan request!
	clrf	BUTTONS_STATE

	; Log that we're going to do an instant scan
	movlw	MSG_INSTANT_SCAN_REQUESTED
	call	LogNoSeparator
	movfw	TIME_REMAINING
	call	LogNoSeparator

	; Wait a bit before doing scan
	clrw
	call	Delay
	call	Delay
	call	Delay
	goto	ActiveMode_DoScan
	

ActiveMode_ReceivedKeyPress1:
	; Check if we got 'Activate' keypress
	btfss	BUTTONS_STATE, ACTIVATE_BTN_BIT
	goto	ActiveMode_ReceivedKeyPress2
	clrf	BUTTONS_STATE

	; We got an 'Activate' button press!
	call	Perform_Reality_Test

	; Incrementation of the TIME_REMAINING is done in the reality test subroutine.
	goto	ActiveMode_Wait_Till_Scan

ActiveMode_ReceivedKeyPress2:
	; Since we got a button press and it wasn't 'Activate', it means it was one of
	; the other two buttons. Abort and go to config mode.

	; Log that program execution was stopped
	movlw	MSG_PROGRAM_STOPPED
	call	LogNoSeparator

	; Check and log battery power
	call	Battery_Test
	call	Log

	return


ActiveMode_DoScan:
	; W00t, time for a REM scan!
	call	Perform_REM_Scan

	; Check if REM was detected
	addlw	.0
	btfss	STATUS, Z
	goto	Scan_REM_Detected

Scan_No_REM_Detected:
	; Set time till next REM scan
	movlw	TIME_UNTIL_NEXT_SCAN
	movwf	TIME_REMAINING
	
	goto	ActiveMode_Wait_Till_Scan


Scan_REM_Detected:
#ifdef TESTING
	; Log that REM was detected
	movlw	MSG_REM_DETECTED
	call	LogNoSeparator
#endif

	; Give dream cues to user
	call	GiveCues

	; If we got a button press, branch to the code that handles that stuff
	movf	BUTTONS_STATE, F
	btfss	STATUS, Z
	goto	ActiveMode_ReceivedKeyPress

	; Is alarm active?
	btfss	ALARM_ACTIVE, 0
	goto	Scan_Alarm_Not_Active

	call	WakeUpUser
	clrw
	call	Delay
	call	Delay
	call	Delay
	call	Perform_Reality_Test
	clrf	BUTTONS_STATE
	
	goto	ActiveMode_Wait_Till_Scan


Scan_Alarm_Not_Active:

	; Set time till next REM scan
	movlw	TIME_UNTIL_NEXT_SCAN
	movwf	TIME_REMAINING

	goto	ActiveMode_Wait_Till_Scan


Scan_Abort_Scan:
	; We received a button press while scanning -- abort scanning.
	
	bcf		PORTC, IR_LED_BIT	; Disable IR led
	movlw	AD_INACTIVE_MODE	; Disable A/D
	movwf	ADCON0				;

	goto	ActiveMode_ReceivedKeyPress


;****************************************************************************
; Perform_REM_Scan()
;   Performs a REM scan. If REM is detected, returns 1 in W. Otherwize, 0
;   is returned.
;****************************************************************************
Perform_REM_Scan:
#ifdef TESTING
	call	Flash_Left_Led
#endif

	; Turn on A/D
	movlw	AD_REM_SCAN_MODE
	movwf	ADCON0

	; Output maximum power to IR led
	bsf		PORTC, IR_LED_BIT	

	; Init temp variable
	clrf	AD_CHANGES					; Number of A/D changes
	clrf	LAST_AD_RESULT				; Last A/D result
	movlw	NUM_REM_SCANS
	movwf	NUM_REMAINING_REM_SCANS		; Number of scans to perform

	; Wait a bit for A/D capacitor to fill up
	movlw	TIME_BETWEEN_AD_SAMPLES
	call	Delay
	call	Delay

	; Get initial A/D result
	bsf		ADCON0, GO
Scan_Init_ADLoop:
	clrwdt
	btfsc	ADCON0, GO
	goto	Scan_Init_ADLoop
#ifndef USE_PRECISE_AD
	movfw	ADRESH
#else
	BANK1
	movfw	ADRESL
	BANK0
#endif
	movwf	LAST_AD_RESULT

Scan_Loop:
	; Check if we got a button press
	movf	BUTTONS_STATE, F
	btfss	STATUS, Z
	goto	Scan_Abort_Scan

	; Disable IR led to save power while waiting
	;bcf		PORTC, IR_LED_BIT

	; Wait until we should scan again
	movlw	TIME_BETWEEN_AD_SAMPLES
	call	Delay
	
	; Enable IR led
	;bsf		PORTC, IR_LED_BIT
	
	; Wait a bit longer...
	call	Delay

	; Perform A/D acquisition
	bsf		ADCON0, GO
Scan_ADLoop:
	clrwdt
	btfsc	ADCON0, GO
	goto	Scan_ADLoop

#ifndef USE_PRECISE_AD
	movfw	ADRESH
#else
	BANK1
	movfw	ADRESL
	BANK0
#endif
	movwf	CURRENT_AD_RESULT

#ifdef TESTING
 #ifdef DETAILLED_SCAN_LOGGING
	; Log A/D result (for debugging)
	movfw	CURRENT_AD_RESULT
	call	LogNoSeparator
 #endif
#endif

	; Check if result is outside of jitter tolerance
	clrf	TEMP				; Set to 0 for later

	movfw	CURRENT_AD_RESULT
	subwf	LAST_AD_RESULT, W	; LAST_AD_RESULT - ADRESH -> W

#ifndef USE_PRECISE_AD	
	btfss	STATUS, C			; Check if result is negative
	subwf	TEMP, W				; If so, do 0 - W -> F to get a positive number
#else
	; Complement result if necessary (when using precise A/D, 0xFF and 0x01 can actually be
	; only 0x02 apart.. so doing 0xFF - 0x01 gives a result that needs to be complemented)
	movwf	TEMP
	btfsc	TEMP, 7
	sublw	0x00
#endif
	

#ifdef TESTING
 #ifdef DETAILLED_SCAN_LOGGING
	; Log difference (for debugging)
	movwf	TEMP
	call	LogNoSeparator
	movfw	TEMP
 #endif
#endif

	; Increment amount of changes if required
	sublw	AD_JITTER_THRESHOLD		; MAX_JITTER - W -> W
	btfss	STATUS, C				; Check if result is negative...
	incf	AD_CHANGES, F			; If so, increment number of changes.
	
	; Store new A/D result
	movfw	CURRENT_AD_RESULT
	movwf	LAST_AD_RESULT



Scan_AD_Cont:
	decfsz	NUM_REMAINING_REM_SCANS, F
	goto	Scan_Loop

	; Scanning complete, disable IR led and A/D
	bcf		PORTC, IR_LED_BIT
	movlw	AD_INACTIVE_MODE
	movwf	ADCON0

	; Log the number of A/D changes
	movlw	MSG_NUMBER_OF_CHANGES
	addwf	AD_CHANGES, W
	;call	TimedLog
	call	LogNoSeparator

#ifdef TESTING
	; Scan is complete -- flash led once
	call	Flash_Left_Led
#endif

	; Check if the number of changes exceeds the threshold
	movlw	MIN_AD_CHANGES_FOR_REM_DETECT
	subwf	AD_CHANGES, W
	btfsc	STATUS, C
	retlw	.1	; REM Detected!
	retlw	.0	; No REM detected.


;****************************************************************************
; GetInitialTimeTillScan()
;   Returns, in multiple of 150-seconds, the initial time till scan
;   based on the data in the INITIAL_DELAY variable. The value is returned
;   in the W register.
;****************************************************************************
GetInitialTimeTillScan:
	; INITIAL_DELAY is 0-based and encodes values in multiple of 15 minutes.
	; Since we want something in multiples of 2.5 minutes, we need to
	; multiply that value by 6 and add 6 to compensate for the 0-based
	; number.
	;movfw	INITIAL_DELAY
	;addwf	INITIAL_DELAY, W
	;addwf	INITIAL_DELAY, W
	;addwf	INITIAL_DELAY, W
	;addwf	INITIAL_DELAY, W
	;addwf	INITIAL_DELAY, W
	;addlw	.6

	; INITIAL_DELAY is 0-based and encodes values in multiple of 15 minutes.
	; Since we want something in multiples of 0.5 minutes, we need to
	; multiply that value by 30 and add 30 to compensate for the 0-based
	; number.
	movfw	INITIAL_DELAY
	movwf	TEMP			; x = INITIAL_DELAY
	swapf	TEMP, F			; Swap upper and lower 4 bits (giving x*16)
	bcf		STATUS, C		; To make sure carry = 0
	rlf		TEMP, F			; Shift left once (giving x*32)
	subwf	TEMP, F			; This gives x*31
	subwf	TEMP, F			; This gives x*30
	movlw	.30				; Compensation for 0-based number
	addwf	TEMP, W
	
	return


;****************************************************************************
; Perform_Reality_Test()
;   This subroutine performs a reality test on the user. The
;   TIME_REMAINING variable is then incremented 
;****************************************************************************
Perform_Reality_Test:
	call	Flash_Both_Leds
	call	Flash_Right_Led	
	call	Flash_Left_Led	
	call	Flash_Left_Led	
	call	Flash_Right_Led	
	call	Flash_Both_Leds
	call	Flash_Both_Leds

	; Increment time remaining till next scan
	movlw	TIME_INCREMENT_ON_REALITY_CHECK
	addwf	TIME_REMAINING, F

	; Check if time remaining is too high -- if so, bring it down to initial delay
	call	GetInitialTimeTillScan
	subwf	TIME_REMAINING, W		; Do (f) - (W) -> (W)
	btfss	STATUS, C				; Check if result is positive -- that means that
									; current remaining time is too high
	goto	Perform_Reality_Test_Continue

	call	GetInitialTimeTillScan
	movwf	TIME_REMAINING

Perform_Reality_Test_Continue:	
	; Log that a reality test was performed
	;movlw	MSG_REALITY_CHECKED
	;call	TimedLog
	movfw	TIME_REMAINING
	call	LogNoSeparator

	return


;****************************************************************************
; GiveCues()
;   This function gives dream cues to the user. It uses the
;   USE_SOUND_SIGNALS, NUMBER_OF_SIGNALS and SIGNAL_INTENSITY configuration
;   variables.
;****************************************************************************
GiveCues:
	; Log that cues were given
	;movlw	MSG_CUES_GIVEN
	;call	LogNoSeparator

	; Figure how many cues should be given
	movlw	.3			; default is 3 cues

	; 1 in NUMBER_OF_SIGNALS = 5 cues
	btfsc	NUMBER_OF_SIGNALS, 0
	movlw	.5

	; 2 in NUMBER_OF_SIGNALS = 10 cues
	btfsc	NUMBER_OF_SIGNALS, 1
	movlw	.10

	movwf	CUES_TEMP
	clrf	CUES_TEMP2

GiveCues_Loop:
	; Abort cues if we get a button press
	movf	BUTTONS_STATE, F
	btfss	STATUS, Z
	goto	GiveCues_Completed

	; Flash leds for ~300ms using selected intensity
	call	Flash_Both_Leds_With_Intensity

	; Check if we should use sound signals
	movf	USE_SOUND_SIGNALS, F
	btfsc	STATUS, Z
	goto	GiveCues_NoSound

	; Check if we should only give sound signal on first cue
	btfss	USE_SOUND_SIGNALS, 0
	goto	GiveCues_Sound
	
	; If we only give sound signal on first cue, check if this is the first cue
	btfsc	CUES_TEMP2, 0
	goto	GiveCues_NoSound

GiveCues_Sound:
	; Activate buzzer for ~300 ms -- not sure how to set intensity with this one yet
	call	LongBuzzer
	goto	GiveCues_Cont

GiveCues_NoSound:
	movlw	.195
	call	Delay
	call	Delay

GiveCues_Cont:
	bsf		CUES_TEMP2, 0	; Signals that we have given at least one cue

	; Check if we should loop
	decfsz	CUES_TEMP, F
	goto	GiveCues_Loop


GiveCues_Completed:
	return


;****************************************************************************
; WakeUpUser()
;   This function wakes up the user. It simply flashes both LEDs and emits
;   a loud noise on the buzzer until the user pressed a button.
;   Note that the function waits for a while before waking up the
;   user (wait time is defined by TIME_UNTIL_ALARM_SOUNDS). If the user
;   presses a button before the alarm is activated, the alarm is aborted.
;****************************************************************************
WakeUpUser:
#ifdef TESTING
	; Log that we're waiting 5 minutes
	;movlw	MSG_PRE_ALARM_DELAY_STARTED
	;call	TimedLog
#endif

	movlw	TIME_UNTIL_ALARM_SOUNDS
	movwf	TIME_REMAINING
WakeUpUser_WaitLoop:
	clrwdt
	
	; Check if we got a button press
	movf	BUTTONS_STATE, F
	btfss	STATUS, Z
	goto	WakeUpUser_Completed

	; Check if required delay has elapsed
	movf	TIME_REMAINING, F
	btfss	STATUS, Z
	goto	WakeUpUser_WaitLoop


	; Log that the alarm was given
	movlw	MSG_ALARM_GIVEN
	call	LogNoSeparator

WakeUpUser_Loop:
	call	LongBuzzer
	bsf		PORTC, LEFT_LED_BIT
	bsf		PORTA, RIGHT_LED_BIT
	call	LongBuzzer
	bcf		PORTC, LEFT_LED_BIT
	bcf		PORTA, RIGHT_LED_BIT

	movf	BUTTONS_STATE, F
	btfsc	STATUS, Z
	goto	WakeUpUser_Loop

WakeUpUser_Completed:
	clrf	BUTTONS_STATE
	return


;****************************************************************************
; RS232_Mode()
;   In this mode, the microcontroller enables the MAX232 chip and awaits
;   the reception of something on the RX line. Once something is received,
;   the content of the EEPROM as well as some other variables is dumped onto
;   the TX line at a rate of 9600 baud.
;****************************************************************************
RS232_Mode:
	; Enable MAX232 chip
	bsf		PORTC, SWITCHED_GROUND_ENABLE_BIT

	; Set TX pin as ouput (that pin is usually kept at high-impedence to
	; avoid power being drained from it when MAX232 is off)
	BANK1
	bcf		TRISC, RS232_TX
	BANK0

	; Set initial TX state
	bsf		PORTC, RS232_TX

	; Flash that we're now in MAX232 mode.
	call	Flash_Both_Leds
	call	Flash_Both_Leds
	call	Flash_Both_Leds

RS232_WaitLoop:
	clrwdt
	btfsc	PORTC, RS232_RX
	goto	RS232_WaitLoop	

	bcf		INTCON, GIE		; Disable interrupts

	; Wait for the activate byte to be done sending
	movlw	.3
	call	Delay

	; Flash right led to show that TX has begun
	call	Fast_Flash_Right_Led

	; Clear checksum
	clrf	RS232_CHECKSUM	

	; Send header (0xAA, 0xCD)
	movlw	0xAA
	call	RS232_SendByte
	movlw	0xCD
	call	RS232_SendByte

	; Dump content of EEPROM
	clrf	TEMP
RS232_EEPROMLoop:
	movfw	TEMP
	call	EEPROM_Read
	call	RS232_SendByte
	incf	TEMP, F
	btfss	TEMP, 7			; We only send 128 bytes, so we stop when 8th bit of TEMP is 1
	goto	RS232_EEPROMLoop

	; Send other important variables
	movlw	AD_REM_SCAN_MODE
	call	RS232_SendByte
	movlw	AD_JITTER_THRESHOLD
	call	RS232_SendByte
	movlw	NUM_REM_SCANS
	call	RS232_SendByte
	movlw	TIME_BETWEEN_AD_SAMPLES
	call	RS232_SendByte
	movlw	MIN_AD_CHANGES_FOR_REM_DETECT
	call	RS232_SendByte
	movlw	TIME_UNTIL_NEXT_SCAN
	call	RS232_SendByte
	movlw	TIME_UNTIL_ALARM_SOUNDS
	call	RS232_SendByte
	movlw	TIME_INCREMENT_ON_REALITY_CHECK
	call	RS232_SendByte

	; Send Footer (0xCD, 0xAA)
	movlw	0xCD
	call	RS232_SendByte
	movlw	0xAA
	call	RS232_SendByte

	; Send checksum, followed by checksummed checksum
	movfw	RS232_CHECKSUM
	call	RS232_SendByte
	movfw	RS232_CHECKSUM
	call	RS232_SendByte
	

	; Flash left led to show that TX has ended
	call	Fast_Flash_Left_Led
	bcf		INTCON, GIE		; Enable interrupts

	goto	RS232_WaitLoop

	

; Misc RS232 functions:

RS232_SendByte: ; Sends a byte on the TX pin. Updates the checksum.
	; Save byte
	movwf	TEMP2

	; Update checksum
	xorwf	RS232_CHECKSUM, F
	
	; Set initial values
	movlw	.8
	movwf	TEMP3

	; Send start bit
	bcf		PORTC, RS232_TX		; 1 us
	call	RS232_BitDelay		; 94 us
	nop							; 1 us
	nop							; 1 us
	nop							; 1 us
	nop							; 1 us
	nop							; 1 us
	
RS232_SendByteLoop:
	; Loop to send the 8 data bits, starting with the least significative
	rrf		TEMP2, F			; 1 us
	
	; 3 us pass before bsf/bcf is executed

	; Check if we should send a one or a zero
	btfsc	STATUS, C			; 1-2 us
	goto	RS232_SendAOne		; 2us

RS232_SendAZero:
	nop							; 1 us
	bcf		PORTC, RS232_TX		; 1 us
	goto	RS232_SendByteCont	; 2 us

RS232_SendAOne:
	bsf		PORTC, RS232_TX		; 1 us
	nop							; 1 us
	nop							; 1 us
RS232_SendByteCont:
	; 3 us pass before this (including bsf/bcf instruction)
	call	RS232_BitDelay		; 94 us

	; Check if all bytes have been sent
	decfsz	TEMP3, F			; 1 us (2 if last bit)
	goto	RS232_SendByteLoop	; 2 us

	; Send stop bit
	nop							; 1 us
	nop							; 1 us
	nop							; 1 us
	nop							; 1 us
	nop							; 1 us
	bsf		PORTC, RS232_TX		; 1 us
	call	RS232_BitDelay		; 96 us
	call	RS232_BitDelay		; 96 us

	return
	

RS232_BitDelay:	; Wait 94us (the other 10 us required are used by program instructions)
								; 2 us for the Call instruction
	movlw	RS232_BIT_LENGTH	; 1 us
	movwf	RS232_DELAYVAR		; 1 us
RS232_BitDelayLoop:
	clrwdt						; 1 us
	decfsz	RS232_DELAYVAR, F	; 1 us
	goto	RS232_BitDelayLoop	; 2 us
	nop							; 1 us
	return						; 2 us

;****************************************************************************
; Battery_Test()
;   Performs a test of the supply battery. Returns the voltage (divided
;   by two) in W.
;****************************************************************************
Battery_Test:
	; Turn on probe transistor
	bsf		PORTC, BATTERY_TEST_ENABLE_BIT

	; Enable A/D
	movlw	AD_BATTERY_PROBE_MODE
	movwf	ADCON0

	; Wait a bit for A/D capacitor to fill up
	movlw	.1
	call	Delay

	; Start A/D conversion
	bsf		ADCON0, GO
Battery_Test_Loop:
	clrwdt
	btfsc	ADCON0, GO
	goto	Battery_Test_Loop

	; Turn off A/D and probe transistor
	bcf		PORTC, BATTERY_TEST_ENABLE_BIT
	movlw	AD_INACTIVE_MODE
	movwf	ADCON0

	; Move result in W
	movfw	ADRESH
	return


;****************************************************************************
; Flash_Battery_Amount(W)
;   Flashes on the leds how much power is left in the battery.
;   3 flash of left led = excellent
;   2 flash of left led = good
;   1 flash of left led = normal
;   1 flash of right led = bad
;   2 flash of right led = insufficient
;   3 flash of right led = battery not present (running on pc power)
;****************************************************************************
Flash_Battery_Amount:
	; Save W
	movwf	TEMP

	; Check for 'excellent' battery power
	movlw	BATTERY_EXCELLENT
	subwf	TEMP, W
	btfss	STATUS, C
	goto	Flash_Battery_Amount_Cont1
	call	Fast_Flash_Left_Led
	call	Fast_Flash_Left_Led
	call	Fast_Flash_Left_Led
	return

Flash_Battery_Amount_Cont1:
	; Check for 'good' battery power
	movlw	BATTERY_GOOD
	subwf	TEMP, W
	btfss	STATUS, C
	goto	Flash_Battery_Amount_Cont2
	call	Fast_Flash_Left_Led
	call	Fast_Flash_Left_Led		
	return

Flash_Battery_Amount_Cont2:
	; Check for 'normal' battery power
	movlw	BATTERY_NORMAL
	subwf	TEMP, W
	btfss	STATUS, C
	goto	Flash_Battery_Amount_Cont3
	call	Fast_Flash_Left_Led
	return

Flash_Battery_Amount_Cont3:
	; Check for 'bad' battery power
	movlw	BATTERY_BAD
	subwf	TEMP, W
	btfss	STATUS, C
	goto	Flash_Battery_Amount_Cont4
	call	Fast_Flash_Right_Led
	return

Flash_Battery_Amount_Cont4:
	; Check for 'insufficient' battery power
	movlw	BATTERY_INSUFFICIENT
	subwf	TEMP, W
	btfss	STATUS, C
	goto	Flash_Battery_Amount_Cont5
	call	Fast_Flash_Right_Led
	call	Fast_Flash_Right_Led
	return

Flash_Battery_Amount_Cont5:
	; If all previous tests failed, then the battery isn't even there!
	call	Fast_Flash_Right_Led
	call	Fast_Flash_Right_Led
	call	Fast_Flash_Right_Led	
	return


;****************************************************************************
; PWM(W) - .1 = 0.4% duty cycle, .255 = 99.6% duty cycle, .0 = 100% duty cycle
;   Outputs on PORTC's IR_LED_BIT pin.
;   Outputs for PWM_PULSE_DURATION periods.
;	Requires GPGs: PWM_DUTY, PWM_LOOPS & PWM_CURRENT_LOOP
;****************************************************************************
PWM:
	movwf	PWM_DUTY			; Contains duration of duty cycle
	movlw	PWM_PULSE_DURATION
	movwf	PWM_LOOPS			; Contains number of loops to be done
	movfw	PWM_DUTY			; Keep duration of duty in W (to be copied back into PWM_DUTY each loop)

PWM_Loop
	movwf	PWM_DUTY			; Put back duration of duty cycle in register

	;clrf	PWM_CURRENT_LOOP	; Put remaining loops = 256
	movwf	TEMP2
	movlw	.16
	movwf	PWM_CURRENT_LOOP
	movfw	TEMP2

	bsf		PORTC, IR_LED_BIT	; Ouputs the high part
PWM2_Loop
	decfsz	PWM_DUTY, F
	Goto	PWM_Continue
	bcf		PORTC, IR_LED_BIT	; Outputs the low part
PWM_Continue
	decfsz	PWM_CURRENT_LOOP, F
	goto	PWM2_Loop
	clrwdt
	decfsz	PWM_LOOPS, F
	goto	PWM_Loop

	return;


;****************************************************************************
; Delay(W) - DelayTime = [(1)+(2)+(2)+(W*768-W)+W+(W*3-1)+(2)]*(OSC/4)cycles
;           (This includes the call & The movlw)
;          - Max Time When W=0xFF, [ 196356 Cycles * (OSC/4) ]
;		   - Max Time When W=0x00, [ 197382 Cycles * (OSC/4) ]
;          - Must Declare INNER & OUTER AS GPR'S
;****************************************************************************
; Values for a 4MHz oscillator:
;	W = 1:		Time = 777 us		Frequency = 1287 Hz
;	W = 2:		Time = 1.548 ms		Frequency = 645.99 Hz
;	W = 7:		Time = 5.403 ms		Frequency = 185.08 Hz
;	W = 13:		Time = 10.029 ms	Frequency = 99.71 Hz
;	W = 20:		Time = 15.426 ms	Frequency = 64.83 Hz
;	W = 26:		Time = 20.052 ms	Frequency = 49.87 Hz
;	W = 33:		Time = 25.449 ms	Frequency = 39.29 Hz
;	W = 39:		Time = 30.075 ms	Frequency = 33.25 Hz
;	W = 52:		Time = 40.098 ms	Frequency = 24.94 Hz
;	W = 65:		Time = 50.121 ms	Frequency = 19.95 Hz
;	W = 97:		Time = 74.793 ms	Frequency = 13.37 Hz
;	W = 130:	Time = 100.236 ms	Frequency = 9.98 Hz
;	W = 162:	Time = 124.908 ms	Frequency = 8.01 Hz
;	W = 195:	Time = 150.351 ms	Frequency = 6.65 Hz
;	W = 227:	Time = 175.023 ms	Frequency = 5.71 Hz
;	W = 0:		Time = 197.382 ms	Frequency = 5.07 Hz
;****************************************************************************
Delay
	movwf	DELAY_OUTER
	clrf	DELAY_INNER
Delay_Loop:
	decfsz	DELAY_INNER, f
	goto	Delay_Loop
	clrwdt
	decfsz	DELAY_OUTER, f
	goto	Delay_Loop
	return


;****************************************************************************
; ShortDelay(W)
;   Delays time = [2 + 1 + (1 + 2) * W - 1 + 1] * (OSC/4)cycles
;****************************************************************************
; Values for a 4MHz oscillator:
;	W = 1:		Time = 6 us		Frequency = 166666 Hz
;	W = 2:		Time = 9 us		Frequency = 111111 Hz
;	W = 4:		Time = 15 us	Frequency = 66666 Hz
;	W = 7:		Time = 24 us	Frequency = 41666 Hz
;	W = 9:		Time = 30 us	Frequency = 33333 Hz
;	W = 16:		Time = 51 us	Frequency = 19607 Hz
;	W = 32:		Time = 99 us	Frequency = 10101 Hz
;	W = 66:		Time = 201 us	Frequency = 4975 Hz
;	W = 99:		Time = 300 us	Frequency = 3333 Hz
;	W = 132:	Time = 399 us	Frequency = 2506 Hz
;	W = 166:	Time = 501 us	Frequency = 1996 Hz
;	W = 199:	Time = 600 us	Frequency = 1666 Hz
;	W = 232:	Time = 699 us	Frequency = 1430 Hz
;	W = 0:		Time = 771 us	Frequency = 1297 Hz
;****************************************************************************
ShortDelay:					; 2 for call
	movwf	DELAY_INNER		; 1
ShortDelay_Loop:
	decfsz	DELAY_INNER, f	; 1 (2 for last loop)
	goto ShortDelay_Loop	; 2
	return					; 1


;****************************************************************************
; LongBuzzer()
;   Makes the buzzer do a sound for ~300ms
;****************************************************************************
LongBuzzer:
	call	Buzzer
	call	Buzzer
	call	Buzzer
	call	Buzzer
	call	Buzzer
	call	Buzzer
	return

;****************************************************************************
; Buzzer()
;   Makes the buzzer do a sound for ~50ms
;****************************************************************************
Buzzer:
	movlw	.64
	movwf	TEMP

	movlw	.1

Buzzer_Loop:
	bsf		PORTC, BUZZER_BIT
	call	Delay
	bcf		PORTC, BUZZER_BIT
	call	Delay

	decfsz	TEMP, F
	goto	Buzzer_Loop

	return


;****************************************************************************
; EEPROM_Read(W)
;   Reads from the EEPROM address specified in the W register. Returns the
;   read value in the W register and switches to BANK0.
;****************************************************************************
EEPROM_Read:
	BANK1
	movwf	EEADR			; Store address in proper register
	bsf		EECON1, RD		; Initiate a read
	movfw	EEDATA			; Put read value in W
	BANK0
	return


;****************************************************************************
; EEPROM_Write(W, EEPROM_DATA)
;   Write the data contained EEPROM_DATA at the EEPROM address specified
;   in W. Switches to BANK0 when done.
;****************************************************************************
EEPROM_Write:
	; Check if address is within range
	movwf	EEPROM_ADDR
	btfsc	EEPROM_ADDR, 7
	goto	EEPROM_Abort_Write

	BANK1
	movwf	EEADR			; Store address in proper register
EEPROM_Write_Cycle:
	movfw	EEPROM_DATA		; Recall data to be stored ...
	movwf	EEDATA			; ... and place it in the proper register.
	bsf		EECON1, WREN	; Enable EEPROM writing
	bcf		INTCON, GIE		; Disable interrupts
	
	movlw	0x55			; Write-unlocking sequence
	movwf	EECON2			;
	movlw	0xAA			;
	movwf	EECON2			;
	
	bsf		EECON1, WR		; Initiate write
	
	bcf		EECON1, WREN	; Disable EEPROM writing

EEPROM_Write_Busy_Wait:
	clrwdt
	btfsc	EECON1, WR				; Check if Write Cycle is complete
	goto	EEPROM_Write_Busy_Wait	; If not, loop it baby

	bsf		INTCON, GIE			; Re-enable interrupts

	; Verify that write was successfull
	movfw	EEPROM_DATA			; Move data that was supposed to be written to W
	bsf		EECON1, RD			; Initiate a read
	xorwf	EEDATA, W			; Compare wanted value with written value
	btfss	STATUS, Z			; Skip next instruction if both values are the same
	goto	EEPROM_Write_Cycle	; Try writing data again

	BANK0
	return

EEPROM_Abort_Write:
	; Write should be aborted
	; Since we probably got here due to an overflow of the LOG_OFFSET
	; variable, set it to a constant so that we don't loop back to 0
	; (and thus overwrite the configuration variables and the start
	; of the log)
	movlw	0x80
	movwf	LOG_OFFSET
	return


;****************************************************************************
; TimedLog(W)
;   Write a log entry to the EEPROM. The format is:
;    (TIME_CODE) (W) (0xDD_SEPARATOR)
;****************************************************************************
#ifndef TEST_SETTINGS
TimedLog:
#endif
	SWAPWF	TEMP
	movfw	TIME_SINCE_BOOT
	goto	Log2


;****************************************************************************
; Log3(W, TEMP, TEMP2)
;   Writes three bytes of data to the EEPROM, followed by the separator
;   character '0xDD'. The data is contained in W, TEMP, and TEMP2; the data
;   is written in that same order.
;****************************************************************************
Log3:
	movwf	EEPROM_DATA		; Move data to be written into data register
	movfw	LOG_OFFSET		; Load EEPROM log address
	call	EEPROM_Write	; Write the data to the EEPROM.
	
	incf	LOG_OFFSET, F	; Increment log address
	movfw	TEMP2			; Puts TEMP2 in W...
	SWAPWF	TEMP			; And does W <--> TEMP
; And continue to the Log2(W, TEMP) code.	

;****************************************************************************
; Log2(W, TEMP)
;   Writes two bytes of data, contained in W and in TEMP, into the EEPROM.
;   The data from W is written first, followed by the data from TEMP.
;   Following that, a separator byte, 0xDD, is written in the EEPROM.
;****************************************************************************
Log2:
	movwf	EEPROM_DATA		; Move data to be written into data register
	movfw	LOG_OFFSET		; Load EEPROM log address
	call	EEPROM_Write	; Write the data to the EEPROM.
	
	incf	LOG_OFFSET, F	; Increment log address
	movfw	TEMP			; Put data from TEMP into W...
; And continue to the Log(W) code.	

;****************************************************************************
; Log(W)
;   Writes one byte of data, contained in W, into the EEPROM. Following that,
;   a separator byte, 0xDD, is written in the EEPROM.
;****************************************************************************
#ifndef TEST_SETTINGS
Log:
#endif
	movwf	EEPROM_DATA		; Move data to be written into data register
	movfw	LOG_OFFSET		; Load EEPROM log address
	call	EEPROM_Write	; Write the data to the EEPROM

	incf	LOG_OFFSET, F	; Increment log address
	movlw	0xDD			; Load separator byte in W...
	movwf	EEPROM_DATA		; .. and store that byte in the data register.
	movfw	LOG_OFFSET		; Load EEPROM log address
	call	EEPROM_Write	; Write the data to the EEPROM

	incf	LOG_OFFSET, F	; Increment log address
	return


;****************************************************************************
; LogNoSeparator(W)
;   Writes one byte of data, contained in W, into the EEPROM. Does not
;   write a separator following the data.
;****************************************************************************
LogNoSeparator:
#ifdef TEST_SETTINGS
Log:
TimedLog:
#endif

	;movwf	EEPROM_DATA		; Move data to be written into data register
	movwf	TEMP

	; Check if last value was compressed
	btfss	EEPROM_LAST_WAS_COMPRESSED, 0
	goto	LogNoSep_Check_For_Compress ; if not
	
	; If so, check if it was same value as this one
	subwf	EEPROM_LAST_WRITTEN_VALUE, W
	btfsc	STATUS, Z
	goto	LogNoSep_Inc_Compressed_Value ; if is the same

	; If value was not the same, then we need to perform a normal write
LogNoSeparator_Normal_Write:
	clrf	EEPROM_LAST_WAS_COMPRESSED
	movfw	TEMP
	movwf	EEPROM_LAST_WRITTEN_VALUE
	movwf	EEPROM_DATA
	goto	LogNoSeparator_Write

LogNoSep_Inc_Compressed_Value:
	incf	EEPROM_COMPRESSED_COUNT, F

	; Check if compressed value is higher than limit
	movlw	0x20
	subwf	EEPROM_COMPRESSED_COUNT, W
	btfsc	STATUS, Z
	goto	LogNoSeparator_Normal_Write ; over the limit, do normal write

	decf	LOG_OFFSET, F
	movfw	EEPROM_COMPRESSED_COUNT
	addlw	MSG_REPEAT_PREVIOUS
	movwf	EEPROM_DATA
	goto	LogNoSeparator_Write

LogNoSep_Check_For_Compress:
	; Check if last written value was same as this one
	subwf	EEPROM_LAST_WRITTEN_VALUE, W
	btfss	STATUS, Z
	goto	LogNoSeparator_Normal_Write ; if not

	; If value was the same as last written value, then write the compression code
	bsf		EEPROM_LAST_WAS_COMPRESSED, 0
	clrf	EEPROM_COMPRESSED_COUNT
	movlw	MSG_REPEAT_PREVIOUS
	movwf	EEPROM_DATA
	goto	LogNoSeparator_Write
	
LogNoSeparator_Write:
	movfw	LOG_OFFSET		; Load EEPROM log address
	call	EEPROM_Write	; Write the data to the EEPROM

	incf	LOG_OFFSET, F	; Increment log address
	return
	

;****************************************************************************
; WipeLog()
;   Wipes the entire log off. Leaves the configuration variables alone.
;****************************************************************************
WipeLog:
	clrf	EEPROM_LAST_WAS_COMPRESSED	; used in LogNoSeparator() function
	movlw	0xFF
	movwf	EEPROM_DATA
	movlw	INITIAL_LOG_OFFSET
	movwf	LOG_OFFSET
	
WipeLog_Loop:
	movfw	LOG_OFFSET
	call	EEPROM_Write
	incf	LOG_OFFSET, F
	movlw	0x80
	subwf	LOG_OFFSET, W
	btfss	STATUS, Z
	goto	WipeLog_Loop

	movlw	INITIAL_LOG_OFFSET
	movwf	LOG_OFFSET

	return
	

;****************************************************************************
; LoadSettings()
;   Loads the configuration data from the EEPROM
;****************************************************************************
LoadSettings:
	movlw	ALARM_ACTIVE_ADDR	
	call	EEPROM_Read
	movwf	ALARM_ACTIVE

	movlw	INITIAL_DELAY_ADDR
	call	EEPROM_Read
	movwf	INITIAL_DELAY

	movlw	USE_SOUND_SIGNALS_ADDR
	call	EEPROM_Read
	movwf	USE_SOUND_SIGNALS

	movlw	NUMBER_OF_SIGNALS_ADDR
	call	EEPROM_Read
	movwf	NUMBER_OF_SIGNALS

	movlw	SIGNAL_INTENSITY_ADDR
	call	EEPROM_Read
	movwf	SIGNAL_INTENSITY

	clrf	MENU_ENTRY6
	clrf	MENU_ENTRY7

	clrwdt
	return


;****************************************************************************
; SaveSettings()
;   Saves the configuration data to the EEPROM
;****************************************************************************
SaveSettings:
	movfw	ALARM_ACTIVE
	movwf	EEPROM_DATA
	movlw	ALARM_ACTIVE_ADDR
	call	EEPROM_Write

	movfw	INITIAL_DELAY
	movwf	EEPROM_DATA
	movlw	INITIAL_DELAY_ADDR
	call	EEPROM_Write

	movfw	USE_SOUND_SIGNALS
	movwf	EEPROM_DATA
	movlw	USE_SOUND_SIGNALS_ADDR
	call	EEPROM_Write

	movfw	NUMBER_OF_SIGNALS
	movwf	EEPROM_DATA
	movlw	NUMBER_OF_SIGNALS_ADDR
	call	EEPROM_Write

	movfw	SIGNAL_INTENSITY
	movwf	EEPROM_DATA
	movlw	SIGNAL_INTENSITY_ADDR
	call	EEPROM_Write

	return


;******************************************************************************
; Fast_Flash_Left_Led
;   Turns on the left led for ~150ms, then turns it off for ~150ms.
;******************************************************************************
Fast_Flash_Left_Led:
	movlw	.97	
	goto	Flash_Left_Led_Fast_Entry_Point


;******************************************************************************
; Flash_Left_Led
;   Turns on the left led for ~300ms, then turns it off for ~300ms.
;******************************************************************************
Flash_Left_Led:
	movlw	.195
Flash_Left_Led_Fast_Entry_Point:
	bsf		PORTC, 	LEFT_LED_BIT
	call	Delay
	call	Delay
	bcf		PORTC, 	LEFT_LED_BIT
	call	Delay
	call	Delay
	return


;******************************************************************************
; Fast_Flash_Right_Led
;   Turns on the right led for ~150ms, then turns it off for ~150ms.
;******************************************************************************
Fast_Flash_Right_Led:
	movlw	.97	
	goto	Flash_Right_Led_Fast_Entry_Point


;******************************************************************************
; Flash_Right_Led
;   Turns on the right led for ~300ms, then turns it off for ~300ms.
;******************************************************************************
Flash_Right_Led:
	movlw	.195
Flash_Right_Led_Fast_Entry_Point
	bsf		PORTA, 	RIGHT_LED_BIT
	call	Delay
	call	Delay
	bcf		PORTA, 	RIGHT_LED_BIT
	call	Delay
	call	Delay
	return


;******************************************************************************
; Flash_Both_Leds
;   Turns on the both leds for ~300ms, then turns it off for ~300ms.
;******************************************************************************
Flash_Both_Leds:
	bsf		PORTC, 	LEFT_LED_BIT
	bsf		PORTA, 	RIGHT_LED_BIT
	call	Buzzer
	call	Buzzer
	clrw
	call	Delay
	bcf		PORTC, 	LEFT_LED_BIT
	bcf		PORTA, 	RIGHT_LED_BIT
	movlw	.195
	call	Delay
	call	Delay
	return


;******************************************************************************
; Flash_Both_Leds_With_Intensity()
;   Turns both leds on for ~300ms using an intensity setting specified in
;   the variable SIGNAL_INTENSITY
;      0 = minimum
;      1 = normal
;      2 = maximum
;******************************************************************************
Flash_Both_Leds_With_Intensity:
	; Figure which code to use to turn on the leds:

	; Check if we should use normal power
	btfsc	SIGNAL_INTENSITY, 0
	goto	Flash_Both_Leds_With_Normal_Int

	; Check if we should use maximum power
	btfsc	SIGNAL_INTENSITY, 1
	goto	Flash_Both_Leds_With_Max_Int

	; Since both these tests failed, we need to use minimum power
Flash_Both_Leds_With_Min_Int:
	movlw	.30
	movwf	TEMP
	movlw	.6
Flash_Both_Leds_With_Min_Loop:
	bsf		PORTC, 	LEFT_LED_BIT
	bcf		PORTC, 	LEFT_LED_BIT
	bsf		PORTA, 	RIGHT_LED_BIT
	bcf		PORTA, 	RIGHT_LED_BIT
	call	Delay

	decfsz	TEMP, F
	goto	Flash_Both_Leds_With_Normal_Loop

	return


Flash_Both_Leds_With_Normal_Int:
	movlw	.193
	movwf	TEMP
	movlw	.1
Flash_Both_Leds_With_Normal_Loop:
	bsf		PORTC, 	LEFT_LED_BIT
	bsf		PORTA, 	RIGHT_LED_BIT
	call	Delay
	bcf		PORTC, 	LEFT_LED_BIT
	bcf		PORTA, 	RIGHT_LED_BIT
	call	Delay
	decfsz	TEMP, F
	goto	Flash_Both_Leds_With_Normal_Loop

	return


Flash_Both_Leds_With_Max_Int:
	; The easiest to implement: turn on the leds, wait 300ms, turn off the leds.
	bsf		PORTC, 	LEFT_LED_BIT
	bsf		PORTA, 	RIGHT_LED_BIT
	movlw	.195
	call	Delay
	call	Delay
	bcf		PORTC, 	LEFT_LED_BIT
	bcf		PORTA, 	RIGHT_LED_BIT	

	return


;******************************************************************************
; INTERRUPT HANDLER
;******************************************************************************
Interrupt_Handler:
	; Save Context
	movwf	W_ISR			; Save W register
	swapf	STATUS, W		; Switch STATUS and W
	movwf	STATUS_ISR		; Save STATUS register
	BANK0					; Switch to Bank 0

	; Check if we got a Timer1 interrupt
	btfsc	PIR1, TMR1IF
	call	Timer1_Interrupt

	; Check if we got a PORTA interrupt
	btfsc	INTCON, RAIF
	call	Button_Press_Interrupt

	; Restore Context
	swapf	STATUS_ISR, W	; Swap old STATUS with W
	movwf	STATUS			; Restore old STATUS
	swapf	W_ISR, F		; Swap old W with itself (to get proper niblet order)
	swapf	W_ISR, W		; Swap old W with W
	
	retfie


;******************************************************************************
; Timer1_Interrupt
;   Handles interrupts from Timer1 overflow
;******************************************************************************
Timer1_Interrupt:
	; Only increment overflows counter at every 2 calls
	incf	TIMER1_MARKER, F		; Increment marker (since we only check,
									; bit 0 this is effectively a toggle)
	btfsc	TIMER1_MARKER, 0		; Check if marker is odd
	goto	Timer1_Interrupt_Cont	; If so, do not increment

	; For testing, decrement 'time remaining' every second instead of every 2.5 minutes
#ifdef TESTING
	decf	TIME_REMAINING, F
#endif

	decf	TIMER1_OVERFLOWS, F		; If so, decrement overflow count.
	btfss	STATUS, Z				; Check if result is 0
	goto	Timer1_Interrupt_Cont	; If not, exit interrupt handler.

	; 150 seconds (2.5 minutes) have passed!
	incf	TIME_SINCE_BOOT, F
	movlw	TIMER1_OVERFLOWS_PER_150_SECS
	movwf	TIMER1_OVERFLOWS

#ifdef	TIMER_CALIBRATION
	call	Flash_Both_Leds
#endif


	; Decrease time remaining register
#ifndef TESTING
	decf	TIME_REMAINING, F
#endif

Timer1_Interrupt_Cont

	bcf		PIR1, TMR1IF	; Clear interrupt
	return


;******************************************************************************
; Button_Press_Interrupt
;   Handles interrupt from changes on port A (aka button presses)
;******************************************************************************
Button_Press_Interrupt:
	; The BUTTON_STATE bits are cleared by loop that receives the events
	btfss	PORTA, ACTIVATE_BTN_BIT
	bsf		BUTTONS_STATE, ACTIVATE_BTN_BIT

	btfss	PORTA, SELECT_BTN_BIT
	bsf		BUTTONS_STATE, SELECT_BTN_BIT

	btfss	PORTA, ENTER_BTN_BIT
	bsf		BUTTONS_STATE, ENTER_BTN_BIT

	; Debounce (no interrupts for 10ms)
	movlw	.13
	call	Delay

	bcf		INTCON, RAIF	; Clear interrupt
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
	fill (goto WDTTrap), 0x03F0-$

;******************************************************************************
; Constants
;   These constants are put at the end of the program in order to be easily
;   accessible when importing a program .hex into the java app
;******************************************************************************
; Starts at 0x03F0
	movlw	AD_REM_SCAN_MODE
	movlw	AD_JITTER_THRESHOLD
	movlw	NUM_REM_SCANS
	movlw	TIME_BETWEEN_AD_SAMPLES
	movlw	MIN_AD_CHANGES_FOR_REM_DETECT
	movlw	TIME_UNTIL_NEXT_SCAN
	movlw	TIME_UNTIL_ALARM_SOUNDS
	movlw	TIME_INCREMENT_ON_REALITY_CHECK
	movlw	TIMER1_OVERFLOWS_PER_150_SECS

	; WDTTrap for the rest
	fill (goto WDTTrap), 0x03FF-$

	END
