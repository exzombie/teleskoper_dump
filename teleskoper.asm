
	list p=16f877

	include "p16f877.inc"

	__CONFIG _BODEN_OFF & _WDT_OFF & _HS_OSC & _PWRTE_ON

;;; CONSTANTS
;;; ==========================================================================

	CONSTANT PORTC_STAT = b'00000111' ; Settings of PORTC

;;; MEMORY LAYOUT
;;; ==========================================================================

	;; Shared memory region, 16 bytes long
	CBLOCK	0x70
	W_TEMP			; Context switch during interrupt
	STATUS_TEMP
	PCLATH_TEMP
	FSR_TEMP
	addrhigh		; Address for longjmp
	addrlow
	argval			; Used to pass arguments to longjmp
	retval			; Used to pass return value from longjmp
	flags
	ENDC

	;; Description of flag bits:
	;;   0 - unused
	;;   1 - transmission in progress
	;;   2 - diagnostic mode
	;;   3 - speed is 0
	;;   4 - old speed was 0
	;;   5 - speed has changed
	;;   6 - deadzone
	;;   7 - unused

	;; Start of BANK0 general purpose memory
	CBLOCK	0x20
	;; Delay macro
	cnt0
 	cnt1
	cnt2
	;; Blinking routine
	noblinks
	nocyc
	ledidx
	;; Magnitude approximation
	mag
	;; Speed and direction
	spdir			; high bits are direction, low bits are speed
	spdir_old
	spdaddr			; Addresses of the routine that determines
	diraddr			; the strings for the chosen mount
	spdaddrh
	diraddrh
	;; Measurements
	x1
	x1_raw
	x2
	x2_raw
	y1
	y1_raw
	y2
	y2_raw
	center
	tmp			; Never rely on this after a subroutine call
	bins:3			; Array for binning of measurements
	binmax
	thresh
	;; BIN2BCD
	BIN
	count
	huns
	tens
	ones
	;; STRINGS:
	strpos			; Pointer to the current character in str
	strlen			; Length of the current string in str
	ENDC

	;; Start of BANK1 general purpose memory
	CBLOCK	0xa0
	str			; Buffer for longer strings, 80 bytes free
	ENDC


;;; MACROS which are needed in the main body of code
;;; (Other macros are defined where they are needed)
;;; ==========================================================================

;;; Delay macro for 12MHz.
;;; Arguments: delay in microseconds; maximum value 2^24-1, minimum value 5.
;;; Uses registers cnt0, cnt1, cnt2.

delay		MACRO	musec
	LOCAL	musec2 = musec
musec2	-=	0x5
	movlw	(musec2 / 0x10000 + 0x1) % 0x100
	movwf	cnt2
musec2	%=	0x10000
	movlw	(musec2 / 0x100 + 0x1) % 0x100
	movwf	cnt1
musec2	%=	0x100
	movlw	(musec2 + 0x1) % 0x100
	movwf	cnt0

	decfsz	cnt0, F
	goto	$-1
	nop
	decfsz	cnt1, F
	goto	$-4
	nop
	decfsz	cnt2, F
	goto	$-7
	nop

	ENDM

;;; ==========================================================================

;;; Shortcut macro for measuring each axis.
;;; Argument: name of axis.

	;; Adjust these constants to match actual wiring
	CONSTANT	X1=1, Y1=0, X2=3, Y2=2

adcmsr		MACRO	chan
	bsf	PORTB, 5	; Enable current through the potentiometer
	IF (chan == 0)
	bcf	ADCON1, CHS0
	bcf	ADCON1, CHS1
	ELSE
	IF (chan == 1)
	bsf	ADCON1, CHS0
	bcf	ADCON1, CHS1
	ELSE
	IF (chan == 2)
	bcf	ADCON1, CHS0
	bsf	ADCON1, CHS1
	ELSE
	IF (chan == 3)
	bsf	ADCON1, CHS0
	bsf	ADCON1, CHS1
	ELSE
	ERROR	"Wrong channel selected!"
	ENDIF
	ENDIF
	ENDIF
	ENDIF
	call	adc_measure
	bcf	PORTB, 5
	ENDM

;;; ==========================================================================

;;; Subtract one register from the other and put absolute value in W.
;;; Arguments: register addresses.

subabs		MACRO	reg1, reg2
	movf	reg1, W
	subwf	reg2, W
	btfsc	STATUS, C	; skip the goto if reg2 < reg1
	goto	$+3
	xorlw	0xff
	addlw	0x1
	ENDM

;;; ==========================================================================

;;; Toggle a LED in PORTC
;;; Argument: LED number

tglled		MACRO	led
	movf	PORTC, W
	xorlw	led
	movwf	PORTC
	ENDM

;;; ==========================================================================

;;; Jump to a computed location, anywhere in memory.
;;; Arguments: registers that store the jump destination.
;;; Origin is stored in addrhigh and addrlow, so calls to long jumps don't stack.

longjmp		MACRO	addrh, addrl
	LOCAL	here
	movlw	HIGH(here)
	movwf	addrhigh
	movlw	LOW(here)
	movwf	addrlow
	movf	addrh, W
	movwf	PCLATH
	movf	addrl, W
	movwf	PCL
here:
	ENDM

;;; Return from a long jump

retjmp		MACRO
	movf	addrhigh, W
	movwf	PCLATH
	movf	addrlow, W
	movwf	PCL
	ENDM

;;; ==========================================================================

;;; Copy a string. Assumes FSR points to the destination.
;;; Arguments: pos is the beginning of the string, len is its length.
;;; Puts len in W.

memcpy		MACRO	pos, len
	LOCAL	pos2 = pos
	WHILE	(pos2 < pos + len)
	movf	pos2, W
	movwf	INDF
	incf	FSR, F
pos2	+=	0x1
	ENDW
	movlw	len
	ENDM

;;; A bunch of macros to insert certain strings. They assume FSR points to
;;; the destination. They put the number of inserted characters in W.

;;; End-of-line
ins_eol		MACRO
	movlw	0xd
	movwf	INDF
	incf	FSR, F
	movlw	0xa
	movwf	INDF
	incf	FSR, F
	movlw	0x2
	ENDM

;;; Space
ins_spc		MACRO
	movlw	0x20
	movwf	INDF
	incf	FSR, F
	movlw	0x1
	ENDM

;;; Comma and space
ins_cmsp	MACRO
	movlw	0x2c
	movwf	INDF
	incf	FSR, F
	movlw	0x20
	movwf	INDF
	incf	FSR, F
	movlw	0x2
	ENDM

;;; Space-dash-space
ins_dashtab	MACRO
	movlw	0x20
	movwf	INDF
	incf	FSR, F
	movlw	0x2d
	movwf	INDF
	incf	FSR, F
	movlw	0x20
	movwf	INDF
	incf	FSR, F
	movlw	0x3
	ENDM

;;; Horizontal tab
ins_tab		MACRO
	movlw	0x9
	movwf	INDF
	incf	FSR, F
	movlw	0x1
	ENDM

;;; ==========================================================================

;;; A fast way to calculate approximate magnitude of a 2-vector.
;;; Arguments: registers holding absolute values of vector components.
;;; Magnitude is returned in mag.
;;; Maximum possible value for the arguments is 185, maximum error is 14.
;;; It uses the Robertson formula with magic constant of 3/8, which is
;;; calculated by a separate routine.

magaprx		MACRO	mag1, mag2
	movf	mag1, W		; Find the maximum value
	subwf	mag2, W
	btfss	STATUS, C
	goto	$+5
	movf	mag1, W		; mag2 >= mag1
	call	magaprx_3_8
	addwf	mag2, W
	goto	$+4
	movf	mag2, W		; mag1 > mag2
	call	magaprx_3_8
	addwf	mag1, W
	movwf	mag
	ENDM

;;; ==========================================================================

;;; The following macro is used in applying deadzone to the measurement.
;;; It takes raw measurement in W and writes it to arg.
;;; If the change between W and the old arg_raw is larger than thresh,
;;; it sets the appropriate flag.

deadzn		MACRO	arg
	movwf	arg
	subabs	arg, arg+1
	subwf	thresh, W
	btfss	STATUS, C
	bsf	flags, 6
	ENDM

;;; A shortcut to shuffle things around after checking for deadzone.
;;; Use this to copy raw measurements from arg to arg_raw and
;;; subtract the center offset.

ddznshf		MACRO	arg
	movf	arg, W
	movwf	arg+1		; arg_raw should be one reg after arg
	subabs	arg, center
	movwf	arg
	ENDM

;;; ==========================================================================

;;; Swap two registers without temporary storage

swapff		MACRO	reg1, reg2
	movf	reg2, W
	xorwf	reg1, F
	xorwf	reg1, W
	movwf	reg2
	xorwf	reg1, F
	ENDM

;;; CODE
;;; ==========================================================================

	ORG	0x0
	goto	start

;;; INTERRUPTS
	ORG	0x4
irq:
	;; Save state on interrupt
	movwf	W_TEMP
	swapf	STATUS, W
	clrf	STATUS
	movwf	STATUS_TEMP
	movf	PCLATH, W
	movwf	PCLATH_TEMP
	clrf	PCLATH
	movf	FSR, W
	movwf	FSR_TEMP

	;; Button press? Disable the interrupt, which will start calibration.
	btfss	INTCON, INTF
	goto	txirq
	bcf	INTCON, INTF
	bcf	INTCON, INTE
	bcf	INTCON, INTF
	goto	endirq


;;; USART code. It fils TXREG from strpos, which should be in BANK1 general
;;; purpose area, i.e. str < strpos < str+0x80. It stops sending when it
;;; reaches str+strlen.
txirq:
	;; Transmit register empty?
	bsf	STATUS, RP0
	btfss	PIR1, TXIF
	goto	endirq
	bcf	STATUS, RP0

	movf	strpos, W
	movwf	FSR
	movf	INDF, W
	movwf	TXREG
	incf	strpos, F

	;; Stop transmitting when finished
	movlw	str
	addwf	strlen, W
	xorwf	strpos, W
	btfss	STATUS, Z
	goto	endirq
	bsf	STATUS, RP0
	bcf	PIE1, TXIE

endirq:
	;; Restore state before returning from interrupt
	movf	FSR_TEMP, W
	movwf	FSR
	movf	PCLATH_TEMP, W
	movwf	PCLATH
	swapf	STATUS_TEMP, W
	movwf	STATUS
	swapf	W_TEMP, F
	swapf	W_TEMP, W
	retfie

;;; BODY
;;; ==========================================================================

start:
	bcf	STATUS, RP0
	bcf	STATUS, RP1

	bsf	STATUS, RP0
	movlw	0x02		; RA0-RA3, RA5 as analog
	movwf	ADCON1
	movlw	b'11011111'	; RB5 as output
	movwf	TRISB
	movlw	b'11111000'	; Set output pins for the LEDs
	movwf	TRISC
	bcf	STATUS, RP0
	movlw	PORTC_STAT	; Enable RC0-RC2 (turn LED off)
	movwf	PORTC
	clrf	PORTB		; Disable the joystick
	movlw	0x80		; ADC clock
	movwf	ADCON0

	;; Setup serial port
	bsf	STATUS, RP0
	bcf	TXSTA, SYNC
	bsf	TXSTA, BRGH
	bcf	TXSTA, TX9
	bsf	TXSTA, TXEN
	;movlw	d'12'		; Baud rate 57.6k at 12MHz, precision 0.1%
	movlw	d'77'		; Baud rate 9.6k at 12MHz, precision 0.16%
	movwf	SPBRG
	bcf	STATUS, RP0
	bsf	RCSTA, SPEN

	clrf	flags

	;; Enable interrupts
	bsf	INTCON, INTE
	bsf	INTCON, PEIE
	bsf	INTCON, GIE

	;; Signal powerup
	movlw	0x2
	call	blink

	;; Enter diagnostic mode if INT was pressed during powerup signal
	btfsc	INTCON, INTE
	goto	wait_calib
	bsf	flags, 2
	delay	d'300'
	bsf	INTCON, INTE

wait_calib:
	;; Wait for calibration
	btfsc	flags, 0
	goto	done_calib
	btfss	INTCON, INTE
	call	calibrate
	tglled	0x1
	delay	d'500000'
	goto	wait_calib

done_calib:
	;; Clear LEDs
	movlw	PORTC_STAT
	movwf	PORTC

	;; Decide what to do next
;	btfsc	flags, 2
	goto	diagnostic_main

	;; Choose the address of the routine that will generate strings
	;; to control the mount. A physical switch will someday be used
	;; to make the choice.
	;; Symbols chosen_{dir,spd} have to be declared as variables because
	;; the addresses they store become defined in second pass af assembly.

	VARIABLE	chosen_dir = human_readable_dir
	VARIABLE	chosen_spd = human_readable_spd
	;VARIABLE	chosen_dir = lx200_dir
	;VARIABLE	chosen_spd = lx200_spd
	movlw	LOW(chosen_dir)
	movwf	diraddr
	movlw	HIGH(chosen_dir)
	movwf	diraddrh
	movlw	LOW(chosen_spd)
	movwf	spdaddr
	movlw	HIGH(chosen_spd)
	movwf	spdaddrh

;;; ---------------------------------------------------------------------------

;;; MAIN LOOP

	clrf	spdir
	clrf	x1_raw
	clrf	x2_raw
	clrf	y1_raw
	clrf	y2_raw

	;; First, jump right ahead and order the mount to stop.
	clrf	strlen
	movlw	str
	movwf	strpos
	movwf	FSR
	bsf	flags, 5
	goto	main_printspd

main:
	btfss	INTCON, INTE
	call	calibrate

	movf	spdir, W
	movwf	spdir_old
	clrf	spdir
	bcf	flags, 3
	bcf	flags, 4
	bcf	flags, 5
	bcf	flags, 6

main_measureloop:
	btfss	INTCON, INTE
	call	calibrate

	delay	d'100000'
	adcmsr	X1
	deadzn	x1
	adcmsr	X2
	deadzn	x2
	adcmsr	Y1
	deadzn	y1
	adcmsr	Y2
	deadzn	y2

	;; See if any measurement is outside deadzone threshold.
	;; If not, measure again. Else, backup the raw values and
	;; subtract center offset.
	btfss	flags, 6
	goto	main_measureloop
	ddznshf	x1
	ddznshf	x2
	ddznshf	y1
	ddznshf	y2

	;; Find the largest measurement for each axis; they determine direction
	;; Intermediate result stored in spdir<4,5>
	movf	x2, W
	subwf	x1, W
	btfss	STATUS, C
	bsf	spdir, 4	; Mark x2 >= x1
	movf	y2, W
	subwf	y1, W
	btfss	STATUS, C
	bsf	spdir, 5	; Mark y2 >= y1

	;; Make sure that {x,y}1 >= {x,y}2
	btfss	spdir, 4
	goto	$+6		; swapff is a 5-instruction macro
	swapff	x1, x2
	btfss	spdir, 5
	goto	$+6
	swapff	y1, y2

	;; Determine the speed using the bins array
	magaprx	x1, y1
	call	binning
	iorwf	spdir, F

	;; Did the speed change?
	movf	spdir, W
	andlw	0xf
	btfsc	STATUS, Z
	bsf	flags, 3	; Mark that speed is zero
	movwf	tmp
	movf	spdir_old, W
	andlw	0xf
	btfsc	STATUS, Z
	bsf	flags, 4	; Mark that old speed was zero
	xorwf	tmp, W
	btfss	STATUS, Z
	bsf	flags, 5

	;; Prepare to overwrite the string
	call	wait_tx
	clrf	strlen
	movlw	str
	movwf	strpos
	movwf	FSR

	;; Determine direction.
	;; For possible values of spdir<4-7> see the description of
	;; string routines at the end of this file.
	btfsc	flags, 3
	goto	main_printspd	; If speed is zero skip this

	movf	y1, W
	movwf	mag
	call	binning
	andlw	0xff
	btfss	STATUS, Z
	goto	$+4
	bsf	spdir, 6	; If y = 0 we have E or W
	bcf	spdir, 5
	goto	main_printdir

	movf	x1, W
	movwf	mag
	call	binning
	andlw	0xff
	btfss	STATUS, Z
	goto	$+6
	bsf	spdir, 6	; If x = 0 we have N or S
	bcf	spdir, 4
	btfsc	spdir, 5
	bsf	spdir, 4
	bsf	spdir, 5

	;; Did the direction change?
main_printdir:
	btfsc	flags, 4	; If old speed was zero always print direction
	goto	main_printdir2
	movf	spdir, W
	andlw	0xf0
	movwf	tmp
	movf	spdir_old, W
	andlw	0xf0
	xorwf	tmp, W
	btfsc	STATUS, Z
	goto	main_printspd

	;; Print direction.
main_printdir2:
	swapf	spdir, W
	andlw	0xf
	movwf	argval
	longjmp	diraddrh, diraddr
	movf	retval, W
	addwf	strlen, F
	call	start_tx

	;; Print speed
main_printspd:
	btfss	flags, 5
	goto	main
	movf	spdir, W
	andlw	0xf
	movwf	argval
	longjmp	spdaddrh, spdaddr
	movf	retval, W
	addwf	strlen, F
	call	start_tx

	goto	main

;;; ---------------------------------------------------------------------------

;;; Diagnostic part. It prints measured values from the potentiometer with
;;; calibration center subtracted, approximate magnitudes as calculated
;;; by magapprox, measured center and extreme position from the calibration
;;; routine.
;;; OUTPUT FORMAT:
;;; x1, y1    sqrt(x1^2+y1^2), sqrt(x2^2+y2^2)    cntr, extrm    bins

diagnostic_main:
	movlw	str
	movwf	strpos
	movwf	FSR
	clrf	strlen

	;; x1_raw, y2_raw

	adcmsr	X1
	movwf	x1_raw
	movwf	BIN
	call	BIN2BCD
	memcpy	huns, 0x3
	addwf	strlen, F
	ins_cmsp
	addwf	strlen, F

	adcmsr	Y1
	movwf	y1_raw
	movwf	BIN
	call	BIN2BCD
	memcpy	huns, 0x3
	addwf	strlen, F
	ins_tab
	addwf	strlen, F

	;; x1, y1
	subabs	x1_raw, center
	movwf	x1
	movwf	BIN
	call	BIN2BCD
	memcpy	huns, 0x3
	addwf	strlen, F
	ins_cmsp
	addwf	strlen, F

	subabs	y1_raw, center
	movwf	y1
	movwf	BIN
	call	BIN2BCD
	memcpy	huns, 0x3
	addwf	strlen, F
	ins_tab
	addwf	strlen, F

	;; sqrt(x1^2+y1^2)
	magaprx	x1, y1
	movwf	BIN
	call	BIN2BCD
	memcpy	huns, 0x3
	addwf	strlen, F
	ins_tab
	addwf	strlen, F

	;; cntr, extrm
	movf	center, W
	movwf	BIN
	call	BIN2BCD

	memcpy	huns, 0x3
	addwf	strlen, F
	ins_tab
	addwf	strlen, F

	;; bins
	movlw	bins
	movwf	tmp
diagnostic_bins:
	movwf	FSR
	movf	INDF, W
	movwf	BIN
	call	BIN2BCD
	movlw	str
	addwf	strlen, W
	movwf	FSR
	memcpy	huns, 0x3
	addwf	strlen, F
	incf	tmp, F
	movlw	binmax+1
	xorwf	tmp, W
	btfsc	STATUS, Z
	goto	diagnostic_end_bins
	ins_cmsp
	addwf	strlen, F
	movf	tmp, W
	goto	diagnostic_bins
	
diagnostic_end_bins:

	ins_eol
	addwf	strlen, F

	call	start_tx

	movlw	0x1
	call	blink

	call	wait_tx
	btfss	INTCON, INTE
	call	calibrate

	goto	diagnostic_main


;;; ROUTINES
;;; ==========================================================================

;;; Wait for USART transmission to complete, as reported by the TXIE flag.
;;; Does NOT check for physical completion (the TRMT flag).

wait_tx:
	bsf	STATUS, RP0
	btfsc	PIE1, TXIE
	goto	$-1
	bcf	STATUS, RP0
	return

;;; Start transmission of a string.
;;; Arguments: strpos is the first character, strlen is the string length.
;;; Transmission is handled via interrupts.

start_tx:
	bsf	STATUS, RP0
	bsf	PIE1, TXIE
	bcf	STATUS, RP0
	return

;;; ==========================================================================

;;; ADC measurement
;;; Select AD channel before calling
;;; Assumes BANK 0, left justified ADRES
;;; Returns with MSB in W
;;; Wait at least 3 instructions (at 12MHz) before calling again

adc_measure:
	bsf	ADCON0, ADON
	delay	d'20'	    	; Minimum wait calculated from datasheet
	bsf	ADCON0, GO	; Start conversion
	btfsc	ADCON0, GO	; Conversion takes minimum 32usec at 12MHz
	goto	$-1
	movf	ADRESH, W
	bcf	ADCON0, ADON
	return

;;; ==========================================================================

;;; Poll RB0 with 50ms debounce

rb0deb:
	btfss	PORTB, 0
	goto	$-1
	delay	d'50000'
	btfss	PORTB, 0
	goto	rb0deb
	return

;;; Calibration

calibrate:
	movlw	PORTC_STAT
	movwf	PORTC


	;; Measure the up, down, left and right directions, in that
	;; order. The user is signalled to perform the measurement by
	;; turning on the red LED. After measuring each direction, the
	;; center is measured. These values are averaged.

	clrf center

	delay	d'300000'
	tglled	0x2

	;; Up
	call	rb0deb
	adcmsr	Y1
	movwf	y1

	tglled	0x2
	delay	d'300000'
	tglled	0x2

	call	rb0deb
	adcmsr	Y1
	movwf	tmp
	bcf	STATUS, C
	rrf	tmp, W
	addwf	center, F

	;; Down
	tglled	0x2
	delay	d'300000'
	tglled	0x2

	call	rb0deb
	adcmsr	Y1
	movwf	y1_raw

	tglled	0x2
	delay	d'300000'
	tglled	0x2

	call	rb0deb
	adcmsr	Y1
	movwf	tmp
	bcf	STATUS, C
	rrf	tmp, W
	addwf	center, F

	;; Left
	tglled	0x2
	delay	d'300000'
	tglled	0x2

	call	rb0deb
	adcmsr	X1
	movwf	x1

	tglled	0x2
	delay	d'300000'
	tglled	0x2

	call	rb0deb
	adcmsr	Y1
	movwf	tmp
	bcf	STATUS, C
	rrf	tmp, W
	addwf	center, F

	;; Right
	tglled	0x2
	delay	d'300000'
	tglled	0x2

	call	rb0deb
	adcmsr	X1
	movwf	x1_raw

	tglled	0x2
	delay	d'300000'
	tglled	0x2

	call	rb0deb
	adcmsr	Y1
	movwf	tmp
	bcf	STATUS, C
	rrf	tmp, W
	addwf	center, F

	bcf	STATUS, C
	rrf	center, F

	tglled	0x2


	;; Find the smallest difference between a measured direction
	;; and the center. This is the range of the stick.

	subabs	center, x1
	movwf	binmax

	subabs	center, x1_raw
	movwf	tmp
	subwf	binmax, W
	movf	tmp, W
	btfsc	STATUS, C
	movwf	binmax

	subabs	center, y1
	movwf	tmp
	subwf	binmax, W
	movf	tmp, W
	btfsc	STATUS, C
	movwf	binmax

	subabs	center, y1_raw
	movwf	tmp
	subwf	binmax, W
	movf	tmp, W
	btfsc	STATUS, C
	movwf	binmax

	;; Set up possible speeds in the following way:
	;; First, there is a threshold, below which the speed is zero:
	;; bins[0] = binmax/8 (threshold)
	;; bins[1] = binmax/2
	;; bins[2] = binmax - binmax/8
	;;							        range
	;;  center     	       	       	       	               	       binmax
	;;  |-------|-----------------------|-----------------------|-------|
	;;       bins[0]               	 bins[1]       	         bins[2]
	;;     	threshold
	;;

	bcf	STATUS, C
	rrf	binmax, W
	movwf	bins+1
	movwf	bins
	bcf	STATUS, C
	rrf	bins, F
	bcf	STATUS, C
	rrf	bins, W
	movwf	bins
	subwf	binmax, W
	movwf	bins+2

	delay	d'1000000'
	bsf	flags, 0
	bcf	INTCON, INTF
	bsf	INTCON, INTE
	return

;;; ==========================================================================

;;; Blinking routine. It cycles three LEDs.
;;; Arguments: W is the number of LED cycles

blink:
	movwf	nocyc
	clrf	ledidx
	bsf	STATUS, C     	; Used by rlf, will start cycle with LED 0

blink_cyc:
	movlw	0x6		; Cycle length
	movwf	noblinks

blink_reloop:
	delay	d'166666'

	;; Toggle the chosen LED
	movlw	0x1
	rlf	ledidx, F
	btfsc	ledidx, 3
	movwf	ledidx
	movf	PORTC, W
	xorwf	ledidx, W
	movwf	PORTC

	decfsz	noblinks, F
	goto	blink_reloop
	decfsz	nocyc, F
	goto	blink_cyc

	return

;;; ==========================================================================

;;; A routine to multiply a number by 3/8, used by magaprx.

magaprx_3_8:
	movwf	mag
	bcf	STATUS, C	; Divide by 8
	rrf	mag, F
	bcf	STATUS, C
	rrf	mag, F
	bcf	STATUS, C
	rrf	mag, F
	movf	mag, W		; Multiply by 3
	addwf	mag, W
	addwf	mag, W
	return

;;; ==========================================================================

;;; Find which bin the given value falls in.
;;; Arguments: mag is the value to be compared with the bins array.
;;; Result is given in W.

binning:
	VARIABLE	binidx = 0
	WHILE (binidx < 4)
	movf	mag, W
	subwf	bins+binidx, W
	btfsc	STATUS, C
	retlw	binidx
binidx	+=	1
	ENDW
	retlw	0x3

;;; ==========================================================================

;;; Convert 8-bit binary to BCD
;;; Adapted from:
;;;    http://www.piclist.com/techref/microchip/math/radix/b2a-8b3d-ab.htm
;;; Argument: BIN
;;; Returns: huns, tens, ones
;;; uses ADD-3 algoerthm

BIN2BCD:
	movlw d'8'
	movwf count
	clrf huns
	clrf tens
	clrf ones

BCDADD3:
	movlw d'5'
	subwf huns, 0
	btfsc STATUS, C
	CALL ADD3HUNS

	movlw d'5'
	subwf tens, 0
	btfsc STATUS, C
	CALL ADD3TENS

	movlw d'5'
	subwf ones, 0
	btfsc STATUS, C
	CALL ADD3ONES

	decf count, 1
	bcf STATUS, C
	rlf BIN, 1
	rlf ones, 1
	btfsc ones,4 ;
	CALL CARRYONES
	rlf tens, 1

	btfsc tens,4 ;
	CALL CARRYTENS
	rlf huns,1
	bcf STATUS, C

	movf count, 0
	btfss STATUS, Z
	GOTO BCDADD3

	movf huns, 0 ; add ASCII Offset
	addlw h'30'
	movwf huns

	movf tens, 0 ; add ASCII Offset
	addlw h'30'
	movwf tens

	movf ones, 0 ; add ASCII Offset
	addlw h'30'
	movwf ones

	RETURN

ADD3HUNS:
	movlw d'3'
	addwf huns,1

	RETURN

ADD3TENS:
	movlw d'3'
	addwf tens,1

	RETURN

ADD3ONES:
	movlw d'3'
	addwf ones,1

	RETURN

CARRYONES:
	bcf ones, 4
	bsf STATUS, C
	RETURN

CARRYTENS:
	bcf tens, 4
	bsf STATUS, C
	RETURN

;;; ==========================================================================

;;; Check whether we have crossed the page boundary.

        IF ($ >= 0x800)
            ERROR "Core routines larger than a page!"
        ENDIF

;;; ==========================================================================

;;; SPEED AND DIRECTION STRINGS
;;; These routines are computed GOTOs that append the requested string to
;;; wherever FSR points. They return the number of written bytes in retval.
;;; Because of their size, these routines are placed in PAGE 1 so that they
;;; don't cross the page boundary. Also, they are 256-word aligned to make
;;; computed GOTOs easier.

;;; Speed strings are indexed in ascending order.

;;; Direction strings are indexed according to the following table,
;;; with indices in base 4:
;;; 00 - NW    02 - SW   10 - W    12 - N
;;; 01 - NE    03 - SE   11 - E    13 - S

;;; Call these subroutines with index in argval.
;;; They return the number of characters in retval.

;;; ==========================================================================

;;; Macro that writes a single character

wrchar		MACRO	char
	movlw	char
	movwf	INDF
	incf	FSR, F
	ENDM

;;; ==========================================================================

;;; Human readable

	ORG	0x800
human_readable_dir:
	wrchar	'D'
	wrchar	'i'
	wrchar	'r'
	wrchar	'e'
	wrchar	'c'
	wrchar	't'
	wrchar	'i'
	wrchar	'o'
	wrchar	'n'
	wrchar	' '

	movf	argval, W
	addwf	PCL, F
	goto	human_readable_dir_NW
	goto	human_readable_dir_NE
	goto	human_readable_dir_SW
	goto	human_readable_dir_SE
	goto	human_readable_dir_W
	goto	human_readable_dir_E
	goto	human_readable_dir_N
	goto	human_readable_dir_S

human_readable_dir_NW:
	wrchar	'N'
	wrchar	'W'
	ins_eol
	movlw	0xc+0x2
	movwf	retval
	retjmp

human_readable_dir_NE:
	wrchar	'N'
	wrchar	'E'
	ins_eol
	movlw	0xc+0x2
	movwf	retval
	retjmp

human_readable_dir_SW:
	wrchar	'S'
	wrchar	'W'
	ins_eol
	movlw	0xc+0x2
	movwf	retval
	retjmp

human_readable_dir_SE:
	wrchar	'S'
	wrchar	'E'
	ins_eol
	movlw	0xc+0x2
	movwf	retval
	retjmp

human_readable_dir_E:
	wrchar	'E'
	ins_eol
	movlw	0xc+0x1
	movwf	retval
	retjmp

human_readable_dir_W:
	wrchar	'W'
	ins_eol
	movlw	0xc+0x1
	movwf	retval
	retjmp

human_readable_dir_S:
	wrchar	'S'
	ins_eol
	movlw	0xc+0x1
	movwf	retval
	retjmp

human_readable_dir_N:
	wrchar	'N'
	ins_eol
	movlw	0xc+0x1
	movwf	retval
	retjmp

human_readable_spd:
	wrchar	'S'
	wrchar	'p'
	wrchar	'e'
	wrchar	'e'
	wrchar	'd'
	wrchar	' '

	movf	argval, W
	addlw	0x30
	movwf	INDF
	incf	FSR, F
	ins_eol
	movlw	0x9
	movwf	retval
	retjmp

	;; Sanity check
        IF (HIGH($) != HIGH(human_readable_dir))
            ERROR "Table crosses 256-word boundary!"
        ENDIF

;;; ==========================================================================

;;; LX200 protocol
;;; This one crosses 256-word boundary, but the both computed gotos are
;;; at the beginning, so it's OK.

	ORG	0x900
lx200_spd:
	wrchar	':'
	movf	argval, W
	addwf	PCL, F
	goto	lx200_spd0
	goto	lx200_spd1
	goto	lx200_spd2
	goto	lx200_spd3

lx200_dir:
	movf	argval, W
	addwf	PCL, F
	goto	lx200_dir_nw
	goto	lx200_dir_ne
	goto	lx200_dir_sw
	goto	lx200_dir_se
	goto	lx200_dir_w
	goto	lx200_dir_e
	goto	lx200_dir_n
	goto	lx200_dir_s

lx200_dir_e:
	wrchar	':'
	wrchar	'Q'
	wrchar	'w'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	'n'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	's'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	'e'
	wrchar	'#'
	movlw	0x10
	goto	lx200_dir_end
lx200_dir_w:
	wrchar	':'
	wrchar	'Q'
	wrchar	'e'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	'n'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	's'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	'w'
	wrchar	'#'
	movlw	0x10
	goto	lx200_dir_end
lx200_dir_n:
	wrchar	':'
	wrchar	'Q'
	wrchar	'w'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	'e'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	's'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	'n'
	wrchar	'#'
	movlw	0x10
	goto	lx200_dir_end
lx200_dir_s:
	wrchar	':'
	wrchar	'Q'
	wrchar	'w'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	'n'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	'e'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	's'
	wrchar	'#'
	movlw	0x10
	goto	lx200_dir_end
lx200_dir_nw:
	wrchar	':'
	wrchar	'Q'
	wrchar	'e'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	's'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	'n'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	'w'
	wrchar	'#'
	movlw	0x10
	goto	lx200_dir_end
lx200_dir_ne:
	wrchar	':'
	wrchar	'Q'
	wrchar	'w'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	's'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	'n'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	'e'
	wrchar	'#'
	movlw	0x10
	goto	lx200_dir_end
lx200_dir_sw:
	wrchar	':'
	wrchar	'Q'
	wrchar	'e'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	'n'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	's'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	'w'
	wrchar	'#'
	movlw	0x10
	goto	lx200_dir_end
lx200_dir_se:
	wrchar	':'
	wrchar	'Q'
	wrchar	'w'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	'n'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	's'
	wrchar	'#'
	wrchar	':'
	wrchar	'M'
	wrchar	'e'
	wrchar	'#'
	movlw	0x10
lx200_dir_end:
	movwf	retval
	retjmp

lx200_spd0:
	wrchar	'Q'
	wrchar	'e'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	'w'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	'n'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	's'
	wrchar	'#'
	wrchar	':'
	wrchar	'Q'
	wrchar	'#'
	movlw	0x13
	goto	lx200_spd_end
lx200_spd1:
	wrchar	'R'
	wrchar	'C'
	wrchar	'#'
	movlw	0x4
	goto	lx200_spd_end
lx200_spd2:
	wrchar	'R'
	wrchar	'M'
	wrchar	'#'
	movlw	0x4
	goto	lx200_spd_end
lx200_spd3:
	wrchar	'R'
	wrchar	'S'
	wrchar	'#'
	movlw	0x4
lx200_spd_end:
	movwf	retval
	retjmp


	end
