; Embedded software used to drive my hobby boat
; driving 1 DC motor, and servo to steer the rudder
; while receiving commands over HC12 (and transmit confirmation)
; 

		processor	16F628A
		radix		dec
		include		p16f628a.inc
		errorlevel	-302
		__config	_CP_OFF & _LVP_OFF & _BOREN_OFF & _MCLRE_OFF & _PWRTE_ON & _WDT_OFF & _INTOSC_OSC_CLKOUT

; constants output bits of PORTA
led		equ	1

; constants output bits of PORTB
servo   equ 4

cblock 0x20 ; Start of general-purpose RAM 80 bytes max
i
j
receivedData
transmitData
DCounter1 	; contains target position for servo (in ms)
DCounter2   ; follows target position for servo (in ms)
DCounter3   ; temp
DCounter4   ; temp
COUNTER_L   ; Counter for Timer1 for polling 50Hz (servo)
COUNTER_H   ; Counter for Timer1 for polling 50Hz (servo)
Power       ; Power
endc

; reset vector
		org	H'00'
		goto setup

; interrupt vector
		org	H'04'
		retfie
		

setup ; Initialize the microcontroller

    banksel PORTA        ; Select the bank for PORTA
    clrf    PORTA        ; Clear PORTA
    movlw 0x07
	movwf CMCON
	
	bsf STATUS, RP0 ; Switch to Bank 1 
	movlw 0xFC ; Set RA1 and RA0 as output (0), others input 
	movwf TRISA ; Load the value into TRISA 
	bcf STATUS, RP0 ; Switch to Bank 0

    banksel PORTB        ; Select the bank for PORTB
    clrf    PORTB        ; Clear PORTB
    movlw   b'00000010'  ; Set RB1 as input (USART RX pin), and RB2 as output (USART TX PIN), and RB4, RB5, RB6
    banksel TRISB        ; Select the bank for TRISB
    movwf   TRISB
	banksel STATUS
    clrf    STATUS       ; Clear STATUS register

    ; USART initialization
	; BSF OPTION_REG, 7  ; Ensure weak pull-ups in PORTB are disabled
    movlw   d'25'        ; Baud rate = 2400/9600 (with 4MHz clock)
    banksel SPBRG        ; Select the bank for SPBRG
    movwf   SPBRG
    banksel TXSTA        ; Select the bank for TXSTA
    bsf     TXSTA, BRGH  ; set High Baud Rate Select bit
    bcf     TXSTA, SYNC  ; Asynchronous mode
    bsf     TXSTA, TXEN  ; Enable transmitter
    banksel RCSTA        ; Select the bank for RCSTA
    bsf     RCSTA, CREN  ; Enable continuous receive
    bsf     RCSTA, SPEN  ; Enable serial port
	
	bsf		STATUS,RP0				;RAM bank 1
	bcf		PIE1,RCIE				;disable UART interrupt on receive
	bcf     PIE1,TXIE               ;disable UART interrupt on transmit
	bcf		STATUS,RP0				;RAM bank 0

	;
    ; Set up Timer1 for Software Servo PWM
    ;
    CLRF COUNTER_L      ; Clear counter low byte
    CLRF COUNTER_H      ; Clear counter high byte
	BSF STATUS, RP0     ; Bank 1
	BCF T1CON, T1CKPS1  ; Set Timer1 prescaler to 1:1
    BCF T1CON, T1CKPS0
    BCF T1CON, TMR1CS   ; Use internal clock source
    BCF STATUS, RP0     ; Bank 0
    CLRF TMR1H          ; Clear Timer1 high byte
    CLRF TMR1L          ; Clear Timer1 low byte
    BSF T1CON, TMR1ON   ; Start Timer1
	
	;
    ; Set up Timer2 for PWM
    ;
	;BSF STATUS, RP0        ; Switch to Bank 1
    ;MOVLW 0x07             ; Prescaler = 1:16 (Timer2)
    ;MOVWF T2CON            ; Configure Timer2 control register
    ;MOVLW 0xFA             ; PR2 = 250 for ~200 Hz PWM frequency
    ;MOVWF PR2              ; Set Timer2 period register
    ;BCF STATUS, RP0        ; Switch to Bank 0
    ;; Set up CCP1 for PWM
    ;BSF STATUS, RP0        ; Switch to Bank 1
    ;MOVLW b'00001100'      ; CCP1 in PWM mode
    ;MOVWF CCP1CON          ; Configure CCP1 control register
    ;BSF CCP1CON, 4		   ; Set 2LSBs of duty cycle in CCP1CON
	;BSF CCP1CON, 5 
	;BCF STATUS, RP0        ; Switch to Bank 0
    ;; Set initial PWM duty cycle (50%)
    ;MOVLW 0x7F             ; Load high 8 bits of duty cycle
    ;MOVWF CCPR1L           ; Set duty cycle (high 8 bits)
    ;; Start Timer2
    ;BSF T2CON, TMR2ON      ; Enable Timer2
	
	
	;STEP 0: 500Hz, 10/25/50/90% duty cycle, prescale 1:16
	
	;STEP 1: Set the PWM period by writting to the PR2 register
	;-------------------------------------------------------------------------------
 
		banksel PR2 
		movlw d'124' 
		movwf PR2
 
	;STEP 2: Set the PWM duty cycle by writing to CCPR1L & CCP1CON
	;--------------------------------------------------------------------------------------------
 
		banksel CCPR1L
		movlw b'01111011' ;99% LSB=b10
		movlw b'01110000' ;90% LSB=b10
		movlw b'01100100' ;80% LSB=b00
		movlw b'01010111' ;70% LSB=b10
		movlw b'01001011' ;60% LSB=b10
		movlw b'00111110' ;50% LSB=b10
		movlw b'00011111' ;25% LSB=b01
		movlw b'00001100' ;10% LSB=b10
		; default
		movlw b'01010111' ;70% LSB=b10
		movwf Power
		; set idel initially
		movlw b'00000000' ;0% 
		movwf CCPR1L
		banksel CCP1CON
		movlw b'00001100'
		movwf CCP1CON
		BCF CCP1CON, 5 
	    BCF	CCP1CON, 4		   ; Set 2LSBs of duty cycle in CCP1CON
		; set motor off
		BCF PORTB, 5
		BCF PORTB, 6
	
	
 
	;Set the TMR2 prescale value and enable TIMER2 by writing to T2CON
	;----------------------------------------------------------------------------------------------
 		movlw b'00000111'
		movwf T2CON
	
	;init
	banksel DCounter1
	movlw d'100' ; 100*15 usec for midpoint position
	movwf DCounter1 ; contains target
	movwf DCounter2 ; moves toward Dcounter1


MainLoop:
	
T2_OVFL_WAIT:
	;
	; The Timer1 interrupt is disabled, do polling on the overflow bit
	;
	banksel TMR1H
	MOVF TMR1H, W          ; Move the value of TMR1H into W
	XORWF COUNTER_H, W     ; Compare with COUNTER_H
	BTFSS STATUS, Z        ; Check the Zero flag in the STATUS register
	GOTO T2_OVFL_WAIT      ; If Z == 0 (a != b), go to begin
	;
	; the next check only works because COUNTER_L <= 224,
	; so there is sufficient time to do the comparison (real-time)
	; (0,32,64,96,128,160,192,224)*
	; https://ww1.microchip.com/downloads/en/devicedoc/31029a.pdf
	; page 40 on how carry is set
DoA:
	MOVF    TMR1L, W       ; Load value in TMR1L into W
    SUBWF   COUNTER_L, W   ; Compute W = COUNTER_L (b) - TMR1L (a)
    BTFSC   STATUS, C      ; Check Carry flag (C = 1 if a <= b, C = 0 if a > b)
	                       ; Skip next instruction, so exit loop, if C = 0 (a > b) 
    GOTO    DoA            ; C = 1 (a <= b), loop back to DoA
    ;
	; jitter on servo by polling is < ~20 usec, provided rest of procedure <<< 20ms
	; generate pulse 
	;
	banksel PORTA
	bsf PORTB, servo
	call delay_1500usec ; precise timing using DCounter2*15usec 
	banksel PORTA
	bcf PORTB, servo
	;
	; update COUNTER_H|COUNTER_L
	;
    CALL add_20000         ; Add 20000 (usec) to the counter to have exact 50Hz period, and ready for next loop
	;
	;update DCounter2 to follow target DCounter1 (target encoded angle rudder) 
	;
	banksel DCounter1 
    movf DCounter1, W    ; Move the value of DCounter1 into the W register
    subwf DCounter2, W   ; Subtract the W register value from DCounter2 (W = DCounter2 - DCounter1)
    btfss STATUS, Z      ; Skip the next instruction if the result is zero (means a == b)
    btfsc STATUS, C      ; Skip the next instruction if the carry bit is set (means a < b)
    goto END_IF          ; If the previous instruction is skipped, it means a <= b, so skip decrementing a
    incf DCounter2, F    ; Decrement DCounter2 (a--)
END_IF:
    movf DCounter2, W    ; Move the value of DCounter2 into the W register
    subwf DCounter1, W   ; Subtract the W register value from REGISTER_B (W = DCounter1 - DCounter2)
    btfss STATUS, Z      ; Skip the next instruction if the result is zero (means a == b)
    btfsc STATUS, C      ; Skip the next instruction if the carry bit is set (means a < b)
    goto END_IF2         ; If the previous instruction is skipped, it means a <= b, so skip decrementing a
    decf DCounter2, F    ; Decrement DCounter2 (a--)
END_IF2:
    ; Continue with the rest of your program
	
	; DELAY - check if necessary..
	movlw   1
	call    delay ; 770 usec; << 20ms!!
		
	; CHECK RECEIVE
	banksel PIR1
    btfss   PIR1, RCIF   ; Check if data is received
    goto    MainLoop     ; Wait until data is received
	banksel RCREG
	movf    RCREG, W     ; Move received data to W register
    banksel receivedData
	movwf   receivedData ; Store received data in receivedData variable
	;bsf PORTA, led
	
    ;
	; analyse received data from RX
	;
	movf	receivedData, W
	; check against 10110011, power off
    XORLW   0xB3        ; XOR W with 0xB2 (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xB2)
	GOTO	End1		; if not equal, go to end
    ; Code for Equal case
	movlw b'00000000' ;0% LSB=b00
	movwf   CCPR1L
	BCF CCP1CON, 5 
    BCF	CCP1CON, 4		   ; Set 2LSBs of duty cycle in CCP1CON
	bcf	PORTA, led
	goto MainLoop
End1:
	
	movf	receivedData, W
	; check against 10110010, power achteruit
	XORLW   0xB2        ; XOR W with 0xB3 (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xB3)
    GOTO    End2        ; If not equal, go to end
    ; Code for Equal case
	BSF PORTB, 5
	BCF PORTB, 6
	movf Power, W
	movwf   CCPR1L
	; only valid for initial Power
	BSF CCP1CON, 5 
    BCF	CCP1CON, 4		   ; Set 2LSBs of duty cycle in CCP1CON
	bcf	PORTA, led ; led OFF for full speed
	goto MainLoop
End2:

	movf	receivedData, W
	; check against 10110001, power vooruit
    XORLW   0xB1        ; XOR W with 0xB1 (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xB1)
    GOTO    End3        ; If not equal, go to end
    ; Code for Equal case
	BCF PORTB, 5
	BSF PORTB, 6
	movf Power, W
	movwf   CCPR1L
	; only valid for initial Power
	BSF CCP1CON, 5 
    BCF	CCP1CON, 4		   ; Set 2LSBs of duty cycle in CCP1CON
	bcf	PORTA, led ; led OFF for full speed
	goto MainLoop
End3:

	movf	receivedData, W
	; check against 10110101, power achteruit-slow, equals 50%
	XORLW   0xB5        ; XOR W with 0xB5 (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xB5)
    GOTO    End4        ; If not equal, go to end
    ; Code for Equal case
	BSF PORTB, 5
	BCF PORTB, 6
	movlw b'00111110' ;50% LSB=b10
	movwf   CCPR1L
	; only valid for initial Power
	BSF CCP1CON, 5 
    BCF	CCP1CON, 4		   ; Set 2LSBs of duty cycle in CCP1CON
	bsf	PORTA, led
	goto MainLoop
End4:


	movf	receivedData, W
	; check against 10110100, power vooruit-slow, equals 50%
    XORLW   0xB4        ; XOR W with 0xB4 (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xB4)
    GOTO    End5        ; If not equal, go to end
    ; Code for Equal case
	BCF PORTB, 5
	BSF PORTB, 6
	movlw b'00111110' ;50% LSB=b10
	movwf   CCPR1L
	; only valid for initial Power
	BSF CCP1CON, 5 
    BCF	CCP1CON, 4		   ; Set 2LSBs of duty cycle in CCP1CON
	bsf	PORTA, led
	goto MainLoop
End5:

	
	; rudder aansturing
	
	movf	receivedData, W
	; 1011-1001 , hoek == 30
	XORLW   0xB9        ; XOR W with 0xB9 (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xB9)
    GOTO    EndR0       ; If not equal, go to end
    ; Code for Equal case
	movlw d'55' ; *15 us
	movwf DCounter1
	goto  AckReceive
EndR0:
	
	movf	receivedData, W
	; 1011-1010 , hoek == 45
	XORLW   0xBA        ; XOR W with 0xBA (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xBA)
    GOTO    EndR1       ; If not equal, go to end
    ; Code for Equal case
	movlw d'70' ; *15 us
	movwf DCounter1
	goto  AckReceive
EndR1:

	movf	receivedData, W
	; 1011-1011 , hoek == 90 
	XORLW   0xBB        ; XOR W with 0xBB (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xBB)
    GOTO    EndR2       ; If not equal, go to end
    ; Code for Equal case
	movlw d'100' ; *15 us
	movwf DCounter1
	goto  AckReceive
EndR2:
	
	movf	receivedData, W
	; 1011-1100 , hoek == 135
	XORLW   0xBC        ; XOR W with 0xBC (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xBC)
    GOTO    EndR3       ; If not equal, go to end
    ; Code for Equal case
	movlw d'130' ; *15 us
	movwf DCounter1
	goto  AckReceive
EndR3:
	
	movf	receivedData, W
	; 1011-1101 , hoek == 150
	XORLW   0xBD        ; XOR W with 0xBD (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xBD)
    GOTO    EndR4       ; If not equal, go to end
    ; Code for Equal case
	movlw d'145' ; *15 us
	movwf DCounter1
	goto  AckReceive
EndR4:
	
	movf	receivedData, W
	; check against 10110111, power up
	XORLW   0xB7        ; XOR W with 0xB7 (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xB7)
    GOTO    EndR5        ; If not equal, go to end
    ; Code for Equal case
	; Set Power to 99% equals 01111011-10
	movlw b'01111011' ;99% LSB=b10
	movwf Power
	goto  AckReceive
EndR5:
	
	movf	receivedData, W
	; check against 10111111, power reset to default
	XORLW   0xBF        ; XOR W with 0xBF (Result in W, flags affected)
    BTFSS   STATUS,Z    ; Check if the Zero flag is set (W == 0xB7)
    GOTO    EndR6        ; If not equal, go to end
    ; Code for Equal case
	; Reset Power
	movlw b'01010111' ;70% LSB=b10
	movwf Power
	goto  AckReceive
EndR6:
	;
	;
	;
	;goto MainLoop ; to show only recognized commands
	;
	;
	; send message back to controller after having received a [valid] rudder command
	; this to avoid collision with receiving further messages on the half-duplex HC12
	; the message format is defined in the controller software
	;
AckReceive:
	movlw '<'
	call    SendByte
	movlw 'B'
	call    SendByte
	movf receivedData, W
	call    SendByte
	movlw '>'
	call    SendByte
	
    goto MainLoop

SendByte:
		;banksel PIR1
	    BTFSS   PIR1, TXIF    ; Check if TX buffer is ready (TXIF = 1)
	    GOTO    SendByte      ; If not ready, wait here
	    MOVWF   TXREG         ; Move WREG (data) to TX register
	    RETURN
		
Timer0_ISR:
    banksel INTCON
    bcf INTCON, T0IF  ; Clear Timer0 interrupt flag
    
    movlw 0x64       ; Load the Timer0 value for 20ms delay
    movwf TMR0
    
    ; Do something (toggle a bit, for example)
    banksel PORTA
	movlw 0x01 ; Load W with the value to XOR (bit 0) 
	xorwf PORTA, 1 ; XOR W with PORTA, result stored back in PORTA
	
    retfie

; -----------------delay-----------------------------------
; Software delay variable with w
; Data:      w
; Variables: i, j

delay	banksel i
		movwf	i		;1 us
		clrf	j		;1 us

dloop		decfsz	j, F		;(j-1) x 1 us + 2 us	(j==0 -> j=256)
		goto	dloop		;(j-1) x 2 us

		decfsz	i, F		;(i-1) x 1 us + 2 us
		goto	dloop		;(i-1) x 2 us

		return			;2 us

; cycles =     (3j-1)*i + (3i-1) + 4
; j=256 ->  (3*256-1)*i + (3i-1) + 4
; total  = 770*i + 3


delay_15usec
	MOVLW 0X01
	;banksel DCounter4
	MOVWF DCounter4
LOOP_15usec:
	DECFSZ DCounter4, 1
	GOTO LOOP_15usec
	NOP
	NOP
	RETURN

delay_1500usec:
	banksel DCounter2
	movf DCounter2, W
	movwf DCounter3
LOOP_1500usec:
	call delay_15usec
	NOP
	NOP
	DECFSZ DCounter3, 1
	GOTO LOOP_1500usec
	RETURN


add_20000:
	    ; Add Low Byte
		banksel COUNTER_L
	    MOVLW LOW(0x4E20)   ; Load low byte of 20,000
	    ADDWF COUNTER_L, F  ; Add to low byte of counter
	    ;banksel STATUS
		BTFSC STATUS, C     ; Check if carry is set
	    INCF COUNTER_H, F   ; Increment high byte if carry occurred

	    ; Add High Byte
	    MOVLW HIGH(0x4E20)  ; Load high byte of 20,000
	    ADDWF COUNTER_H, F  ; Add to high byte of counter
		
	    RETURN              ; Return to polling loop
	
		end

		;
		;
		;
		;
		;
		;
		;
		;
		
		Setting of PWM in CCP is based on 2 formulas:

		1)  Period= [PR2 + 1] x 4 x Tosc x TMR2-prescale-value

		2)  Duty cycle= (CCPR1L : CCP1CON <5:4>) x Tosc x TMR2-prescale-value

		1) can be rearranged to:

		       PR2=  Fosc
		            ------------------------------  - 1
		             Fpulse x Timer2-prescale x 4 

		       where Fosc= frequency of crystal, Fpulse= frequency of pulse

		2) can be rearranged to :

		       CCPR1L: CCP1CON <5:4> = Duty cycle x Fosc
		                              --------------------
		                               TMR2-prescale-value

		       where CCPR1L: CCP1CON<5:4> means- value in CCPR1L register and 
		                                         bit 4 and bit 5 of CCP1CON register



		We have to set the values in PR2 and CCPR1L and CCP1CON.

		Example:
 
		Pulses we want= 2 khz, 
		Fosc= 4 Mhz
		Duty cycle= 50%
		Scale= 4

		     PR2= 4000000
		          ----------------  - 1
		           2000 x 4 x 4

		         = 124


		      Duty cycle = 0.5
		                   -------------- = 0.00025  (50% of period of 2khz)
		                    2000    

      
		      CCPR1L: CCP1CON<5:4>= 0.00025 X 4000000
		                            -------------------
		                                    4

		                          = 250

		                          Now convert this number to binary= 11111010

		       CCP1CON bit 4= 0, bit 5=1
		       CCPR1L= 00111110

		       (nearly finish)

		Once we have these values, the rest is easy:

		* Configure RB3 as output
		* Configure CCP module for PWM operation in CCP1CON register by setting
		  1 to bit 2 and bit 3
		* Set up TMR2 prescale value and enable TMR2 by writing to T2CON:
		  in this case- 00000101

		That's it.
                    

		Frequency
		(1) PR2=[(Fosc)/(4∗TMR2Prescale∗PWMFrequency)]−1
		(2) PR2=[4Mhz/(4∗16∗500Hz)]–1=124
		Duty Cycle
		(3) PWMxDCH:PWMxDCL=4∗(PR2+1)∗DutyCycleRatio
		(4) PWMxDCH:PWMxDCL=4∗(124+1)∗0.5=250
		(5) PWMxDCH=00111110
		(6) PWMxDCL=10


		Met 90% duty cycle:
		PWMxDCH:PWMxDCL==450
		0111-0000 10
		
		Met 80% duty cycle:
		PWMxDCH:PWMxDCL==400
		0110-0100 00

		Met 25% duty cycle:
		PWMxDCH:PWMxDCL==125
		00001100-10

		Met 10% duty cycle:
		PWMxDCH:PWMxDCL==50
		00001100-10

		Met 70% duty cycle:
		PWMxDCH:PWMxDCL==350
		01010111-10

		Met 99% duty cycle:
		PWMxDCH:PWMxDCL==494
		01111011-10
		
		Met 60% duty cycle:
		PWMxDCH:PWMxDCL==302
		01001011-10

		Andersom: 
		PR2=249
		(1) PR2=[(Fosc)/(4∗TMR2Prescale∗PWMFrequency)]−1
		PWMF = Fosc/(4*PRE*(PR2+1))
		= 4000000/(4*250*16)
		= 250
		(3)
		DutyCycleRatio = PWMxDC/(4*(PR2+1))
		= 48/4000 == 1,2 %



		
		en voorbeeld delay
		;bsf PORTA, led
		;movlw	234		;180183us
		;call	delay
		;movlw	234		;180183us
		;call	delay
		;movlw	181		;139373us -> total = 499739us
		;call	delay
		;bcf PORTA, led
	
