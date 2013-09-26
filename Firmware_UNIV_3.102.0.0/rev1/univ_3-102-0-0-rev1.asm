;==============================================================================
;    HAPCAN - Home Automation Project (http://hapcan.com)
;    Copyright (C) 2012 Jacek Siwilo
;
;    This program is free software: you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation, either version 3 of the License, or
;    (at your option) any later version.
;
;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.
;
;    You should have received a copy of the GNU General Public License
;    along with this program.  If not, see <http://www.gnu.org/licenses/>.
;==============================================================================
;
;    Filename:              univ_3-102-0-0-rev1.asm
;    Associated diagram:    univ_3-102-0-x.sch                                         
;    Author:                Jacek Siwilo                           
;                                                                   
;==============================================================================
;       Revision History
;==============================================================================
;Rev:	Date:       Details:
;0      04.2012	    Original version                                   
;1      07.2012     Added Tibbo reset RST with MD button pressed     
;==============================================================================
;===  FIRMWARE DEFINITIONS  ===================================================
;==============================================================================
	#define		ATYPE	.102			    ;application type
	#define		AVERS	.00			    ;application version
	#define		FVERS	.00			    ;firmware version

	#define		FREV1	.00			    ;firmware revision
	#define		FREV2	.01			   ;firmware revision
;==============================================================================
;===  NEEDED FILES  ===========================================================
;==============================================================================
	LIST P=18F26K80                         ;directive to define processor
	#include <P18F26K80.INC>                ;processor specific variable definitions
	#include "univ_3-102-0-0-rev1_cfg.inc"  ;firmware config
;==============================================================================
;===  FIRMWARE CHECKSUM  ======================================================
;==============================================================================
FIRMCHKSM   code    0x001000
    DB      0x6D,0x19,0x65,0xFF,0xFF,0xFF,0xFF,0xFF
;            |    |    |_________________________________________________ FSum0
;            |    |______________________________________________________ FSum1
;            |___________________________________________________________ FSum2
;==============================================================================
;===  FIRMWARE ID  ============================================================
;==============================================================================
FIRMID      code    0x001010
    DB      0x30, 0x00, 0x03,ATYPE,AVERS,FVERS,FREV1,FREV2
;            |     |     |     |     |     |     |_____|_____ firmware revision
;            |     |     |     |     |     |__________________ firmware version 
;            |     |     |     |     |_____________________ application version
;            |     |     |     |______________________________ application type
;            |     |     |________________________________ hardware version '3'
;            |_____|______________________________________ hardware type 'UNIV'
;==============================================================================
;===  MOVED VECTORS  ==========================================================
;==============================================================================
;PROGRAM RESET VECTOR
FIRMRESET   code    0x1020
		goto	Main
;PROGRAM HIGH PRIORITY INTERRUPT VECTOR
FIRMHIGHINT code    0x1030
		call	HighInterrupt
        retfie
;RECEIVED MESSAGE VECTOR
FIRMLOWINT  code    0x1040
		call	LowInterrupt
        retfie

;==============================================================================
;===  FIRMWARE STARTS  ========================================================
;==============================================================================
FIRMSTART   code    0x001050
;------------------------------------------------------------------------------
;---  LOW PRIORITY INTERRUPT  -------------------------------------------------
;------------------------------------------------------------------------------
LowInterrupt
        movff	STATUS,STATUS_LOW       ;save STATUS register
        movff	WREG,WREG_LOW           ;save working register
        movff	BSR,BSR_LOW             ;save BSR register
        movff	FSR0L,FSR0L_LOW         ;save other registers used in high int
        movff	FSR0H,FSR0H_LOW
        movff	FSR1L,FSR1L_LOW
        movff	FSR1H,FSR1H_LOW

    ;main firmware ready flag
        movlb   0x1
        btfss   FIRMFLAG,0
        bra     ExitLowInterrupt       ;main firmware is not ready yet
    ;CAN buffer
        movlb   0x1
        btfsc   CANFULL,0  
        call    WriteToCanRxFIFO		;receive and save message in RAM

ExitLowInterrupt
        movff	BSR_LOW,BSR             ;restore BSR register
        movff	WREG_LOW,WREG           ;restore working register
        movff	STATUS_LOW,STATUS       ;restore STATUS register
        movff	FSR0L_LOW,FSR0L         ;restore other registers used in high int
        movff	FSR0H_LOW,FSR0H
        movff	FSR1L_LOW,FSR1L
        movff	FSR1H_LOW,FSR1H
    return

;------------------------------------------------------------------------------
;---  HIGH PRIORITY INTERRUPT  ------------------------------------------------
;------------------------------------------------------------------------------
HighInterrupt
        movff	STATUS,STATUS_HIGH      ;save STATUS register
        movff	WREG,WREG_HIGH          ;save working register
        movff	BSR,BSR_HIGH            ;save BSR register
        movff	FSR0L,FSR0L_HIGH        ;save other registers used in high int
        movff	FSR0H,FSR0H_HIGH
        movff	FSR1L,FSR1L_HIGH
        movff	FSR1H,FSR1H_HIGH

    ;main firmware ready flag
        movlb   0x1
        btfss   FIRMFLAG,0
        bra     ExitHighInterrupt       ;main firmware is not ready yet
    ;uart
        movlb   0x1
        tstfsz  UARTCNT                 ;check if uart received anything
        call	UartReceived
    ;timer2
		btfsc	PIR1,TMR2IF		    	;Timer2 interrupt? (8.192ms)
		rcall	Timer2Interrupt

ExitHighInterrupt
        movff	BSR_HIGH,BSR            ;restore BSR register
        movff	WREG_HIGH,WREG          ;restore working register
        movff	STATUS_HIGH,STATUS      ;restore STATUS register
        movff	FSR0L_HIGH,FSR0L        ;restore other registers used in high int
        movff	FSR0H_HIGH,FSR0H
        movff	FSR1L_HIGH,FSR1L
        movff	FSR1H_HIGH,FSR1H
    return

;------------------------------------------------------------------------------
; Routine:			TIMER 2 INTERRUPT
;------------------------------------------------------------------------------
; Overview:			8192us periodical interrupt
;------------------------------------------------------------------------------
Timer2Interrupt
        rcall   Timer2Init              ;restart timer
        rcall   JumperStatus            ;read jumper
    return
;-----------------------
Timer2Init
        movlb   0xF
        bcf     PMD1,TMR2MD             ;enable timer 2                  
		clrf	TMR2                    ;set 8.192ms
		movlw	b'01111111'				;start timer, prescaler=16, postscaler=10
		movwf	T2CON
        bsf     IPR1,TMR2IP             ;high priority for interrupt
		bcf		PIR1,TMR2IF			    ;clear timer's flag
		bsf		PIE1,TMR2IE			    ;interrupt on
    return 
;-----------------------
JumperStatus                            ;Transfers jumper status to MD line of Tibbo
        bcf     LATC,0                  ;MD latch to low
        btfss   PORTB,7                 ;skip if jumper high (open)
        bcf     TRISC,0                 ;jumpel low, so set MD as output
        btfsc   PORTB,7                 ;skip if jumper low (closed)
        bsf     TRISC,0                 ;jumper high, so set MD as input (pull up in Tibbo)     
    return

;==============================================================================
;===  MAIN PROGRAM  ===========================================================
;==============================================================================
Config0  EQU R0     ;translation enabled byte (0x00 - translation disabled)

Main:
    ;initiate ports
        ;PORT A leave as it is with analog inputs
        ;PORT B
        movlb   0xF
        movlw   b'10000000'             ;pull up on interrupt pins 
        movwf   WPUB
        bcf     INTCON2,RBPU            ;enable pull up
        bcf     TRISB,TRISB6            ;unconnected RB6; others with pullups
        ;PORT C leave as it is with digital inputs with pullup from Tibbo
    ;Tibbo reset RST with MD button pressed
        call    Tibbo_MD_RST            ;Tibbo reset RST with MD button pressed
    ;node configuration
        call    FIFOInitialization      ;FIFO buffers init
        call    ReadEepromConfig        ;node config from eeprom
    ;initiate timer 2
        call    Timer2Init              ;8.192ms periodical interrupt
    ;firmware started   
        movlb   0x1
        bsf     FIRMFLAG,0              ;set flag "firmware started and ready for interrupts"

Loop:
        clrwdt
        call    UartReceiveProc         ;check if any msg received on UART
        call    CanReceiveProc          ;check if any msg received on CAN
	bra		Loop

;-----------------------
UartReceiveProc
        tstfsz  URXCNT                  ;any msg received?
        bra     $ + 4                   ;yes
    return                              ;no
;make sure there was not writing to UART RX FIFO during reading
        movlw   URXFIFOSIZE             ;is buffer full?
        cpfseq  URXCNT
        bra     $ +.20                  ;no, so do not worry and go to ReadFromUartRxFIFO
        bcf     FIFOFLAG,UartRxFifoWrite;clear "Write to UART RX FIFO Buffer appeared" flag
        call    ReadFromUartRxFIFO
        btfss   FIFOFLAG,UartRxFifoWrite;skip if "Write to UART RX FIFO Buffer appeared"
        call    UartMsgTransform
        call    CanTransmit
    return                              ;exit because new message was written in the same buffer possition
        call    ReadFromUartRxFIFO
        call    UartMsgTransform
        call    CanTransmit
    return
;-----------------------
CanReceiveProc
        tstfsz  CRXCNT                  ;any msg received?
        bra     $ + 4                   ;yes
    return                              ;no
;make sure there was not writing to CAN RX FIFO during reading
        movlw   CRXFIFOSIZE             ;is buffer full?
        cpfseq  CRXCNT
        bra     $ +.20                  ;no, so do not worry and go to ReadFromCanRxFIFO
        bcf     FIFOFLAG,CanRxFifoWrite;clear "Write to CAN RX FIFO Buffer appeared" flag
        call    ReadFromCanRxFIFO
        btfss   FIFOFLAG,CanRxFifoWrite;skip if "Write to CAN RX FIFO Buffer appeared"
        call    CanMsgTransform
        call    UartTransmit
    return                              ;exit because new message was written in the same buffer possition
        call    ReadFromCanRxFIFO
        call    CanMsgTransform
        call    UartTransmit
    return

;------------------------------------------------------------------------------
; Routine:			TIBBO RST + MD
;------------------------------------------------------------------------------
; Overview:			Tibbo gest into programming mode
;                   (rescue option when tibbo not visible in Device Explorer)
;------------------------------------------------------------------------------
Tibbo_MD_RST                            ;Tibbo reset RST with MD button pressed
        btfsc   PORTB,7                 ;skip if jumper low (closed)
    return
        bcf     LATC,0                  ;MD latch to low
        bcf     LATC,1                  ;RST latch to low
        bcf     TRISC,0                 ;set MD as output - MD goes low
        bcf     TRISC,1                 ;set RST as output - start reset
		setf	R0	                    ;wait 25ms
TibboL2	setf	R1
TibboL1	decfsz	R1
		bra		TibboL1
		decfsz	R0
		bra		TibboL2
        bsf     TRISC,1                 ;set RST as intput - stop reset
        clrwdt        
        bra     $ - 2                   ;stay in loop - manual restart needed 

;------------------------------------------------------------------------------
; Routine:			READ CONFIG EEPROM
;------------------------------------------------------------------------------
; Overview:			Reads config from eeprom
;------------------------------------------------------------------------------
ReadEepromConfig
        bcf     EECON1,EEPGD            ;access data EEPROM memory
        bcf     EECON1, CFGS            ;access data FLASH or EEPROM
        movlw   0x08                    ;start reading from 00h EEPROM address
        movwf   EEADR
        lfsr    FSR0,Config0            ;start writing to address of Config0
        movlw   0x01                    ;x successive registers
ReadConfig
        bsf     EECON1,RD               ;read data from EEPROM
        movff   EEDATA,POSTINC0         ;move EEPROM DATA to register indicated by FSR0 and increment FSR0
        incf    EEADR                   ;increment address of EEPROM
        decfsz  WREG                    ;all bytes read?
        bra     ReadConfig              ;not yet - read more
    return

;------------------------------------------------------------------------------
; Routine:			UART received
;------------------------------------------------------------------------------
; Overview:			Takes message from UART buffer and sends to UART RX FIFO
;------------------------------------------------------------------------------
UartReceived
	;checking message
        movlb	0x1
        tstfsz  UARTOVF                 ;buffer overflow?
        bra     ExitUartReceived        ;yes, so exit
        tstfsz  UARTCNT                 ;any byte received?
        call	WriteToUartRxFIFO       ;yes, so go write fifo
ExitUartReceived
        movlb	0x1
        clrf	UARTCNT  				;clear uart received flag
    return

;------------------------------------------------------------------------------
; Routine:          UART TRANSMIT
;------------------------------------------------------------------------------
; Overview:			Load message from FIFO to UART
;------------------------------------------------------------------------------
UartTransmit
        movlb   0x0
        tstfsz  CRXtFIFOCNT             ;anything to transmit? 
        bra     $ + 4
    return                              ;no

        tstfsz  CRXtFIFOTRSF            ;pure CAN message or buffer transformed?
        bra     UartTransmitStart       ;transformed

        ;prepare pure CAN message
        movlw	0xAA					;start byte
        movwf	CRXtFIFO0
        movlw	0xA5					;stop byte
        movwf	CRXtFIFO14
        clrf	CRXtFIFO13              ;checksum
        lfsr    FSR0,CRXtFIFO1          ;point at first reg
        movlw   .12
        movwf   CRXtFIFO15              ;use temp reg
        movf    POSTINC0,W              ;read next reg
        addwf   CRXtFIFO13              ;add reg to temp checksum
        decfsz  CRXtFIFO15              ;read all 12 regs
        bra     $ - 6                   

UartTransmitStart
		bsf		TXSTA,TXEN				;enable uart transmission
		lfsr	FSR1,CRXtFIFO0			;point at first register of CAN RX temp FIFO buffer
UartTransmitLp
		btfss	PIR1,TX1IF				;check if TXREG if empty and ready to be loaded
		bra		$ - 2				
		movff	POSTINC1,TXREG			;move data from can receive buffer to uart transmit buffer
		decfsz	CRXtFIFOCNT    			;all bytes read?
		bra		UartTransmitLp			;not yet - read more
	return

;------------------------------------------------------------------------------
; Routine:              FIFO INITIALIZE
;------------------------------------------------------------------------------
; Overview:             Initialization of Received & Transmit FIFO buffers
;------------------------------------------------------------------------------
FIFOInitialization

URXFIFOSIZE    EQU     .42              ;size of UART RX FIFO buffer
CRXFIFOSIZE    EQU     .42              ;size of CAN RX FIFO buffer

        clrf    URXTOP                  ;clear stack top UART RX
        clrf    URXCNT                  ;clear stack counter UART RX
        clrf    URXCNTMX                ;clear max stack counter UART RX
        clrf    CRXTOP                  ;clear stack top CAN RX
        clrf    CRXCNT                  ;clear stack counter CAN RX
        clrf    CRXCNTMX                ;clear max stack counter CAN RX
        clrf    FIFOFLAG                ;clear fifo flags
    ;clear buffer
        lfsr	FSR0,0x200
        movlw   .6                      ;3 banks
        movwf   URXFIFOR1
        setf    URXFIFOR0
ClearFIFO2
        movlw   0x00
ClearFIFO1
        movff   URXFIFOR0,POSTINC0
        decfsz  WREG
        bra     ClearFIFO1
        decfsz  URXFIFOR1
        bra     ClearFIFO2
    return

;------------------------------------------------------------------------------
; Routine:          WRITE MESSAGE TO UART RX FIFO
;------------------------------------------------------------------------------
; Overview:         UART Received message is saved to FIFO buffer
;------------------------------------------------------------------------------
WriteToUartRxFIFO
        movlb   0x0                         ;point at FIFO registers
    ;point at destination address (top of FIFO)
		lfsr	FSR1,0x3E0					;point at first reg in FIF0 (0x400-32)
        movff   URXTOP,URXFIFOR0            ;copy stack top
        incf    URXFIFOR0                      
        movlw   .32
AddRXFIFOAdr:
        addwf   FSR1L                       ;add multiply 32 to FSR1
        bnc     $ + 4                       ;go to next ram bank if overflow
        incf    FSR1H
        decfsz  URXFIFOR0
        bra     AddRXFIFOAdr

    ;copy to FIFO
		lfsr	FSR0,0x110					;point at source (first register in receive UART buffer bRB0)
		movlw	.32 						;read 32 successive registers
        movwf   URXFIFOR0                      
CopyToUartRxFIFOLoop:
		movff	POSTINC0,POSTINC1			;move data from receive buffer to destination buffer
		decfsz	URXFIFOR0                   ;all 32 bytes read?
		bra		CopyToUartRxFIFOLoop		;not yet - read more

    ;update stack top
        incf    URXTOP                      ;increment top of stack
        movlw   URXFIFOSIZE                 ;stack depth
        xorwf   URXTOP,W                    ;overflow
        bnz     $ + 4                       ;no
        clrf    URXTOP                      ;yes, so set top to zero
    ;update stack counter
        movlw   URXFIFOSIZE                 ;stack depth
        xorwf   URXCNT,W                    ;overflow?
        bz      $ + 6                       ;yes
        incf    URXCNT                      ;no, so increment stack counter
        bra     $ + 6
    ;update stack counter max value
        infsnz  URXCNTMX                    ;indicate overflow ;over 0xFF?
        decf    URXCNTMX                    ;yes, so go back to 0xFF
        movf    URXCNT,W
        cpfsgt  URXCNTMX                    ;compare, skip if RXCNTMX>RXCNT
        movwf   URXCNTMX                    ;save cnt as max value

ExitWriteToUartRxFIFO
        bsf     FIFOFLAG,UartRxFifoWrite    ;set "Write to UART RX Buffer appeared" flag
    return                  	        ;end of Save msg to UART FIFO procedure

;------------------------------------------------------------------------------
; Routine:          READ MESSAGE FROM UART RX FIFO
;------------------------------------------------------------------------------
; Overview:         Reads message from UART RX FIFO buffer to temp UART buffer
;------------------------------------------------------------------------------
ReadFromUartRxFIFO
        movlb   0x0                         ;point at FIFO registers
        tstfsz  URXCNT                      ;any on stack?
        bra     $ + 4                       ;yes
    return                                  ;stack empty
    ;find first reg in FIFO              
        movf    URXCNT,W                       
        subwf   URXTOP,W                    ;if(RXTOP-RXCNT>0)
        bnn     $ + 4                       ;Adr = RXTOP-RXCNT      
        addlw   URXFIFOSIZE                 ;else: RXTOP-RXCNT+RXFIFOSIZE
    ;get address of first FIFO reg
        movwf   URXFIFOR1          
		lfsr	FSR1,0x3E0					;point at first reg in FIF0 (0x400-32)  
        incf    URXFIFOR1
        movlw   .32   
GetRXFIFOAdr:
        addwf   FSR1L                       ;add multiply 32 to FSR1
        bnc     $ + 4                       ;go to next ram bank if overflow
        incf    FSR1H
        decfsz  URXFIFOR1
        bra     GetRXFIFOAdr

    ;copy from FIFO
		lfsr	FSR0,URXtFIFO0				;point at first register in RXtemp FIFO buffer
		movlw	.32 						;read 32 successive registers              
CopyFromRXFIFOLoop:
		movff	POSTINC1,POSTINC0			;move data from receive buffer to destination buffer
		decfsz	WREG                        ;all 32 bytes read?
		bra		CopyFromRXFIFOLoop			;not yet - read more

    ;update FIFO stack top
        ;top stack stays unchanged
    ;update FIFO stack counter
        decf    URXCNT

ExitReadFromUartRxFIFO
    return                  	            ;end of read msg from FIFO procedure

;------------------------------------------------------------------------------
; Routine:          UART MESSAGE TRANSFORM
;------------------------------------------------------------------------------
; Overview:         Transforms from ASCII to HEX if found in config
;------------------------------------------------------------------------------
UartMsgTransform
        movlb   0x0
;check if translation in enabled
        tstfsz  Config0        
        bra     $ + 4
        bra     ExitUartMsgTransform    ;translation disabled

;get box address
        setf    UBOXADR                 ;first box minus 1
UARTNextBOX	
		incf 	UBOXADR,F			    ;point at next BOX
		movlw	.128 				    ;check if it is last box
		cpfseq	UBOXADR  				;skip if BOXADR is euqal WREG	
		bra		$ + 4
	bra		ExitUartMsgTransform        ;all boxes done

;read 48 regs from flash
		clrf	TBLPTRU                 ;set address of flash
		movlw	.48 				    ;start writing form first register of BOXxx
        mulwf   UBOXADR                 ;get TBLPTR value 48xBOXADR (range 0x008000 - 0x0097D0)
        movff   PRODH,TBLPTRH
        movff   PRODL,TBLPTRL  
        movlw   0x80                    ;add 0x008000 to TBLPTR
        addwf   TBLPTRH
		lfsr	FSR0,UBOX0  			;start writing to...
		movlw	.48                     ;48 registers			
UARTCopyData
		tblrd*+					    	;read and move to TABLAT and increment address
		movff	TABLAT,POSTINC0		    ;move flash data to register indicated by FSR0 and increment FSR0
		decfsz	WREG				    ;all Bank bytes read?
		bra		UARTCopyData

;check if BOX is enabled
        tstfsz  UBOX30
        bra     $ + 4
		bra		UARTNextBOX			    ;box disabled

;Compare box to receive buffer
        movf    UBOX31,W                ;number of bytes the same in box and message received?
        xorwf   URXtFIFOCNT,W
		bnz		UARTNextBOX             ;no, so do not compare
        lfsr    FSR0,UBOX0              ;get first byte of box (UART part)
        lfsr    FSR1,URXtFIFO0          ;get first byte of UART temp FIFO buffer
UARTCompareMsgs
        movf    POSTINC0,W              ;compare values in FSR0 and FSR1
        xorwf   POSTINC1,W
		bnz		UARTNextBOX             ;not the same
        decfsz  UBOX31                  ;compare each byte in the message
        bra     UARTCompareMsgs

;load CAN message from box to temp UART FIFO buffer
        lfsr    FSR0,UBOX32             ;get first byte of box (CAN part)
        lfsr    FSR1,URXtFIFO1          ;get first byte of temp FIFO buffer
		movlw	.12                     ;12 registers			
UARTCopyCANmsg
		movff	POSTINC0,POSTINC1
		decfsz	WREG				    ;all Bank bytes read?
		bra		UARTCopyCANmsg

;add start, checksum, stop etc
        movlw   0xAA                    ;start byte
        movwf   URXtFIFO0
        movlw   0xA5                    ;stop byte
        movwf   URXtFIFO14
        clrf    URXtFIFO13              ;checksum
        lfsr    FSR0,URXtFIFO1          ;point at first reg
        movlw   .12
        movwf   URXtFIFO15              ;use temp reg
        movf    POSTINC0,W              ;read next reg
        addwf   URXtFIFO13              ;add reg to temp checksum
        decfsz  URXtFIFO15              ;read all 12 regs
        bra     $ - 6                   
        movlw   .15                     ;update message counter
        movwf   URXtFIFOCNT

ExitUartMsgTransform
    return

;------------------------------------------------------------------------------
; Routine:          WRITE MESSAGE TO CAN RX FIFO
;------------------------------------------------------------------------------
; Overview:         CAN Received message is saved to temp FIFO buffer
;------------------------------------------------------------------------------
WriteToCanRxFIFO
        movlb   0x0                         ;point at FIFO registers
    ;point at destination address (top of FIFO)
		lfsr	FSR1,0x1F4					;point at first reg in FIF0 (0x200-12)
        movff   CRXTOP,CRXFIFOR0            ;copy stack top
        incf    CRXFIFOR0                      
        movlw   .12
AddCRXFIFOAdr:
        addwf   FSR1L                       ;add multiply 32 to FSR1
        bnc     $ + 4                       ;go to next ram bank if overflow
        incf    FSR1H
        decfsz  CRXFIFOR0
        bra     AddCRXFIFOAdr

    ;copy to FIFO
		lfsr	FSR0,0x101					;point at source
		movlw	.12 						;read 12 successive registers
        movwf   CRXFIFOR0                      
CopyToCRXFIFOLoop:
        movlw   .8
        xorwf   CRXFIFOR0,W                 ;first 4 regs passed?
        bnz     $ + 4                       ;no
        incf    FSR0L                       ;yes, so pass RXBDLC reg
		movff	POSTINC0,POSTINC1			;move data from receive buffer to destination buffer
		decfsz	CRXFIFOR0                   ;all 12 bytes read?
		bra		CopyToCRXFIFOLoop			;not yet - read more

    ;update stack top
        incf    CRXTOP                      ;increment top of stack
        movlw   CRXFIFOSIZE                 ;stack depth
        xorwf   CRXTOP,W                    ;overflow
        bnz     $ + 4                       ;no
        clrf    CRXTOP                      ;yes, so set top to zero
    ;update stack counter
        movlw   CRXFIFOSIZE                 ;stack depth
        xorwf   CRXCNT,W                    ;overflow?
        bz      $ + 6                       ;yes
        incf    CRXCNT                      ;no, so increment stack counter
        bra     $ + 6
    ;update stack counter max value
        infsnz  CRXCNTMX                    ;indicate overflow ;over 0xFF?
        decf    CRXCNTMX                    ;yes, so go back to 0xFF
        movf    CRXCNT,W
        cpfsgt  CRXCNTMX                    ;compare, skip if TXCNTMX>TXCNT
        movwf   CRXCNTMX                    ;save cnt as max value

ExitWriteToCanRxFIFO
        bsf     FIFOFLAG,CanRxFifoWrite    ;set "Write to CAN RX Buffer appeared" flag
    return                  	        ;end of Save msg to FIFO procedure

;------------------------------------------------------------------------------
; Routine:          READ MESSAGE FROM CAN RX FIFO
;------------------------------------------------------------------------------
; Overview:         Reads message from CAN RX FIFO buffer and saves in CAN temp buffer
;------------------------------------------------------------------------------
ReadFromCanRxFIFO
        movlb   0x0                     ;point at FIFO registers
        tstfsz  CRXCNT                  ;any on stack?
        bra     $ + 4                   ;yes
    return                              ;stack empty
    ;find first reg in FIFO              
        movf    CRXCNT,W                       
        subwf   CRXTOP,W                ;if(TXTOP-TXCNT>0)
        bnn     $ + 4                   ;Adr = TXTOP-TXCNT      
        addlw   CRXFIFOSIZE             ;else: TXTOP-TXCNT+TXFIFOSIZE
    ;get address of first FIFO reg
        movwf   CRXFIFOR1          
		lfsr	FSR1,0x1F4				;point at first reg in FIF0 (0x200-12)  
        incf    CRXFIFOR1
        movlw   .12  
GetCRXFIFOAdr:
        addwf   FSR1L                   ;add multiply 32 to FSR1
        bnc     $ + 4                   ;go to next ram bank if overflow
        incf    FSR1H
        decfsz  CRXFIFOR1
        bra     GetCRXFIFOAdr

    ;copy from FIFO
		lfsr	FSR0,CRXtFIFO1			;point at first register of CAN RX FIFO temp buffer
		movlw	.12 					;read 12 successive registers  
CopyFromCRXFIFOLoop:
		movff	POSTINC1,POSTINC0		;move data from receive buffer to destination buffer
		decfsz	WREG                    ;all 12 bytes read?
		bra		CopyFromCRXFIFOLoop		;not yet - read more

        clrf    CRXtFIFOTRSF 		    ;clear flag "buffer transformed"
		movlw	.15                     ;update message counter
        movwf   CRXtFIFOCNT

    ;update FIFO stack top
        ;top stack stays unchanged
    ;update FIFO stack counter
        decf    CRXCNT

ExitReadFromCanRxFIFO
    return                  	        ;end of read msg from FIFO procedure

;------------------------------------------------------------------------------
; Routine:          CAN MESSAGE TRANSFORM
;------------------------------------------------------------------------------
; Overview:         Transforms from HEX to ASCII or different HEX if found in config
;------------------------------------------------------------------------------
CanMsgTransform
        movlb   0x0
;check if translation in enabled
        tstfsz  Config0        
        bra     $ + 4
        bra     ExitUartMsgTransform    ;translation disabled

;get box address
        setf    CBOXADR                 ;first box minus 1
CanNextBOX	
		incf 	CBOXADR,F			    ;point at next BOX
		movlw	.128 				    ;check if it is last box
		cpfseq	CBOXADR  			    ;skip if BOXADR is euqal WREG	
		bra		$ + 4
	bra		ExitCanMsgTransform         ;all boxes done

;read 48 regs from flash
		clrf	TBLPTRU                 ;set address of flash
		movlw	.48 				    ;start writing form first register of BOXxx
        mulwf   CBOXADR                 ;get TBLPTR value 48xBOXADR (range 0x00A000 - 0x00B7D0)
        movff   PRODH,TBLPTRH
        movff   PRODL,TBLPTRL  
        movlw   0xA0                    ;add 0x00A000 to TBLPTR
        addwf   TBLPTRH
		lfsr	FSR0,CBOX0  			;start writing to CAN box temp buffer
		movlw	.48                     ;48 registers			
CanCopyData
		tblrd*+					    	;read and move to TABLAT and increment address
		movff	TABLAT,POSTINC0		    ;move flash data to register indicated by FSR0 and increment FSR0
		decfsz	WREG				    ;all Bank bytes read?
		bra		CanCopyData

;check if BOX is enabled
        tstfsz  CBOX46
        bra     $ + 4
		bra		CanNextBOX			    ;box disabled

;Compare box to receive buffer
        lfsr    FSR0,CBOX0              ;get first byte of CAN box (CAN part)
        lfsr    FSR1,CRXtFIFO1          ;get first byte of CAN temp FIFO buffer
        movlw   .12                     ;compare 12 bytes
        movwf   CBOX46                  ;use this reg as temp reg
CanCompareMsgs
        movf    POSTINC0,W              ;compare values in FSR0 and FSR1
        xorwf   POSTINC1,W
		bnz		CanNextBOX              ;not the same
        decfsz  CBOX46                  ;compare each byte in the message
        bra     CanCompareMsgs

;load UART message from box to temp CAN RX FIFO buffer
        lfsr    FSR0,CBOX16             ;get first byte of CAN box (UART part)
        lfsr    FSR1,CRXtFIFO0          ;get first byte of CAN temp FIFO buffer
        movlw   .30                     ;make sure counter is not greater than 30
        cpfsgt  CBOX47                  ;skip if counter greater than 30
        movf    CBOX47,W                ;copy number of bytes shown in CBOX47 reg
        movwf   CBOX47                  ;update message counter
CanCopyUARTmsg
		movff	POSTINC0,POSTINC1
		decfsz	WREG				    ;all buffer bytes read?
		bra		CanCopyUARTmsg
        setf    CRXtFIFOTRSF 		    ;set flag "buffer transformed"
        movff   CBOX47,CRXtFIFOCNT      ;copy counter to CAN temp FIFO buffer

ExitCanMsgTransform
    return

;------------------------------------------------------------------------------
; Routine:          CAN TRANSMIT ROUTINE
;------------------------------------------------------------------------------
; PreCondition:		Data saved in transmit buffer TXBD0 - TXBD7
;------------------------------------------------------------------------------
CanTransmit:
    ;check if message contain 15 bytes
        movlb   0x0                     ;point at right bank
		movlw	.15
		xorwf	URXtFIFOCNT,W
		bnz		ExitCANTransmit		    ;no, so don't transmit

		movlw	0xAA					;is start byte ok?
		xorwf	URXtFIFO0,W
		bnz     ExitCANTransmit		    ;no, so don't transmit

		movlw	0xA5					;is stop byte ok?
		xorwf	URXtFIFO14,W
		bnz     ExitCANTransmit		    ;no, so don't transmit

		clrf	URXtFIFO15              ;checksum - use this reg as temp
        lfsr    FSR0,URXtFIFO1          ;point at first reg
        movlw   .12
        movwf   URXtFIFO16              ;use temp reg
        movf    POSTINC0,W              ;read next reg
        addwf   URXtFIFO15              ;add reg to temp checksum
        decfsz  URXtFIFO16              ;read all 12 regs
        bra     $ - 6                   
		movf	URXtFIFO13,W            ;compare checksum, ok?
		xorwf	URXtFIFO15,W
		bnz     ExitCANTransmit         ;no, so don't transmit

	;prepare bits in URXFIFO2
		btfsc	URXtFIFO2,4					;what is the fourth bit?
		bra		$ + 6						;it is 1, so set it in fourth position
		bcf		URXtFIFO2,1					;it is 0, so clear it in fourth position
		bra		$ + 4
		bsf		URXtFIFO2,1
		bsf		URXtFIFO2,3					;set EXIDE bit
TXB2
		movlw	b'00000100'					;access TXB2 buffer
		movwf	CANCON
		btfsc	RXB0CON,3					;check if TXB2 is free skip if free
		bra		TXB1
		lfsr	FSR0,0xF00					;point at first register of transmit buffer 2
		call	WriteBuffer					;
		bsf		RXB0CON,3					;request trasmition & clear TXABT, TXLARB, TXERR
		bra     ExitCANTransmit             ;now wait for TXB2IF (transmit successful)
TXB1
		movlw	b'00000110'					;access TXB1 buffer
		movwf	CANCON
		btfsc	RXB0CON,3					;check if TXB1 is free skip if free
		bra		TXB2
		lfsr	FSR0,0xF10					;point at first register of transmit buffer 1
		call	WriteBuffer					;
		bsf		RXB0CON,3					;request trasmition & clear TXABT, TXLARB, TXERR
		bra     ExitCANTransmit	            ;now wait for TXB1IF (transmit successful)
ExitCANTransmit
        return

;--------------------------
WriteBuffer:
		lfsr	FSR1,URXtFIFO1				;point at first register of source buffer URXFIFO0
        clrf    POSTINC0                    ;clear TXxCON
		movlw	.12                         ;read 12 successive registers
        movwf   URXtFIFO17                  ;temp reg
WriteBufferLoop:
        movlw   .8
        xorwf   URXtFIFO17,W                ;first 4 regs passed?
        bnz     $ + 4                       ;no
        incf    FSR0L                       ;yes, so pass TXBDLC reg
		movff	POSTINC1,POSTINC0			;move data from transmit buffer to destination buffer
		decfsz	URXtFIFO17					;all 13 bytes read?
		bra		WriteBufferLoop				;not yet - read more
        movlw   0x08                        ;move 8 to DLC reg
        movwf   RXB0DLC
	return



;==============================================================================
;===  END OF MAIN PROGRAM  ====================================================
;==============================================================================
		END
