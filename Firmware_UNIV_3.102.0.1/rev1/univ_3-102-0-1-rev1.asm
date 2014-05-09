;==============================================================================
;   HAPCAN - Home Automation Project Firmware (http://hapcan.com)
;   Copyright (C) 2014 hapcan.com
;
;   This program is free software: you can redistribute it and/or modify
;   it under the terms of the GNU General Public License as published by
;   the Free Software Foundation, either version 3 of the License, or
;   (at your option) any later version.
;
;   This program is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;   GNU General Public License for more details.
;
;   You should have received a copy of the GNU General Public License
;   along with this program.  If not, see <http://www.gnu.org/licenses/>.
;==============================================================================
;   Filename:              univ_3-102-0-1-revX.asm
;   Associated diagram:    univ_3-102-0-x.sch
;   Author:                Jacek Siwilo                                         
;   Note:                  Ethernet interface                                                          
;==============================================================================
;   Revision History
;   Rev:  Date:     Details:
;   0     11.2013   Original version
;   1     05.2013   Minor corrections
;==============================================================================
;===  FIRMWARE DEFINITIONS  =================================================== 
;==============================================================================
    #define    ATYPE    .102                          ;application type [0-255]
    #define    AVERS    .0                         ;application version [0-255]
    #define    FVERS    .1                            ;firmware version [0-255]

    #define    FREV     .1                         ;firmware revision [0-65536]
;==============================================================================
;===  NEEDED FILES  ===========================================================
;==============================================================================
    LIST P=18F26K80                              ;directive to define processor
    #include <P18F26K80.INC>           ;processor specific variable definitions
    #include "univ_3-102-0-1-rev1.inc"                       ;project variables
    #include "univ3-fake_bootloader-rev2.inc"    ;fake bootloader for debugging
INCLUDEDFILES   code  
    #include "univ3-routines-rev3.inc"                     ;UNIV 3 CPU routines
    #include "univ3-I2C-rev0.inc"                                 ;I2C routines

;==============================================================================
;===  CONFIG  DATA  ===========================================================
;==============================================================================
EEPROM      code                                                ;default config
    org 0xF00008
    DE      0xFF,0x00,0xFF,0xFF ;RTC calibrated to 0, send RTC msg to CAN & UART
;==============================================================================
;===  FIRMWARE CHECKSUM  ======================================================
;==============================================================================
FIRMCHKSM   code    0x001000
    DB      0x65, 0xF0, 0x21, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF            
;==============================================================================
;===  FIRMWARE ID  ============================================================
;==============================================================================
FIRMID      code    0x001010
    DB      0x30, 0x00, 0x03,ATYPE,AVERS,FVERS,FREV>>8,FREV
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
FIRMRESET   code    0x001020
        goto    Main
;PROGRAM HIGH PRIORITY INTERRUPT VECTOR
FIRMHIGHINT code    0x001030
        call    HighInterrupt
        retfie
;PROGRAM LOW PRIORITY INTERRUPT VECTOR
FIRMLOWINT  code    0x001040
        call    LowInterrupt
        retfie

;==============================================================================
;===  FIRMWARE STARTS  ========================================================
;==============================================================================
FIRMSTART   code    0x001050
;------------------------------------------------------------------------------
;---  LOW PRIORITY INTERRUPT  -------------------------------------------------
;------------------------------------------------------------------------------
LowInterrupt
        movff   STATUS,STATUS_LOW           ;save STATUS register
        movff   WREG,WREG_LOW               ;save working register
        movff   BSR,BSR_LOW                 ;save BSR register
        movff   FSR0L,FSR0L_LOW             ;save other registers used in high int
        movff   FSR0H,FSR0H_LOW
        movff   FSR1L,FSR1L_LOW
        movff   FSR1H,FSR1H_LOW

    ;main firmware ready flag
        banksel FIRMREADY
        btfss   FIRMREADY,0
        bra     ExitLowInterrupt            ;main firmware is not ready yet
    ;CAN buffer
        banksel CANFULL
        tstfsz  CANFULL                     ;check if CAN received anything
        call    CANInterrupt                ;proceed with CAN interrupt

ExitLowInterrupt
        movff   BSR_LOW,BSR                 ;restore BSR register
        movff   WREG_LOW,WREG               ;restore working register
        movff   STATUS_LOW,STATUS           ;restore STATUS register
        movff   FSR0L_LOW,FSR0L             ;restore other registers used in high int
        movff   FSR0H_LOW,FSR0H
        movff   FSR1L_LOW,FSR1L
        movff   FSR1H_LOW,FSR1H
    return

;------------------------------------------------------------------------------
;---  HIGH PRIORITY INTERRUPT  ------------------------------------------------
;------------------------------------------------------------------------------
HighInterrupt
        movff   STATUS,STATUS_HIGH          ;save STATUS register
        movff   WREG,WREG_HIGH              ;save working register
        movff   BSR,BSR_HIGH                ;save BSR register
        movff   FSR0L,FSR0L_HIGH            ;save other registers used in high int
        movff   FSR0H,FSR0H_HIGH
        movff   FSR1L,FSR1L_HIGH
        movff   FSR1H,FSR1H_HIGH

    ;main firmware ready flag
        banksel FIRMREADY
        btfss   FIRMREADY,0
        bra     ExitHighInterrupt           ;main firmware is not ready yet
    ;uart
        banksel UARTCNT
        tstfsz  UARTCNT                     ;check if uart received anything
        call    UartInterrupt
    ;timer0
        btfsc   INTCON,TMR0IF               ;Timer0 interrupt? (1000ms)
        rcall   Timer0Interrupt
    ;timer2
        btfsc   PIR1,TMR2IF                 ;Timer2 interrupt? (8.192ms)
        rcall   Timer2Interrupt

ExitHighInterrupt
        movff   BSR_HIGH,BSR                ;restore BSR register
        movff   WREG_HIGH,WREG              ;restore working register
        movff   STATUS_HIGH,STATUS          ;restore STATUS register
        movff   FSR0L_HIGH,FSR0L            ;restore other registers used in high int
        movff   FSR0H_HIGH,FSR0H
        movff   FSR1L_HIGH,FSR1L
        movff   FSR1H_HIGH,FSR1H
    return

;------------------------------------------------------------------------------
; Routine:          CAN INTERRUPT
;------------------------------------------------------------------------------
; Overview:         Saves message to to FIFO
;------------------------------------------------------------------------------
CANInterrupt
        call    Copy_RXB_RXFIFOIN           ;copies received message to CAN RX FIFO input buffer
        call    WriteToCanRxFIFO            ;saves message to FIFO
    return

;------------------------------------------------------------------------------
; Routine:          UART INTERRUPT
;------------------------------------------------------------------------------
; Overview:         Takes message from UART buffer and sends to UART RX FIFO
;------------------------------------------------------------------------------
UartInterrupt
    ;checking message
        banksel UARTOVF
        tstfsz  UARTOVF                     ;buffer overflow?
        bra     ExitUartInterrupt           ;yes, so exit
        movlw   .13                         ;system message?
        xorwf   UARTCNT,W                     
        bz      UartInterruptToFifo
        movlw   .15                         ;message to CAN?
        xorwf   UARTCNT,W                     
        bnz     ExitUartInterrupt           ;other message, so exit
UartInterruptToFifo
        call    WriteToUartRxFIFO           ;yes, so go write fifo
ExitUartInterrupt
        banksel UARTOVF
        clrf    UARTCNT                     ;clear uart received flag
    return

;------------------------------------------------------------------------------
; Routine:          TIMER 0 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         1000ms periodical interrupt
;------------------------------------------------------------------------------
Timer0Interrupt:
        call    Timer0Initialization32MHz   ;restart 1000ms Timer 
        call    UpdateUpTime                ;counts time from restart
        call    UpdateTransmitTimer         ;increment transmit timer (seconds after last transmission)
        banksel TIMER0_1000ms
        setf    TIMER0_1000ms               ;timer 0 interrupt occurred flag
    return

;------------------------------------------------------------------------------
; Routine:          TIMER 2 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         8192us periodical interrupt
;------------------------------------------------------------------------------
Timer2Interrupt
        rcall   Timer2Initialization        ;restart timer
        rcall   JumperStatus                ;read jumper
    return
;-------------------------------
Timer2Initialization
        movlb   0xF
        bcf     PMD1,TMR2MD                 ;enable timer 2                  
        clrf    TMR2                        ;set 8.192ms
        movlw   b'01111111'                 ;start timer, prescaler=16, postscaler=10
        movwf   T2CON
        bsf     IPR1,TMR2IP                 ;high priority for interrupt
        bcf     PIR1,TMR2IF                 ;clear timer's flag
        bsf     PIE1,TMR2IE                 ;interrupt on
    return 
;--------------------------------
JumperStatus                                ;Transfers jumper status to MD line of Tibbo
        bcf     LATC,0                      ;MD latch to low
        btfss   PORTB,7                     ;skip if jumper high (open)
        bcf     TRISC,0                     ;jumper low, so set MD as output
        btfsc   PORTB,7                     ;skip if jumper low (closed)
        bsf     TRISC,0                     ;jumper high, so set MD as input (pull up in Tibbo)     
    return

;==============================================================================
;===  MAIN PROGRAM  ===========================================================
;==============================================================================
Main:
    ;disable global interrupts for startup
        call    DisAllInt                   ;disable all interrupts
    ;firmware initialization
        rcall   PortInitialization          ;prepare processor ports
        call    Tibbo_MD_RST                ;Tibbo rescue reset
        call    GeneralInitialization       ;read eeprom config, clear other registers
        call    FIFOInitialization          ;prepare CAN FIFO buffers
        call    UartFIFOInitialization      ;prepare UART FIFO buffers
        rcall   RTCInitialization           ;Real Time Clock initial settings
        call    Timer0Initialization32MHz   ;Timer 0 initialization for 1s periodical interrupt  
        call    Timer2Initialization        ;Timer 2 initialization for 8192us periodical interrupt
    ;firmware ready
        banksel FIRMREADY
        bsf     FIRMREADY,0                 ;set flag "firmware started and ready for interrupts"
    ;enable global interrupts
        call    EnAllInt                    ;enable all interrupts

Loop:
        clrwdt
        rcall   UartReceiveProc             ;check if any msg in UART RX FIFOif so - process it
        rcall   CanReceiveProc              ;check if any msg in CAN RX FIFO if so - process it
        call    TransmitProcedure           ;check if any msg in CAN TX FIFO and if so - transmit it
        rcall   RTC_Main                    ;process RTC
        rcall   OnceA1000ms                 ;do routines only after 1000ms interrupt
    bra     Loop

;-------------------------------
OnceA1000ms                                 ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER0_1000ms
        tstfsz  TIMER0_1000ms               ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
        call    UpdateHealthRegs            ;saves health maximums to eeprom
        banksel TIMER0_1000ms
        clrf    TIMER0_1000ms
    return

;-------------------------------
UartReceiveProc
        banksel URXCNT
        tstfsz  URXCNT                      ;any msg received?
        bra     $ + 4                       ;yes
    return                                  ;no
        ;make sure there was not writing to UART RX FIFO during reading
        bcf     INTCON,GIEH                 ;high interrupt off
        call    ReadFromUartRxFIFO
        bsf     INTCON,GIEH                 ;high interrupt on
        ;check if system message
        call    UartProcessSystemMessage    ;system message for this module? (returns WREG=0 if message was ok)
        tstfsz  WREG                        ;system message was ok?
        call    UartProcessNormalMessage    ;send message to CAN
    return
                              
;-------------------------------
CanReceiveProc
        banksel RXCNT
        tstfsz  RXCNT                       ;any msg received?
        bra     $ + 4                       ;yes
    return                                  ;no
        ;make sure there was not writing to CAN RX FIFO during reading
        bcf     INTCON,GIEL                 ;low interrupt off
        call    ReadFromCanRxFIFO
        bsf     INTCON,GIEL                 ;low interrupt on
        ;recognize the message
        banksel RXFIFO0
        movlw   0x1F                        ;check if RXFIFO0 > 1Fh
        cpfsgt  RXFIFO0                     ;skip if yes
        call    ProcessSystemMessage        ;system message for this module?
        call    UartTransmit                ;send message to UART
    return



;==============================================================================
;===  FIRMWARE ROUTINES  ======================================================
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          PORT INITIALIZATION
;------------------------------------------------------------------------------
; Overview:         It sets processor pins. All unused pins should be set as
;                   outputs and driven low
;------------------------------------------------------------------------------
PortInitialization                          ;default all pins set as analog (portA,B) or digital (portB,C) inputs 
    ;PORT A
        banksel ANCON0                      ;select memory bank
        ;0-digital, 1-analog input
        movlw   b'00000011'                 ;(x,x,x,AN4,AN3,AN2,AN1-boot_mode,AN0-volt)
        movwf   ANCON0
        ;output level
        clrf    LATA                        ;all low
        ;0-output, 1-input
        movlw   b'00000011'                 ;all outputs except, bit<1>-boot_mode, bit<0>-volt
        movwf   TRISA        
    ;PORT B - inputs with pull ups from Tibbo EM500 rest as outputs
        ;0-digital, 1-analog input
        movlw   b'00000111'                 ;(x,x,x,x,x,AN10,AN9,AN8)
        movwf   ANCON1
        ;output level
        clrf    LATB                        ;all low
        ;0-output, 1-input
        movlw   b'10111011'                 ;all input except CanTX & RB6
        movwf   TRISB
        ;pull up
        movlw   b'10000000'                 ;enable pull up on RB7 pin (jumper)
        movwf   WPUB   
        bcf     INTCON2,RBPU                ;enable pull ups on port B                
    ;PORT C - leave as it is with digital inputs with pull ups from Tibbo EM500
        ;output level
        clrf    LATC                        ;all low
        ;0-output, 1-input
        movlw   b'11111111'                 ;all input 
        movwf   TRISC
    return

;------------------------------------------------------------------------------
; Routine:          TIBBO RST + MD
;------------------------------------------------------------------------------
; Overview:         Tibbo gets into programming mode
;                   (rescue option when tibbo is not visible in Device Explorer)
;                   Procedure only for backwards compatibility for hardware
;                   version older than 3. In hardware ver 3 jumper is wired
;                   directly to Tibbo MD pin 
;------------------------------------------------------------------------------
Tibbo_MD_RST                                ;Tibbo reset RST with MD button pressed
        btfsc   PORTB,7                     ;skip if jumper low (closed)
    return
        bcf     LATC,0                      ;MD latch to low
        bcf     LATC,1                      ;RST latch to low
        bcf     TRISC,0                     ;set MD as output - MD goes low
        bcf     TRISC,1                     ;set RST as output - start reset
        setf    R0                          ;wait 25ms

TibboL2 setf    R1
TibboL1 decfsz  R1
        bra     TibboL1
        decfsz  R0
        bra     TibboL2
        bsf     TRISC,1                     ;set RST as input - stop reset
        clrwdt        
        bra     $ - 2                       ;stay in loop - manual restart needed 

;------------------------------------------------------------------------------
; Routine:          CAN NODE STATUS
;------------------------------------------------------------------------------
; Overview:         It prepares status messages when status request was
;                   received on CAN side
;------------------------------------------------------------------------------
NodeStatusRequest
        banksel TXFIFOIN0
        movlw   0x30
        movwf   TXFIFOIN0
        movlw   0x00
        movwf   TXFIFOIN1
        bsf     TXFIFOIN1,0                 ;set response bit
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4                   ;unused
        movff   RTCYEAR,TXFIFOIN5
        movff   RTCMONTH,TXFIFOIN6
        movff   RTCDATE,TXFIFOIN7
        movff   RTCDAY,TXFIFOIN8
        movff   RTCHOUR,TXFIFOIN9
        movff   RTCMIN,TXFIFOIN10
        movff   RTCSEC,TXFIFOIN11
        call    WriteToCanTxFIFO
    return

;------------------------------------------------------------------------------
; Routine:          DO INSTRUCTION
;------------------------------------------------------------------------------
; Overview:         Executes instruction immediately or sets timer for later
;                   execution
;------------------------------------------------------------------------------
DoInstructionRequest                        ;Recognize instruction
        banksel INSTR1
        movlw   0x00                        ;instruction 00?
        xorwf   INSTR1,W
        bz      Instr00
        movlw   0x01                        ;instruction 01?
        xorwf   INSTR1,W
        bz      Instr01
    bra     ExitDoInstructionRequest        ;exit if unknown instruction

;-------------------------------
;Instruction execution
Instr00                                     ;set time
        banksel RTCHOUR_NEW
        movff   INSTR2,RTCHOUR_NEW          ;move time regs
        movff   INSTR3,RTCMIN_NEW
        movff   INSTR4,RTCSEC_NEW
        bcf     RTCSEC_NEW,7                ;make sure enable bit is 0
        call    RTC_SetTime
        bra     ExitDoInstructionRequest 
Instr01                                     ;set date
        movff   INSTR2,RTCYEAR_NEW          ;move date regs
        movff   INSTR3,RTCMONTH_NEW
        movff   INSTR4,RTCDATE_NEW
        movff   INSTR5,RTCDAY_NEW
        call    RTC_SetDate
        bra     ExitDoInstructionRequest 
ExitDoInstructionRequest
        setf    INSTR1                        ;clear instruction
    return


;==============================================================================
;                   UART ROUTINES                        
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          UART TRANSMIT
;------------------------------------------------------------------------------
; Overview:         Loads message from CAN RX FIFO output to UART port
;------------------------------------------------------------------------------
UartTransmit
        banksel RXFIFO0
        ;calculate message checksum
        clrf    UTXR0                       ;checksum in working reg 0
        lfsr    FSR0,RXFIFO0                ;point at first reg
        movlw   .12 
        movwf   RXFIFOR1                    ;use working reg 1
        movf    POSTINC0,W                  ;read next reg
        addwf   UTXR0                       ;add reg to temp checksum
        decfsz  RXFIFOR1                    ;read all 12 regs
        bra     $ - 6                   

UartTransmitStart
        bsf     TXSTA,TXEN                  ;enable uart transmission
    ;start byte
        movlw   0xAA                    
        btfss   PIR1,TX1IF                  ;check if TXREG if empty and ready to be loaded
        bra     $ - 2                
        movff   WREG,TXREG                  ;move data to transmit buffer
    ;send 12 bytes
        lfsr    FSR1,RXFIFO0                ;point at first register of CAN RX FIFO out buffer
        movlw   .12 
        movwf   RXFIFOR1                    ;use working reg 1
UartTransmitLp
        btfss   PIR1,TX1IF                  ;check if TXREG if empty and ready to be loaded
        bra     $ - 2                
        movff   POSTINC1,TXREG              ;move data from can receive buffer to uart transmit buffer
        decfsz  RXFIFOR1                    ;all bytes read?
        bra     UartTransmitLp              ;not yet - read more
    ;checksum
        movf    UTXR0,W
        btfss   PIR1,TX1IF                  ;check if TXREG if empty and ready to be loaded
        bra     $ - 2                
        movff   WREG,TXREG                  ;move data to transmit buffer
    ;stop byte
        movlw   0xA5                    
        btfss   PIR1,TX1IF                  ;check if TXREG if empty and ready to be loaded
        bra     $ - 2                
        movff   WREG,TXREG                  ;move data to transmit buffer
    return

;------------------------------------------------------------------------------
; Routine:          UART FIFO INITIALIZE
;------------------------------------------------------------------------------
; Overview:         Initialization of UART Receive FIFO buffers
;------------------------------------------------------------------------------
UartFIFOInitialization
;
URXFIFOSIZE    EQU     .42                  ;size of UART RX FIFO buffer
URXFIFORAM     EQU     0x700                ;beginning of UART RX FIFO in RAM

        banksel URXTOP                      ;choose the right bank
        clrf    URXTOP                      ;clear stack top UART RX
        clrf    URXCNT                      ;clear stack counter UART RX
        clrf    URXCNTMX                    ;clear max stack counter UART RX
    return

;------------------------------------------------------------------------------
; Routine:          WRITE MESSAGE TO UART RX FIFO
;------------------------------------------------------------------------------
; Overview:         Writes message from UART receive buffer (15 bytes
;                   beginning from UART0) into UART RX FIFO
;------------------------------------------------------------------------------
WriteToUartRxFIFO
        banksel URXTOP                      ;point at FIFO registers
    ;point at destination address (top of FIFO)
        lfsr    FSR1,URXFIFORAM-.16         ;point at first reg in FIF0 (URXFIFORAM-16)
        movff   URXTOP,URXFIFOR0            ;copy stack top
        incf    URXFIFOR0                      
        movlw   .16
AddURXFIFOAdr:
        addwf   FSR1L                       ;add multiply 32 to FSR1
        bnc     $ + 4                       ;go to next ram bank if overflow
        incf    FSR1H
        decfsz  URXFIFOR0
        bra     AddURXFIFOAdr

    ;copy to FIFO
        lfsr    FSR0,UART0                  ;point at source (first register in receive UART buffer)
        movlw   .15                         ;read 15 successive registers
        movwf   URXFIFOR0                      
CopyToUartRxFIFOLoop:
        movff   POSTINC0,POSTINC1           ;move data from receive buffer to destination buffer
        decfsz  URXFIFOR0                   ;all 15 bytes read?
        bra     CopyToUartRxFIFOLoop        ;not yet - read more
        lfsr    FSR0,UARTCNT                ;point at byte counter in receive UART buffer
        movff   POSTINC0,POSTINC1           ;move data from receive buffer to destination buffer

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
    return                              ;end of Save msg to UART FIFO procedure

;------------------------------------------------------------------------------
; Routine:          READ MESSAGE FROM UART RX FIFO
;------------------------------------------------------------------------------
; Overview:         Reads next message in queue from UART RX FIFO and saves in
;                   UART RX FIFO output buffer (16 bytes beginning from URXFIFO0)
;;------------------------------------------------------------------------------
ReadFromUartRxFIFO
        banksel URXCNT                      ;choose the right bank
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
        lfsr    FSR1,URXFIFORAM-.16         ;point at first reg in FIF0 (URXFIFORAM-16)  
        incf    URXFIFOR1
        movlw   .16   
GetURXFIFOAdr:
        addwf   FSR1L                       ;add multiply 16 to FSR1
        bnc     $ + 4                       ;go to next ram bank if overflow
        incf    FSR1H
        decfsz  URXFIFOR1
        bra     GetURXFIFOAdr

    ;copy from FIFO
        lfsr    FSR0,URXFIFO0               ;point at first register in RXtemp FIFO buffer
        movlw   .16                         ;read 16 successive registers              
CopyFromUartRxFIFOLoop:
        movff   POSTINC1,POSTINC0           ;move data from receive buffer to destination buffer
        decfsz  WREG                        ;all 16 bytes read?
        bra     CopyFromUartRxFIFOLoop      ;not yet - read more

    ;update FIFO stack top
        ;top stack stays unchanged
    ;update FIFO stack counter
        decf    URXCNT

ExitReadFromUartRxFIFO
    return                                  ;end of read msg from FIFO procedure

;------------------------------------------------------------------------------
; Routine:          UART PROCESS NORMAL MESSAGE
;------------------------------------------------------------------------------
; Overview:         It moves uart message to CAN TX FIFO buffer
;;------------------------------------------------------------------------------
UartProcessNormalMessage
        banksel    URXFIFO0
    ;message contains 15 bytes?
        movlw   .15                         ;check if URXFIFOCNT = 15
        xorwf   URXFIFOCNT,W
        bz      $ + 4                       ;yes
UartProcessNormalMessageError
    retlw   0x01                            ;exit with error

    ;start byte
        movlw   0xAA                        ;is start byte ok?
        xorwf   URXFIFO0,W
        bz      $ + 4                
    bra UartProcessNormalMessageError       ;no
    ;stop byte
        movlw   0xA5                        ;is stop byte ok?
        xorwf   URXFIFO14,W
        bz      $ + 4    
    bra UartProcessNormalMessageError       ;no
    ;checksum
        clrf    URXFIFOR0                   ;checksum in working reg 0
        lfsr    FSR0,URXFIFO1               ;point at first reg
        movlw   .12 
        movwf   URXFIFOR1                   ;use working reg 1
        movf    POSTINC0,W                  ;read next reg
        addwf   URXFIFOR0                   ;add reg to temp checksum
        decfsz  URXFIFOR1                   ;read all 12 regs
        bra     $ - 6  
        movf    URXFIFOR0,W                 ;checksums ok?
        xorwf   URXFIFO13,W
        bz      $ + 4    
    bra UartProcessNormalMessageError       ;no   

    ;copy from UART RX FIFO OUT buffer to CAN TX FIFO IN buffer
        lfsr    FSR0,URXFIFO1               ;point at first register in UART buffer
        lfsr    FSR1,TXFIFOIN0              ;point at first register in CAN buffer
        movlw   .12                         ;read 12 successive registers              
CopyFromUartToCanLoop
        movff   POSTINC0,POSTINC1           ;move data from receive buffer to destination buffer
        decfsz  WREG                        ;all 12 bytes read?
        bra     CopyFromUartToCanLoop       ;not yet - read more

    ;move CAN TX FIFO IN buffer to CAN TX FIFO        
        call    WriteToCanTxFIFO
    retlw   0x00 

;------------------------------------------------------------------------------
; Routine:          UART PROCESS SYSTEM MESSAGE
;------------------------------------------------------------------------------
; Overview:         It checks if received system message for this module
;;------------------------------------------------------------------------------
UartProcessSystemMessage
        banksel URXFIFO0
    ;system message?
        movlw   0x20                        ;check if URXFIFO1 < 20h
        cpfslt  URXFIFO1                    ;skip if yes
        bra UartProcessSystemMessageError   ;no
    ;message contains 13 bytes?
        movlw   .13                         ;check if URXFIFOCNT = 13
        xorwf   URXFIFOCNT,W
        bz      $ + 4                       ;yes
UartProcessSystemMessageError
    retlw   0x01                            ;exit with error
    
    ;start byte
        movlw   0xAA                        ;is start byte ok?
        xorwf   URXFIFO0,W
        bz      $ + 4                
    bra UartProcessSystemMessageError       ;no
    ;stop byte
        movlw   0xA5                        ;is stop byte ok?
        xorwf   URXFIFO12,W
        bz      $ + 4    
    bra UartProcessSystemMessageError       ;no
    ;checksum
        clrf    URXFIFOR0                   ;checksum in working reg 0
        lfsr    FSR0,URXFIFO1               ;point at first reg
        movlw   .10 
        movwf   URXFIFOR1                   ;use working reg 1
        movf    POSTINC0,W                  ;read next reg
        addwf   URXFIFOR0                   ;add reg to temp checksum
        decfsz  URXFIFOR1                   ;read all 10 regs
        bra     $ - 6  
        movf    URXFIFOR0,W                 ;checksums ok?
        xorwf   URXFIFO11,W
        bz      $ + 4    
    bra UartProcessSystemMessageError       ;no   
    ;frame type byte1
        movlw   0x10                        ;equals 0x10?
        xorwf   URXFIFO1,W
        bz      USM_10xx
        movlw   0x11                        ;equals 0x11?
        xorwf   URXFIFO1,W
        bz      USM_11xx    
    bra UartProcessSystemMessageError       ;no 
    ;frame type byte2
USM_10xx
        movlw   0x90                        ;node status request?
        xorwf   URXFIFO2,W
        bz      UARTStatusRequest
        movlw   0xA0                        ;execute instruction request?
        xorwf   URXFIFO2,W
        bz      UARTExecuteInstruction
    bra UartProcessSystemMessageError       ;none of them
USM_11xx
        movlw   0x30                        ;uptime request?
        xorwf   URXFIFO2,W
        bz      UARTUptimeRequest
    bra UartProcessSystemMessageError       ;none of them

;-------------------------------
UARTStatusRequest
        movlw   0x30
        movwf   RXFIFO0
        movlw   0x00
        movwf   RXFIFO1
        bsf     RXFIFO1,0                   ;set response bit
        movff   NODENR,RXFIFO2              ;node id
        movff   GROUPNR,RXFIFO3
        setf    RXFIFO4                     ;unused
        movff   RTCYEAR,RXFIFO5
        movff   RTCMONTH,RXFIFO6
        movff   RTCDATE,RXFIFO7
        movff   RTCDAY,RXFIFO8
        movff   RTCHOUR,RXFIFO9
        movff   RTCMIN,RXFIFO10
        movff   RTCSEC,RXFIFO11
        call    UartTransmit
    retlw   0x00                            ;uart system message was ok
;-------------------------------
UARTExecuteInstruction
        movff   URXFIFO3,INSTR1             ;move data from message to INSTRx regs
        movff   URXFIFO4,INSTR2
        movff   URXFIFO5,INSTR3
        movff   URXFIFO6,INSTR4
        movff   URXFIFO7,INSTR5
        call    DoInstructionRequest        ;call DoInstruction in main program
    retlw   0x00                            ;uart system message was ok
;-------------------------------
UARTUptimeRequest
        movlw   0x11
        movwf   RXFIFO0
        movlw   0x30
        movwf   RXFIFO1
        bsf     RXFIFO1,0                   ;set response bit
        movff   NODENR,RXFIFO2              ;node id
        movff   GROUPNR,RXFIFO3
        setf    RXFIFO4                     ;unused
        setf    RXFIFO5                     ;unused
        setf    RXFIFO6                     ;unused
        setf    RXFIFO7                     ;unused
        movff   UPTIME3,RXFIFO8
        movff   UPTIME2,RXFIFO9
        movff   UPTIME1,RXFIFO10
        movff   UPTIME0,RXFIFO11
        call    UartTransmit
    retlw   0x00                            ;uart system message was ok

;==============================================================================
;                   RTC ROUTINES                        
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          RTC INITIALIZATION
;------------------------------------------------------------------------------
; Overview:         Reads RTC and sends to CAN if not sent yet
;------------------------------------------------------------------------------
RTCInitialization
        call    I2C_Recovery                ;make sure slave doesn't lock up i2c bus
        call    I2C_MasterInitialization    ;prepare i2c bus
        rcall   RTC_SetCharger              ;enable super cap charging
        rcall   RTC_SetCalibration          ;sets calibration regs
    return
;-------------------------------
RTC_SetCharger
        call    I2C_WaitForIdle             ;wait till module is idle
        call    I2C_SendStart
        movlw   b'11010000'                 ;slave address + WRITE bit
        call    I2C_SendByte
        movlw   0x08                        ;address pointer (charger register)   
        call    I2C_SendByte
        movlw   b'10101011'                 ;enable charger, diode and 4k resistor)
        call    I2C_SendByte
        call    I2C_SendStop
    return
;-------------------------------
RTC_SetCalibration
        call    I2C_WaitForIdle             ;wait till module is idle
        call    I2C_SendStart
        movlw   b'11010000'                 ;slave address + WRITE bit
        call    I2C_SendByte
        movlw   0x07                        ;address pointer (calibration register)   
        call    I2C_SendByte
        movff   CONFIG1,WREG                ;move cal reg from eeprom
        bsf     WREG,7                      ;OUT=1, FT/OUT output set high
        bcf     WREG,6                      ;FT=0, FT/OUT stable (no frequency)
        call    I2C_SendByte
        call    I2C_SendStop
    return

;------------------------------------------------------------------------------
; Routine:          RTC SET TIME & DATE
;------------------------------------------------------------------------------
; Overview:         Sends new date and time to RTC
;------------------------------------------------------------------------------
RTC_SetTime
        call    I2C_WaitForIdle             ;wait till module is idle
        call    I2C_SendStart
        movlw   b'11010000'                 ;slave address + WRITE bit
        call    I2C_SendByte
        movlw   0x00                        ;address pointer (time)   
        call    I2C_SendByte
        movf    RTCSEC_NEW,W                ;send time
        call    I2C_SendByte
        movf    RTCMIN_NEW,W
        call    I2C_SendByte
        movf    RTCHOUR_NEW,W
        call    I2C_SendByte
        call    I2C_SendStop
    return
;-------------------------------
RTC_SetDate
        call    I2C_WaitForIdle             ;wait till module is idle
        call    I2C_SendStart
        movlw   b'11010000'                 ;slave address + WRITE bit
        call    I2C_SendByte
        movlw   0x03                        ;address pointer (date)   
        call    I2C_SendByte
        movf    RTCDAY_NEW,W                ;send date
        call    I2C_SendByte
        movf    RTCDATE_NEW,W
        call    I2C_SendByte
        movf    RTCMONTH_NEW,W
        call    I2C_SendByte
        movf    RTCYEAR_NEW,W
        call    I2C_SendByte
        call    I2C_SendStop
    return

;------------------------------------------------------------------------------
; Routine:          RTC MAIN
;------------------------------------------------------------------------------
; Overview:         Maintains all RTC routines
;------------------------------------------------------------------------------
RTC_Main
        rcall   RTC_ReadTime                ;read current time
        rcall   RTC_CheckIfTimeChanged      ;compare time with read previously
        tstfsz  WREG                        ;time changed?
    return                                  ;no
        rcall   RTC_CheckSchedule           ;check if current time matches schedule
        rcall   RTC_SendPeriodicalToCan     ;send periodically to CAN if enabled
        rcall   RTC_SendPeriodicalToUART    ;send periodically to UART if enabled
    return

;------------------------------------------------------------------------------
; Routine:          RTC READ CURRENT TIME
;------------------------------------------------------------------------------
; Overview:         Reads RTC time via I2C bus
;------------------------------------------------------------------------------
RTC_ReadTime        
    ;make sure module in not busy
        call    I2C_WaitForIdle             ;wait till module is idle
    ;set address pointer in slave device
        call    I2C_SendStart
        movlw   b'11010000'                 ;slave address + WRITE bit
        call    I2C_SendByte
        movlw   0x00                        ;address pointer (start reading from byte 0)   
        call    I2C_SendByte
    ;read device
        call    I2C_SendReStart             ;repeated start
        ;slave address
        movlw   b'11010001'                 ;slave address + READ bit
        call    I2C_SendByte
        ;read data
        call    I2C_ReceiveByte_andACK
        movwf   RTCSEC                      ;seconds
        call    I2C_ReceiveByte_andACK
        movwf   RTCMIN                      ;minutes
        call    I2C_ReceiveByte_andACK
        movwf   RTCHOUR                     ;hours
        call    I2C_ReceiveByte_andACK
        movwf   RTCDAY                      ;day
        call    I2C_ReceiveByte_andACK
        movwf   RTCDATE                     ;data
        call    I2C_ReceiveByte_andACK
        movwf   RTCMONTH                    ;month
        call    I2C_ReceiveByte_andStop
        movwf   RTCYEAR                     ;year
        ;remove unwanted bits
        movlw   b'01111111'
        andwf   RTCSEC                      ;seconds
        movlw   b'00111111'
        andwf   RTCHOUR                     ;hours
        movlw   b'00000111'
        andwf   RTCDAY                      ;day
        movlw   b'00111111'
        andwf   RTCDATE                     ;data
        movlw   b'00011111'
        andwf   RTCMONTH                    ;month
    return

;------------------------------------------------------------------------------
; Routine:          RTC CHECK IF TIME CHANGED
;------------------------------------------------------------------------------
; Overview:         Compares current time with previous one and saves new
;                   values if different
;------------------------------------------------------------------------------
RTC_CheckIfTimeChanged
;        movf    RTCSEC,W                   ;accurate to second
;        xorwf   RTCSEC_S,W
;        bnz     CompareTimeDiff
        movf    RTCMIN,W                    ;accurate to minute
        xorwf   RTCMIN_S,W
        bnz     RTC_TimeChanged
        movf    RTCHOUR,W
        xorwf   RTCHOUR_S,W
        bnz     RTC_TimeChanged
        movf    RTCDAY,W
        xorwf   RTCDAY_S,W
        bnz     RTC_TimeChanged
        movf    RTCDATE,W
        xorwf   RTCDATE_S,W
        bnz     RTC_TimeChanged
        movf    RTCMONTH,W
        xorwf   RTCMONTH_S,W
        bnz     RTC_TimeChanged
        movf    RTCYEAR,W
        xorwf   RTCYEAR_S,W
        bnz     RTC_TimeChanged
    retlw   0x01                            ;time the same
RTC_TimeChanged                             ;time is different, so save new values
        movff   RTCSEC,RTCSEC_S
        movff   RTCMIN,RTCMIN_S
        movff   RTCHOUR,RTCHOUR_S
        movff   RTCDAY,RTCDAY_S
        movff   RTCDATE,RTCDATE_S
        movff   RTCMONTH,RTCMONTH_S
        movff   RTCYEAR,RTCYEAR_S
    retlw   0x00

;------------------------------------------------------------------------------
; Routine:          RTC CHECK SCHEDULE
;------------------------------------------------------------------------------
; Overview:         Checks if current time matches schedule, if so then
;                   programmed CAN message is sent
;------------------------------------------------------------------------------
RTC_CheckSchedule
        banksel CONFIG4                     ;is schedule turned on?
        tstfsz  CONFIG4
        bra     $ + 4
    return 

;-------------------------------
        banksel BOXADR                      ;select bank
        setf    BOXADR                      ;first box minus 1 (range 0-127)
;get BOX by BOX
NextScheduleBOX    
        banksel BOXADR                      ;select bank
        incf    BOXADR,F                    ;point at next BOX
        movlw   .128                        ;check if it is last box
        cpfseq  BOXADR                      ;skip if BOXADR is euqal WREG    
        bra     $ + 4
    bra     RTC_CheckScheduleExit
;-------------
;read 20 regs from flash
        clrf    TBLPTRU                     ;set address of flash
        movlw   .32                         ;start writing form first register of BOXxx
        mulwf   BOXADR                      ;get TBLPTR value 32xBOXADR (range 0x008800 - 0x0097E0)
        movff   PRODH,TBLPTRH
        movff   PRODL,TBLPTRL  
        movlw   0x88                        ;add 0x008800 to TBLPTR
        addwf   TBLPTRH
        lfsr    FSR0,BOXFIL1                ;start writing to first reg in BOX temp buffer
        movlw   .20                         ;20 registers            
CopyScheduleBox
        tblrd*+                             ;read and move to TABLAT and increment address
        movff   TABLAT,POSTINC0             ;move flash data to register indicated by FSR0 and increment FSR0
        decfsz  WREG                        ;all BOX bytes read?
        bra     CopyScheduleBox
;-------------
;check if box is enabled
        tstfsz  BOXFIL1                     ;check first byte in box
        bra     $ + 4
        bra     NextScheduleBOX             ;box disabled
;-------------
;compare box to current time
        ;year
        movlw   0xAA                        ;any year?
        xorwf   BOXFIL2,W
        bz      $ + 8                       ;yes, so skip checking
        movf    RTCYEAR,W
        xorwf   BOXFIL2,W
        bnz     NextScheduleBOX             ;bytes don't match
        ;month
        movlw   0xAA                        ;any month?
        xorwf   BOXFIL3,W
        bz      $ + 8                       ;yes, so skip checking
        movf    RTCMONTH,W
        xorwf   BOXFIL3,W
        bnz     NextScheduleBOX             ;bytes don't match
        ;date
        movlw   0xAA                        ;any day of month?
        xorwf   BOXFIL4,W
        bz      $ + 8                       ;yes, so skip checking
        movf    RTCDATE,W
        xorwf   BOXFIL4,W
        bnz     NextScheduleBOX             ;bytes don't match
        ;day
        clrf    RTCDAYBIT                   ;convert BCD format to bit eg 1-b'00000001', 2-b'00000010' etc
        movf    RTCDAY,W
        bsf     STATUS,C
        rlcf    RTCDAYBIT                   ;rotate left bit '1' RTCDAY times       
        decfsz  WREG
        bra     $ - 4
        movf    RTCDAYBIT,W                 ;is this day included?
        andwf   BOXFIL5,W
        bz      NextScheduleBOX             ;no
        ;hour
        movlw   0xAA                        ;any hour?
        xorwf   BOXFIL6,W
        bz      $ + 8                       ;yes, so skip checking
        movf    RTCHOUR,W
        xorwf   BOXFIL6,W
        bnz     NextScheduleBOX             ;bytes don't match
        ;minute
        movlw   0xAA                        ;any minute?
        xorwf   BOXFIL7,W
        bz      $ + 8                       ;yes, so skip checking
        movf    RTCMIN,W
        xorwf   BOXFIL7,W
        bnz     NextScheduleBOX             ;bytes don't match
        ;second
;        movlw   0xAA                        ;any second?
;        xorwf   BOXFIL8,W
;        bz      $ + 8                       ;yes, so skip checking
;        movf    RTCSEC,W
;        xorwf   BOXFIL8,W
;        bnz     NextScheduleBOX             ;bytes don't match
;-------------
;prepare CAN message to send
        movff   BOXFIL9,TXFIFOIN0           ;copy message from box
        movff   BOXFIL10,TXFIFOIN1
        movff   BOXFIL11,TXFIFOIN2
        movff   BOXFIL12,TXFIFOIN3
        movff   BOXFIL1C,TXFIFOIN4                
        movff   BOXFIL2C,TXFIFOIN5
        movff   BOXFIL3C,TXFIFOIN6
        movff   BOXFIL4C,TXFIFOIN7
        movff   BOXFIL5C,TXFIFOIN8
        movff   BOXFIL6C,TXFIFOIN9
        movff   BOXFIL7C,TXFIFOIN10
        movff   BOXFIL8C,TXFIFOIN11
        call    WriteToCanTxFIFO
;-------------
;make this message visible on UART
        movff   BOXFIL9,RXFIFO0             ;copy message from box
        movff   BOXFIL10,RXFIFO1
        movff   BOXFIL11,RXFIFO2
        movff   BOXFIL12,RXFIFO3
        movff   BOXFIL1C,RXFIFO4                
        movff   BOXFIL2C,RXFIFO5
        movff   BOXFIL3C,RXFIFO6
        movff   BOXFIL4C,RXFIFO7
        movff   BOXFIL5C,RXFIFO8
        movff   BOXFIL6C,RXFIFO9
        movff   BOXFIL7C,RXFIFO10
        movff   BOXFIL8C,RXFIFO11
        call    UartTransmit
    bra     NextScheduleBOX

;-------------
RTC_CheckScheduleExit
    return

;------------------------------------------------------------------------------
; Routine:          RTC SEND PERIODICALLY
;------------------------------------------------------------------------------
; Overview:         Compares current time with previous one and sends if needed
;                   to CAN & UART
;------------------------------------------------------------------------------
RTC_SendPeriodicalToCan
        banksel CONFIG2                     ;is periodical sending to CAN turned on?
        tstfsz  CONFIG2
        bra     $ + 4
    return                                  ;no, so exit
        banksel TXFIFOIN0
        movlw   0x30
        movwf   TXFIFOIN0
        movlw   0x00
        movwf   TXFIFOIN1
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4                   ;unused
        movff   RTCYEAR,TXFIFOIN5
        movff   RTCMONTH,TXFIFOIN6
        movff   RTCDATE,TXFIFOIN7
        movff   RTCDAY,TXFIFOIN8
        movff   RTCHOUR,TXFIFOIN9
        movff   RTCMIN,TXFIFOIN10
        movff   RTCSEC,TXFIFOIN11
        call    WriteToCanTxFIFO
    return
;-------------------------------
RTC_SendPeriodicalToUART
        banksel CONFIG3                     ;is periodical sending to UART turned on?
        tstfsz  CONFIG3
        bra     $ + 4
    return  
        banksel RXFIFO0
        movlw   0x30
        movwf   RXFIFO0
        movlw   0x00
        movwf   RXFIFO1
        movff   NODENR,RXFIFO2              ;node id
        movff   GROUPNR,RXFIFO3
        setf    RXFIFO4                     ;unused
        movff   RTCYEAR,RXFIFO5
        movff   RTCMONTH,RXFIFO6
        movff   RTCDATE,RXFIFO7
        movff   RTCDAY,RXFIFO8
        movff   RTCHOUR,RXFIFO9
        movff   RTCMIN,RXFIFO10
        movff   RTCSEC,RXFIFO11
        call    UartTransmit
    return
;==============================================================================
;===  END OF MAIN PROGRAM  ====================================================
;==============================================================================
        END
