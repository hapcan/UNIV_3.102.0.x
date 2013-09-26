	NOLIST
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
;==========================================================================
;
;   Filename:           univ_3-102-0-0-rev1_cfg.inc
;   Date:               June 2012
;
;   Author:             Jacek Siwilo
;		
;==========================================================================

;==========================================================================
;       Register Definitions
;==========================================================================
;general use registers
R0			EQU		H'0000'
R1			EQU		H'0001'
R2			EQU		H'0002'
R3			EQU		H'0003'
R4			EQU		H'0004'
R5			EQU		H'0005'
R6			EQU		H'0006'
R7			EQU		H'0007'
R8			EQU		H'0008'
R9			EQU		H'0009'
R10			EQU		H'000A'
R11			EQU		H'000B'
R12			EQU		H'000C'
R13			EQU		H'000D'
R14			EQU		H'000E'
R15			EQU		H'000F'
R16			EQU		H'0010'
R17			EQU		H'0011'
R18			EQU		H'0012'
R19			EQU		H'0013'
R20			EQU		H'0014'
R21			EQU		H'0015'
R22			EQU		H'0016'
R23			EQU		H'0017'
R24			EQU		H'0018'
R25			EQU		H'0019'
R26			EQU		H'001A'
R27			EQU		H'001B'
R28			EQU		H'001C'
R29			EQU		H'001D'
R30			EQU		H'001E'
R31			EQU		H'001F'

;temporary saved registers
STATUS_HIGH EQU     H'0030'
WREG_HIGH   EQU     H'0031'
BSR_HIGH    EQU     H'0032'
FSR0L_HIGH  EQU     H'0033'
FSR0H_HIGH  EQU     H'0034'
FSR1L_HIGH  EQU     H'0035'
FSR1H_HIGH  EQU     H'0036'
STATUS_LOW  EQU     H'0037'
WREG_LOW    EQU     H'0038'
BSR_LOW     EQU     H'0039'
FSR0L_LOW   EQU     H'003A'
FSR0H_LOW   EQU     H'003B'
FSR1L_LOW   EQU     H'003C'
FSR1H_LOW   EQU     H'003D'

;FIFO regs
URXTOP      EQU	    H'0040'
URXCNT      EQU	    H'0041'
URXCNTMX    EQU	    H'0042'
URXFIFOR0   EQU	    H'0043'
URXFIFOR1   EQU	    H'0044'
CRXTOP      EQU	    H'0045'
CRXCNT      EQU	    H'0046'
CRXCNTMX    EQU	    H'0047'
CRXFIFOR0   EQU	    H'0048'
CRXFIFOR1   EQU	    H'0049'
FIFOFLAG    EQU     H'004A'
UartRxFifoWrite EQU H'0000'     ;"Write to UART RX FIFO Buffer appeared" flag
CanRxFifoWrite  EQU H'0001'     ;"Write to CAN RX FIFO Buffer appeared" flag

;Other regs
UBOXADR     EQU     H'005E'     ;addres of current UART flash box
CBOXADR     EQU     H'005F'     ;addres of current CAN flash box


;temp FIFO UART RX buffer
URXtFIFO0    EQU     H'0060'     ;buffer begining
URXtFIFO1    EQU     H'0061'
URXtFIFO2    EQU     H'0062'
;...
URXtFIFO13   EQU     H'006D'
URXtFIFO14   EQU     H'006E'
URXtFIFO15   EQU     H'006F'
URXtFIFO16   EQU     H'0070'
URXtFIFO17   EQU     H'0071'
;...
URXtFIFO29   EQU     H'007D'     ;buffer end
URXtFIFOOVFL EQU     H'007E'     ;buffer overflow flag
URXtFIFOCNT  EQU     H'007F'     ;buffer counter

;temp FIFO CAN RX buffer	
CRXtFIFO0    EQU     H'0080'     ;buffer begining
CRXtFIFO1    EQU     H'0081'
;...
CRXtFIFO13   EQU     H'008D'
CRXtFIFO14   EQU     H'008E'
CRXtFIFO15   EQU     H'008F'
;...
CRXtFIFO29   EQU     H'009D'     ;buffer end
CRXtFIFOTRSF EQU     H'009E'     ;buffer has been transformed flag
CRXtFIFOCNT  EQU     H'009F'     ;buffer counter

;UART box temp buffer
UBOX0       EQU     H'00A0'     ;box begining
;...
UBOX30      EQU     H'00BE'     ;disable box if 0x00
UBOX31      EQU     H'00BF'     ;message byte counter
UBOX32      EQU     H'00C0'     ;box begining (CAN part)
;...
UBOX47      EQU     H'00CF'     ;box end

;CAN box temp buffer
CBOX0       EQU     H'00D0'     ;box begining (CAN part)
;...
CBOX16      EQU     H'00E0'     ;box begining (UART part)
;...
CBOX46      EQU     H'00FE'     ;disable box if 0x00
CBOX47      EQU     H'00FF'     ;message byte counter


;==============================
;===  BOOTLOADER REGISTERS  ===
;==============================
;CAN RX buffer
RxBCON      EQU		H'0100'
CANFRAME1   EQU		H'0101'
CANFRAME2   EQU		H'0102'
CANNODE     EQU		H'0103'
CANGROUP    EQU		H'0104'
CANDLC      EQU		H'0105'
CAND0       EQU		H'0106'
CAND1       EQU		H'0107'
CAND2       EQU		H'0108'
CAND3       EQU		H'0109'
CAND4       EQU		H'010A'
CAND5       EQU		H'010B'
CAND6       EQU		H'010C'
CAND7       EQU		H'010D'
CANFULL     EQU     H'010E'     ;CAN buffer full flag
FIRMFLAG    EQU     H'010F'     ;firmware started completely flag

;uart RX buffer
UART0       EQU		H'0110'
UART1       EQU		H'0111'
UART2       EQU		H'0112'
UART3       EQU		H'0113'
UART4       EQU		H'0114'
UART5       EQU		H'0115'
UART6       EQU		H'0116'
UART7       EQU		H'0117'
UART8       EQU		H'0118'
UART9       EQU		H'0119'
UART10      EQU		H'011A'
UART11      EQU		H'011B'
UART12      EQU		H'011C'
UART13      EQU		H'011D'
UART14      EQU		H'011E'
UART15      EQU		H'011F'
UART16      EQU		H'0120'
UART17      EQU		H'0121'
UART18      EQU		H'0122'
UART19      EQU		H'0123'
UART20      EQU		H'0124'
UART21      EQU		H'0125'
UART22      EQU		H'0126'
UART23      EQU		H'0127'
UART24      EQU		H'0128'
UART25      EQU		H'0129'
UART26      EQU		H'012A'
UART27      EQU		H'012B'
UART28      EQU		H'012C'
UART29      EQU		H'012D'
UARTOVF     EQU		H'012E'     ;overflow indicator
UARTCNT     EQU		H'012F'     ;uart buffer counter

;bootloader other regs
UARTON      EQU		H'016F'     ;indicates that UART bootloader is on when 0xFF

    LIST
