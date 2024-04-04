; Written by Anders Nielsen, 2023-2024
; License: https://creativecommons.org/licenses/by-nc/4.0/legalcode

.feature string_escapes ; Allow c-style string escapes when using ca65
.feature org_per_seg
.feature c_comments

BAUDRATE=9600 ; Max 9600
BAUDSTEP=9600 / BAUDRATE - 1; Must be an integer

RIOT = $80
DRA     = RIOT + $00 ;DRA ('A' side data register)
DDRA    = RIOT + $01 ;DDRA ('A' side data direction register)
DRB     = RIOT + $02 ;DRB ('B' side data register)
DDRB    = RIOT + $03 ;('B' side data direction register)
READTDI = RIOT + $04 ;Read timer (disable interrupt)

WEDGC   = RIOT + $04 ;Write edge-detect control (negative edge-detect,disable interrupt)
RRIFR   = RIOT + $05 ;Read interrupt flag register (bit 7 = timer, bit 6 PA7 edge-detect) Clear PA7 flag
A7PEDI  = RIOT + $05 ;Write edge-detect control (positive edge-detect,disable interrupt)
A7NEEI  = RIOT + $06 ;Write edge-detect control (negative edge-detect, enable interrupt)
A7PEEI  = RIOT + $07 ;Write edge-detect control (positive edge-detect enable interrupt)

READTEI = RIOT + $0C ;Read timer (enable interrupt)
WTD1DI  = RIOT + $14 ; Write timer (divide by 1, disable interrupt)
WTD8DI  = RIOT + $15 ;Write timer (divide by 8, disable interrupt)
WTD64DI = RIOT + $16 ;Write timer (divide by 64, disable interrupt)
WTD1KDI = RIOT + $17 ;Write timer (divide by 1024, disable interrupt)

WTD1EI  = RIOT + $1C ;Write timer (divide by 1, enable interrupt)
WTD8EI  = RIOT + $1D ;Write timer (divide by 8, enable interrupt)
WTD64EI = RIOT + $1E ;Write timer (divide by 64, enable interrupt)
WTD1KEI = RIOT + $1F ;Write timer (divide by 1024, enable interrupt)

; Bitmasks for setting and clearing bit 1 of DDRA to input and output
BITMASK_SET_BIT1    = $02   ; Binary: 0000 0010
BITMASK_CLEAR_BIT1  = $FD   ; Binary: 1111 1101

; Bitmasks for setting and clearing signals in Data Register B (DRB)
BITMASK_SET_RLSBLE   = $04
BITMASK_CLEAR_RLSBLE = $FB

BITMASK_SET_RMSBLE    = $08
BITMASK_CLEAR_RMSBLE  = $F7

BITMASK_SET_ROM_OE    = $10
BITMASK_CLEAR_ROM_OE  = $EF

BITMASK_SET_CTRL_LE   = $20
BITMASK_CLEAR_CTRL_LE = $DF

BITMASK_SET_ROM_CE    = $80
BITMASK_CLEAR_ROM_CE  = $7F

; Bitmasks for additional signals
BITMASK_SET_VPE_TO_VPP   = %00000001
BITMASK_SET_A9_VPP_ENABLE = %00000010
BITMASK_SET_VPE_ENABLE    = %00000100
BITMASK_SET_REG_DISABLE = %10000000

; Bitmasks to clear corresponding bits
BITMASK_CLEAR_VPE_TO_VPP   = %11111110
BITMASK_CLEAR_A9_VPP_ENABLE = %11111101
BITMASK_CLEAR_VPE_ENABLE    = %11111011
BITMASK_CLEAR_REG_DISABLE     = %01111111

.include "macros.s"
;includes the "print" macro for printing strings

.segment "ZEROPAGE"
I2CADDR:  .res 1 ; Reserve 1 byte for I2CADDR
inb:      .res 1 ; Reserve 1 byte for inb - Used for Serial and I2C
outb:     .res 1 ; Reserve 1 byte for outb - Used for Serial and I2C
xtmp:     .res 1 ; Reserve 1 byte for xtmp
stringp:  .res 2 ; Reserve 2 bytes for stringp (stringp + 1)
;Stringp +1 free for temp.
mode:     .res 1 ; Reserve 1 byte for mode
rxcnt:    .res 1 ; Reserve 1 byte for rxcnt
txcnt:    .res 1 ; Reserve 1 byte for txcnt
runpnt:   .res 2 ; Reserve 2 bytes for runpnt
cursor:   .res 1 ; Reserve 1 byte for cursor ; SSD1306
scroll:   .res 1 ; Reserve 1 byte for scroll ; SSD1306
tflags:   .res 1 ; Reserve 1 byte for tflags ; SSD1306
serialbuf: .res 0 ; Reserve 1 byte for serialbuf - Used for text display and userland program storage

timer2  = stringp ; We're not going to be printing strings while waiting for timer2

romaddr = runpnt
longdelay = txcnt

.SEGMENT "USERLAND"
.org $0e ; Just to make listing.txt match
userland:

;Enable VPE and halt - for calibration of VPE
;lda #BITMASK_SET_REG_DISABLE
;jsr latchctrl
;halthere:
;jmp halthere

jsr clonetow27c512 ; Clone and verify (print to SSD1306)
jmp halt 
/*
LDA #BITMASK_SET_REG_DISABLE | BITMASK_SET_VPE_ENABLE | BITMASK_SET_VPE_TO_VPP
JSR latchctrl  ; Latch the updated control signals

lda #$ff 
sta DDRA
jsr delay_short ; ~2ms
jsr delay_short ; ~2ms

;jsr clonetow27c512nowipe
;jsr clonerom2
*/

;This snippit checks if ROM is identical to ROM in programmer
/*
lda #%01000000 ; Indicator 
jsr latchctrl
print verifying
jsr clonecheck ; Verify

lda #%01000000
jsr latchctrl

;print romnotblankstr
lda #'$'
jsr ssd1306_sendchar

lda romaddr+1
jsr printbyte ; Print address of first mismatch ($1000 if OK)
lda romaddr
jsr printbyte
print bytesverifiedstr
jmp halt ; Back to main
*/

/* ;This snippet checks if a ROM is blank
checkblank:
print verifying
jsr blankcheck ; Verify erasure
cpx #$ff ; Check byte value 
bne notblank
print romblankstr

notblank:
lda romaddr+1
jsr printbyte ; Print address of first mismatch ($1000 if OK)
lda romaddr
jsr printbyte
print bytesverifiedstr

/*
lda #$FA
ldy #10
ldx #0
jsr writerom
*/


; This snippet programs ROM contents to a 2732 ROM (if VPE is connected correctly and configured to 21V )
/*print cloning

lda #50
sta longdelay ; Must be initialized to either 0 for 100us ROMs or number of ms for slow programming Roms

LDA #BITMASK_SET_REG_DISABLE | BITMASK_SET_VPE_ENABLE 
JSR latchctrl  ; Latch the updated control signals

lda #$ff 
sta DDRA
jsr delay_long ; ~256ms
lda #$ff 
jsr delay_long ; ~256ms
lda #$ff 
jsr delay_long ; ~256ms

;jsr clonerom2 ; Write 65uino ROM
*/

;This snippet waits for a serial byte to arrive - and prints it as hex when it does
/*
w4serial:
lda DRA ; Check serial 3c
and #$01 ; 2c
bne w4serial ; 2c
jsr serial_rx ; Get character
jsr printbyte
jmp w4serial

*/

halt:

ldy #0
lda #$02 ; Bit 1 is serial TX (Output)
sta DDRA
sty mode
sty txcnt
sty rxcnt
jmp main ; Get ready for new code


.segment "RODATA"
.org $1000 ; Not strictly needed with CA65 but shows correct address in listing.txt
  nmi:
  irq:
  reset:
          cld ; Because you never know
          sei ; Probably not needed with the 6507
          ;Set stack pointer and clear RAM
          ldx #$7f
          txs
          lda #0
  clearzp:
          sta $00,x
          dex
          bne clearzp
          sta $00,x

          JSR latchctrl  ; Latch 0 into control register 

          lda #%01010000 ; Init DRB bit 4 and 6 to 1, rest to 0.
          sta DRB
          lda #%10111100 ; Bit 0, 1 are SCL and SDA, bit 6 is input button.
          sta DDRB ; Set B register direction to #%10111100
          ; Reset state of DDRA is $00.

          lda #%11111001 ; Rest of port is input
          sta DRA
          lda #$02 ; Bit 1 is serial TX (Output)
          sta DDRA

jsr qsdelay
lda #$3C ; 78 on the back of the module is 3C << 1
sta I2CADDR
jsr ssd1306_init
jsr qsdelay
jsr ssd1306_clear

bit DRB
bvs welcomemsg
lda #1
sta mode
print ready

clc
bcc main ; BRA

welcomemsg:
print welcome

main:
bit DRB
bpl ledoff ; LED on, turn it off
lda DRB    ; LED off, turn it on
and #$7f   ; Bit low == ON
sta DRB
jmp l71
ledoff:
lda DRB
ora #$80 ; High == off
sta DRB
l71:
lda #244
bit DRB
bvs quartersecond
sta WTD64DI ; 244*64 = 15616 ~= 16ms
jsr ssd1306_clear ; We only end up here if button is pressed
bne wait ; BRA
quartersecond:
sta WTD1KDI ; 244 * 1024 = 249856 ~= quarter second
bne wait ; BRA

gonoserial:
jmp noserial
wait:
lda DRA ; Check serial 3c
/*
and #%11111011 ; CTS low
sta DRA
*/
and #$01 ; 2c
bne gonoserial ; 2c
tay ; A already 0
sta txcnt
lda #64 ; RX wait loop below is 16 cycles
sta timer2

rx:
dec timer2
beq rxtimeout ; Branch if timeout
lda DRA ; Check serial 3c
and #$01 ; 2c
bne rx ; Wait for RX until timeout
lda #64
sta timer2 ; Reset timer
gorx:
jsr serial_rx ; 6c
sta serialbuf, y
cpy #128-(<userland)-9 ; Leaves 9 bytes for stack
beq rx_err
iny
bne rx ; BRA (Y never 0)
rx_err:
sty rxcnt
/*
lda DRA ;
ora #4 ; CTS high
sta DRA

lda #$13 ; XOFF
jsr serial_tx ; Inform sender we're out of buffer space
*/
lda #<overflow
sta stringp
lda #>overflow
sta stringp+1
jsr ssd1306_wstring
clc
bcc tx ; BRA
rxtimeout:
sty rxcnt
lda mode
beq txt
cmp #1
bne txt
;Time to parse data instead of txt - aka, our bootloader!
jsr ssd1306_clear
lda #<loaded
sta stringp
lda #>loaded
sta stringp+1
jsr ssd1306_wstring

lda rxcnt
jsr printbyte

lda #<bytes
sta stringp
lda #>bytes
sta stringp+1
jsr ssd1306_wstring

waittorun:
jsr qsdelay
jsr qsdelay
jsr ssd1306_clear
lda #0
sta cursor
jsr ssd1306_setline
lda #0
jsr ssd1306_setcolumn

;lda #<userland
;sta runpnt
;lda #>userland
;sta runpnt+1

;jmp (runpnt)
jmp userland

txt:
/*
lda DRA ;
ora #4 ; CTS high
sta DRA

lda #$13 ; XOFF
jsr serial_tx ; Inform sender to chill while we write stuff to screen
*/

tx:
ldy txcnt
bne notfirst
lda serialbuf, y
cmp #$01 ; SOH
bne notfirst
sta mode
jmp main ; Bootloader character via serial
notfirst:
cpy rxcnt
beq txdone
lda serialbuf, y
jsr ssd1306_sendchar
inc txcnt
jmp tx
noserial:
lda READTDI
bne gowait ; Loop until timer runs out
jmp main ; loop
txdone:
/*
lda DRA ;
and #$fb ; CTS low
sta DRA

lda #$11 ; XON
jsr serial_tx
*/
gowait:
jmp wait

.include "i2c.s" ; i2c rutines specifically for the 65uino. Provides i2c_start, i2cbyteout, i2cbytein, i2c_stop - expects a few 
.include "ssd1306.s" ; SSD1306 routines specifically for the 65uino. Provides ssd1306_init, ssd1306_clear, ssd1306_sendchar, ssd1306_setline, ssd1306_cmd, ssd1306_wstring, printbyte

ready:.asciiz "Ready to load code... "
loading:.asciiz "Loading... "
overflow:.asciiz "OF!"
loaded: .asciiz "Loaded "
bytes: .asciiz " bytes of data. Running code in 1 second."

welcome:
.byte "Hi!             I'm the 65uino! I'm a 6502 baseddev board. Come learn everything about me!"
.byte $00

erasing:.asciiz "\nErasing...\n"
m27c512rom:.asciiz "M27C512"
w27c512rom:.asciiz "W27C512" ; Continues
foundwithid:.asciiz " found with ID: "
unrecognized:.asciiz "Unrecognized ROM"
blankcheckstr:.asciiz "Performing blank check... "
romblankstr: .asciiz "ROM blank. \n"
cloning: .asciiz "Cloning internal ROM.. "
verifying: .asciiz "Verifying.. \n"
romnotblankstr: .asciiz "First mismatch found at addr: $"
bytesverifiedstr: .asciiz " bytes verified OK"

clonetow27c512:
lda #%01000000 ; Indicator cloning started
jsr latchctrl

jsr identifyrom ; Returns ROM ID in romaddr(+1)
lda romaddr
cmp #$DA ; Winbond
bne notwb
lda romaddr+1
cmp #8 ; 08 = W27C512
bne notwb

wipe:
print erasing ; Print macro in macros.s
jsr erasew27c512 ; First we erase it


clonetow27c512nowipe:
lda #0
sta longdelay ; Must be initialized to either 0 for 100us ROMs or number of ms for slow programming Roms

print cloning
jsr cloneROM

print verifying
jsr clonecheck ; Verify

lda #%01000000
jsr latchctrl

;print romnotblankstr
lda #'$'
jsr ssd1306_sendchar

lda romaddr+1
jsr printbyte ; Print address of first mismatch ($1000 if OK)
lda romaddr
jsr printbyte
print bytesverifiedstr
rts

notwb:
print unrecognized
print foundwithid 
rts

identifyrom:
jsr getromid
stx romaddr ; tmp
sty romaddr+1

cpx #$DA ; Winbond
beq winbond
cpx #$20 ; ST
beq st
bne fail

winbond:
cpy #$08
bne fail
print w27c512rom
jmp nofail

st:
cpy #$3D
bne fail
print m27c512rom
jmp nofail

fail:
print unrecognized
nofail:
print foundwithid
lda romaddr
jsr printbyte
lda romaddr+1
jsr printbyte
rts

;Returns manufacturer ID in X and device ID in Y
getromid:
lda #BITMASK_SET_REG_DISABLE
jsr latchctrl
lda #BITMASK_SET_VPE_TO_VPP | BITMASK_SET_REG_DISABLE | BITMASK_SET_A9_VPP_ENABLE
jsr latchctrl
lda #$50
jsr delay_long ; Stabilize VPP
ldy #0
sty DRA
lda DRB
ora #BITMASK_SET_RLSBLE | BITMASK_SET_RMSBLE
sta DRB
and #BITMASK_CLEAR_RMSBLE & BITMASK_CLEAR_RLSBLE & BITMASK_CLEAR_ROM_CE
sta DRB
sty DDRA ; DRA input
and #BITMASK_CLEAR_ROM_OE
sta DRB
ldx DRA
ora #BITMASK_SET_ROM_OE
sta DRB
ldy #$ff
sty DDRA
ldy #$01
sty DRA
ora #BITMASK_SET_RLSBLE
sta DRB
and #BITMASK_CLEAR_RLSBLE
sta DRB
ldy #0
sty DDRA
and #BITMASK_CLEAR_ROM_OE
sta DRB
ldy DRA
ora #BITMASK_SET_ROM_OE | BITMASK_SET_ROM_CE
sta DRB
lda #BITMASK_SET_REG_DISABLE
jsr latchctrl
rts

clonecheck:
;Init
lda #0
sta romaddr
sta DRA
tay
tax

lda #$10 ; ROM is mapped at $1000
sta romaddr+1

lda #$FF
sta DDRA
LDA DRB
ORA #BITMASK_SET_RMSBLE | BITMASK_SET_RLSBLE ; Latch 0 into address registers
STA DRB
AND #BITMASK_CLEAR_RMSBLE & BITMASK_CLEAR_RLSBLE ; Clear LE and output
STA DRB

tya
sta DDRA ; A input
LDA DRB
AND #BITMASK_CLEAR_ROM_CE & BITMASK_CLEAR_ROM_OE;Output ROM data
sta DRB
lda DRA
cmp (romaddr),y
bne clonecheckdone
clc ; Let's make sure

checknextaddress:
lda DRB
ora #BITMASK_SET_ROM_OE
sta DRB
iny
bne nexta
inx
beq clonecheckdone
clc ; Wonder if this is needed
lda romaddr+1
adc #1
sta romaddr+1
cmp #$20
beq clonecheckdone
stx DRA
jsr latchmsb2 ; Could unwrap this but only a little bit faster
nexta:
sty DRA
LDA #$FF
sta DDRA
; Set RLSBLE (bit 2) to latch the lower byte
LDA DRB
ORA #BITMASK_SET_RLSBLE
STA DRB
AND #BITMASK_CLEAR_RLSBLE
STA DRB
lda #0
sta DDRA
LDA DRB
AND #BITMASK_CLEAR_ROM_OE;Output ROM data
sta DRB
lda DRA
cmp (romaddr),y
beq checknextaddress ; Bytes match, continue testing
clonecheckdone:
sty romaddr
stx romaddr+1
ldx DRA ; Return byte found in X 
lda DRB
ora #BITMASK_SET_ROM_OE | BITMASK_SET_ROM_CE
sta DRB
rts

blankcheck:
;Init
lda #0
sta romaddr
sta romaddr+1
sta DRA
tay
tax
jsr latchctrl

lda #$FF
sta DDRA
LDA DRB
ORA #BITMASK_SET_RMSBLE | BITMASK_SET_RLSBLE ; Latch 0 into address registers
STA DRB
AND #BITMASK_CLEAR_RMSBLE & BITMASK_CLEAR_RLSBLE ; Clear LE and output
STA DRB

tya ; Clear A
sta DDRA ; A input
LDA DRB
AND #BITMASK_CLEAR_ROM_CE & BITMASK_CLEAR_ROM_OE ;Output ROM data
sta DRB
lda DRA
eor #$ff
bne checkdone

nextaddress:
lda DRB
ora #BITMASK_SET_ROM_OE
sta DRB
inx
bne next
iny
beq checkdone
sty DRA
jsr latchmsb2 ; Could unwrap this but only a little bit faster
next:
stx DRA
LDA #$FF
sta DDRA
; Set RLSBLE (bit 2) to latch the lower byte
LDA DRB
ORA #BITMASK_SET_RLSBLE
STA DRB
AND #BITMASK_CLEAR_RLSBLE
STA DRB
lda #0
sta DDRA
LDA DRB
AND #BITMASK_CLEAR_ROM_OE;Output ROM data
sta DRB
lda DRA
eor #$ff
beq nextaddress
checkdone:
stx romaddr
sty romaddr+1
ldx DRA ; Return byte found in X 
lda DRB
ora #BITMASK_SET_ROM_OE | BITMASK_SET_ROM_CE
sta DRB
rts

cloneROM:
LDA #BITMASK_SET_REG_DISABLE
JSR latchctrl  ; Latch the updated control signals

LDA #BITMASK_SET_REG_DISABLE | BITMASK_SET_VPE_ENABLE | BITMASK_SET_VPE_TO_VPP
JSR latchctrl  ; Latch the updated control signals

lda #$ff 
sta DDRA
jsr delay_short ; ~2ms

clonerom2:
lda #0 ; Address latches to 0
sta romaddr ; Zero LSB 
sta DRA
tay
LDA DRB
ORA #BITMASK_SET_RMSBLE
STA DRB
AND #BITMASK_CLEAR_RMSBLE
STA DRB

lda #$10

nextpage:
sta romaddr+1
sec 
sbc #$10 ; ROM is mapped offset $1000
sta DRA
LDA DRB
ORA #BITMASK_SET_RMSBLE
STA DRB
AND #BITMASK_CLEAR_RMSBLE
STA DRB
pageloop:
sty DRA ; Latch next address
LDA DRB
ORA #BITMASK_SET_RLSBLE
STA DRB
AND #BITMASK_CLEAR_RLSBLE
STA DRB

lda (romaddr),Y
sta DRA
lda DRB
AND #BITMASK_CLEAR_ROM_CE ; Programming pulse start
sta DRB
lda longdelay
beq defaultpulse
jsr delay_long
clc
bcc pulsed
defaultpulse:
lda #11
jsr delay_short
pulsed:
lda DRB
ORA #BITMASK_SET_ROM_CE ; Programming pulse end
sta DRB
iny
bne pageloop
lda romaddr+1
adc #0 ; C is set
cmp #$20
bne nextpage

lda #0
JSR latchctrl  ; Latch the updated control signals
rts

writerom: ; Expects byte to burn in A, ROM address in $XY
; Returns: Nothing, but leaves voltages ON!(!)
pha 
LDA #BITMASK_SET_REG_DISABLE
JSR latchctrl  ; Latch the updated control signals
LDA #BITMASK_SET_REG_DISABLE | BITMASK_SET_VPE_ENABLE | BITMASK_SET_VPE_TO_VPP
JSR latchctrl  ; Latch the updated control signals
lda #$ff
sta DDRA
jsr delay_long
pla

writeaddress:
;Expects byte to burn in A, ROM address in $XY, voltages set for burning
; Returns nothing
; Destroys X, Y, A and ctrl register.
pha
txa
jsr latchmsb
sty DRA
jsr latchlsb2
pla
sta DRA
;Falls through to writerom

writerom2: ; Expects address and voltages to be set and DDRA = $FF
lda DRB
AND #BITMASK_CLEAR_ROM_CE
sta DRB

lda #11 
jsr delay_short

lda DRB
ORA #BITMASK_SET_ROM_CE
sta DRB

 ; Leaves CTRL register with voltages on!
rts

erasew27c512: ; Assumes DDRA is $FF (output)
lda #0
sta DRA
lda DRB ; Make sure CE & OE not already LOW and clear address latches
ora #BITMASK_SET_ROM_CE|BITMASK_SET_ROM_OE | BITMASK_SET_RLSBLE | BITMASK_SET_RMSBLE
sta DRB
and #BITMASK_CLEAR_RLSBLE & BITMASK_CLEAR_RMSBLE ; Latch 0 
sta DRB
; Set CTRL_REGISTER to enable the regulator and set VPE/14V on A9
LDA #BITMASK_SET_REG_DISABLE
jsr latchctrl
LDA #BITMASK_SET_REG_DISABLE | BITMASK_SET_VPE_ENABLE | BITMASK_SET_A9_VPP_ENABLE
JSR latchctrl  ; Latch the updated control signals
lda #50
jsr delay_long ; Stabilize VPE

lda #$ff ; Set DRA to OUTPUT
sta DDRA
sta DRA

lda DRB
AND #BITMASK_CLEAR_ROM_CE
sta DRB

lda #106
jsr delay_long

lda DRB
ORA #BITMASK_SET_ROM_CE
sta DRB

lda #BITMASK_SET_REG_DISABLE
jsr latchctrl ; Disable high voltage regulator outputs - leave voltage reg ON
rts

;Routines
latchctrl:
  STA DRA            ; Store the low byte in DRA
  ; Set DDRA to OUTPUT
  LDA #$FF
  sta DDRA
latchctrl2:
  ; Set CTRL_LE (bit 2) to latch the lower byte
  LDA DRB
  ORA #BITMASK_SET_CTRL_LE
  STA DRB

  ; Clear CTRL_LE (bit 2) to release the latch
  ;LDA DRB ; Don't need to reload
  AND #BITMASK_CLEAR_CTRL_LE
  STA DRB
rts

latchlsb:
  STA DRA            ; Store the low byte in DRA
  ; Set DDRA to OUTPUT
  LDA #$FF
  sta DDRA
  ; Set RLSBLE (bit 2) to latch the lower byte
latchlsb2:
  LDA DRB
  ORA #BITMASK_SET_RLSBLE
  STA DRB

  ; Clear RLSBLE (bit 2) to release the latch
  ;LDA DRB ; Don't need to reload
  AND #BITMASK_CLEAR_RLSBLE
  STA DRB
rts

latchmsb:
sta DRA
latchmsb2:
lda #$FF
sta DDRA
; Set RMSBLE (bit 3) to latch the higher byte
LDA DRB
ORA #BITMASK_SET_RMSBLE
STA DRB

; Clear RMSBLE (bit 3) to release the latch
;LDA DRB ; Don't need to reload
AND #BITMASK_CLEAR_RMSBLE
STA DRB
rts

readrom:
  ; Takes ROM address to read in romaddr(+1)
  ; Returns data in Y
  lda romaddr+1
  sta DRA
  jsr latchmsb2
  LDA romaddr        ; Load the low byte of the address
  sta DRA
  jsr latchlsb2
  ; Set DRA to input (all bits of DDRA = 0)
  LDY #$00
  STY DDRA

  ; Set ROM_CE (bit 7) and ROM_OE (bit 4) to enable the ROM chip
  ;LDA DRB ; Don't need to reload
  AND #BITMASK_CLEAR_ROM_CE & BITMASK_CLEAR_ROM_OE
  STA DRB

  ; Load the byte from the ROM into the accumulator
  LDY DRA            ; Assuming the ROM is connected to DRA

  ; Set ROM_CE (bit 7) and ROM_OE (bit 4) to disable the ROM chip
  LDA DRB
  ORA #BITMASK_SET_ROM_CE | BITMASK_SET_ROM_OE
  STA DRB
rts

qsdelay:
lda #244
sta WTD1KDI ; 244 * 1024 = 249856 ~= quarter second
waitqs:
lda READTDI
bne waitqs ; Loop until timer runs out
rts

; jsr = 6 cycles
; sta (zp) = 3 cycles
; (WTD8DI -1) * 8 cycles
; We can ignore branches while timer not 0
;lda (zp) = 3 cycles
; bne = 2 cycles (not taken since timer expired)
; rts = 6 cycles
; = 20 + ((WTD8DI - 1) * 8) cycles

delay_short:
sta WTD8DI ; Divide by 8 = A contains ticks to delay/8
shortwait:
nop; Sample every 8 cycles instead of every 6
lda READTDI
bne shortwait
rts

delay_long:
sta WTD1KDI
wait1k:
lda READTDI
bne wait1k ; Loop until timer runs out
rts

;Returns byte in A - assumes 9600 baud = ~104us/bit, 1 cycle = 1us (1 MHz)
;We should call this ASAP when RX pin goes low - let's assume it just happened (13 cycles ago)
serial_rx:
;Minimum 13 cycles before we get here
lda #(15+19*BAUDSTEP) ; 1.5 period-ish ; 2 cycles - 15 for 9600 baud, 34 for 4800
jsr delay_short ; 140c
ldx #8 ; 2 cycles
;149 cycles to get here
serial_rx_loop: ;103 cycles
lda DRA ; Read RX bit 0 ; 3 cycles
lsr ; Shift received bit into carry - in many cases might be safe to just lsr DRA ; 2 cycles
ror inb ; Rotate into MSB 5 cycles
lda #(9+13*BAUDSTEP) ; 2 cycles ;9 for 9600 baud, 22 for 4800 baud (add 104us == 104 / 8 = 13)
jsr delay_short ; Delay until middle of next bit - overhead; 84 cycles
nop ; 2c
dex ; 2c
bne serial_rx_loop ; 3 cycles
;Should already be in the middle of the stop bit
; We can ignore the actual stop bit and use the time for other things
; Received byte in inb
lda inb ; Put in A
rts

serial_tx:
sta outb
lda #$fd ; Inverse bit 1
and DRA
sta DRA ; Start bit
lda #(8+13*BAUDSTEP) ; 2c ; 9600 = 8, 4800 = 21
jsr delay_short ; 20 + (8-1)*8 = 76c ; Start bit total 104 cycles - 104 cycles measured
nop ; 2c
nop ; 2c
ldx #8 ; 2c
serial_tx_loop:
lsr outb ; 5c
lda DRA ; 3c
bcc tx0 ; 2/3c
ora #2 ; TX bit is bit 1 ; 2c
bcs bitset ; BRA 3c
tx0:
nop ; 2c
and #$fd ; 2c
bitset:
sta DRA ; 3c
; Delay one period - overhead ; 101c total ; 103c measured
lda #(8+13*BAUDSTEP) ; 2c ; 9600 8, 4800 21
jsr delay_short ; 20 + (8-1)*8 = 76c
dex ; 2c
bne serial_tx_loop ; 3c
nop; 2c ; Last bit 98us counted, 100us measured
nop; 2c
nop; 2c
nop; 2c
lda DRA ;3c
ora #2 ; 2c
sta DRA ; Stop bit 3c
lda #(8+13*BAUDSTEP) ; 2c ; 9600 8, 4800 21
jsr delay_short
rts

serial_wstring:
ldy #0
txstringloop:
lda (stringp),y
beq stringtxd
jsr serial_tx
iny
bne txstringloop
stringtxd:
rts ; In case of overflow

bytetoa: ;This SR puts LSB in A and MSB in HXH - as ascii using hextoa.
pha
lsr
lsr
lsr
lsr
clc
jsr hextoa
sta xtmp
pla
and #$0F
jsr hextoa
rts

hextoa:
; wozmon-style
; and #%00001111  ; Mask LSD for hex print.
; Already masked when we get here.
ora #'0'        ; Add '0'.
cmp #'9'+1      ; Is it a decimal digit?
bcc ascr        ; Yes, output it.
adc #$06        ; Add offset for letter.
ascr:
rts

.segment "VECTORS6502"
.ORG $1ffa
.word nmi,reset,irq
.reloc