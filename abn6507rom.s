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

; Bitmasks for setting and clearing signals in Data Register B (DRB) (as hex)
BITMASK_RLSBLE   = $04
BITMASK_RMSBLE     = $08
BITMASK_ROM_OE    = $10
BITMASK_CTRL_LE   = $20
BITMASK_ROM_CE    = $80

; Bitmasks for additional signals (as binary)
BITMASK_VPE_TO_VPP    = %00000001
BITMASK_A9_VPP_ENABLE = %00000010
BITMASK_VPE_ENABLE    = %00000100
BITMASK_P1_VPP_ENABLE = %00001000
BITMASK_REG_DISABLE   = %10000000

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

buffer_length = cursor ; This is fine... Maybe
romaddr = runpnt
fifobufferpnt = rxcnt ; Means it'll be overwritten when we receive a command.
longdelay = txcnt

;ROM profile definition
VPP_PIN_MSK   = 1 ; 0 == OE/VPP, 1 == Pin 1
VPP_DROP_MSK  = 2 ; 0 == Dont drop, 1 == Drop the regulator voltage for programming and ID
ID_A9_VPP_MSK = 4 ; 0 == VPP = VPP during ID,  1 == VPP = VCC during ID
IDROM         = 8 ; 0 == Doesn't support ID, 1 == Supports ID 
RES2          = 16
RES3          = 32 
RES4          = 64 ; 
RES5          = 128

.SEGMENT "USERLAND"
.org $0e ; Just to make listing.txt match
userland:

jsr ssd1306_clear

lda #0
sta runpnt
sta runpnt+1

lda #'$'
jsr fastprint 
lda runpnt
jsr printbyte
lda #':'
jsr fastprint
lda #' '
;Jump to selection

;jsr identifyrom

;jsr checkblank

/*
; This snippet just twiddles LED's
; Set DDRA to $FF to configure port A as output
lda #$FF
sta DDRA

; Initialize stringp to 1 to represent the pattern of LED lights
lda #1
sta stringp

; Loop to continuously twiddle the LED pattern
twiddle:
    ; Rotate left the LED pattern stored in stringp
    lda stringp
    rol
    sta stringp

    ; Activate the LEDs according to the pattern
    jsr latchctrl       ; Control the LEDs connected to the control register
    lda stringp
    jsr latchmsb       ; Control the LEDs connected to the MSB register
    lda stringp
    jsr latchlsb       ; Control the LEDs connected to the LSB register

    ; Introduce a delay to slow down the LED pattern change
    jsr delay_long
    jsr delay_long

    ; Jump back to the beginning of the loop
    jmp twiddle
 */

;This snippet will clone what's on the internal ROM to a w27c512 in less than two seconds
;jsr clonetow27c512 ; Clone and verify (print to SSD1306)
;jmp halt 

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

/*
lda #$FA
ldy #10
ldx #0
jsr writerom
*/

; This snippet programs ROM contents to a 2732 ROM (if VPE is jumpered correctly on the back of the board and VPE calibrated to 21V )
/*print cloning

lda #50
sta longdelay ; Must be initialized to either 0 for 100us ROMs or number of ms for slow programming Roms

LDA #BITMASK_REG_DISABLE | BITMASK_P1_VPP_ENABLE
JSR latchctrl  ; Latch the updated control signals

;Let's make absolutely sure the regulator is stable 
lda #$ff 
sta DDRA
jsr delay_long ; ~256ms
lda #$ff 
jsr delay_long ; ~256ms
lda #$ff 
jsr delay_long ; ~256ms

;jsr clonerom2 ; Write 65uino ROM
*/

halt:
lda #$02 ; Bit 1 is serial TX (Output)
sta DDRA
;sty mode
jmp main ; Get ready for new code

fifobuffer:
.res 4

.segment "RODATA"
.org $1000  ; Start address for code (for clarity, not strictly needed)

nmi:
irq:
reset:
    cld        ; Clear decimal mode flag
    sei        ; Disable interrupts (may not be necessary with 6507)
    
    ; Set stack pointer and clear zero page RAM
    ldx #$7f   ; Load X register with 127 (stack starts from top of memory)
    txs        ; Transfer X to stack pointer
    lda #0     ; Clear accumulator
    
clearzp: 
    sta $00,x  ; Clear zero page RAM from $007F to $0000
    dex        ; Decrement counter
    bne clearzp  ; Continue clearing until every bit of RAM is clear

.ifdef runprogrammer ; Only need this if using the Relatively Universal ROM Programmer
    JSR latchctrl  ; Call latchctrl subroutine to latch 0 into control register 
.endif 

    lda #%01010000  ; Initialize DRB with bit 4 and 6 set to 1, rest to 0
    sta DRB
    lda #%10111100  ; Set B register direction: Bit 0, 1 are SCL and SDA, bit 6 is input button
    sta DDRB

    lda #%11111011  ; Set A register default - 
    sta DRA
    lda #$02       ; Set bit 1 of DDRA for serial TX (Output)
    sta DDRA

    lda #244
    jsr delay_long   ; Delay for a short time

    lda #$3C      ; Address of the device (78 on the back of the module is 3C << 1)
    sta I2CADDR
    jsr ssd1306_init  ; Initialize SSD1306 display
    lda #244
    jsr delay_long   ; Delay for a short time
    jsr ssd1306_clear  ; Clear display

; Main routine for controlling LED and handling button press
    jmp mainmenu ; Returns to main below

gomenu:
jmp selectormove

main:
    bit DRB               ; Check the status of DRB (Data Register B)
    bvc gomenu                    ;Button pressed, cancel this and go update menu
    bpl ledoff           ; If DRB is clear (LED on), branch to ledoff to turn it off
    lda DRB               ; Load DRB (LED off)
    and #$7f             ; Clear bit 7 to turn the LED on
    sta DRB               ; Store the modified value back to DRB
    jmp l71               ; Jump to l71 to continue execution

ledoff:
    lda DRB               ; Load DRB (LED on)
    ora #$80            ; Set bit 7 to turn the LED off
    sta DRB               ; Store the modified value back to DRB

l71:
    lda #244             ; Load accumulator with value 244 (approx. 16ms delay)
    bit DRB               ; Check the status of DRB
    bvs quartersecond   ; Button isn't pressed branch to quartersecond
    jmp selectormove       ; Button pressed, change main menu selection

quartersecond:
    sta WTD1KDI         ; Set timer for approximately quarter of a second (244 * 1024 ≈ 249856 cycles)
    bne wait               ; Branch (BRA - Branch Always)

gonoserial:
jmp noserial

wait:
bit DRB
bvc gomenu
lda DRA ; Check serial 3c
/* Flow control disabled
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
/* Flow control disabled
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
beq bootload
.ifdef runprogrammer
cmp #2 
beq programmer
.endif
bootload:
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
lda #250
jsr delay_long
jsr ssd1306_clear

jmp userland
.ifdef runprogrammer
programmer:
JMP runprogrammer
.endif

txt:
/* Flow control disabled
lda DRA ;
ora #4 ; CTS high
sta DRA
lda #$13 ; XOFF
jsr serial_tx ; Inform sender to chill while we write stuff to screen
*/

tx:
ldy txcnt           ;Check if this is the first byte of the frame
bne notfirst
lda serialbuf, y    ;If it is, we check if it's a command (not ascii)
cmp #$01 ; SOH
beq boot
cmp #$AA
.ifdef runprogrammer
beq programmer
.endif
bne notfirst
boot:
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
/* Flow control disabled
lda DRA ;
and #$fb ; CTS low
sta DRA

lda #$11 ; XON
jsr serial_tx
*/
gowait:
jmp wait

/*
stepperdriver:
sta mode
lda #$0C ; Set bits 2 and 3 as outputs
sta DDRA

; Initialize direction (default to clockwise, bit 3 = 0)
lda #$08
sta xtmp

more:
; Check button press (DRB bit 6 = low)
bit DRB      ; Load DRB register
bvs noleft ; Skip if bit 6 is high (button not pressed)
lda #8
sta xtmp
noleft:
bit DRA
bvs no_change
; Toggle direction
lda #0
sta xtmp

no_change:
ldx #1
spin:
lda xtmp     ; Load the direction from xtmp
ora #$04     ; Combine with step (bit 2)
sta DRA      ; Write to DRA (step + direction)
lda #61      ; 500uS delay
jsr delay_short
lda xtmp     ; Load direction again (to clear step while keeping direction)
sta DRA      ; Clear step (only direction bit remains)
lda #61
jsr delay_short
dex
bne spin

lda mode
jsr delay_long
jmp more
*/

.include "flappylarus.s" ; Flappy Larus game routines
.include "i2c.s" ; i2c rutines specifically for the 65uino. Provides i2c_start, i2cbyteout, i2cbytein, i2c_stop - expects a few 
.include "ssd1306.s" ; SSD1306 routines specifically for the 65uino. Provides ssd1306_init, ssd1306_clear, ssd1306_sendchar, ssd1306_setline, ssd1306_cmd, ssd1306_wstring, printbyte
;.include "programmer.s" ; ; Relatively Universal ROM Programmer 6502 Firmware

ready:.asciiz "Ready to load code... "
loading:.asciiz "Loading... "
overflow:.asciiz "OF!"
loaded: .asciiz "Loaded "
bytes: .asciiz " bytes of data."

welcome:
;.asciiz "Hi!             I'm the 65uino! I'm a 6502 baseddev board. Come learn everything about me!"
;.word $0000

flappylarusstr:     .asciiz "Flappy Larus" ;0 
codemonstr:         .asciiz "Run Userland" ;1
i2cscannerstr:      .asciiz "I2C Scanner" ;2
terminalstr:        .asciiz "Terminal" ;3
.ifdef runprogrammer
idromstr:           .asciiz "Check ROM ID" ;4
eraseromstr:        .asciiz "Erase ROM" ;5
blankcheckromstr:   .asciiz "Blank check IC" ;6
.endif
airplanestr:        .asciiz "Airplane mode" ;7

longpress:
jsr ssd1306_clear
lda itemsel
beq startflappylarus
cmp #1
beq gouserland
cmp #3
bne l477
jsr ssd1306_clear ; Clear screen to get ready for serial data
lda #0 
sta mode
jmp wait
l477:
.ifdef runprogrammer
cmp #4
beq startidentifyrom
cmp #5
beq starteraserom
cmp #6
beq startblankcheck
.endif
bne airplanemode

gouserland:
jmp userland

startflappylarus:
jsr flappylarus
sec
bcs pausehere

.ifdef runprogrammer
startblankcheck:
jsr blankcheck
sec
bcs pausehere

starteraserom:
jsr flappylarus
sec
bcs pausehere

startidentifyrom:
jsr identifyrom
;fall to pause
.endif 

pausehere:
lda #100
jsr delay_long ; Pause long enough that user probably has released button
actuallyhere:
bit DRB
bvs actuallyhere ; Button released
jmp main

airplanemode:
.byte $F2 ; JAM - locks up CPU, preventing more instructions from loading

menutable:
.word flappylarusstr
.word codemonstr
.word i2cscannerstr
.word terminalstr
.ifdef runprogrammer
.word idromstr
.word eraseromstr
.word blankcheckromstr
.endif
.word airplanestr
.word $0000

wait_getserial:
  lda #$02            ; Set bit 1 for serial TX (Output), clear bit 0 for serial RX (Input)
  sta DDRA            ; Store to configure port direction
wait_getserial2:
  lda DRA ; Check serial 3c
  and #$01 ; 2c
  bne wait_getserial2 ; 2c
  jsr serial_rx ; Get character
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
nop; 2c fix
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

mainmenu:
itemsel = runpnt
lda #8
sta itemsel
jmp shortpress ; BRA

gomain:
jmp main

selectormove:
;If button is pressed
lda #$ff
sta WTD1KDI
waitbounce:
lda READTDI
beq golongpress ; Timeout
bit DRB
bvs shortpress ; Button released before timeout 
bvc waitbounce ; BRA - Loop until timer runs out.

golongpress:
jmp longpress

shortpress: ; Notice! Not a subroutine!
;Increment selection
inc itemsel
reloop:
ldx #0
stx cursor
menuloop:
stx mode
txa
lsr ; Divide by two to convert pointer index to line number
jsr ssd1306_setline
lda #0
jsr ssd1306_zerocolumn
ldx mode
lda menutable, X ; Grab item pointer L
bne notprinted 
sta stringp
lda menutable+1,X ; Grab item pointer H
beq printedmenu
sta stringp+1
bne l693
notprinted:
sta stringp
lda menutable+1,X ; Grab item pointer H
sta stringp+1
l693:
stx mode ; Save counter value
ldy #0
txa 
lsr ; Convert index counter to item number
cmp itemsel
beq selected
lda tflags
and #$7f
sta tflags ; Invert off
clc
bcc notselected; BRA
selected:
lda tflags
ora #$80 ; Invert text
sta tflags
notselected:
jsr stringloop
ldx mode ; Restore
inx
inx 
bne menuloop ; BRA

printedmenu:
txa
lsr ; Convert index to item number
sec 
sbc #1
cmp itemsel
bcs notlastitem ; Selected item over number of items
lda #0
sta itemsel
beq reloop
notlastitem:
jmp main

.segment "VECTORS6502"
.ORG $1ffa
.word nmi,reset,irq
.reloc