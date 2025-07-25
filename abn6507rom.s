; Written by Anders Nielsen, 2023-2025
; License: https://creativecommons.org/licenses/by-nc/4.0/legalcode

.feature string_escapes ; Allow c-style string escapes when using ca65
.feature org_per_seg
.feature c_comments

;assembleprogrammer = YES
assemblelarus = 1
assemblesdr = 1

BAUDRATE=4800 ; Max 9600 - 4800 leaves room to do stuff without framing errors
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
tflags:   .res 1 ; Reserve 1 byte for tflags ; Bit usage: 0=reserved, 1=monitor byte selected, 3=i2c ack(0)/nack(1), 4=monitor h/l nibble select, 6=fast text mode, 7=invert text; others reserved.
serialbuf: .res 0 ; Reserve 1 byte for serialbuf - Used for text display and userland program storage

timer2  = stringp ; We're not going to be printing strings while waiting for timer2

buffer_length = cursor ; This is fine... Maybe
romaddr = runpnt
fifobufferpnt = rxcnt ; Means it'll be overwritten when we receive a command.
longdelay = txcnt

scan_addr = inb ; We don't need incoming byte while scanning i2c

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

jsr setup_si5351   ; Initialize Si5351

; Disable outputs
lda #3 
ldy #$ff    
jsr i2c_write_register

;jsr validate_si5351_init
;
;  lda #44
;  jsr read_i2c_reg
;  lda inb
;  jsr serialbyte ; See what comes back
; lda #$60
; sta I2CADDR

; lda #16
; ldy #$0F
; jsr i2c_write_register

lda #45
ldy #30
jsr i2c_write_register

lda #53
ldy #30
jsr i2c_write_register

 lda #166
 ldy #64
 jsr i2c_write_register

    lda #29
    ldy #12
    jsr i2c_write_register



; Reset PLLs
  lda #177
  ldy #$AC
  jsr i2c_write_register


; Enable available outputs
lda #3 
ldy #$f8
jsr i2c_write_register

here:
jmp here

; val = runpnt
; reg = runpnt+1

;  validate_si5351_init:
;     lda #<si5351_init_data
;     sta ptr
;     lda #>si5351_init_data
;     sta ptr+1

;     validate_next:
;     ldy #0
;     lda (ptr),y
;     cmp #$FF
;     beq done
;     sta reg
;     jsr serialbyte        ; Print register address

;     iny
;     lda (ptr),y
;     sta val

;     lda reg
;     jsr read_i2c_reg      ; read register value into inb

;     lda inb
;     cmp val
;     beq ok                ; Skip print if matched

;    ; lda #' '
;     jsr serial_tx

;     lda val
;     jsr serialbyte

;     ;lda #' '
;     jsr serial_tx

;     lda inb
;     jsr serialbyte

; ok:
;     lda #$0A
;     jsr serial_tx

;     clc
;     lda ptr
;     adc #2
;     sta ptr
;     lda ptr+1
;     adc #0
;     sta ptr+1

;     jmp validate_next

; done:
;     rts

; ;Read i2c ROM at address 0x50 and dump it as ascii via serial
; lda #$50          ; Load the I2C address of the ROM (0x50)
; sta I2CADDR       ; Store it in the I2CADDR variable
; jsr i2c_start     ; Start the I2C communication
; ;Send address  
; lda #$00          ; Load the address to read from (0x00)
; jsr i2cbyteout   ; Send the address to the ROM

; receiveloop:
; ;Read data
; jsr i2cbytein    ; Read the data from the ROM
; lda inb          ; Load the received data into the accumulator
; ;Convert to ASCII
; jsr hextoa        ; Convert the data to ASCII
; ;Send data via serial
; jsr serial_tx     ; Send the data via serial
; jmp receiveloop ; Loop back to receive the next byte

;print notinromstr
;print modeenabled

;lda #0
;sta xtmp
;jmp stepperdriver

;Jump to selection
;jsr identifyrom
;jsr checkblank

; Below should be placed in the zero page
; Inputs
; freq0:        .res 1   ; Frequency LSB
; freq1:        .res 1
; freq2:        .res 1
; freq3:        .res 1   ; Frequency MSB

; ; Internal
; dividend0:    .res 1   ; 750_750_751 (LSB first)
; dividend1:    .res 1
; dividend2:    .res 1
; dividend3:    .res 1

; quotient0:    .res 1   ; Output: quotient (LSB first)
; quotient1:    .res 1
; quotient2:    .res 1
; quotient3:    .res 1

; remainder0:   .res 1
; remainder1:   .res 1
; remainder2:   .res 1
; remainder3:   .res 1

; temp0:        .res 1
; temp1:        .res 1
; temp2:        .res 1
; temp3:        .res 1

; phase_lo:     .res 1
; phase_hi:     .res 1

halt:
;lda #$02 ; Bit 1 is serial TX (Output)
;sta DDRA
;sty mode
;jmp main ; Get ready for new code

fifobuffer:
;.res 4

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

    bit DRB
    bvc waithere ; Skip clearing RAM if USR btn held down during reset

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
;Fall through
jmp mainmenu

gomenu:
jmp selectormove

waithere:
bit DRB
bvc waithere ; Stay here until USR released

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

stepperdriver:
lda #10
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
ldx #20
spin:
lda xtmp     ; Load the direction from xtmp
ora #$04     ; Combine with step (bit 2)
sta DRA      ; Write to DRA (step + direction)
lda #15      ; 500uS delay
jsr delay_short
lda xtmp     ; Load direction again (to clear step while keeping direction)
sta DRA      ; Clear step (only direction bit remains)
lda #10
jsr delay_short
dex
bne spin

lda mode
jsr delay_long
jmp more


.ifdef assemblelarus
.include "flappylarus.s" ; Flappy Larus game routines
.endif
.include "i2c.s" ; i2c rutines specifically for the 65uino. Provides i2c_start, i2cbyteout, i2cbytein, i2c_stop - expects a few 
.include "ssd1306.s" ; SSD1306 routines specifically for the 65uino. Provides ssd1306_init, ssd1306_clear, ssd1306_sendchar, ssd1306_setline, ssd1306_cmd, ssd1306_wstring, printbyte
.ifdef assembleprogrammer 
.include "programmer.s" ; ; Relatively Universal ROM Programmer 6502 Firmware
.endif

.ifdef assemblesdr
.include "si5351.s" ; Si5351 routines specifically for the 65uino. Provides si5351_init, si5351_set_freq, si5351_set_phase
.endif

ready:.asciiz "Ready to load code... "
loading:.asciiz "Loading... "
overflow:.asciiz "OF!"
loaded: .asciiz "Loaded "
bytes: .asciiz " bytes of data."
modeenabled: .asciiz " enabled. Press RST to exit."
notinromstr: .asciiz "Code not in rom / not"
scanning: .asciiz "Scanning i2c busFound: "

welcome:
;.asciiz "Hi!             I'm the 65uino! I'm a 6502 baseddev board. Come learn everything about me!"
;.word $0000

flappylarusstr:     .asciiz "Flappy Larus" ;0 
userlandstr:        .asciiz "Run Userland $0E" ;1
codemonstr:         .asciiz "Code Monitor" ;2
i2cscannerstr:      .asciiz "I2C Scanner" ;3
terminalstr:        .asciiz "Terminal" ;4
.ifdef runprogrammer
idromstr:           .asciiz "Check ROM ID" ;4
eraseromstr:        .asciiz "Erase ROM" ;5
blankcheckromstr:   .asciiz "Blank check IC" ;6
.endif
airplanestr:        .asciiz "Airplane mode" ;7
stepperstr:         .asciiz "St. motor ctrl"

longpress:
jsr ssd1306_clear
lda itemsel
beq startflappylarus
cmp #1
beq gouserland
cmp #2
beq gomon
cmp #3
beq i2cscan ; I2C scanner
cmp #4
bne l477
jsr ssd1306_clear ; Clear screen to get ready for serial data
lda #0 
sta mode
jmp wait
l477:
.ifdef runprogrammer
cmp #5
beq startidentifyrom
cmp #6
beq starteraserom
cmp #7
beq startblankcheck
.endif
cmp #8
beq airplanemode
cmp #9 
jmp stepperdriver

gouserland:
jmp userland

gomon:
jsr monitor
jmp main

startflappylarus:
.ifdef flappylarus
jsr flappylarus
.else
jmp notinrom
.endif
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
print airplanestr
print modeenabled
.byte $F2 ; JAM - locks up CPU, preventing more instructions from loading

i2cscan:
jsr ssd1306_clear
print scanning
jsr i2c_scan
jmp main

notinrom:
print notinromstr
print modeenabled
jmp pausehere

menutable:
.word flappylarusstr
.word userlandstr
.word codemonstr
.word i2cscannerstr
.word terminalstr
.ifdef runprogrammer
.word idromstr
.word eraseromstr
.word blankcheckromstr
.endif
.word airplanestr
.word stepperstr
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

serialbyte:
    pha ; Save A
    jsr bytetoa
    pha
    lda xtmp
    jsr serial_tx
    pla
    jsr serial_tx
    pla ; Restore A
rts

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
itemsel = txcnt
lda #8
sta itemsel
bne shortpress ; BRA

gomain:
jmp main

debouncebtn:
;If button is pressed
lda #$ff
sta WTD1KDI
waitbounce:
lda READTDI
beq btntimeout
bit DRB
bvs short ; Button released before timeout 
bvc waitbounce ; BRA - Loop until timer runs out.
btntimeout:
sec
rts
short:
clc
rts

selectormove:
jsr debouncebtn
bcc shortpress

golongpress:
jmp longpress

menuitem = rxcnt

shortpress: ; Notice! Not a subroutine!
;Increment selection
inc itemsel
reloop:
ldx #0
stx cursor
jsr setcursor
menuloop:
stx menuitem
txa
lsr ; Divide by two to convert pointer index to line number
jsr ssd1306_setline
lda #0
jsr ssd1306_zerocolumn
ldx menuitem
lda menutable, X ; Grab item pointer L
bne notprinted 
sta stringp
lda menutable+1,X ; Grab item pointer H
beq printedmenu
sta stringp+1
bne l693 ; BRA
notprinted:
sta stringp
lda menutable+1,X ; Grab item pointer H
sta stringp+1
l693:
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
ora #$C0 ; Invert text and fast
sta tflags
notselected:
; Save x?
jsr ssd1306_wstring ; Destroys X
ldx menuitem ; Restore
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
lda #0 
sta mode
jmp main ; Release control

monitor:
selectedbyte = txcnt
lastnewpage = rxcnt

jsr ssd1306_clear

lda #$40 ; Fast
sta tflags

tya
sta lastnewpage
sta selectedbyte
sta runpnt+1
sta stringp
sta stringp+1
newscreen:
sta runpnt

nextline:
jsr setcursor
lda #'$'
jsr ssd1306_sendchar
lda runpnt
jsr printbyte
lda #':'
jsr ssd1306_sendchar
printram:
inc cursor ; Space
jsr setcursor
jsr checkline
lda (runpnt),Y
pha
lda runpnt
cmp selectedbyte
bne notit
lda tflags
ora #$C0 ; Invert and fast
bne it ; BRA
notit:
lda #$42 ; Leave selection and fast bit
and tflags
it:
sta tflags
pla
jsr printbyte
;;; If selected, we loop here
lda tflags
and #2 ; Selected flag
beq unselect
lda runpnt
cmp selectedbyte
bne afterunselect
changevalue:
bit DRB
bvs changevalue ; Wait for button press
jsr debouncebtn ; Short or long?
bcs startover ; Long press, deselect or next nibble

;Go back two cursor positions
dec cursor
dec cursor
jsr setcursor

lda tflags
and #$10 ; Check nibble flag (high nibble if set, low nibble if clear)
beq lownibble ; Branch to low nibble increment

; High nibble increment
lda stringp+1 
clc
adc #$10 ; Increment high nibble
sta stringp+1 ; Save offset
clc ; Ignore carry
adc (runpnt),y ; Add the current value
adc stringp ; Add the lower nibble
jmp nibbled ; Skip low nibble increment

lownibble:
lda stringp
clc
adc #1 ; Increment low nibble
and #$0F ; Mask out the high nibble
sta stringp
adc (runpnt),Y ; Add the incremented value to the low nibble

nibbled:
jsr printbyte
jmp changevalue ; BRA
startover:
lda tflags ; Check nibble flag
and #$10 ; Set = high nibble
bne savebyte
lda tflags
ora #$10 ; Set high nibble
sta tflags
l850:
bit DRB
bvc l850 ; Wait for button release
bne changevalue ; BRA
savebyte:
lda stringp+1 ; Load high nibble
clc
adc (runpnt),Y ; Load previous value
adc stringp ; Add low nibble - this might add to high nibble :/ 
sta (runpnt),Y
jsr ssd1306_clear
lda lastnewpage
jmp newscreen
;;;
unselect:
lda #$40 ; Clear nibble select, inversion and selection
sta tflags
afterunselect:
lda runpnt ; Increment pointer
adc #1
sta runpnt
and #$03   ; Check for newline
bne goprintram
lda cursor 
bne gonextline

l122:
bit DRB
bvs l122

jsr debouncebtn
bcc nextitem
bcs selectitem
;Select current item

goprintram:
jmp printram ; BRA

gonextline:
jmp nextline

nextitem:
lda selectedbyte
adc #1
sta selectedbyte
and #$1f ; Check for rollover to next page
beq nextpage ; Selected above range, move range
lda lastnewpage
bcc gnewscreen ; BRA

nextpage:
lda lastnewpage
adc #$20 ; One page+
sta lastnewpage
bne gnewscreen ; BRA

selectitem:
tya ; Assume 0
sta stringp ; Clear nibble offsets
sta stringp+1
lda #2
ora tflags
sta tflags
l877:
bit DRB
bvc l877
gnewscreen:
lda lastnewpage
jmp newscreen ; BRA

.segment "VECTORS6502"
.ORG $1ffa
.word nmi,reset,irq
.reloc