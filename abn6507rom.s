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
jsr flappylarus

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
ldy #0
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

    JSR latchctrl  ; Call latchctrl subroutine to latch 0 into control register 

    lda #%01010000  ; Initialize DRB with bit 4 and 6 set to 1, rest to 0
    sta DRB
    lda #%10111100  ; Set B register direction: Bit 0, 1 are SCL and SDA, bit 6 is input button
    sta DDRB

    lda #%11111001  ; Set A register direction: Rest of port is input
    sta DRA
    lda #$02       ; Set bit 1 of DDRA for serial TX (Output)
    sta DDRA

    jsr qsdelay   ; Delay for a short time

    lda #$3C      ; Address of the device (78 on the back of the module is 3C << 1)
    sta I2CADDR
    jsr ssd1306_init  ; Initialize SSD1306 display
    jsr qsdelay   ; Delay for a short time
    jsr ssd1306_clear  ; Clear display

    bit DRB       ; Test if DRB is set
    bvs welcomemsg  ; Branch if overflow flag is set (indicating button not pressed)
    lda #1        ; Set mode to 1 (normal mode)
    sta mode

    print ready  ; Print "ready" message

    clc           ; Clear carry flag
    bcc main      ; Branch to main subroutine

welcomemsg:
    print welcome  ; Print "welcome" message

; Main routine for controlling LED and handling button press

main:
    bit DRB               ; Check the status of DRB (Data Register B)
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
    bvs quartersecond  ; If the overflow flag is set, branch to quartersecond

    ; If the overflow flag is clear, set the timer for 16ms
    sta WTD64DI           ; Set timer for approximately 16ms
    jsr ssd1306_clear    ; Clear the display (assuming this subroutine exists)
    bne wait               ; Branch (BRA - Branch Always)

quartersecond:
    sta WTD1KDI         ; Set timer for approximately quarter of a second (244 * 1024 ≈ 249856 cycles)
    bne wait               ; Branch (BRA - Branch Always)

gonoserial:
jmp noserial

wait:
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
cmp #2 
beq programmer
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
jsr qsdelay
jsr qsdelay
jsr ssd1306_clear
lda #0
sta cursor
jsr ssd1306_setline
lda #0
jsr ssd1306_setcolumn

jmp userland
programmer:
JMP runprogrammer

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
beq programmer
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

.include "flappylarus.s" ; Flappy Larus game routines
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
w27c512rom:.asciiz "W27C512" 
foundwithid:.asciiz " found with ID: "
unrecognized:.asciiz "Unrecognized ROM"
blankcheckstr:.asciiz "Performing blank check... "
romblankstr: .asciiz "ROM blank. \n"
cloning: .asciiz "Cloning internal ROM.. "
verifying: .asciiz "Verifying.. \n"
romnotblankstr: .asciiz "First mismatch found at addr: $"
bytesverifiedstr: .asciiz " bytes verified OK"
pressbutton: .asciiz "Press USR button "
toexit: .asciiz "to exit calibration."
finished: .asciiz "Finished."

runprogrammer:
lda userland+1 ; Command
cmp #1
beq sendromtoserial
cmp #2
beq burnromfromserial
cmp #3 
beq eraserom
;cmp #4 
jmp calibrateVEP

stoppage = userland ; Steal a byte of RAM

sendromtoserial:
jsr ssd1306_clear
print verifying
lda #<userland+1  ;Executed from ROM we can use all of userland for buffer (faster)
sta fifobufferpnt
lda #>userland+1
sta fifobufferpnt+1
lda userland+2  ; Buffer length
sta buffer_length
lda userland+3  ; Addr LSB
sta romaddr
lda userland+4  ; MSB
sta romaddr+1
lda userland+5  ; Stoppage - stop reading ROM when this rolls over. 
sta stoppage

fetchromandsendserial:
jsr rom_fillbuf
ldy #$7f
jsr serial_wbuf
lda romaddr+1
cmp #$ff
bne notlastread
lda #0
sta stoppage
lda romaddr+1
notlastread:
cmp stoppage
bne fetchromandsendserial
    lda romaddr
    jsr latchlsb        ; Just to reflect the actual next address that would've been read
jmp finish

eraserom:
vendorid = userland+2 
serialid = userland+3
jsr ssd1306_clear
jsr idanderaserom
jmp finish

burnromfromserial:
;Enable VPE
lda #<userland+1  ;Executed from ROM we can use all of userland for buffer (faster)
sta fifobufferpnt
lda #>userland+1
sta fifobufferpnt+1
lda userland+2  ; Buffer length
sta buffer_length
lda userland+3
sta stoppage

lda #BITMASK_VPE_TO_VPP
jsr latchctrl
lda #BITMASK_REG_DISABLE | BITMASK_VPE_TO_VPP
jsr latchctrl
lda #$ff ; Max delay
jsr delay_long ; Should probably check with a scope how long it actually takes to charge the regulator output cap.
lda #0
sta romaddr
sta romaddr+1
lda #2 
sta DRA
sta DDRA
ldy #$7f
fetchandburnrom:
jsr serial_rbuf ; Takes a bit value for delay_short in Y
LDA #BITMASK_REG_DISABLE | BITMASK_VPE_TO_VPP | BITMASK_VPE_ENABLE ; This should be fixed to match the programming profile
JSR latchctrl  ; Latch the updated control signals
jsr rom_writebuf ; This needs to be fixed to allow for different profiles
LDA #BITMASK_REG_DISABLE | BITMASK_VPE_TO_VPP ; This should be fixed to match the programming profile
JSR latchctrl  ; Latch the updated control signals
lda romaddr+1
cmp #$ff
bne notlast
lda #0
sta stoppage
lda romaddr+1
notlast:
ldy #$7f
cmp stoppage
bne fetchandburnrom
lda #0 
jsr latchctrl
jsr delay_long
jsr serial_rbuf ; Let's tell sender we've finished with this
jmp finish

calibrateVEP:
jsr ssd1306_clear
print pressbutton
print toexit
lda #$0A
jsr ssd1306_sendchar
lda #BITMASK_REG_DISABLE
jsr latchctrl
waithere:
bit DRB
bvs waithere
finish:
print finished
lda #0
sta mode
jsr latchctrl
lda #2 ; Fix serial
sta DDRA
jmp main

;-----------------------------------------------------
; Function: serial_rbuf
; Description: Reads data from serial communication
;              and fills a buffer in RAM.
; Parameters: None
; Returns: None
;-----------------------------------------------------

serial_rbuf:
    lda #$02            ; Set bit 1 for serial TX (Output), clear bit 0 for serial RX (Input)
    sta DDRA            
    sta DRA ; Make sure 
    tya
    jsr delay_short     ; Delay long enough for a serial stop condition (high)
    lda #$AA            ; Better pattern than XON
    jsr serial_tx       ; Tell host we're ready to receive
    lda #244
    sta WTD1KDI ; 244 * 1024 = 249856 ~= quarter second
rxbufloop:
    lda READTDI         ; Timeout - hope we don't need to hang around longer
    beq rxbufdone   
    lda DRA             ; Read port register
    and #$01            ; Mask A to extract bit 0 (serial RX)
    bne rxbufloop       ; If bit is not clear, continue waiting - needs to be fixed with a timeout
    ldy #0              ; Clear y register
rxloop:
    lda DRA             ; Read port register
    and #$01            ; Mask to extract bit 0 (serial RX)
    bne rxloop       ; If bit is not clear, continue waiting
    jsr serial_rx       ; Read byte from serial receiver
    sta (fifobufferpnt),y   ; Store byte in buffer at index Y
    iny                 ; Increment index Y
    cpy buffer_length   ; Check if buffer is full
    bne rxloop          ; If not, continue reading
    rts
rxbufdone:
    lda #$1f 
    sta romaddr+1       ; Indicate we're done even if timeout
    rts                 ; Return from subroutine


;-----------------------------------------------------
; Function: rom_fillbuf
; Description: Reads data from an external ROM chip
;              and fills a buffer in RAM.
; Parameters: None
; Returns: None
;-----------------------------------------------------
rom_fillbuf:
    lda romaddr+1       ; Load high byte of ROM address into A
    jsr latchmsb        ; Call subroutine to set up ROM chip with MSB
    lda romaddr         ; Load low byte of ROM address into A
    jsr latchlsb       ; Call subroutine to set up ROM chip with LSB
    ldy #0              ; Clear Y register
fillbufloop:
    jsr readrom        ; Call subroutine to read byte from ROM returned in X
    txa
    sta (fifobufferpnt),y    ; Store byte in fifobuffer at index Y
    inc romaddr         ; Increment ROM address
    bne samepage        ; Branch if low byte did not overflow
    inc romaddr+1       ; Increment high byte if low byte overflowed
    lda romaddr+1       ; Load high byte of ROM address again    
    jsr latchmsb        ; Call subroutine to set up ROM chip with MSB again
samepage:
    iny                 ; Increment index Y
    cpy buffer_length   ; Check if all bytes have been read
    bne fillbufloop     ; If not, loop back to read next byte
    rts                 ; Return from subroutine

;-----------------------------------------------------
; Function: rom_writebuf
; Description: Reads data from buffer
;              and writes to ROM.
; Parameters: Assumes romaddr contains first ROM address
; Returns: None
;-----------------------------------------------------
rom_writebuf:          
    lda romaddr+1       ; Load high byte of ROM address into A
    jsr latchmsb        ; Call subroutine to set up ROM chip with MSB
    lda romaddr         ; Load low byte of ROM address into A
    jsr latchlsb       ; Call subroutine to set up ROM chip with LSB
    ldy #0              ; Clear Y register
writebufloop:
    LDA #BITMASK_REG_DISABLE | BITMASK_VPE_TO_VPP | BITMASK_VPE_ENABLE ; This should be fixed to match the programming profile
    sta DRA
    JSR latchctrl2  ; Latch the updated control signals
    lda (fifobufferpnt),y    ; Store byte in fifobuffer at index Y    
    ; Write ROM
    sta DRA
    jsr writerom2
    LDA #BITMASK_REG_DISABLE | BITMASK_VPE_TO_VPP ; This should be fixed to match the programming profile
    sta DRA
    jsr latchctrl2
    inc romaddr         ; Increment ROM address
    bne samepage2        ; Branch if low byte did not overflow
    inc romaddr+1       ; Increment high byte if low byte overflowed
    lda romaddr+1       ; Load high byte of ROM address again
    jsr latchmsb        ; Call subroutine to set up ROM chip with MSB again
samepage2:
    lda romaddr
    jsr latchlsb
    iny                 ; Increment index Y
    cpy buffer_length   ; Check if all bytes have been read
    bne writebufloop     ; If not, loop back to read next byte
    rts                 ; Return from subroutine

;-----------------------------------------------------

;-----------------------------------------------------
; Function: serial_wbuf
; Description: Sends data from a buffer serially.
; Parameters: None
; Returns: None
;-----------------------------------------------------
serial_wbuf:
    lda #$02            ; Set bit 1 for serial TX (Output)
    sta DDRA            ; Store to configure port direction
    sta DRA
    tya 
    jsr delay_short     ; Might need this to let the line go inactive long enough for receiver to sync start of byte
    lda #$AA            ; Start of frame
    jsr serial_tx    
    ldy #0              ; Clear Y register
txbufloop:
    lda (fifobufferpnt),y  ; Load byte from buffer at index Y
    jsr serial_tx       ; Send byte serially
    iny                 ; Increment index Y
    cpy buffer_length            ; Check if all bytes have been sent
    bne txbufloop       ; If not, loop back to send next byte
buftxd:
    rts                 ; Return from subroutine

;-----------------------------------------------
; Function: idanderaserom
; Description: ID's and erases an EEPROM
; Inputs: None
; Outputs: None
;-----------------------------------------------
idanderaserom:
    jsr identifyrom          ; Returns ROM ID in romaddr(+1)
    lda romaddr
    cmp vendorid            ; Check if it's a Winbond ROM
    bne badrom               ; Branch if not Winbond
    lda romaddr+1
    cmp serialid                   ; Check if it's a W27C512 ROM
    bne badrom               ; Branch if not W27C512
    ; If Winbond W27C512 detected, proceed with cloning
    print erasing           ; Print macro in macros.s
    jsr erasew27c512        ; First, erase the ROM
rts
badrom:
lda #$0A
jsr ssd1306_sendchar
print unrecognized
rts

;-----------------------------------------------
; Function: clonetow27c512
; Description: Cloning routine for W27C512 EEPROM
; Inputs: None
; Outputs: None
;-----------------------------------------------
clonetow27c512:
    lda #%01000000           ; Indicator cloning started
    jsr latchctrl

    jsr identifyrom          ; Returns ROM ID in romaddr(+1)
    lda romaddr
    cmp #$DA                ; Check if it's a Winbond ROM
    bne notwb               ; Branch if not Winbond
    lda romaddr+1
    cmp #8                   ; Check if it's a W27C512 ROM
    bne notwb               ; Branch if not W27C512

    ; If Winbond W27C512 detected, proceed with cloning

wipe:
    print erasing           ; Print macro in macros.s
    jsr erasew27c512        ; First, erase the ROM

;Second entry point
clonetow27c512nowipe:
    lda #0
    sta longdelay           ; Must be initialized to either 0 for 100us ROMs or number of ms for slow programming ROMs

    print cloning
    jsr cloneROM

    print verifying
    jsr clonecheck          ; Verify

    lda #%01000000
    jsr latchctrl

    lda #'$'
    jsr ssd1306_sendchar   ; Print "$" as the start of the address marker

    lda romaddr+1
    jsr printbyte           ; Print address of first mismatch ($1000 if OK)
    lda romaddr
    jsr printbyte
    print bytesverifiedstr

    rts

notwb:
    print unrecognized
    print foundwithid
    rts

;-----------------------------------------------
; Function: identifyrom
; Description: Identify ROM type
; Inputs: None
; Outputs: None
;-----------------------------------------------
identifyrom:
    jsr getromid
    stx romaddr             ; Store ROM ID temporarily
    sty romaddr+1

    cpx #$DA                ; Check if it's a Winbond ROM
    beq winbond
    cpx #$20                ; Check if it's an ST ROM
    beq st
    jmp fail                ; Jump to fail if unrecognized

winbond:
    cpy #$08
    bne fail                ; Jump to fail if not W27C512
    print w27c512rom        ; Print ROM type
    jmp nofail              ; Jump to nofail

st:
    cpy #$3D
    bne fail                ; Jump to fail if not M27C512
    print m27c512rom        ; Print ROM type
    jmp nofail              ; Jump to nofail

fail:
    print unrecognized
nofail:
    print foundwithid
    lda romaddr
    jsr printbyte           ; Print ROM ID
    lda romaddr+1
    jsr printbyte
    rts

;Returns manufacturer ID in X and device ID in Y
getromid:
lda #BITMASK_REG_DISABLE
jsr latchctrl
/*
lda romprofile
and #ID_A9_VPP_MSK
beq enablevppduringid
*/

lda #BITMASK_VPE_TO_VPP | BITMASK_REG_DISABLE | BITMASK_A9_VPP_ENABLE
jsr latchctrl
lda #$50
jsr delay_long ; Stabilize VPP
ldy #0
sty DRA
lda DRB
ora #BITMASK_RLSBLE | BITMASK_RMSBLE 
sta DRB
and #~BITMASK_RMSBLE & ~BITMASK_RLSBLE & ~BITMASK_ROM_CE & $FF ; &$FF to make inversions 8 bits
sta DRB
sty DDRA ; DRA input
and .lobyte(~BITMASK_ROM_OE) ; .lobyte() is an alternative to & $FF
sta DRB
ldx DRA
ora #BITMASK_ROM_OE
sta DRB
ldy #$ff
sty DDRA
ldy #$01
sty DRA
ora #BITMASK_RLSBLE
sta DRB
and #~BITMASK_RLSBLE &$FF
sta DRB
ldy #0
sty DDRA
and #~BITMASK_ROM_OE&$FF
sta DRB
ldy DRA
ora #BITMASK_ROM_OE | BITMASK_ROM_CE
sta DRB
lda #BITMASK_REG_DISABLE
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
ORA #BITMASK_RMSBLE  | BITMASK_RLSBLE ; Latch 0 into address registers
STA DRB
AND #~BITMASK_RMSBLE & $ff& ~BITMASK_RLSBLE ; Clear LE and output
STA DRB

tya
sta DDRA ; A input
LDA DRB
AND #~BITMASK_ROM_CE&$FF & ~BITMASK_ROM_OE&$FF;Output ROM data
sta DRB
lda DRA
cmp (romaddr),y
bne clonecheckdone
clc ; Let's make sure

checknextaddress:
lda DRB
ora #BITMASK_ROM_OE
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
ORA #BITMASK_RLSBLE
STA DRB
AND #~BITMASK_RLSBLE &$FF&$FF
STA DRB
lda #0
sta DDRA
LDA DRB
AND #~BITMASK_ROM_OE&$FF;Output ROM data
sta DRB
lda DRA
cmp (romaddr),y
beq checknextaddress ; Bytes match, continue testing
clonecheckdone:
sty romaddr
stx romaddr+1
ldx DRA ; Return byte found in X 
lda DRB
ora #BITMASK_ROM_OE | BITMASK_ROM_CE
sta DRB
rts

;This snippet checks if a ROM is blank
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
ORA #BITMASK_RMSBLE  | BITMASK_RLSBLE ; Latch 0 into address registers
STA DRB
AND #~BITMASK_RMSBLE & $ff& ~BITMASK_RLSBLE ; Clear LE and output
STA DRB

tya ; Clear A
sta DDRA ; A input
LDA DRB
AND #~BITMASK_ROM_CE&$FF & ~BITMASK_ROM_OE&$FF ;Output ROM data
sta DRB
lda DRA
eor #$ff
bne checkdone

nextaddress:
lda DRB
ora #BITMASK_ROM_OE
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
ORA #BITMASK_RLSBLE
STA DRB
AND #~BITMASK_RLSBLE &$FF
STA DRB
lda #0
sta DDRA
LDA DRB
AND #~BITMASK_ROM_OE&$FF;Output ROM data
sta DRB
lda DRA
eor #$ff
beq nextaddress
checkdone:
stx romaddr
sty romaddr+1
ldx DRA ; Return byte found in X 
lda DRB
ora #BITMASK_ROM_OE | BITMASK_ROM_CE
sta DRB
rts

cloneROM:
LDA #BITMASK_REG_DISABLE
JSR latchctrl  ; Latch the updated control signals
LDA #BITMASK_REG_DISABLE | BITMASK_VPE_ENABLE | BITMASK_VPE_TO_VPP
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
ORA #BITMASK_RMSBLE 
STA DRB
AND #$ff & ~BITMASK_RMSBLE
STA DRB

lda #$10

nextpage:
sta romaddr+1
sec 
sbc #$10 ; ROM is mapped offset $1000
sta DRA
LDA DRB
ORA #BITMASK_RMSBLE 
STA DRB
AND #$ff & ~BITMASK_RMSBLE
STA DRB
pageloop:
sty DRA ; Latch next address
LDA DRB
ORA #BITMASK_RLSBLE
STA DRB
AND #~BITMASK_RLSBLE & $FF
STA DRB

lda (romaddr),Y
sta DRA
lda DRB
AND #~BITMASK_ROM_CE&$FF ; Programming pulse start
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
ORA #BITMASK_ROM_CE ; Programming pulse end
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
LDA #BITMASK_REG_DISABLE
JSR latchctrl  ; Latch the updated control signals
LDA #BITMASK_REG_DISABLE | BITMASK_VPE_ENABLE | BITMASK_VPE_TO_VPP ; Probably want to make this part separate or dynamic. 
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
AND #~BITMASK_ROM_CE&$FF
sta DRB

lda #11 ; Hardcoded to W27C512. Must fix. 
jsr delay_short

lda DRB
ORA #BITMASK_ROM_CE
sta DRB

 ; Leaves CTRL register with voltages on!
rts

erasew27c512: ; Assumes DDRA is $FF (output)
lda #0
sta DRA
lda DRB ; Make sure CE & OE not already LOW and clear address latches
ora #BITMASK_ROM_CE|BITMASK_ROM_OE | BITMASK_RLSBLE | BITMASK_RMSBLE 
sta DRB
and #~BITMASK_RLSBLE & ~BITMASK_RMSBLE & $ff; Latch 0 
sta DRB
; Set CTRL_REGISTER to enable the regulator and set VPE/14V on A9
LDA #BITMASK_REG_DISABLE
jsr latchctrl
lda #50 ; ~50ms
jsr delay_long ; Stabilize VPE
LDA #BITMASK_REG_DISABLE | BITMASK_VPE_ENABLE | BITMASK_A9_VPP_ENABLE
JSR latchctrl  ; Latch the updated control signals
lda #50; ~50ms
jsr delay_long ; Stabilize VPE

lda #$ff ; Set DRA to OUTPUT
sta DDRA
sta DRA

lda DRB
AND #~BITMASK_ROM_CE&$FF
sta DRB

lda #106
jsr delay_long

lda DRB
ORA #BITMASK_ROM_CE
sta DRB

lda #BITMASK_REG_DISABLE
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
  ORA #BITMASK_CTRL_LE
  STA DRB

  ; Clear CTRL_LE (bit 2) to release the latch
  ;LDA DRB ; Don't need to reload
  AND #~BITMASK_CTRL_LE &$FF
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
  ORA #BITMASK_RLSBLE
  STA DRB

  ; Clear RLSBLE (bit 2) to release the latch
  ;LDA DRB ; Don't need to reload
  AND #~BITMASK_RLSBLE & $FF
  STA DRB
rts

latchmsb:
sta DRA
latchmsb2:
lda #$FF
sta DDRA
; Set RMSBLE (bit 3) to latch the higher byte
LDA DRB
ORA #BITMASK_RMSBLE 
STA DRB

; Clear RMSBLE (bit 3) to release the latch
;LDA DRB ; Don't need to reload
AND #$ff & ~BITMASK_RMSBLE
STA DRB
rts

readrom:
  ; Takes ROM address to read in romaddr(+1)
  ; Returns data in X
  lda romaddr+1
  sta DRA
  jsr latchmsb2
readrom2:  
  LDA romaddr        ; Load the low byte of the address
  sta DRA
  jsr latchlsb2
  ; Set DRA to input (all bits of DDRA = 0)
  LDX #$00
  STX DDRA

  ; Set ROM_CE (bit 7) and ROM_OE (bit 4) to enable the ROM chip
  ;LDA DRB ; Don't need to reload
  AND #~BITMASK_ROM_CE& $FF & ~BITMASK_ROM_OE&$FF
  STA DRB

  ; Load the byte from the ROM into the accumulator
  LDX DRA            ; Assuming the ROM is connected to DRA

  ; Set ROM_CE (bit 7) and ROM_OE (bit 4) to disable the ROM chip
  LDA DRB
  ORA #BITMASK_ROM_CE | BITMASK_ROM_OE
  STA DRB
rts

wait_getserial:
  lda #$02            ; Set bit 1 for serial TX (Output), clear bit 0 for serial RX (Input)
  sta DDRA            ; Store to configure port direction
wait_getserial2:
  lda DRA ; Check serial 3c
  and #$01 ; 2c
  bne wait_getserial2 ; 2c
  jsr serial_rx ; Get character
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

.segment "VECTORS6502"
.ORG $1ffa
.word nmi,reset,irq
.reloc