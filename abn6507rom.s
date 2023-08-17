; Written by Anders Nielsen, 2023
; License: https://creativecommons.org/licenses/by-nc/4.0/legalcode

.feature string_escapes ; Allow c-style string escapes when using ca65
.feature org_per_seg
.feature c_comments

I2CRAM = $00

I2CADDR = I2CRAM
inb     = I2CRAM +1
outb    = I2CRAM +2
xtmp    = I2CRAM +3
stringp = I2CRAM +4 ; (and +5)

timer2  = stringp + 2
rxcnt   = stringp + 3
txcnt   = rxcnt +1
runpnt  = txcnt +1
mode    = runpnt +2
serialbuf = runpnt +3 ; Address #12 / 0x0c

SCL     = 1 ; DRB0 bitmask
SCL_INV = $FE ; Inverted for easy clear bit
SDA     = 2 ; DRB1 bitmask
SDA_INV = $FD

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

.segment "USERLAND"
userland:
lda #<userstring
sta stringp
lda #>userstring
sta stringp+1
jsr ssd1306_wstring

halt:
jmp halt

userstring:
.asciiz "Now we can test whatever code we want with our new serial bootloader! #65uino"

.segment "RODATA"
.org $F000 ; Not strictly needed with CA65 but shows correct address in listing.txt
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

          lda #$BC ; Bit 0, 1 are SCL and SDA, bit 6 is input button.
          sta DDRB ; Set B register direction to #%10111100
          ; Reset state of DDRA is $00.

          lda #$ff ; Rest of port is input
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
lda #<ready
sta stringp
lda #>ready
sta stringp+1
jsr ssd1306_wstring
clc
bcc main

welcomemsg:
lda #<welcome
sta stringp
lda #>welcome
sta stringp+1
jsr ssd1306_wstring

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
lda #'O'
jsr serial_tx
lda #'K'
jsr serial_tx
bne wait ; BRA
quartersecond:
sta WTD1KDI ; 244 * 1024 = 249856 ~= quarter second
bne wait ; BRA

gonoserial:
jmp noserial
wait:
lda DRA ; Check serial 3c
and #$01 ; 2c
bne gonoserial ; 2c
tay ; A already 0
sta txcnt
sta timer2

rx:
dec timer2
beq rxtimeout ; Branch if timeout
lda DRA ; Check serial 3c
and #$01 ; 2c
bne rx ; Wait for RX until timeout
sta timer2 ; Reset timer
jsr serial_rx ; 6c
sta serialbuf, y
cpy #128-21 ; Leaves 9 bytes for stack
beq rx_err
iny
bne rx ; BRA (Y never 0)
rx_err:
sty rxcnt
lda #<overflow
sta stringp
lda #>overflow
sta stringp+1
jsr ssd1306_wstring
clc
bcc tx ; BRA
rxtimeout:
lda mode
beq txt
cmp #1
bne txt
;Time to parse data instead of txt - aka, our bootloader!
sty rxcnt
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
bit DRB
bvs waittorun
jsr ssd1306_clear

lda #$0c
sta runpnt
lda #0
sta runpnt+1

jmp (runpnt)

txt:
sty rxcnt
tx:
ldy txcnt
cpy rxcnt
beq gowait
lda serialbuf, y
jsr ssd1306_sendchar
jsr i2c_stop
inc txcnt
jmp tx
noserial:
lda READTDI
bne gowait ; Loop until timer runs out
jmp main ; loop
gowait:
jmp wait

ready:
.asciiz "Ready to load code... "

loading:
.asciiz "Loading... "

overflow:
.asciiz "Buffer overflow!"

loaded:
.asciiz "Loaded "

bytes:
.asciiz " bytes of data. Press user button to clear screen and run code."

welcome:
.byte "Hi!             I'm the 65uino! I'm a 6502 baseddev board. Come learn everything about me!"
.byte $00

;Routines

i2c_start:
  lda I2CADDR
  rol ; Shift in carry
  sta outb ; Save addr + r/w bit

  lda #SCL_INV
  and DDRB
  sta DDRB ; Start with SCL as input HIGH - that way we can inc/dec from here

  lda #SDA ; Ensure SDA is output low before SCL is LOW
  ora DDRB
  sta DDRB
  lda #SDA_INV
  and DRB
  sta DRB

  lda #SCL_INV ; Ensure SCL is low when it turns to output
  and DRB
  sta DRB
  inc DDRB ; Set to output by incrementing the direction register == OUT, LOW

  ; Fall through to send address + RW bit
  ; After a start condition we always send the address byte so we don't need to RTS+JSR again here

i2cbyteout:
  lda #SDA_INV ; In case this is a data byte we set SDA LOW
  and DRB
  sta DRB
  ldx #8
  bne first ; BRA - skip INC since first time already out, low
I2Cbyteloop:
  inc DDRB ; SCL out, low
first:
  asl outb ; MSB to carry
  bcc seti2cbit0 ; If bit was low
  lda DDRB       ; else set it high
  and #SDA_INV
  sta DDRB
  bcs wasone ; BRA doesn't exist on 6507
seti2cbit0:
  lda DDRB
  ora #SDA
  sta DDRB
  wasone:
  dec DDRB
  dex
  bne I2Cbyteloop

  inc DDRB

  lda DDRB ; Set SDA to INPUT (HIGH)
  and #SDA_INV
  sta DDRB

  dec DDRB ; Clock high
  lda DRB  ; Check ACK bit
  sec
  and #SDA
  bne nack
  clc ; Clear carry on ACK
  nack:
  inc DDRB ; SCL low
  rts

i2cbytein:
  ; Assume SCL is low from address byte
  lda DDRB  ; SDA, input
  and #SDA_INV
  sta DDRB
  lda #0
  sta inb
  ldx #8
i2cbyteinloop:
  clc
  dec DDRB ; SCL HIGH
  lda DRB ; Let's read after SCL goes high
  and #SDA
  beq got0
  sec
  got0:
  rol inb ; Shift bit into the input byte
  inc DDRB ; SCL LOW
  dex
  bne i2cbyteinloop

  lda DDRB ; Send NACK == SDA high (only single bytes for now)
  and #SDA_INV
  sta DDRB
  dec DDRB ; SCL HIGH
  inc DDRB ; SCL LOW
rts

i2c_stop:
  lda DDRB ; SDA low
  ora #SDA
  sta DDRB
  dec DDRB ; SCL HIGH
  lda DDRB ; Set SDA high after SCL == Stop condition
  and #SDA_INV
  sta DDRB
  rts

ssd1306_init:
  clc
  jsr i2c_start
  ldy #0
  initloop:
  lda ssd1306_inittab, y
  cmp #$ff
  beq init_done
  sta outb
  jsr i2cbyteout
  iny
  bne initloop ; BRA
  init_done:
  jsr i2c_stop
  rts

  ssd1306_clear:
  clc ; Write
  jsr i2c_start
  lda #$40 ; Co bit 0, D/C# 1
  sta outb
  jsr i2cbyteout
  ;outb is already 0
  ldy #0
  clearcolumn:
  jsr i2cbyteout
  jsr i2cbyteout
  jsr i2cbyteout
  jsr i2cbyteout
  dey
  bne clearcolumn ; Inner loop
  jsr i2c_stop
  rts

ssd1306_sendchar:
tay ; Save out byte
clc ; Write
jsr i2c_start
lda #$40 ; Co bit 0, D/C 1
sta outb
jsr i2cbyteout
;outb already 0
jsr i2cbyteout ; Send 0
lda fontc1-$20, y ; Get font column pixels
sta outb
jsr i2cbyteout
lda fontc2-$20, y ; Get font column pixels
sta outb
jsr i2cbyteout
lda fontc3-$20, y ; Get font column pixels
sta outb
jsr i2cbyteout
lda fontc4-$20, y ; Get font column pixels
sta outb
jsr i2cbyteout
lda fontc5-$20, y ; Get font column pixels
sta outb
jsr i2cbyteout
jsr i2cbyteout ; Send 0
jsr i2cbyteout ; Send 0
;Leaving i2c tx open
tya
jsr serial_tx
rts

ssd1306_wstring:
ldy #0
stringloop:
lda (stringp),y
beq sent
sty xtmp
jsr ssd1306_sendchar
ldy xtmp
iny
bne stringloop
sent:
jsr i2c_stop
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

;Returns byte in A - assumes 9600 baud = ~104us/bit, 1 cycle = 1us (1 MHz)
;We should call this ASAP when RX pin goes low - let's assume it just happened (13 cycles ago)
serial_rx:
;Minimum 13 cycles before we get here
lda #15 ; 1.5 period-ish ; 2 cycles
jsr delay_short ; 140c
ldx #8 ; 2 cycles
;149 cycles to get here
serial_rx_loop: ;103 cycles
lda DRA ; Read RX bit 0 ; 3 cycles
lsr ; Shift received bit into carry - in many cases might be safe to just lsr DRA ; 2 cycles
ror inb ; Rotate into MSB 5 cycles
lda #9 ; 2 cycles
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
lda #8 ; 2c
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
lda #8 ; 2c
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
lda #8 ; 2c
jsr delay_short
rts

printbyte:
    pha ; Save A
    jsr bytetoa
    pha
    lda xtmp
    jsr ssd1306_sendchar
    jsr i2c_stop
    pla
    jsr ssd1306_sendchar
    jsr i2c_stop
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
        ;    and #%00001111  ; Mask LSD for hex print.
        ; Already masked when we get here.
            ora #'0'        ; Add '0'.
            cmp #'9'+1      ; Is it a decimal digit?
            bcc ascr        ; Yes, output it.
            adc #$06        ; Add offset for letter.
        ascr:
            rts



ssd1306_inittab:
.byte $ae   ; Turn off display
.byte $d5   ; set display clock divider
.byte $f0   ; 0x80 default - $f0 is faster for less tearing
.byte $a8   ; set multiplex
.byte $3f   ; for 128x64
.byte $40   ; Startline 0
.byte $8d   ; Charge pump
.byte $14   ; VCCstate 14
.byte $a1   ; Segment remap
.byte $c8   ; Com output scan direction
.byte $20   ; Memory mode
.byte $00   ;
.byte $da   ; Com-pins
.byte $12
.byte $fe   ; Set contrast - full power!
.byte $7f   ; About half
.byte $d9   ; Set precharge
.byte $11   ; VCC state 11
.byte $a4   ; Display all on resume
.byte $af   ; Display on
.byte $b0, $10, $00 ; Page 0, column 0.
.byte $ff ; Stop byte

.include "./95char5x7font.s" ; Font

.segment "VECTORS6502"
.ORG $fffa
.word nmi,reset,irq
.reloc
