; The code below writes text to a 128x64 SSD1306 i2c OLED
; and reads out raw temperature from a BMP180 i2c module
; Written by Anders Nielsen, 2023
; License: https://creativecommons.org/licenses/by-nc/4.0/legalcode

.feature string_escapes ; Allow c-style string escapes when using ca65
.feature org_per_seg
.feature c_comments

I2CRAM = $00

I2CADDR = I2CRAM
inb    = I2CRAM +1
outb   = I2CRAM +2
I2CREG = I2CRAM +3
xtmp   = I2CRAM+4
ytmp   = I2CRAM+5
stringp = I2CRAM+6 ; & +1


;I2CADDR = $78 ; 0x3c << 1
SDA   = 2; DRB0 bitmask
SDA_INV = $FD
SCL   = 1; DRB1 bitmask
SCL_INV = $FE

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
.org $0008
jmp main

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

          lda #$BC ; Bit 0, 1 are SCL, SDA, bit 6 is input button
          sta DDRB ; Set whole B register to output
          ; Reset state of DDRA is $00.

          lda #$ff ; Rest of port is input
          sta DRA ; And it's supposed to start high
          lda #$02 ; Bit 1 is serial TX (Output)
          sta DDRA

lda #'O'
jsr serial_tx
lda #'K'
jsr serial_tx
lda #$0D
jsr serial_tx
lda #$0A
jsr serial_tx



          ldy #$7f
  i2cscan:
          tya
          sta I2CADDR
          jsr i2c_start
          jsr i2c_stop
          dey
          bne i2cscan

jsr i2c_test
ldy #$2e ; BMP180 cmd to read uncompensated temperature
lda #$f4 ; Register
jsr i2c_write

;On cold start we need to wait a bit here to let OLED voltages stabilize

jsr delay_qs

lda #$3C ; 78 on back of module is 3C << 1.
sta I2CADDR
jsr ssd1306_init
jsr ssd1306_clear
lda #0
jsr ssd1306_setpage

lda #<welcome
sta stringp
lda #>welcome
sta stringp+1
jsr ssd1306_wstring

lda #7
jsr ssd1306_setpage
jsr bmp180_printtemp

lda #0
jsr ssd1306_setpage

main:
lda DRA ; Check for serial RX
and #$01 ;
bne noserial ; Break
jsr serial_rx
jsr ssd1306_wchar
noserial:
;jsr i2c_test
;ldy #$2e ; BMP180 cmd to read uncompensated temperature
;lda #$f4 ; Register
;jsr i2c_write

bit DRB
bpl ledoff ; LED on, turn it off
lda DRB    ; LED off, turn it on
and #$7f ; Bit low = ON
sta DRB
jmp l57
ledoff:
lda DRB
ora #$80
sta DRB
l57:
lda #244
bit DRB ; Button is in the V flag. Button pressed = 0, unpressed = 1.
bvs quartersecond
jsr ssd1306_clear
lda #244
sta WTD64DI ; 244*64 = 15616 ~= 16ms
bne wait ; BRA
quartersecond:
sta WTD1KDI ; 244 * 1024 = 249856 ~= quarter second

wait:
lda DRA ; Check for serial RX
and #$01 ;
beq main ; Receiving - Break wait
lda READTDI
bne wait ; Loop until timer runs out

jmp main ; loop

welcome:
.byte "Hi!             I'm the 65uino! I'm a 6502 baseddev board. Come learn everything about me!"
.byte $00

delay_qs:
lda #244
sta WTD1KDI ; 244 * 1024 = 249856 ~= quarter second
coldwait:
lda READTDI
bne coldwait ; Loop until timer runs out
rts

delay_short:
sta WTD8DI ; Divide by 8 = A contains ticks to delay/8
shortwait:
lda READTDI
bne shortwait
rts

bmp180_printtemp:
lda #$77
sta I2CADDR
lda #$F7 ; LSB
jsr i2c_read
jsr bytetoa
pha ; LSN
tya
pha ; MSN

lda #$F6 ; MSB
jsr i2c_read
jsr bytetoa
pha ; LSN
tya
pha ; MSN

lda #$3C ; 78 on back of module is 3C << 1.
sta I2CADDR
pla
jsr ssd1306_wchar
pla
jsr ssd1306_wchar
pla
jsr ssd1306_wchar
pla
jsr ssd1306_wchar
rts

i2c_start: ; i2c addr in I2CADDR and RW bit is in C
  lda I2CADDR
  rol ; Shift in carry
  sta outb ; Save addr + rw bit

  lda #SCL_INV ; Start with SCL as INPUT HIGH. If bit 0 is used we can continue from here with inc/dec.
  and DDRB
  sta DDRB

  lda #SDA ; Ensure SDA is output low before SCL is LOW
  ora DDRB
  sta DDRB
  lda #SDA_INV
  and DRB
  sta DRB

  lda #SCL_INV ; Ensure SCL is low when output (currently high because it's an input)
  and DRB
  sta DRB
  inc DDRB ; Set to output by incrementing the direction register
  ; Fall through to send address + RW bit.
  ; After a start condition we always SEND the address byte, so we don't need to return.

  ; Send an i2c byte - byte is in outb
  ; From here on we can assume OUTPUTs are LOW and INPUTS are HIGH.
  ; Maybe some of the juggling above is not necessary but let's not assume for now
i2cbyteout:
  lda #SDA_INV ; In case this is a data byte we set SDA LOW
  and DRB
  sta DRB
  ldx #8
  bne first ; BRA - skip the inc since SCL is already OUTPUT LOW
I2CADDRloop: ; At start of loop SDA and SCL are both OUTPUT LOW
  inc DDRB ;
first:
  asl outb ; Put MSB in carry
  bcc seti2cbit0 ; If data bit was low
  lda DDRB       ; else set it high
  and #SDA_INV
  sta DDRB
  bcs wasone ; BRA
seti2cbit0:
  lda DDRB
  ora #SDA
  sta DDRB
  wasone:
  dec DDRB
  dex
  bne I2CADDRloop

  inc DDRB
  lda DDRB ; Set SDA to INPUT (=HIGH)
  and #SDA_INV
  sta DDRB

  dec DDRB ; Set clock HIGH
  lda DRB ; Check ACK bit
  clc
  and #SDA
  bne nack ; Didn't go low?
  sec ; Set carry to indicate ACK
  nack:
  inc DDRB ; SCL low
  rts

i2cbytein: ; Assume SCL is LOW
  lda DDRB       ; Set SDA to input
  and #SDA_INV
  sta DDRB
  lda #0
  sta inb
  ldx #8
byteinloop:
  clc ; Clearing here for more even cycle
  dec DDRB ; SCL HIGH
  lda DRB; Let's read after SCL goes high
  and #SDA
  beq got0
  sec
  got0:
  rol inb ; Shift carry into input byte
  inc DDRB ; SCL LOW
  dex
  bne byteinloop
  lda DDRB ; Send NACK == SDA high (because we're ony fetching single bytes)
  and #SDA_INV
  sta DDRB
  dec DDRB ; SCL HIGH
  inc DDRB ; SCL LOW
rts ; Input byte in inb

i2c_stop:
  lda DDRB ; SDA low
  ora #SDA
  sta DDRB
  dec DDRB ; SCL HIGH
  lda DDRB       ; Set SDA high after SCL == Stop condition.
  and #SDA_INV
  sta DDRB
  rts

i2c_test:
  lda #$77 ; Address $77 = BMP180 address
  sta I2CADDR
  lda #$D0 ; Register in A
  jsr i2c_read
  sec
  rts
  failed:
  lda #0
  rts

;i2c_read takes i2c address in I2CADDR and register in A
; Returns: Data byte in A
i2c_read:
  pha
  clc ; We "write" the register we want to read from
  jsr i2c_start
  bcc readfail
  pla
  sta outb
  jsr i2cbyteout
  ;jsr i2c_stop
  sec ; Now we read the register data
  jsr i2c_start ; Restart and read byte
  jsr i2cbytein
  jsr i2c_stop
  lda inb
  readfail:
  rts

;i2c_write takes i2c address in I2CADDR, register in A, and data in Y
; Returns: Last ACK in C
  i2c_write:
  pha ; Save output byte
  clc ; "write" the address
  jsr i2c_start
  bcc wreadfail
  pla ; Load output byte
  sta outb
  clc ; Clear to write
  jsr i2cbyteout
  tya
  sta outb
  jsr i2cbyteout
  jsr i2c_stop
  wreadfail:
  rts

ssd1306_w:
  pha ; I2COUTBYTE
  clc ; Write flag
  jsr i2c_start
  lda #$40 ; Co bit 0, D/C# 1
  sta outb
  jsr i2cbyteout
  pla ; I2COUTBYTE
  sta outb
  jsr i2cbyteout
  jsr i2c_stop
  rts

ssd1306_clear:
clc
jsr i2c_start
lda #$40 ; Co bit 0, D/C# 1
sta outb
jsr i2cbyteout
ldx #64
clearrow:
ldy #128
stx xtmp ; Save outer counter
clearcolumn:
lda #0
clc ; Clear previous ACK
jsr i2cbyteout ; outb should stay 0 for the duration
dey
bne clearcolumn
ldx xtmp ; Restore counter
dex
bne clearrow
jsr i2c_stop
rts

ssd1306_clearcolumn:
lda #$21 ; Set column cmd
jsr ssd1306_cmd
;lda #$40 ; Data byte
;sta outb
;jsr i2cbyteout
lda #0 ; Start column 0
sta outb
jsr i2cbyteout
lda #$7f ; End column 127
sta outb
jsr i2cbyteout
jsr i2c_stop
rts

; Takes page (0-7) in A
;Destroys ytmp
ssd1306_setpage:
sta ytmp
jsr ssd1306_clearcolumn
lda #$22 ; Set page cmd
jsr ssd1306_cmd
;lda #$40 ; Data byte?
;sta outb
;jsr i2cbyteout
lda ytmp
beq page0
sta outb
jsr i2cbyteout
dec ytmp
lda ytmp
sta outb
jsr i2cbyteout
pageset:
jsr i2c_stop
rts

page0:
sta outb
jsr i2cbyteout
lda #7
sta outb
jsr i2cbyteout
jsr i2c_stop
rts

; ssd1306_cmd takes command in A
;Overwrites I2CADDR
; Returns: Nothing
ssd1306_cmd:
pha ; I2COUTBYTE
lda #$3c
sta I2CADDR
clc ; Write flag
jsr i2c_start
lda #0 ; Co = 0, D/C# = 0
sta outb
jsr i2cbyteout
pla
sta outb
jsr i2cbyteout
;jsr i2c_stop
rts

ssd1306_wchar:
  jsr ssd1306_sendchar
  jsr i2c_stop
  rts

ssd1306_wstring:
  ldy #0
  stringloop:
  lda (stringp),y
  beq sent
  sty ytmp
  pha
  jsr serial_tx
  pla
  jsr ssd1306_sendchar
  ldy ytmp
  iny
  bne stringloop
  sent:
  jsr i2c_stop
  rts

ssd1306_sendchar:
sec
sbc #$20 ; We start from 0x20
pha ; Save out byte
lda #$3C ; 78 on back of module is 3C << 1.
sta I2CADDR
clc ; Write
jsr i2c_start
lda #$40 ; Co bit 0, D/C# 1
sta outb
jsr i2cbyteout
lda #0
sta outb
jsr i2cbyteout
pla ; Fetch out byte
tay
lda fontc1, y
sta outb
jsr i2cbyteout
lda fontc2, y
sta outb
jsr i2cbyteout
lda fontc3, y
sta outb
jsr i2cbyteout
lda fontc4, y
sta outb
jsr i2cbyteout
lda fontc5, y
sta outb
jsr i2cbyteout
lda #0
sta outb
jsr i2cbyteout
clc ; Assuming ACK in carry, outb still 0
lda #0
jsr i2cbyteout
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
  bne initloop; BRA
  init_done:
  JSR i2c_stop
  rts

ssd1306_inittab:
.byte $ae		;turn off display
.byte $d5		;set display clock div
.byte $f0		;ratio 0x80 default - speed up with $f0 to minimize screen tearing
.byte $A8		;set multiplex
.byte $3f		; 128x64
;.byte $d3		;set display offset
;.byte $00    ; None
.byte $40		;set startline
.byte $8d		;charge pump
.byte $14		;vccstate 14
.byte $a1 ; Segment re-map
.byte $c8 ; Com output scan direction
.byte $20		;memorymode
.byte $00 ;
.byte $da		;set com pins
.byte $12		; 02 128x32 12 ; ??
.byte $81	  ; set contrast
.byte $7f		;contrast, $3f = One quarter
.byte $d9		;set precharge
.byte $11		;vcc state f1
;.byte $db		;set vcom detect
;.byte $20   ; 0.77V (Default)
.byte $a4		;display all on resume
;.byte $a6		;A6 = normal display, A7 = invert
.byte $af		;display on
.byte $b0, $10, $00 ; Zero page and column
.byte $ff ; Stop byte

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


bytetoa: ;This SR puts LSB in A and MSB in Y - as ascii using hextoa.
    pha
    lsr
    lsr
    lsr
    lsr
    clc
    jsr hextoa
    tay
    pla
    and #$0F
    jsr hextoa
    rts

 serial_tx: ; Takes output in A
sta outb
lda #$fd
and DRA
sta DRA ; Start BIT
; Delay one period
lda #21
jsr delay_short
ldx #8
serial_tx_loop:
lsr outb
lda DRA
bcc tx0
ora #2 ; TX bit is bit 1
bcs bitset
tx0:
and #$fd
bitset:
sta DRA
; Delay one period - overhead
lda #21
jsr delay_short
dex
bne serial_tx_loop
lda DRA
ora #2
sta DRA ; Stop bit
lda #21
jsr delay_short
rts

serial_rx: ; Returns byte in A - assumes 4800 baud
;We're only here if RX pin is low - let's assume it just happened
lda #31 ; 1.5 period
jsr delay_short
ldx #8
serial_rx_loop:
lda DRA ; Read RX bit 0
lsr ; Shift received bit into carry - in many cases it might be safe to just lsr DRA
ror inb ; Rotate into MSB
lda #21
jsr delay_short ; Delay until middle of next bit
dex
bne serial_rx_loop
lda #11
jsr delay_short ; Delay until start of stop bit so we don't mistake last bit of 0 for a new start bit
;We can ignore stop bit and use the time for other things
;Received byte in inb
lda inb
rts

.include "95char5x7font.s"

.segment "VECTORS6502"
.ORG $fffa
.word nmi,reset,irq
.reloc
