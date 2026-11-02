; hello.s - a minimal stand-alone ROM for the 65uino (and the emulator)
;
; Shows how little is needed to drive the board from scratch:
;   - bit-banged I2C on RIOT port B (PB0 = SCL, PB1 = SDA, open-drain by
;     toggling the data-direction bits, exactly like the 65uino BIOS does)
;   - SSD1306 init + text using a small 5x7 font
;   - the RIOT interval timer for a 50 ms tick
;   - the green LED on PB7 (active low) and the BTN on PB6 (active low)
;   - decimal mode (SED) for a BCD seconds counter
;
; Build:  ca65 hello.s -o hello.o && ld65 -C hello.cfg -o hello.bin hello.o
; License: CC BY-NC 4.0, like the 65uino project it was written for.

.setcpu "6502"

DRA     = $80
DDRA    = $81
DRB     = $82
DDRB    = $83
READT   = $84          ; read timer (IRQ disabled)
WT1K    = $97          ; write timer, /1024 prescaler (IRQ disabled)

OLED    = $3C << 1     ; SSD1306 I2C address, write

; Map the characters we have glyphs for onto glyph numbers.
.charmap $20, 0   ; ' '
.charmap $30, 1   ; '0'
.charmap $31, 2   ; '1'
.charmap $32, 3   ; '2'
.charmap $33, 4   ; '3'
.charmap $34, 5   ; '4'
.charmap $35, 6   ; '5'
.charmap $36, 7   ; '6'
.charmap $37, 8   ; '7'
.charmap $38, 9   ; '8'
.charmap $39, 10   ; '9'
.charmap $3A, 11   ; ':'
.charmap $41, 12   ; 'A'
.charmap $42, 13   ; 'B'
.charmap $43, 14   ; 'C'
.charmap $44, 15   ; 'D'
.charmap $45, 16   ; 'E'
.charmap $46, 17   ; 'F'
.charmap $47, 18   ; 'G'
.charmap $48, 19   ; 'H'
.charmap $49, 20   ; 'I'
.charmap $4A, 21   ; 'J'
.charmap $4B, 22   ; 'K'
.charmap $4C, 23   ; 'L'
.charmap $4D, 24   ; 'M'
.charmap $4E, 25   ; 'N'
.charmap $4F, 26   ; 'O'
.charmap $50, 27   ; 'P'
.charmap $51, 28   ; 'Q'
.charmap $52, 29   ; 'R'
.charmap $53, 30   ; 'S'
.charmap $54, 31   ; 'T'
.charmap $55, 32   ; 'U'
.charmap $56, 33   ; 'V'
.charmap $57, 34   ; 'W'
.charmap $58, 35   ; 'X'
.charmap $59, 36   ; 'Y'
.charmap $5A, 37   ; 'Z'
.charmap $21, 38   ; '!'
.charmap $2D, 39   ; '-'
.charmap $2E, 40   ; '.'
EOS = $FF

.segment "ZEROPAGE"
outb:   .res 1
ptr:    .res 2
ticks:  .res 1
secs:   .res 2          ; BCD seconds, low byte first
btnprev:.res 1
inv:    .res 1
bx:     .res 1
dx:     .res 1
wpage:  .res 1
wcol:   .res 1
cnt:    .res 1
ysave:  .res 1
tmp:    .res 1
buf:    .res 5

.segment "CODE"
reset:
        cld
        sei
        ldx #$7F
        txs
        lda #0
        sta DRB         ; port B outputs low: LED on, SCL/SDA ready to pull low
        sta DDRA
        sta secs
        sta secs+1
        sta ticks
        sta inv
        sta bx
        lda #2
        sta dx
        lda #$40
        sta btnprev
        lda #$80
        sta DDRB        ; PB7 (LED) output, everything else input/released
        lda #20         ; ~20 ms for the OLED to power up
        jsr delay

        lda #<init_tab
        sta ptr
        lda #>init_tab
        sta ptr+1
        lda #init_len
        sta cnt
        jsr send_cmds
        jsr oled_clear

        lda #0
        ldx #0
        ldy #<s_hello
        jsr print_str
        lda #1
        ldx #0
        ldy #<s_demo
        jsr print_str
        lda #3
        ldx #0
        ldy #<s_btn
        jsr print_str
        lda #4
        ldx #0
        ldy #<s_secs
        jsr print_str
        lda #7
        ldx #0
        ldy #<s_url
        jsr print_str
        jsr show_secs

loop:
        lda #49         ; 49 * 1024 cycles = ~50 ms at 1 MHz
        sta WT1K
@wait:  lda READT
        bne @wait

        jsr ball

        lda DRB         ; button edge detect
        and #$40
        cmp btnprev
        beq @nobtn
        sta btnprev
        cmp #0
        bne @nobtn      ; released
        lda inv
        eor #1
        sta inv
        ora #$A6        ; A6 = normal, A7 = inverse
        jsr cmd1
@nobtn:
        inc ticks
        lda ticks
        cmp #10
        beq @led
        cmp #20
        bne loop
        lda #0
        sta ticks
        sed             ; BCD increment
        clc
        lda secs
        adc #1
        sta secs
        lda secs+1
        adc #0
        sta secs+1
        cld
        jsr show_secs
@led:
        lda DRB         ; keep PB0..PB6 output latches at 0!
        and #$80
        eor #$80
        sta DRB
        jmp loop

; ---- helpers ----
delay:  ; A * 1024 cycles
        sta WT1K
@w:     lda READT
        bne @w
        rts

print_str:              ; A = page, X = column, Y = low byte of string in strings page
        sty ptr
        ldy #>strings
        sty ptr+1
        ; fall through
print_at:               ; A = page, X = column, ptr = string (EOS terminated)
        jsr oled_window
        jsr data_begin
        ldy #0
@ch:    lda (ptr),y
        cmp #EOS
        beq @done
        sty ysave
        sta tmp         ; glyph offset = index * 5
        asl
        asl
        clc
        adc tmp
        tay
        lda #5
        sta cnt
@col:   lda font,y
        jsr i2c_byte
        iny
        dec cnt
        bne @col
        lda #0          ; 1 pixel gap
        jsr i2c_byte
        ldy ysave
        iny
        bne @ch
@done:  jmp i2c_stop

show_secs:
        lda secs+1
        jsr hi_digit
        sta buf
        lda secs+1
        jsr lo_digit
        sta buf+1
        lda secs
        jsr hi_digit
        sta buf+2
        lda secs
        jsr lo_digit
        sta buf+3
        lda #EOS
        sta buf+4
        lda #<buf
        sta ptr
        lda #0
        sta ptr+1
        lda #4
        ldx #54
        jmp print_at
hi_digit:
        lsr
        lsr
        lsr
        lsr
lo_digit:
        and #$0F
        clc
        adc #1          ; glyph 1 = '0'
        rts

ball:   ; a 6 px ball bouncing along page 6
        lda #6
        ldx bx
        jsr oled_window
        jsr data_begin
        ldy #6
@erase: lda #0
        jsr i2c_byte
        dey
        bne @erase
        jsr i2c_stop
        lda bx
        cmp #120
        bne @n1
        lda #$FE
        sta dx
@n1:    lda bx
        bne @n2
        lda #2
        sta dx
@n2:    lda bx
        clc
        adc dx
        sta bx
        lda #6
        ldx bx
        jsr oled_window
        jsr data_begin
        ldy #0
@draw:  lda ballgfx,y
        jsr i2c_byte
        iny
        cpy #6
        bne @draw
        jmp i2c_stop

oled_clear:
        lda #0
        ldx #0
        jsr oled_window
        jsr data_begin
        ldy #0
@c:     lda #0
        jsr i2c_byte
        lda #0
        jsr i2c_byte
        lda #0
        jsr i2c_byte
        lda #0
        jsr i2c_byte
        dey
        bne @c
        jmp i2c_stop

oled_window:            ; A = first page, X = first column; window to page 7 / column 127
        sta wpage
        stx wcol
        jsr cmd_begin
        lda #$21
        jsr i2c_byte
        lda wcol
        jsr i2c_byte
        lda #127
        jsr i2c_byte
        lda #$22
        jsr i2c_byte
        lda wpage
        jsr i2c_byte
        lda #7
        jsr i2c_byte
        jmp i2c_stop

cmd1:                   ; send the single command in A
        pha
        jsr cmd_begin
        pla
        jsr i2c_byte
        jmp i2c_stop

send_cmds:              ; ptr = table, cnt = length
        jsr cmd_begin
        ldy #0
@l:     lda (ptr),y
        jsr i2c_byte
        iny
        cpy cnt
        bne @l
        jmp i2c_stop

cmd_begin:              ; START, address, control byte $00 (command stream)
        jsr i2c_start
        lda #OLED
        jsr i2c_byte
        lda #$00
        jmp i2c_byte

data_begin:             ; START, address, control byte $40 (data stream)
        jsr i2c_start
        lda #OLED
        jsr i2c_byte
        lda #$40
        jmp i2c_byte

; ---- bit-banged I2C: DDR bit = 1 pulls the line low, 0 releases it ----
i2c_start:              ; both lines released on entry
        lda DRB
        and #$FC        ; make sure the PB0/PB1 output latches are 0
        sta DRB
        lda DDRB
        ora #$02        ; SDA low while SCL high = START
        sta DDRB
        ora #$01        ; SCL low
        sta DDRB
        rts

i2c_stop:               ; SCL low on entry
        lda DDRB
        ora #$02        ; SDA low
        sta DDRB
        and #$FE        ; SCL high
        sta DDRB
        and #$FD        ; SDA high while SCL high = STOP
        sta DDRB
        rts

i2c_byte:               ; send A, MSB first; SCL low on entry/exit; C = 1 on NACK
        sta outb
        ldx #8
@bit:   lda DDRB
        asl outb
        bcs @one
        ora #$02        ; data 0: pull SDA low
        bne @set
@one:   and #$FD        ; data 1: release SDA
@set:   sta DDRB
        and #$FE        ; SCL high (slave samples)
        sta DDRB
        ora #$01        ; SCL low
        sta DDRB
        dex
        bne @bit
        and #$FD        ; release SDA for the ACK bit
        sta DDRB
        and #$FE        ; SCL high
        sta DDRB
        lda DRB
        and #$02
        cmp #$01        ; C = 1 if SDA high (NACK)
        lda DDRB
        ora #$01        ; SCL low
        sta DDRB
        rts

init_tab:
        .byte $AE             ; display off
        .byte $D5, $80        ; clock divide
        .byte $A8, $3F        ; multiplex 64
        .byte $D3, $00        ; display offset 0
        .byte $40             ; start line 0
        .byte $8D, $14        ; charge pump on
        .byte $20, $00        ; horizontal addressing
        .byte $A1, $C8        ; segment remap, COM scan remap (upright)
        .byte $DA, $12        ; COM pins
        .byte $81, $CF        ; contrast
        .byte $D9, $F1        ; pre-charge
        .byte $DB, $40        ; VCOMH
        .byte $A4, $A6, $AF   ; resume RAM, normal, display on
init_len = * - init_tab

ballgfx: .byte $18, $3C, $7E, $7E, $3C, $18

.align 256
strings:
s_hello: .byte "HELLO 65UINO!", EOS
s_demo:  .byte "EMULATOR DEMO", EOS
s_btn:   .byte "BTN: INVERT", EOS
s_secs:  .byte "SECONDS:", EOS
s_url:   .byte "WWW.IMANIA.DK", EOS

.include "font.s"

.segment "VECTORS"
        .word reset, reset, reset   ; NMI, RESET, IRQ (the 6507 has no NMI/IRQ pins)
