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
  lda #0
  sta cursor
  sta tflags ; Reset scroll
  jsr ssd1306_setline
  lda #0
  jsr ssd1306_setcolumn
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

  lda #$d3 ; Clear scroll
  jsr ssd1306_cmd
  lda #0
  sta scroll
  sta outb
  jsr i2cbyteout
  jsr i2c_stop
  return:
  rts

gonewline:
jmp newline

ssd1306_sendchar:
cmp #$0D ; Newline
beq gonewline
cmp #$0A ; CR - also newline
beq gonewline
cmp #$08 ; Backspace
beq backspace
cmp #$7f ; Delete - also backspace
beq backspace
cmp #$0C ; Form feed, CTRL+L on your keyboard.
bne startprint
;jsr ssd1306_clear
rts
startprint:
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
jsr i2c_stop
;tya
;jsr serial_tx
lda cursor
clc
adc #1
and #127
sta cursor

lda scroll
asl ; Convert scroll offset to cursor count - units. 8 << 2 == 16 == Second line
clc ; Need this?
adc cursor
and #$7F ; Throw away only the top bit since scroll offset might have it set

bne l451 ; Again - not taking scroll offset into account..
lda #1 ; We reached wraparound so we start scrolling
sta tflags ; Terminal flags
l451:
lda cursor
and #$0F ; Check if we started a new line and need to reset cursor position.
bne nonewline
jsr ssd1306_setcolumn
lda tflags ; Check scroll flag
beq nonewline
jsr ssd1306_scrolldown
nonewline:
rts

backspace:
;jsr serial_tx ; Echo back the backspace/del
lda cursor
bne noroll ; No roll back to 127
lda #128
noroll:
sec
sbc #1
sta cursor ; Save
and #$0F ; Discard page
cmp #$0F ; Wrapped back a line
bne nocolwrap
ldy #0
sty tflags ; Scroll off
nocolwrap:
jsr ssd1306_setcolumn ; Set new column
lda cursor
lsr
lsr
lsr
lsr ; Shift bits to get current line (16 chars per line == first four bits = character = next three bits line)
jsr ssd1306_setline

clc ; Write
jsr i2c_start
lda #$40 ; Co bit 0, D/C 1
sta outb
ldy #9
send0:
jsr i2cbyteout ; cmd byte + 8 x Send 0
dey
bne send0
jsr i2c_stop

setcursor:
lda cursor
setcursor2:
and #$0F ; Discard page
jsr ssd1306_setcolumn ; Set new column
lda cursor
lsr
lsr
lsr
lsr ; Shift bits to get current line (16 chars per line == first four bits = character = next three bits line)
jsr ssd1306_setline
zeropos:
rts

newline:
;jsr serial_tx ; Echo the newline
lda cursor
adc #16
and #$70 ; Ensure range - ignore character position
sta cursor
lda scroll
asl ; Convert scroll offset to cursor count - units. 8 << 2 == 16 == Second line
clc ; Need this?
adc cursor
and #$70 ; Throw away top bit since scroll offset might have it set
bne nowrap ; Now factoring in scroll offset!
ldy #1
sty tflags
nowrap:
lda cursor
lsr
lsr
lsr
lsr ; Shift bits to get current line (16 chars per line == first four bits = character = next three bits line)
jsr ssd1306_setline
lda #0
jsr ssd1306_setcolumn ; CR
lda tflags
beq notscrolling
jsr ssd1306_scrolldown
notscrolling:
rts

;Scroll but don't clear the first line
ssd1306_scrollup_noclear:
  lda #$d3
  jsr ssd1306_cmd
  lda scroll
  sec
  sbc #8 ; Doesn't care about bits 6+7
  sta scroll
  sta outb
  jsr i2cbyteout
  jsr i2c_stop
rts

ssd1306_scrolldown:
  lda #$d3
  jsr ssd1306_cmd
  lda scroll
  adc #8 ; Doesn't care about bits 6+7
  sta scroll
  sta outb
  jsr i2cbyteout
  jsr i2c_stop

  clc ; Write
  jsr i2c_start
  lda #$40 ; Co bit 0, D/C 1
  sta outb
  ldy #128 ; Command byte + One line/page... Writing the last column might increase the page pointer..
  ; And the last column should always be clear anyway, so 128 instead of 129 so we don't reset to the first
  ;column on the wrong page.
  sendblanks:
  jsr i2cbyteout ; Send 0
  dey
  bne sendblanks
  jsr i2c_stop
  jsr ssd1306_resetcolumn ; Column always 0 after scroll
  rts

  ssd1306_resetcolumn:
  lda #$21 ; Set column command (0-127)
  jsr ssd1306_cmd
  jsr i2cbyteout ; outb already 0
  lda #$7f ; 127
  sta outb
  jsr i2cbyteout
  jsr i2c_stop
  rts

ssd1306_setcolumn:
asl ; 15 >> >> >> 120
asl
asl
pha
lda #$21 ; Set column command (0-127)
jsr ssd1306_cmd
pla
sta outb
jsr i2cbyteout
lda #$7f ; 127
sta outb
jsr i2cbyteout
jsr i2c_stop
rts

;Takes line(page) in A
ssd1306_setline:
pha ; Save line
lda #$22 ; Set page cmd
jsr ssd1306_cmd
pla ; Fetch line
sta outb
jsr i2cbyteout
lda #7 ; Ensure range
sta outb
jsr i2cbyteout
jsr i2c_stop
rts

;Takes command in A
ssd1306_cmd:
pha ; Save command
lda #$3c ; SSD1306 address
sta I2CADDR
clc ;Write flag
jsr i2c_start
;A is 0 == Co = 0, D/C# = 0
sta outb
jsr i2cbyteout
pla ; Fetch command
sta outb
jsr i2cbyteout
rts

ssd1306_prints:
stx stringp+1
sty stringp
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
rts

printbyte:
    pha ; Save A
    jsr bytetoa
    pha
    lda xtmp
    jsr ssd1306_sendchar
    pla
    jsr ssd1306_sendchar
    pla ; Restore A
    rts

fastprint:
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
jsr i2c_stop

lda cursor
clc
adc #1
and #127
sta cursor
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