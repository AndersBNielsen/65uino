; The code below writes text to a 128x64 SSD1306 i2c OLED
; and reads out raw temperature from a BMP180 i2c module
; Written by Anders Nielsen, 2023
; License: https://creativecommons.org/licenses/by-nc/4.0/legalcode

.feature string_escapes ; Allow c-style string escapes when using ca65
.feature org_per_seg
.feature c_comments

I2CRAM = $00

inb    = I2CRAM
outb   = I2CRAM +1
I2CREG = I2CRAM +2
xtmp   = I2CRAM+3
ytmp   = I2CRAM+4
stringp = I2CRAM+5 ; & +1
I2CADDR = I2CRAM+7

;I2CADDR = $78 ; 0x3c << 1
SDA   = 2; DRB0 bitmask
SDA_INV = $FD
SCL   = 1; DRB1 bitmask
SCL_INV = $FE
PORTB   = DRB


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

main:
jsr i2c_test
ldy #$2e ; BMP180 cmd to read uncompensated temperature
lda #$f4 ; Register
jsr i2c_write

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
lda READTDI
bne wait ; Loop until timer runs out

lda #7
jsr ssd1306_setpage
jsr bmp180_printtemp

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

i2c_start: ; i2c addr a constant and RW bit is in C
  lda I2CADDR
  rol ; Shift in carry
  ;ora I2CADDR
  sta outb ; Save addr + rw bit

  lda #SCL_INV ; Start with SCL as INPUT HIGH. If bit 0 is used we can continue from here with inc/dec.
  and DDRB
  sta DDRB

  lda #SDA ; Ensure SDA is output low before SCL is LOW
  ora DDRB
  sta DDRB
  lda #SDA_INV
  and PORTB
  sta PORTB

  lda #SCL_INV ; Ensure SCL is low when output (currently high because it's an input)
  and PORTB
  sta PORTB
  inc DDRB ; Set to output by incrementing the direction register
  ; Fall through to send address + RW bit.
  ; After a start condition we always SEND a byte, so we don't need to return.

  ; Send an i2c byte - byte is in outb
  ; From here on we can assume OUTPUTs are LOW and INPUTS are HIGH.
  ; Maybe some of the juggling above is not necessary but let's not assume for now
i2cbyteout:
  lda #SDA_INV ; In case this is a data byte we set SDA LOW
  and PORTB
  sta PORTB
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
  lda PORTB ; Check ACK bit
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
  lda PORTB; Let's read after SCL goes high
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

fontc1:
.byte $00	;	20 	32
.byte $00	;	21 	33 	!
.byte $00	;	22 	34 
.byte $14	;	23 	35 	#
.byte $24	;	24 	36 	$
.byte $23	;	25 	37 	%
.byte $36	;	26 	38 	&
.byte $00	;	27 	39 	'
.byte $00	;	28 	40 	(
.byte $00	;	29 	41 	)
.byte $14	;	2A 	42 	*
.byte $08	;	2B 	43 	+
.byte $00	;	2C 	44 
.byte $08	;	2D 	45 	-
.byte $00	;	2E 	46 	.
.byte $20	;	2F 	47 	/
.byte $3E	;	30 	48 	0
.byte $00	;	31 	49 	1
.byte $42	;	32 	50 	2
.byte $21	;	33 	51 	3
.byte $18	;	34 	52 	4
.byte $27	;	35 	53 	5
.byte $3C	;	36 	54 	6
.byte $03	;	37 	55 	7
.byte $36	;	38 	56 	8
.byte $06	;	39 	57 	9
.byte $00	;	3A 	58 	:
.byte $00	;	3B 	59 	;
.byte $08	;	3C 	60 	<
.byte $14	;	3D 	61 	=
.byte $00	;	3E 	62 	>
.byte $02	;	3F 	63 	?
.byte $32	;	40 	64 	@
.byte $7E	;	41 	65 	A
.byte $7F	;	42 	66 	B
.byte $3E	;	43 	67 	C
.byte $7F	;	44 	68 	D
.byte $7F	;	45 	69 	E
.byte $7F	;	46 	70 	F
.byte $3E	;	47 	71 	G
.byte $7F	;	48 	72 	H
.byte $00	;	49 	73 	I
.byte $20	;	4A 	74 	J
.byte $7F	;	4B 	75 	K
.byte $7F	;	4C 	76 	L
.byte $7F	;	4D 	77 	M
.byte $7F	;	4E 	78 	N
.byte $3E	;	4F 	79 	O
.byte $7F	;	50 	80 	P
.byte $3E	;	51 	81 	Q
.byte $7F	;	52 	82 	R
.byte $46	;	53 	83 	S
.byte $01	;	54 	84 	T
.byte $3F	;	55 	85 	U
.byte $1F	;	56 	86 	V
.byte $3F	;	57 	87 	W
.byte $63	;	58 	88 	X
.byte $07	;	59 	89 	Y
.byte $61	;	5A 	90 	Z
.byte $7F	;	5B 	91 	[
.byte $15	;	5C 	92 	'\'
.byte $00	;	5D 	93 	]
.byte $04	;	5E 	94 	^
.byte $40	;	5F 	95 	_
.byte $00	;	60 	96 	`
.byte $20	;	61 	97 	a
.byte $7F	;	62 	98 	b
.byte $38	;	63 	99 	c
.byte $38	;	64	100 	d
.byte $38	;	65	101 	e
.byte $08	;	66	102 	f
.byte $0C	;	67	103 	g
.byte $7F	;	68	104 	h
.byte $00	;	69	105 	i
.byte $20	;	6A	106 	j
.byte $7F	;	6B	107 	k
.byte $00	;	6C	108 	l
.byte $7C	;	6D	109 	m
.byte $7C	;	6E	110 	n
.byte $38	;	6F	111 	o
.byte $7C	;	70	112 	p
.byte $08	;	71	113 	q
.byte $7C	;	72	114 	r
.byte $48	;	73	115 	s
.byte $04	;	74	116 	t
.byte $3C	;	75	117 	u
.byte $1C	;	76	118 	v
.byte $3C	;	77	119 	w
.byte $44	;	78	120 	x
.byte $0C	;	79	121 	y
.byte $44	;	7A	122 	z
.byte $00	;	7B	123 	{
.byte $00	;	7C	124 	|
.byte $00	;	7D	125 	}
.byte $08	;	7E	126 	~
.byte $08	; 7F	127

fontc2:
.byte $00	;	20 	32
.byte $00	;	21 	33 	!
.byte $07	;	22 	34 
.byte $7F	;	23 	35 	#
.byte $2A	;	24 	36 	$
.byte $13	;	25 	37 	%
.byte $49	;	26 	38 	&
.byte $05	;	27 	39 	'
.byte $1C	;	28 	40 	(
.byte $41	;	29 	41 	)
.byte $08	;	2A 	42 	*
.byte $08	;	2B 	43 	+
.byte $50	;	2C 	44 
.byte $08	;	2D 	45 	-
.byte $60	;	2E 	46 	.
.byte $10	;	2F 	47 	/
.byte $51	;	30 	48 	0
.byte $42	;	31 	49 	1
.byte $61	;	32 	50 	2
.byte $41	;	33 	51 	3
.byte $14	;	34 	52 	4
.byte $45	;	35 	53 	5
.byte $4A	;	36 	54 	6
.byte $01	;	37 	55 	7
.byte $49	;	38 	56 	8
.byte $49	;	39 	57 	9
.byte $36	;	3A 	58 	:
.byte $56	;	3B 	59 	;
.byte $14	;	3C 	60 	<
.byte $14	;	3D 	61 	=
.byte $41	;	3E 	62 	>
.byte $01	;	3F 	63 	?
.byte $49	;	40 	64 	@
.byte $11	;	41 	65 	A
.byte $49	;	42 	66 	B
.byte $41	;	43 	67 	C
.byte $41	;	44 	68 	D
.byte $49	;	45 	69 	E
.byte $09	;	46 	70 	F
.byte $41	;	47 	71 	G
.byte $08	;	48 	72 	H
.byte $41	;	49 	73 	I
.byte $40	;	4A 	74 	J
.byte $08	;	4B 	75 	K
.byte $40	;	4C 	76 	L
.byte $02	;	4D 	77 	M
.byte $04	;	4E 	78 	N
.byte $41	;	4F 	79 	O
.byte $09	;	50 	80 	P
.byte $41	;	51 	81 	Q
.byte $09	;	52 	82 	R
.byte $49	;	53 	83 	S
.byte $01	;	54 	84 	T
.byte $40	;	55 	85 	U
.byte $20	;	56 	86 	V
.byte $40	;	57 	87 	W
.byte $14	;	58 	88 	X
.byte $08	;	59 	89 	Y
.byte $51	;	5A 	90 	Z
.byte $41	;	5B 	91 	[
.byte $16	;	5C 	92 	'\'
.byte $41	;	5D 	93 	]
.byte $02	;	5E 	94 	^
.byte $40	;	5F 	95 	_
.byte $01	;	60 	96 	`
.byte $54	;	61 	97 	a
.byte $48	;	62 	98 	b
.byte $44	;	63 	99 	c
.byte $44	;	64	100 	d
.byte $54	;	65	101 	e
.byte $7E	;	66	102 	f
.byte $52	;	67	103 	g
.byte $08	;	68	104 	h
.byte $44	;	69	105 	i
.byte $40	;	6A	106 	j
.byte $10	;	6B	107 	k
.byte $41	;	6C	108 	l
.byte $04	;	6D	109 	m
.byte $08	;	6E	110 	n
.byte $44	;	6F	111 	o
.byte $14	;	70	112 	p
.byte $14	;	71	113 	q
.byte $08	;	72	114 	r
.byte $54	;	73	115 	s
.byte $3F	;	74	116 	t
.byte $40	;	75	117 	u
.byte $20	;	76	118 	v
.byte $40	;	77	119 	w
.byte $28	;	78	120 	x
.byte $50	;	79	121 	y
.byte $64	;	7A	122 	z
.byte $08	;	7B	123 	{
.byte $00	;	7C	124 	|
.byte $41	;	7D	125 	}
.byte $08	;	7E	126 	~
.byte $1C	; 7F	127

fontc3:
.byte $00	;	20 	32
.byte $4F	;	21 	33 	!
.byte $00	;	22 	34 
.byte $14	;	23 	35 	#
.byte $7F	;	24 	36 	$
.byte $08	;	25 	37 	%
.byte $55	;	26 	38 	&
.byte $03	;	27 	39 	'
.byte $22	;	28 	40 	(
.byte $22	;	29 	41 	)
.byte $3E	;	2A 	42 	*
.byte $3E	;	2B 	43 	+
.byte $30	;	2C 	44 
.byte $08	;	2D 	45 	-
.byte $60	;	2E 	46 	.
.byte $08	;	2F 	47 	/
.byte $49	;	30 	48 	0
.byte $7F	;	31 	49 	1
.byte $51	;	32 	50 	2
.byte $45	;	33 	51 	3
.byte $12	;	34 	52 	4
.byte $45	;	35 	53 	5
.byte $49	;	36 	54 	6
.byte $71	;	37 	55 	7
.byte $49	;	38 	56 	8
.byte $49	;	39 	57 	9
.byte $36	;	3A 	58 	:
.byte $36	;	3B 	59 	;
.byte $22	;	3C 	60 	<
.byte $14	;	3D 	61 	=
.byte $22	;	3E 	62 	>
.byte $51	;	3F 	63 	?
.byte $79	;	40 	64 	@
.byte $11	;	41 	65 	A
.byte $49	;	42 	66 	B
.byte $41	;	43 	67 	C
.byte $41	;	44 	68 	D
.byte $49	;	45 	69 	E
.byte $09	;	46 	70 	F
.byte $49	;	47 	71 	G
.byte $08	;	48 	72 	H
.byte $7F	;	49 	73 	I
.byte $41	;	4A 	74 	J
.byte $14	;	4B 	75 	K
.byte $40	;	4C 	76 	L
.byte $0C	;	4D 	77 	M
.byte $08	;	4E 	78 	N
.byte $41	;	4F 	79 	O
.byte $09	;	50 	80 	P
.byte $51	;	51 	81 	Q
.byte $19	;	52 	82 	R
.byte $49	;	53 	83 	S
.byte $7F	;	54 	84 	T
.byte $40	;	55 	85 	U
.byte $40	;	56 	86 	V
.byte $38	;	57 	87 	W
.byte $08	;	58 	88 	X
.byte $70	;	59 	89 	Y
.byte $49	;	5A 	90 	Z
.byte $41	;	5B 	91 	[
.byte $7C	;	5C 	92 	'\'
.byte $41	;	5D 	93 	]
.byte $01	;	5E 	94 	^
.byte $40	;	5F 	95 	_
.byte $02	;	60 	96 	`
.byte $54	;	61 	97 	a
.byte $44	;	62 	98 	b
.byte $44	;	63 	99 	c
.byte $44	;	64	100 	d
.byte $54	;	65	101 	e
.byte $09	;	66	102 	f
.byte $52	;	67	103 	g
.byte $04	;	68	104 	h
.byte $7D	;	69	105 	i
.byte $44	;	6A	106 	j
.byte $28	;	6B	107 	k
.byte $7F	;	6C	108 	l
.byte $18	;	6D	109 	m
.byte $04	;	6E	110 	n
.byte $44	;	6F	111 	o
.byte $14	;	70	112 	p
.byte $14	;	71	113 	q
.byte $04	;	72	114 	r
.byte $54	;	73	115 	s
.byte $44	;	74	116 	t
.byte $40	;	75	117 	u
.byte $40	;	76	118 	v
.byte $38	;	77	119 	w
.byte $10	;	78	120 	x
.byte $50	;	79	121 	y
.byte $54	;	7A	122 	z
.byte $36	;	7B	123 	{
.byte $7F	;	7C	124 	|
.byte $36	;	7D	125 	}
.byte $2A	;	7E	126 	~
.byte $2A	; 7F	127

fontc4:
.byte $00	;	20 	32
.byte $00	;	21 	33 	!
.byte $07	;	22 	34 
.byte $7F	;	23 	35 	#
.byte $2A	;	24 	36 	$
.byte $64	;	25 	37 	%
.byte $22	;	26 	38 	&
.byte $00	;	27 	39 	'
.byte $41	;	28 	40 	(
.byte $1C	;	29 	41 	)
.byte $08	;	2A 	42 	*
.byte $08	;	2B 	43 	+
.byte $00	;	2C 	44 
.byte $08	;	2D 	45 	-
.byte $00	;	2E 	46 	.
.byte $04	;	2F 	47 	/
.byte $45	;	30 	48 	0
.byte $40	;	31 	49 	1
.byte $49	;	32 	50 	2
.byte $4B	;	33 	51 	3
.byte $7F	;	34 	52 	4
.byte $45	;	35 	53 	5
.byte $49	;	36 	54 	6
.byte $09	;	37 	55 	7
.byte $49	;	38 	56 	8
.byte $29	;	39 	57 	9
.byte $00	;	3A 	58 	:
.byte $00	;	3B 	59 	;
.byte $41	;	3C 	60 	<
.byte $14	;	3D 	61 	=
.byte $14	;	3E 	62 	>
.byte $09	;	3F 	63 	?
.byte $41	;	40 	64 	@
.byte $11	;	41 	65 	A
.byte $49	;	42 	66 	B
.byte $41	;	43 	67 	C
.byte $22	;	44 	68 	D
.byte $49	;	45 	69 	E
.byte $09	;	46 	70 	F
.byte $49	;	47 	71 	G
.byte $08	;	48 	72 	H
.byte $41	;	49 	73 	I
.byte $3F	;	4A 	74 	J
.byte $22	;	4B 	75 	K
.byte $40	;	4C 	76 	L
.byte $02	;	4D 	77 	M
.byte $10	;	4E 	78 	N
.byte $41	;	4F 	79 	O
.byte $09	;	50 	80 	P
.byte $21	;	51 	81 	Q
.byte $29	;	52 	82 	R
.byte $49	;	53 	83 	S
.byte $01	;	54 	84 	T
.byte $40	;	55 	85 	U
.byte $20	;	56 	86 	V
.byte $40	;	57 	87 	W
.byte $14	;	58 	88 	X
.byte $08	;	59 	89 	Y
.byte $45	;	5A 	90 	Z
.byte $00	;	5B 	91 	[
.byte $16	;	5C 	92 	'\'
.byte $7F	;	5D 	93 	]
.byte $02	;	5E 	94 	^
.byte $40	;	5F 	95 	_
.byte $04	;	60 	96 	`
.byte $54	;	61 	97 	a
.byte $44	;	62 	98 	b
.byte $44	;	63 	99 	c
.byte $48	;	64	100 	d
.byte $54	;	65	101 	e
.byte $01	;	66	102 	f
.byte $52	;	67	103 	g
.byte $04	;	68	104 	h
.byte $40	;	69	105 	i
.byte $3D	;	6A	106 	j
.byte $44	;	6B	107 	k
.byte $40	;	6C	108 	l
.byte $04	;	6D	109 	m
.byte $04	;	6E	110 	n
.byte $44	;	6F	111 	o
.byte $14	;	70	112 	p
.byte $18	;	71	113 	q
.byte $04	;	72	114 	r
.byte $54	;	73	115 	s
.byte $40	;	74	116 	t
.byte $20	;	75	117 	u
.byte $20	;	76	118 	v
.byte $40	;	77	119 	w
.byte $28	;	78	120 	x
.byte $50	;	79	121 	y
.byte $4C	;	7A	122 	z
.byte $41	;	7B	123 	{
.byte $00	;	7C	124 	|
.byte $08	;	7D	125 	}
.byte $1C	;	7E	126 	~
.byte $08	; 7F	127

fontc5:
.byte $00	;	20 	32
.byte $00	;	21 	33 	!
.byte $00	;	22 	34 
.byte $14	;	23 	35 	#
.byte $12	;	24 	36 	$
.byte $62	;	25 	37 	%
.byte $50	;	26 	38 	&
.byte $00	;	27 	39 	'
.byte $00	;	28 	40 	(
.byte $00	;	29 	41 	)
.byte $14	;	2A 	42 	*
.byte $08	;	2B 	43 	+
.byte $00	;	2C 	44 
.byte $08	;	2D 	45 	-
.byte $00	;	2E 	46 	.
.byte $02	;	2F 	47 	/
.byte $3E	;	30 	48 	0
.byte $00	;	31 	49 	1
.byte $46	;	32 	50 	2
.byte $31	;	33 	51 	3
.byte $10	;	34 	52 	4
.byte $39	;	35 	53 	5
.byte $30	;	36 	54 	6
.byte $07	;	37 	55 	7
.byte $36	;	38 	56 	8
.byte $1E	;	39 	57 	9
.byte $00	;	3A 	58 	:
.byte $00	;	3B 	59 	;
.byte $00	;	3C 	60 	<
.byte $14	;	3D 	61 	=
.byte $08	;	3E 	62 	>
.byte $06	;	3F 	63 	?
.byte $3E	;	40 	64 	@
.byte $7E	;	41 	65 	A
.byte $36	;	42 	66 	B
.byte $22	;	43 	67 	C
.byte $1C	;	44 	68 	D
.byte $41	;	45 	69 	E
.byte $01	;	46 	70 	F
.byte $7A	;	47 	71 	G
.byte $7F	;	48 	72 	H
.byte $00	;	49 	73 	I
.byte $01	;	4A 	74 	J
.byte $41	;	4B 	75 	K
.byte $40	;	4C 	76 	L
.byte $7F	;	4D 	77 	M
.byte $7F	;	4E 	78 	N
.byte $3E	;	4F 	79 	O
.byte $06	;	50 	80 	P
.byte $5E	;	51 	81 	Q
.byte $46	;	52 	82 	R
.byte $31	;	53 	83 	S
.byte $01	;	54 	84 	T
.byte $3F	;	55 	85 	U
.byte $1F	;	56 	86 	V
.byte $3F	;	57 	87 	W
.byte $63	;	58 	88 	X
.byte $07	;	59 	89 	Y
.byte $43	;	5A 	90 	Z
.byte $00	;	5B 	91 	[
.byte $15	;	5C 	92 	'\'
.byte $00	;	5D 	93 	]
.byte $04	;	5E 	94 	^
.byte $40	;	5F 	95 	_
.byte $00	;	60 	96 	`
.byte $78	;	61 	97 	a
.byte $38	;	62 	98 	b
.byte $20	;	63 	99 	c
.byte $7F	;	64	100 	d
.byte $18	;	65	101 	e
.byte $02	;	66	102 	f
.byte $3E	;	67	103 	g
.byte $78	;	68	104 	h
.byte $00	;	69	105 	i
.byte $00	;	6A	106 	j
.byte $00	;	6B	107 	k
.byte $00	;	6C	108 	l
.byte $78	;	6D	109 	m
.byte $78	;	6E	110 	n
.byte $38	;	6F	111 	o
.byte $08	;	70	112 	p
.byte $7C	;	71	113 	q
.byte $08	;	72	114 	r
.byte $20	;	73	115 	s
.byte $20	;	74	116 	t
.byte $7C	;	75	117 	u
.byte $1C	;	76	118 	v
.byte $3C	;	77	119 	w
.byte $44	;	78	120 	x
.byte $3C	;	79	121 	y
.byte $44	;	7A	122 	z
.byte $00	;	7B	123 	{
.byte $00	;	7C	124 	|
.byte $00	;	7D	125 	}
.byte $08	;	7E	126 	~
.byte $08	; 7F	127

.segment "VECTORS6502"
.ORG $fffa
.word nmi,reset,irq
.reloc
