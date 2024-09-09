;Flappy Larus constants and variables
GRAVITY = 160
debounce_flag = mode
loopcounter = runpnt
random = rxcnt
loopcounter2 = runpnt+1
gamespeed = txcnt
obstaclecnt = stringp
obstacleh   = stringp+1
score = halt-1

initgame:

jsr ssd1306_clear

lda #6
sta gamespeed
sta loopcounter2
lda #0 
sta score
sta loopcounter
lda #8
sta obstaclecnt

restart:
lda #0 
sta scroll

lda #0
sta cursor
jsr setcursor
lda #'8' ; Cloud
jsr fastprint
lda #$11
sta cursor
jsr setcursor2
lda #'8' ; Cloud
jsr fastprint
lda #$31
sta cursor
jsr setcursor2
lda #'8' ; Cloud
jsr fastprint
lda #$40
sta cursor
jsr setcursor2
lda #'8' ; Cloud
jsr fastprint
lda #$61
sta cursor
jsr setcursor2
lda #'8' ; Cloud
jsr fastprint

lda #$67
sta cursor
jsr setcursor
lda #'E'
jsr fastprint
jsr delay_long

waitforflap:
bit DRB
bvs waitforflap
rts

checkstate:
jsr checkfall
jsr checkbounds2
bcc gameover

jsr checkflap
jsr checkbounds2
bcc gameover

lda READTDI ; Try to grab a somewhat random value depending on what the timer is at
sta random
sec
rts

gameover:
rts

checkfall:
lda READTDI ; Time to fall?
bne notyet

; Fall one position
dec cursor
jsr setcursor
lda #' '
jsr fastprint
lda #'E'
jsr fastprint
; Reset timer

lda #GRAVITY
sta WTD1KDI ; Gravity timer
notyet:
rts

checkbounds2:
lda #$80
sec
sbc scroll
asl ; Convert scroll offset to cursor count - units. 8 << 2 == 16 == Second line
clc
adc cursor
and #$7F ; Throw away only the top bit since scroll offset might have it set
cmp #$70
bcs gameover2
cmp #$5F
bcc gameover2
sec ; Return C = 1
rts
gameover2:
clc ; Return C = 0 to indicate gameover
rts 

checkcollision2:
lda #$80
sec
sbc scroll
asl ; Convert scroll offset to cursor count - units. 8 << 2 == 16 == Second line
clc
adc cursor
and #$7F ; Throw away only the top bit since scroll offset might have it set
pha
sec
sbc obstacleh
cmp #$60
bcc gameover3
pla
clc
adc #16
sec
sbc #4
sbc obstacleh
cmp #$70
bcs gameover3
sec ; Return C = 1
rts
gameover3:
clc ; Return C = 0 to indicate gameover
rts 

clearcursor:
;Clear current cursor position
dec cursor
jsr setcursor
lda #' '
jsr fastprint
rts

checkflap:
getinput:
bit DRB
bvs notpressed

lda debounce_flag
bne debounce
lda #1 ; Only one flap per press, so we set a flag we already flappeds
sta debounce_flag
jsr clearcursor

; Draw Larus
lda cursor
sec 
sbc #2
sta cursor
jsr setcursor2
lda #'|'
jsr fastprint
lda #255
sta WTD1KDI ; Gravity timer

bne debounce

notpressed:
lda #0
sta debounce_flag

debounce:
rts

makeobstacle:
lda cursor
pha
lda #$70
sta cursor
jsr setcursor
clc ; Write
jsr i2c_start
lda #$40 ; Co bit 0, D/C 1
sta outb
jsr i2cbyteout
lda random
and #$07
pha
asl
asl
asl
tay
hitboxes:
lda #$ff
sta outb
jsr i2cbyteout
dey
bne hitboxes

ldy #32
opening:
jsr i2cbyteout
dey
bne opening
pla
sta obstacleh
lda #16
sbc #4 ; Opening
sbc obstacleh ; First hitboxes
asl
asl
asl
tay
hitboxes2:
lda #$ff
sta outb
jsr i2cbyteout
dey
bne hitboxes2
jsr i2c_stop

pla
sta cursor
jsr setcursor
rts

flappylarus:
jsr initgame

gameloop:
jsr checkstate
bcc flappylarus ; Game over

dec loopcounter
bne notyetx
dec loopcounter2
bne notyetx
jsr clearscore
jsr ssd1306_scrollup_noclear
jsr clearcursor
lda cursor
sec
sbc #17
sta cursor
jsr setcursor2
lda #'E'
jsr fastprint
jsr printscore


lda obstaclecnt
sec
sbc #1
sta obstaclecnt
beq clearobstacle
cmp #7
beq drawobstacle
cmp #2
beq checkcollision
bne nexttick

drawobstacle:
jsr makeobstacle

bcc nexttick

clearobstacle:
;later
lda #8
sta obstaclecnt
bne nexttick

checkcollision:
jsr checkcollision2
bcc flappylarus ; Game Over
inc score

nexttick:
lda gamespeed
sta loopcounter2
lda cursor

notyetx:

jmp gameloop
rts ; Never get here

printscore:
lda cursor
pha
and #$f0
adc #$1d
sta cursor
jsr setcursor2
lda score
jsr printbyte
pla 
sta cursor
jsr setcursor2
rts

clearscore:
lda cursor
pha
and #$f0
adc #$1c
sta cursor
jsr setcursor2
lda #' '
jsr fastprint
lda #' '
jsr fastprint
pla 
sta cursor
jsr setcursor2
rts