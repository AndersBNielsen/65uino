/*
This file is example code 

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