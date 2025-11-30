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


/*


    lda #$60
    sta I2CADDR

lda #<testfreq
sta ptr
lda #>testfreq
sta ptr+1

jsr write_ms2

lda #18
ldy #$0C ; 2mA, frac mode
jsr i2c_write_register

*/
/*
lda #62
ldy #$11
jsr i2c_write_register
*/
/*
  ; Reset PLLs (register 177)
  lda #177
  ldy #$AC
  jsr i2c_write_register

  ; Enable CLK0 & CLK1 outputs (register 3 → clear disable bits)
  lda #3
  ldy #$f8       ; 0b11111100 → enable CLK0 & CLK1, disable CLK2
  jsr i2c_write_register
*/

jsr setup_si5351

lda #%11110010   ; DRA4–DRA7 + TX as outputs
sta DDRA

lda #<frq_89650000_data
sta ptr
lda #>frq_89650000_data
sta ptr+1

jsr setup_frequency_from_lut

stayhere:
jsr delay_long
lda DRB
and #$7f
sta DRB
lda #10
jsr delay_long
lda DRB
ora #$80
sta DRB
bne stayhere    ; BRA

frq_89650000_data:  .byte 1, 244, 0, 10, 141, 0, 0, 28, 0, 1, 0, 1, 128, 0, 0, 0, 7
/*
testfreq:
.byte 004, 000, 0, 008, 129, 000, 000, 000, 021

write_ms2:
    ldy #58
ms2loop:
    tya              ; Save register number on stack
    pha
    lda (ptr),y      ; Load data byte
    tay              ; Y = data
    pla              ; A = register number
    pha
    jsr i2c_write_register ;reg is in A, val in Y
    pla 
    tay
    iny
    cpy #66           ; Only 8 registers for MS2
    bne ms2loop
    rts
*/
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

*/