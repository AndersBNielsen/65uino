;Driver for si5351

; ; Below should be placed in the zero page
; ; Inputs
; freq0        .res 1   ; Frequency LSB
; freq1        .res 1
; freq2        .res 1
; freq3        .res 1   ; Frequency MSB

; ; Internal
; dividend0    .res 1   ; 750_750_751 (LSB first)
; dividend1    .res 1
; dividend2    .res 1
; dividend3    .res 1

; quotient0    .res 1   ; Output: quotient (LSB first)
; quotient1    .res 1
; quotient2    .res 1
; quotient3    .res 1

; remainder0   .res 1
; remainder1   .res 1
; remainder2   .res 1
; remainder3   .res 1

; temp0        .res 1
; temp1        .res 1
; temp2        .res 1
; temp3        .res 1

; phase_lo     .res 1
; phase_hi     .res 1


ptr     = stringp     ; zero-page 16-bit pointer (not conflicting with outb or I2CADDR since we're not printing strings while changing clocks)

setup_si5351:
    lda #<si5351_init_data
    sta ptr
    lda #>si5351_init_data
    sta ptr+1

next_reg:
    ldy #0
    lda (ptr),y
    cmp #$FF
    beq done_init

    ; Start I2C and send address
    lda #$60
    sta I2CADDR
    clc                 ; Write mode
    jsr i2c_start       ; sends address

    lda (ptr),y         ; register number
    sta outb
    jsr i2cbyteout

    iny
    lda (ptr),y         ; data byte
    sta outb
    jsr i2cbyteout

    jsr i2c_stop        ; always stop after each write

    ; Advance ptr by 2 bytes
    clc
    lda ptr
    adc #2
    sta ptr
    lda ptr+1
    adc #0
    sta ptr+1

    jmp next_reg

done_init:
    rts

; ; Compute phase offset = 750,750,751 / frequency
; ; Inputs: freq0..3 = frequency in Hz (LSB first)
; ; Output: phase_lo/hi = offset in counts (LSB first)

; compute_phase_offset:
;     ; Load constant: 750,750,751 = $2CD145EF
;     lda #$EF
;     sta dividend0
;     lda #$45
;     sta dividend1
;     lda #$D1
;     sta dividend2
;     lda #$2C
;     sta dividend3

;     jsr div_32_by_32

;     ; Store 16-bit result
;     lda quotient0
;     sta phase_lo
;     lda quotient1
;     sta phase_hi
;     rts

; ; dividend0-3 ÷ freq0-3 → quotient0-3
; ; Uses remainder0-3 internally

; div_32_by_32:
;     lda #0
;     sta quotient0
;     sta quotient1
;     sta quotient2
;     sta quotient3

;     sta remainder0
;     sta remainder1
;     sta remainder2
;     sta remainder3

;     ldx #32
; div_loop:
;     ; Shift dividend left into remainder
;     asl dividend3
;     rol dividend2
;     rol dividend1
;     rol dividend0

;     rol remainder3
;     rol remainder2
;     rol remainder1
;     rol remainder0

;     ; Subtract divisor from remainder
;     sec
;     lda remainder0
;     sbc freq0
;     sta temp0
;     lda remainder1
;     sbc freq1
;     sta temp1
;     lda remainder2
;     sbc freq2
;     sta temp2
;     lda remainder3
;     sbc freq3
;     sta temp3

;     bcc skip_subtract

;     ; Successful subtract → update remainder
;     lda temp0
;     sta remainder0
;     lda temp1
;     sta remainder1
;     lda temp2
;     sta remainder2
;     lda temp3
;     sta remainder3

;     ; Set bit in quotient
;     rol quotient0
;     rol quotient1
;     rol quotient2
;     rol quotient3
;     jmp div_continue

; skip_subtract:
;     ; No subtract → shift in 0
;     clc
;     rol quotient0
;     rol quotient1
;     rol quotient2
;     rol quotient3

; div_continue:
;     dex
;     bne div_loop
;     rts



si5351_init_data:


  ; ====== Output Enable Control
  .byte 3, $F8     ; Enable all outputs (0 = enabled)

; ====== CLKx Control (Registers 16–23)
  .byte 16, $80    ; CLK0: Powerdown (0 = on), PLLA, no invert, 8 mA, int mode
  .byte 17, $80    ; CLK1: Powerdown (0 = on), PLLA, no invert, 8 mA, int mode
  .byte 18, $80   ; CLK2: Powerdown (0 = on), PLLA, no invert, 8 mA, int mode
  .byte 19, $80    ; CLK3 Powerdown (0 = on)
  .byte 20, $80    ; CLK4 Powerdown (0 = on)
  .byte 21, $80    ; CLK5 Powerdown (0 = on)
  .byte 22, $80    ; CLK6 Powerdown (0 = on)
  .byte 23, $80    ; CLK7 Powerdown (0 = on)

  ; ====== PLLA Configuration: Set PLLA to 700 MHz (25 MHz × 28)
  ; Registers 26–33
  ; Integer mode: Mult = 28, Num = 0, Denom = 1
  ; P1 = 128 × Mult - 512 = 3584 = $0C00
  ; P2 = 0, P3 = 1 (integer mode)

  .byte 26, $00    ; MSNA P3[15:8]   = 0
  .byte 27, $01    ; MSNA P3[7:0]    = 1 → P3 = 1
  .byte 28, $00    ; MSNA Reserved + P1[17:16] = 0
  .byte 29, $0C    ; MSNA P1[15:8]   = $0C == 700MHz. 0x02 = 200MHz(Underclocked) -->50Mhz/div--> 0x11 == 950MHz (Overclocked)
  .byte 30, $00    ; MSNA P1[7:0]    = $00 → P1 = $0C00 = 3072
  .byte 31, $00    ; MSNA P3[19:16] + P2[19:16] = 0
  .byte 32, $00    ; MSNA P2[15:8]   = 0
  .byte 33, $00    ; MSNA P2[7:0]    = 0 → P2 = 0

  ; ====== MS0 Configuration (CLK0): Divide by 100 → 700 MHz / 100 = 7 MHz
  ; Integer mode: Div = 100, Num = 0, Den = 1
  ; P1 = 128 × 100 - 512 = 12288 = $3000
  ; P2 = 0, P3 = 1

  .byte 42, $00    ; MS0 P3[15:8]    = 0
  .byte 43, $01    ; MS0 P3[7:0]     = 1 → P3 = 1
  .byte 44, $00    ; MS0 R_DIV + DIVBY4 + P1[17:16] = 0 + 0 + (12288 >> 16) = $00
  .byte 45, $30    ; MS0 P1[15:8]    = (12288 >> 8) & $FF = $30
  .byte 46, $00    ; MS0 P1[7:0]     = 
  .byte 47, $00    ; MS0 P3[19:16] + P2[19:16] = 0
  .byte 48, $00    ; MS0 P2[15:8]    = 0
  .byte 49, $00    ; MS0 P2[7:0]     = 0

  ; ====== MS1 Configuration (CLK1): Same as MS0 → 7 MHz
  ; Exact same parameters as above

  .byte 50, $00    ; MS1 P3[15:8]    = 0
  .byte 51, $01    ; MS1 P3[7:0]     = 1 → P3 = 1
  .byte 52, $00    ; MS1 R_DIV + DIVBY4 + P1[17:16] = $00
  .byte 53, $30    ; MS1 P1[15:8]    = $30
  .byte 54, $00    ; MS1 P1[7:0]     = $00
  .byte 55, $00    ; MS1 P3[19:16] + P2[19:16] = 0
  .byte 56, $00    ; MS1 P2[15:8]    = 0
  .byte 57, $00    ; MS1 P2[7:0]     = 0

; ====== CLK1 Phase Offset
; Each unit = 1 / (4 × PLL freq) = 1 / (4 × 700 MHz) = ~0.357 ns
; 90° phase shift @ 7 MHz = 35.7 ns → 35.7 / 0.357 ≈ 100 units
.byte 166, 100   ; Phase offset for CLK1 (100 units ≈ 90° shift @ 7 MHz)

  .byte 177, $AC   ; PLL Reset: 0 = reset PLLA, 1 = reset PLLB

  ; ====== Output Enable Control
  .byte 3, $F8     ; Enable all outputs (0 = enabled) (CLK3-CLK7 are disabled)

  ; ====== CLKx Control (Registers 16–17)
  ; Bit 7 = Powerdown (0 = on)
  ; Bit 6 = PLL select (0 = PLLA, 1 = PLLB)
  ; Bit 5 = Invert (0 = normal)
  ; Bits 4–2 = Drive strength (0 = 2 mA, 3 = 8 mA)
  ; Bits 1–0 = MSx mode (0 = int)
  .byte 16, $0F    ; CLK0: PLLA, no invert, 8 mA, frac mode because clk1 is
  .byte 17, $0F    ; CLK1: PLLA, no invert, 8 mA, frac mode (can't use int mode with phase offset)

  ; ====== End of Table
  .byte $FF        ; End marker

