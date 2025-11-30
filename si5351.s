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

BTN_DRB2 = %00000100  ; DRB2 mask (increase)
BTN_DRB3 = %00001000  ; DRB3 (unused)
BTN_DRB4 = %00010000  ; DRB4 (unused)
BTN_DRB5 = %00100000  ; DRB5 mask (decrease)
BTN_DRB6 = %01000000  ; DRB6 mask (set freq)

freq_index = cursor      ; Current index, range 0–15
debounce_mask = tflags   ; Temporary debounce bitmask
temp = xtmp

setup_sdr_mode:
jsr setup_si5351

  lda #%11110010   ; DRA4–DRA7 + TX as outputs
  sta DDRA

  ldy #1               ; Example index: 1 = 7.030 MHz or any valid one
  sty freq_index  ; Initialize frequency index
  jsr get_freq_ptr     ; returns ptr to freq table (or $0000 for off)

  lda ptr
  ora ptr+1
  beq skip_setup       ; If ptr == 0, skip config

  jsr update_leds
  jsr setup_frequency_from_lut

skip_setup:

button_loop:
  ; === Check DRB5 (Increase Index) ===
  lda DRB
  and #BTN_DRB5
  bne check_drb2

  lda #BTN_DRB5
  jsr debounce_masked
  bcc check_drb2       ; Bounced/released

  lda freq_index
  cmp #15
  beq check_drb2       ; Already at max
  inc freq_index
  jsr update_leds
  jmp end_button_loop

check_drb2:
  ; === Check DRB2 (Decrease Index) ===
  lda DRB
  and #BTN_DRB2
  bne check_drb6

  lda #BTN_DRB2
  jsr debounce_masked
  bcc check_drb6       ; Bounced/released
  lda freq_index
  beq check_drb6       ; Already at 0
  dec freq_index
  jsr update_leds
  jmp end_button_loop

check_drb6:
  ; === Check DRB6 (Set Frequency) ===
  lda DRB
  and #BTN_DRB6
  bne end_button_loop  ; Not pressed

  lda #BTN_DRB6
  jsr debounce_masked
  bcc end_button_loop  ; Bounced/released

  ldy freq_index
  jsr get_freq_ptr     ; returns ptr to freq table (or $0000 for off)
  jsr setup_frequency_from_lut    ; Set frequency from freq_index
  lda freq_index ; Save current index for LED update
  pha 
  ldy #0
  sty freq_index ; Reset index to 0
  jsr update_leds ;Blink LEDs to indicate frequency change
  lda #50
  jsr delay_long
  pla
  sta freq_index ; Restore freq_index
  jsr update_leds ; Update LEDs to show new frequency

end_button_loop:
jmp button_loop

; Input: A = bitmask (e.g., #%00000100 for DRB2)
; Clobbers: A, X
; Returns: Carry set = stable press, Carry clear = bounce/released
debounce_masked:
  sta debounce_mask     ; store bitmask
  lda #$90         ; Set debounce time 
  sta WTD1KDI           ; start debounce timer

wait_masked:
  lda READTDI
  beq debounce_timeout  ; timer ran out

  lda DRB
  and debounce_mask
  bne debounce_short    ; button released

  beq wait_masked       ; still pressed → keep waiting

debounce_timeout:
  sec                   ; debounce passed
  rts

debounce_short:
  clc                   ; button bounced or released early
  rts

update_leds:
  lda freq_index     ; Load current index (0–15)
  asl                ; Shift left 4 times to go into DRA4–7
  asl
  asl
  asl
  eor #%11110000  ; Invert bits to match LED logic
  sta temp

  lda DRA            ; Read current port state
  and #%00001111     ; Clear upper nibble
  ora temp           ; Merge with shifted index
  sta DRA            ; Write to port
  rts

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

setup_frequency_from_lut:
  ; Disable all CLK outputs (register 3)
  lda #3
  ldy #$ff
  jsr i2c_write_register
    ; ptr -> combined record: 8 bytes PLL, then 9 bytes MS table
    ldy #0
    ; write registers 26..33 (PLL)
pll_write_loop: ; We can't use y or x so this looks a bit messy
    tya 
    pha ; Save loop index
    lda (ptr),y
    ; Set target register = 26 + index (we'll put it in A then Y will contain value)
    tay              ; Save data in Y
    pla              ; A = register index
    pha              ; Save loop index
    clc
    adc #26
    jsr i2c_write_register
    pla ; restore Y loop index
    tay
    iny
    cpy #8
    bne pll_write_loop

    ; Now write MS table inline at ptr+8
    ; Set ptr to point to ms table (ptr+8)
    clc
    lda ptr
    adc #8
    sta ptr
    lda ptr+1
    adc #0
    sta ptr+1

    ; write MS registers (42..49 and 50..57) from the inline table
    jsr write_ms0_ms1

    ; Reset PLLs (reg 177)
    lda #177
    ldy #$AC
    jsr i2c_write_register

    ; Enable CLK0 & CLK1 outputs (reg 3)
    lda #3
    ldy #$FC
    jsr i2c_write_register

    rts

; Write MS0 and MS1 registers 42–57 using shared 8-byte data table
; IN: ptr = pointer to 8-byte data table
; Uses: A, X, Y
; Clobbers: A, X, Y

write_ms0_ms1:
    ldy #0
msxloop:
    tya
    pha                  ; Save full loop index (0–15)

    and #$07             ; A = Y & 7 (index into data)
    tay
    lda (ptr), y         ; A = data byte
    tay                  ; Y = data

    pla                  ; A = loop index again
    pha                  ; Save again for later
    clc
    adc #42              ; A = target register number
    jsr i2c_write_register

    pla
    tay                  ; Y = loop counter again
    iny
    cpy #16
    bne msxloop
    ldy #8
    ;CLK0 90deg offset
    lda (ptr),y ; Get the 90deg offset value
    tay
    lda #165 ; CLK0 register
    jsr i2c_write_register
    rts


; get_freq_ptr
; --------------
; Y = index (0–15)
; If Y = 0 → returns ptr = $0000
; Else     → loads 2-byte pointer from freq_table into ptr

get_freq_ptr:
  tya
  asl                    ; Multiply Y by 2 (2 bytes per entry)
  tax
  lda freq_table,x       ; Load low byte of pointer
  sta ptr
  lda freq_table+1,x     ; Load high byte
  sta ptr+1
  rts

; freq=7025000Hz MS=86 VCO=604150000Hz PLL=24,83,500 P=2581,124,500
; PLL regs 26..33: MSNA P3[15:8], P3[7:0], P1[17:16]+..., P1[15:8], P1[7:0], P2[19:16], P2[15:8], P2[7:0]
; MS0 regs 42..49: P3,P3,P1[..],P1[15:8],P1[7:0],P2[..],P2[15:8],P2[7:0]
; MS1 regs 50..57: P3,P3,P1[..],P1[15:8],P1[7:0],P2[..],P2[15:8],P2[7:0]
; Phase/Integer divider: reg 165 (phase offset for CLK0) - last byte
frq_7025000_data: .byte 1, 244, 0, 10, 21, 0, 0, 124, 0, 1, 0, 41, 0, 0, 0, 0, 86

; freq=7075000Hz MS=85 VCO=601375000Hz PLL=24,11,200 P=2567,8,200
frq_7075000_data: .byte 0, 200, 0, 10, 7, 0, 0, 8, 0, 1, 0, 40, 128, 0, 0, 0, 85

; freq=7125000Hz MS=85 VCO=605625000Hz PLL=24,9,40 P=2588,32,40
frq_7125000_data: .byte 0, 40, 0, 10, 28, 0, 0, 32, 0, 1, 0, 40, 128, 0, 0, 0, 85

; freq=7175000Hz MS=84 VCO=602700000Hz PLL=24,27,250 P=2573,206,250
frq_7175000_data: .byte 0, 250, 0, 10, 13, 0, 0, 206, 0, 1, 0, 40, 0, 0, 0, 0, 84

; freq=7225000Hz MS=84 VCO=606900000Hz PLL=24,69,250 P=2595,82,250
frq_7225000_data: .byte 0, 250, 0, 10, 35, 0, 0, 82, 0, 1, 0, 40, 0, 0, 0, 0, 84

; freq=7275000Hz MS=83 VCO=603825000Hz PLL=24,153,1000 P=2579,584,1000
frq_7275000_data: .byte 3, 232, 0, 10, 19, 0, 2, 72, 0, 1, 0, 39, 128, 0, 0, 0, 83

; freq=89650000Hz MS=7 VCO=627550000Hz PLL=25,51,500 P=2701,28,500
; PLL regs 26..33: MSNA P3[15:8], P3[7:0], P1[17:16]+..., P1[15:8], P1[7:0], P2[19:16], P2[15:8], P2[7:0]
; MS0 regs 42..49: P3,P3,P1[..],P1[15:8],P1[7:0],P2[..],P2[15:8],P2[7:0]
; MS1 regs 50..57: P3,P3,P1[..],P1[15:8],P1[7:0],P2[..],P2[15:8],P2[7:0]
; Phase/Integer divider: reg 165 (phase offset for CLK0) - last byte
; freq=89750000Hz MS=7 VCO=628250000Hz PLL=25,13,100 P=2704,64,100
frq_89750000_data: .byte 0, 100, 0, 10, 144, 0, 0, 64, 0, 1, 0, 1, 128, 0, 0, 0, 7

frq_93350000_data: .byte 1, 244, 0, 11, 17, 0, 1, 76, 0, 1, 0, 1, 128, 0, 0, 0, 7

; freq=97850000Hz MS=7 VCO=684950000Hz PLL=27,199,500 P=2994,472,500
frq_97850000_data: .byte 1, 244, 0, 11, 178, 0, 1, 216, 0, 1, 0, 1, 128, 0, 0, 0, 7

; freq=98150000Hz MS=7 VCO=687050000Hz PLL=27,241,500 P=3005,348,500
frq_98150000_data: .byte 1, 244, 0, 11, 189, 0, 1, 92, 0, 1, 0, 1, 128, 0, 0, 0, 7

; freq=100250000Hz MS=6 VCO=601500000Hz PLL=24,3,50 P=2567,34,50
frq_100250000_data: .byte 0, 50, 0, 10, 7, 0, 0, 34, 0, 1, 0, 1, 0, 0, 0, 0, 6

; freq=102750000Hz MS=6 VCO=616500000Hz PLL=24,33,50 P=2644,24,50
frq_102750000_data: .byte 0, 50, 0, 10, 84, 0, 0, 24, 0, 1, 0, 1, 0, 0, 0, 0, 6

; freq=104550000Hz MS=6 VCO=627300000Hz PLL=25,23,250 P=2699,194,250
frq_104550000_data: .byte 0, 250, 0, 10, 139, 0, 0, 194, 0, 1, 0, 1, 0, 0, 0, 0, 6

; freq=106050000Hz MS=6 VCO=636300000Hz PLL=25,113,250 P=2745,214,250
frq_106050000_data: .byte 0, 250, 0, 10, 185, 0, 0, 214, 0, 1, 0, 1, 0, 0, 0, 0, 6

; freq=144850000Hz MS=5 VCO=724250000Hz PLL=28,97,100 P=3196,16,100
frq_144850000_data: .byte 0, 100, 0, 12, 124, 0, 0, 16, 0, 1, 0, 0, 128, 0, 0, 0, 5

; Pointer table for generated frequencies
freq_table:
  .word frq_7025000_data
  .word frq_7075000_data
  .word frq_7125000_data
  .word frq_7175000_data
  .word frq_7225000_data
  .word frq_7275000_data
  .word frq_89750000_data ; P3
  .word frq_93350000_data ; P1
  .word frq_97850000_data ; NORD
  .word frq_98150000_data ; P4
  .word frq_100250000_data ; The Voice
  .word frq_102750000_data ; Radio4
  .word frq_104550000_data ; ANR
  .word frq_106050000_data ; NOVA
  .word frq_144850000_data ; APRS
  

; ====== Si5351 Initialization Data ======
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
  .byte 46, $00    ; MS0 P1[7:0]     = $00
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
.byte 165, 100   ; Phase offset for CLK0 (100 units ≈ 90° shift @ 7 MHz)

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
; Each unit = 1 / (4 × PLL freq) = 1 / (4 × 700 MHz) = ~0.357 ns
; 90° phase shift @ 7 MHz = 35.7 ns → 35.7 / 0.357 ≈ 100 units
.byte 165, 100   ; Phase offset for CLK0 (100 units ≈ 90° shift @ 7 MHz)

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

