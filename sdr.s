; =============================================
; SDR ADC capture and RAM dump routines (6502)
; ---------------------------------------------
; - Captures interleaved I/Q samples from ADC
; - Stores into external RAM in banked pages
; - Dumps captured bytes over serial
; =============================================
readadcstart:
ldy #0
sty bankcount
sty ptr              ; ptr low = 0
lda #$8e
sta DDRA      ; Set A7/D7 as output
lda #$82
sta countandclock
ldx #2
stx DRA            ; Serial TX high, clock off
; Fall through 

;jsr ramtest_init
; ---------------------------------------------
; readadc
; Configure port, iterate pages/banks, capture two bytes per step:
;   - Turn clock ON (DRA)
;   - Read I ($0401) → store
;   - Turn clock OFF (DRA)
;   - Read Q ($0400) → store
; Advances ptr across pages and bankcount across banks.
; ---------------------------------------------

readadc:
lda #08
sta ptr+1

adcinloop:
	lda countandclock  ; Should have $80 set at all times
	sta DRA            ; Turn on clock

	lda $0401          ; Read ADC I channel
	sta (ptr),y        ; Store in external RAM
	iny 
	stx DRA            ; Turn off clock - should have $80 clear
	lda $0400          ; Read ADC Q channel
	sta (ptr),y        ; Store in external RAM
	iny
	bne adcinloop    ; Continue within page
	bit tflags
	bvs adcdone

	; Advance to next page
	lda ptr+1
	clc
	adc #1
	sta ptr+1
	cmp #$0c
	bne adcinloop    ; Next page

	; Advance to next bank (0..3)
	lda bankcount      ; Next bank
	clc
	adc #1
	sta bankcount
	cmp #4
	beq adcdone        ; End after last bank
	asl               ; Prepare bank select bits
	asl
	ora #2
	tax
	ora #$82
	sta countandclock  ; Update control mask for next bank
	bne readadc  ; BRA

adcdone:
	; Finalize: ensure clock off, TX high
	lda #0
	sta bankcount      
	lda #2 
	sta DRA            ; Turn off clock, Serial TX high

rts

; ---------------------------------------------
; ramout
; Iterate pages and banks, dump bytes via serial.
; Mirrors the traversal used in readadc.
; ---------------------------------------------
ramout:
lda #$AB
jsr serial_tx
lda #$AB
jsr serial_tx

ramoutnopre:
; External RAM at $0800
lda #8
sta ptr+1

; Dump four pages of RAM as hex via serial x 4 banks
userland_loop:
lda (ptr),y
jsr serial_tx
iny
bne userland_loop
lda ptr+1
clc
adc #1
sta ptr+1
cmp #$0c    
bne userland_loop

lda bankcount
clc
adc #1
sta bankcount
cmp #4
beq ramoutdone
asl
asl
ora #2
sta DRA            ; Switch bank
bne ramoutnopre ; BRA
ramoutdone:
rts

; Coefficients: 2*cos(2*pi*k/64) in signed Q1.8
; Fs ≈ 27,778 Hz → bin spacing ≈ 434.0 Hz (N=64)
; Target ~1 kHz steps using k = [2,5,7,9,12,14,16,18]
; Centers ≈ [0.87k, 2.17k, 3.04k, 3.91k, 5.21k, 6.08k, 6.96k, 7.83k] Hz
; (Q0.7 tables removed - using Q1.14 tables only)

; Q1.14 coefficients (16-bit little-endian) for 2*cos(2*pi*k/64) scaled to Q1.14
; These values should be calculated offline; placeholder values derived from q0.7*128
; (Q1.14 tables replaced by split low/high byte arrays later)

; General purpose 8x8 -> 16 multiply: td_a * td_b = td_prod_hi:td_prod_lo
.proc mul8
    txa
    pha
    lda td_b
    pha             ; save multiplier
    lda xtmp
    pha
    lda td_tmp_hi
    pha

    lda #$00
    sta td_prod_lo
    sta td_prod_hi

    lda td_a
    sta xtmp
    lda #$00
    sta td_tmp_hi

    ldx #$08
mul8_loop:
    lda td_b
    and #$01
    beq :+
    clc
    lda td_prod_lo
    adc xtmp
    sta td_prod_lo
    lda td_prod_hi
    adc td_tmp_hi
    sta td_prod_hi
:
    asl xtmp
    rol td_tmp_hi
    lsr td_b
    dex
    bne mul8_loop

    pla
    sta td_tmp_hi
    pla
    sta xtmp
    pla
    sta td_b
    pla
    tax
    rts
.endproc

; Signed 8x8 -> 16 multiply wrapper: td_a * td_b (signed)
.proc mul8_signed
	; Inputs: td_a (signed 8-bit), td_b (signed 8-bit)
	; Output: td_prod_hi:td_prod_lo (signed 16-bit)
	; Preserves X across call (delegates to mul8 which preserves X)
	; Determine sign of result
	lda td_a
	and #$80
	sta td_tmp      ; td_tmp bit7 = sign(a)
	lda td_b
	and #$80
	eor td_tmp      ; bit7 = sign(a) ^ sign(b)
	sta td_tmp      ; td_tmp bit7 = sign(result)
	; Make td_a absolute
	lda td_a
	bpl :+
	eor #$FF
	clc
	adc #$01
	:
	sta td_a
	; Make td_b absolute
	lda td_b
	bpl :+
	eor #$FF
	clc
	adc #$01
	:
	sta td_b
	; Perform unsigned multiply
	jsr mul8
	; Apply result sign if needed (two's complement negate)
	lda td_tmp
	and #$80
	beq :+
	lda td_prod_lo
	eor #$FF
	clc
	adc #$01
	sta td_prod_lo
	lda td_prod_hi
	eor #$FF
	adc #$00
	sta td_prod_hi
	:
	rts
.endproc

; ---------------------------------------------
; Debug helpers (ROM)
; ---------------------------------------------
.proc dbg_putc ; safe single-char print preserving X/Y
	; A contains the character to print; preserve X and Y without touching A
	stx dbg_saved_x ; save X in global ZP temp
	sty td_saved_y ; save Y in new workspace temp (avoid clobbering td_b)
	jsr serial_tx ; send A
	ldx dbg_saved_x ; restore X
	ldy td_saved_y; restore Y
	rts
.endproc

.segment "WORKSPACE"
.ORG $0050
; Zero-page workspace for tone detection (fits within $50-$7F window)
td_s_prev_lo: .res 1
td_s_prev_hi: .res 1
td_power_lo: .res 1
td_power_hi: .res 1
td_bin_power_lo: .res 1
td_bin_power_hi: .res 1
; removed td_coeff_q07 and td_sin_q07 - use td_coeff_lo/td_coeff_hi
td_tmp: .res 1
td_sample: .res 1
td_max_bin: .res 1
td_prod_lo: .res 1
td_prod_hi: .res 1
td_a:       .res 1
td_b:       .res 1
td_tmp_lo:  .res 1
td_tmp_hi:  .res 1
td_tmp32_0: .res 1
td_tmp32_1: .res 1
td_tmp32_2: .res 1
td_tmp32_3: .res 1
td_tmp32_4: .res 1
td_tmp32_5: .res 1
td_saved_y: .res 1
dbg_saved_x: .res 1
td_coeff_lo: .res 1
td_coeff_hi: .res 1
td_s_prev2_lo: .res 1
td_s_prev2_hi: .res 1
;td_mul_debug: .res 1  ; removed - unused
;td_samples_shown: .res 1  ; removed - unused
;td_sample_idx: .res 1  ; removed - unused
; (alt multiply temps removed to save ROM)

.segment "RODATA"
; Q1.14 coefficients (signed 16-bit, little endian) for bins [2,5,7,9,12,14,16,18]
; Store Q1.14 coeffs split into low/high byte arrays for indexed access
coeff_q14_lo:
	.byte $C5,$71,$79,$9A,$7E,$7C,$00,$83
coeff_q14_hi:
	.byte $3E,$38,$31,$28,$18,$0C,$00,$F3
sin_q14_lo:
	.byte $7C,$2B,$9A,$79,$21,$C5,$00,$C5
sin_q14_hi:
	.byte $0C,$1E,$28,$31,$3B,$3E,$40,$3E

.proc td_compute_t
	; Wrapper that preserves X and Y then forwards to Q1.14 computation
	txa
	pha
	tya
	pha
	jsr td_compute_t_q14
	pla
	tay
	pla
	tax
	rts
.endproc


; 16-bit Q1.14 multiply helper: r = (coeff(16) * val(16)) >> 14 (arith)
.proc td_mul_q14_shift14
	; Inputs: td_tmp_hi:td_tmp_lo = val (16-bit signed)
	;         td_coeff_hi:td_coeff_lo = coeff (16-bit signed)
	; Output: td_tmp_hi:td_tmp_lo = result (16-bit signed) = (coeff*val) >> 14 (arith)
	sty td_saved_y
	txa
	pha
	jsr td_mul_16x16       ; produces signed 32-bit in td_tmp32_3..0
	jsr td_shr14_arith     ; arithmetic >>14, places low 16 bits in td_tmp_hi:td_tmp_lo
	ldy td_saved_y
	pla
	tax
	rts
.endproc

; Signed 16x16 -> signed 32 multiply
; Inputs:
;   td_coeff_hi:td_coeff_lo (signed)
;   td_tmp_hi:td_tmp_lo     (signed)
; Output:
;   td_tmp32_3:td_tmp32_2:td_tmp32_1:td_tmp32_0

.proc td_mul_16x16
    sty td_saved_y
    txa
    pha

    ; clear accumulator
    lda #$00
    sta td_tmp32_0
    sta td_tmp32_1
    sta td_tmp32_2
    sta td_tmp32_3

    ; sign = coeff_hi ^ tmp_hi
    lda td_coeff_hi
    eor td_tmp_hi
    and #$80
    sta td_tmp

    ; abs(coeff)
    lda td_coeff_hi
    bpl coeff_pos
    lda td_coeff_lo
    eor #$FF
    sta td_coeff_lo
    lda td_coeff_hi
    eor #$FF
    sta td_coeff_hi
    clc
    lda td_coeff_lo
    adc #$01
    sta td_coeff_lo
    lda td_coeff_hi
    adc #$00
    sta td_coeff_hi
coeff_pos:

    ; abs(tmp)
    lda td_tmp_hi
    bpl tmp_pos
    lda td_tmp_lo
    eor #$FF
    sta td_tmp_lo
    lda td_tmp_hi
    eor #$FF
    sta td_tmp_hi
    clc
    lda td_tmp_lo
    adc #$01
    sta td_tmp_lo
    lda td_tmp_hi
    adc #$00
    sta td_tmp_hi
tmp_pos:

    ; P0 = lo * lo
    lda td_coeff_lo
    sta td_a
    lda td_tmp_lo
    sta td_b
    jsr mul8
    lda td_prod_lo
    sta td_tmp32_0
    lda td_prod_hi
    sta td_tmp32_1

    ; P1 = hi * lo << 8
    lda td_coeff_hi
    sta td_a
    lda td_tmp_lo
    sta td_b
    jsr mul8
    clc
    lda td_tmp32_1
    adc td_prod_lo
    sta td_tmp32_1
    lda td_tmp32_2
    adc td_prod_hi
    sta td_tmp32_2
    lda td_tmp32_3
    adc #$00
    sta td_tmp32_3

    ; P2 = lo * hi << 8
    lda td_coeff_lo
    sta td_a
    lda td_tmp_hi
    sta td_b
    jsr mul8
    clc
    lda td_tmp32_1
    adc td_prod_lo
    sta td_tmp32_1
    lda td_tmp32_2
    adc td_prod_hi
    sta td_tmp32_2
    lda td_tmp32_3
    adc #$00
    sta td_tmp32_3

    ; P3 = hi * hi << 16
    lda td_coeff_hi
    sta td_a
    lda td_tmp_hi
    sta td_b
    jsr mul8
    clc
    lda td_tmp32_2
    adc td_prod_lo
    sta td_tmp32_2
    lda td_tmp32_3
    adc td_prod_hi
    sta td_tmp32_3

    ; apply sign
    lda td_tmp
    bpl done
    lda td_tmp32_0
    eor #$FF
    sta td_tmp32_0
    lda td_tmp32_1
    eor #$FF
    sta td_tmp32_1
    lda td_tmp32_2
    eor #$FF
    sta td_tmp32_2
    lda td_tmp32_3
    eor #$FF
    sta td_tmp32_3
    clc
    lda td_tmp32_0
    adc #$01
    sta td_tmp32_0
    lda td_tmp32_1
    adc #$00
    sta td_tmp32_1
    lda td_tmp32_2
    adc #$00
    sta td_tmp32_2
    lda td_tmp32_3
    adc #$00
    sta td_tmp32_3

done:
    ldy td_saved_y
    pla
    tax
    rts
.endproc

; Arithmetic right shift by 14 of td_tmp32_3..0; returns low16 in td_tmp_hi:td_tmp_lo
.proc td_shr14_arith
	ldx #$0E
shr14_loop:
	lda td_tmp32_3
	and #$80
	beq shr14_clr
	sec
	jmp shr14_ror
shr14_clr:
	clc
shr14_ror:
	ror td_tmp32_3
	ror td_tmp32_2
	ror td_tmp32_1
	ror td_tmp32_0
	dex
	bne shr14_loop
	lda td_tmp32_0
	sta td_tmp_lo
	lda td_tmp32_1
	sta td_tmp_hi
	rts
.endproc

; Small wrapper to call td_mul_q14_shift14 from userland without using ZP
; Inputs: A=val_lo, X=val_hi, Y=coeff_index
; Effect: computes (coeff * val)>>14 and prints result (td_tmp_hi/lo)
; (td_call_and_print wrapper removed to reduce ROM size)

; 16-bit Q1.14 t computation: t = (coeff_q14 * (s_prev<<1)) >> 14
.proc td_compute_t_q14
    lda td_s_prev_lo
    sta td_tmp_lo
    lda td_s_prev_hi
    sta td_tmp_hi
    asl td_tmp_lo
    rol td_tmp_hi

    lda coeff_q14_lo,x
    sta td_coeff_lo
    lda coeff_q14_hi,x
    sta td_coeff_hi

    jsr td_mul_q14_shift14
    rts
.endproc

; 16-bit Q1.14 recurrence update and final energy (re/im)
.proc detect_tone_q14_16bit
	lda #$08
	sta ptr+1
	ldx #$00
	stx ptr
	stx td_max_bin
	stx td_power_lo
	stx td_power_hi
bin_loop:
	; reset per-bin energy
	lda #$00
	sta td_bin_power_lo
	sta td_bin_power_hi
	; init prevs
	sta td_s_prev_lo
	sta td_s_prev_hi
	sta td_s_prev2_lo
	sta td_s_prev2_hi
	; process 64 samples
	ldy #$00
@sample_loop2:
	lda (ptr),y
	lsr ; Reduce resolution
	sec
	sbc #$80        ; center to signed 8-bit
	sta td_sample
	; compute t
	jsr td_compute_t
	; s = x + t - s_prev2 (16-bit: sign-extend x, then add t, then subtract s_prev2)
	; build x16_hi in td_tmp32_5
	lda td_sample
	and #$80
	beq :+
	lda #$FF
	bne :++
:
	lda #$00
:
	sta td_tmp32_5   ; x16_hi
	; sum = x16 + t
	clc
	lda td_sample    ; x16_lo
	adc td_tmp_lo    ; + t_lo
	sta td_tmp_lo    ; sum_lo
	lda td_tmp32_5   ; x16_hi
	adc td_tmp_hi    ; + t_hi + carry
	sta td_tmp_hi    ; sum_hi
	; s = sum - s_prev2
	sec
	lda td_tmp_lo
	sbc td_s_prev2_lo
	sta td_tmp_lo
	lda td_tmp_hi
	sbc td_s_prev2_hi
	sta td_tmp_hi
	; rotate prevs
	lda td_s_prev_lo
	sta td_s_prev2_lo
	lda td_s_prev_hi
	sta td_s_prev2_hi
	lda td_tmp_lo
	sta td_s_prev_lo
	lda td_tmp_hi
	sta td_s_prev_hi
	iny
	iny
	cpy #$80
	bne @sample_loop2
	; compute re = s_prev - (coeff*s_prev2)>>14
	lda coeff_q14_lo,x
	sta td_coeff_lo
	lda coeff_q14_hi,x
	sta td_coeff_hi
	lda td_s_prev2_lo
	sta td_tmp_lo
	lda td_s_prev2_hi
	sta td_tmp_hi
	jsr td_mul_q14_shift14
	sec
	lda td_s_prev_lo
	sbc td_tmp_lo
	sta td_prod_lo
	lda td_s_prev_hi
	sbc td_tmp_hi
	sta td_prod_hi
	; im = (sin_q14 * s_prev2)>>14
	lda sin_q14_lo,x
	sta td_coeff_lo
	lda sin_q14_hi,x
	sta td_coeff_hi
	lda td_s_prev2_lo
	sta td_tmp_lo
	lda td_s_prev2_hi
	sta td_tmp_hi
	jsr td_mul_q14_shift14
	lda td_tmp_lo
	sta td_tmp32_4
	lda td_tmp_hi
	sta td_tmp32_5
	; abs(re)
	lda td_prod_hi
	and #$80
	beq repos2
	lda td_prod_lo
	eor #$FF
	sta td_tmp32_0
	lda td_prod_hi
	eor #$FF
	sta td_tmp32_1
	inc td_tmp32_0
	beq reabs2
	inc td_tmp32_1
	jmp reabs2
repos2:
	lda td_prod_lo
	sta td_tmp32_0
	lda td_prod_hi
	sta td_tmp32_1
reabs2:
	; abs(im)
	lda td_tmp32_5
	and #$80
	beq impos2
	lda td_tmp32_4
	eor #$FF
	sta td_tmp32_2
	lda td_tmp32_5
	eor #$FF
	sta td_tmp32_3
	inc td_tmp32_2
	beq imabs2
	inc td_tmp32_3
	jmp imabs2
impos2:
	lda td_tmp32_4
	sta td_tmp32_2
	lda td_tmp32_5
	sta td_tmp32_3
imabs2:
	; sum abs_re + abs_im
	clc
	lda td_tmp32_0
	adc td_tmp32_2
	sta td_tmp_lo
	lda td_tmp32_1
	adc td_tmp32_3
	sta td_tmp_hi
	; (diagnostics removed) - only print index and energy
	; store per-bin energy
	lda td_tmp_lo
	sta td_bin_power_lo
	lda td_tmp_hi
	sta td_bin_power_hi
	
	bit tflags
	bpl :+ 
	; print index and energy
	txa
	jsr printsafebyte
	lda #$20
	jsr dbg_putc
	lda td_bin_power_hi
	jsr printsafebyte
	lda td_bin_power_lo
	jsr printsafebyte
	lda #$0A
	jsr dbg_putc
	:
	jsr td_select_max
	inx
	cpx #$08
	beq end_bins2
	jmp bin_loop
end_bins2:
	rts
.endproc

	; removed legacy detect_tone_recur24 wrapper

; Minimal debug stubs to save ROM space
.proc printsafebyte
    pha ; Save A
    jsr bytetoa
    pha
    lda xtmp
    jsr dbg_putc
    pla
    jsr dbg_putc
    pla ; Restore A
	rts
.endproc

.proc td_select_max
	; Compare td_tmp (E) with td_power, update max and index
	; Compare per-bin energy (td_bin_power_*) against global max (td_power_*)
	lda td_bin_power_hi
	cmp td_power_hi
	bcc @done
	bne @set
	lda td_bin_power_lo
	cmp td_power_lo
	bcc @done
@set:
	lda td_bin_power_lo
	sta td_power_lo
	lda td_bin_power_hi
	sta td_power_hi
	txa
	sta td_max_bin
@done:
	rts
.endproc

ledpb2347:
pha
lda DDRB
ora #$9C ; Set PB2,PB3,PB4,PB7 as outputs
sta DDRB

lda DRB
ora #$9C
sta DRB   ; Set PB2,PB3,PB4,PB7 high
pla
cmp #0
bne :+
lda DRB
and #$fb ; PB2 low if tone detected = LED On
sta DRB
:
cmp #1
bne :+
lda DRB
and #$f7 ; PB3 low if tone detected = LED On
sta DRB
:
cmp #2
bne :+
lda DRB
and #$ef ; PB4 low if tone detected = LED On
sta DRB
:
cmp #3
bne :+
lda DRB
and #$7f ; PB7 low if tone detected = LED On
sta DRB
:
rts