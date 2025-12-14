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
	sty bankcount      ; Hope Y is 0
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
	; Correct unsigned 8x8 -> 16 using 16-bit accumulator shift-and-add
	; Inputs: td_a (multiplicand), td_b (multiplier)
	; Output: td_prod_hi:td_prod_lo
	; Preserve X across call
	txa
	pha
	lda #$00
	sta td_prod_lo
	sta td_prod_hi
	; 16-bit accumulator acc_hi:acc_lo starts as td_a
	lda td_a
	sta xtmp          ; reuse xtmp as acc_lo
	lda #$00
	sta td_tmp_hi     ; acc_hi
	ldx #$08
mul8_loop:
	lda td_b
	and #$01
	beq mul8_skip_add
	; prod += acc (16-bit)
	clc
	lda td_prod_lo
	adc xtmp          ; acc_lo
	sta td_prod_lo
	lda td_prod_hi
	adc td_tmp_hi     ; acc_hi
	sta td_prod_hi
mul8_skip_add:
	; acc <<= 1 (16-bit)
	asl xtmp
	rol td_tmp_hi
	; td_b >>= 1
	lsr td_b
	dex
	bne mul8_loop
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
td_mul_debug: .res 1
td_samples_shown: .res 1
td_sample_idx: .res 1
; (alt multiply temps removed to save ROM)
.segment "RODATA"
; Using Q1.14 coeffs and safe zero-page temps


; Q1.14 coefficients (signed 16-bit, little endian) for bins [2,5,7,9,12,14,16,18]
; Store Q1.14 coeffs split into low/high byte arrays for indexed access
coeff_q14_lo:
	.byte $8A,$E3,$F2,$34,$FC,$F9,$00,$07
coeff_q14_hi:
	.byte $7D,$70,$62,$51,$30,$18,$00,$E7
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
	; Uses td_tmp32_0..td_tmp32_5 for partials and td_prod_lo/hi for temporary P3
	; Preserve Y and X across this helper (caller uses Y and X heavily)
	sty td_saved_y
	txa
	pha
	; Compute P0 = coeff_lo * val_lo
	lda td_coeff_lo
	sta td_a
	lda td_tmp_lo
	sta td_b
	jsr mul8_signed
	lda td_prod_lo
	sta td_tmp32_0   ; P0_lo
	lda td_prod_hi
	sta td_tmp32_1   ; P0_hi
	; (P0 computed above in td_tmp32_0/1)
	; Compute P1 = coeff_hi * val_lo
	lda td_coeff_hi
	sta td_a
	lda td_tmp_lo
	sta td_b
	jsr mul8_signed
	lda td_prod_lo
	sta td_tmp32_2   ; P1_lo
	lda td_prod_hi
	sta td_tmp32_3   ; P1_hi
	; (P1 computed above in td_tmp32_2/3)
	; Compute P2 = coeff_lo * val_hi
	lda td_coeff_lo
	sta td_a
	lda td_tmp_hi
	sta td_b
	jsr mul8_signed
	lda td_prod_lo
	sta td_tmp32_4   ; P2_lo
	lda td_prod_hi
	sta td_tmp32_5   ; P2_hi
	; (P2 computed above in td_tmp32_4/5)
	; Compute P3 = coeff_hi * val_hi (result in td_prod_lo/hi)
	lda td_coeff_hi
	sta td_a
	lda td_tmp_hi
	sta td_b
	jsr mul8_signed
	; td_prod_lo/hi = P3_lo/P3_hi
	; (P3 in td_prod_lo/hi)

	; Assemble 32-bit product acc3:acc2:acc1:acc0 into td_tmp32_3..0
	; acc0 = P0_lo (td_tmp32_0)
	; acc1 = P0_hi + P1_lo + P2_lo
	lda td_tmp32_1
	clc
	adc td_tmp32_2
	adc td_tmp32_4
	sta td_tmp32_1
	; save carry from acc1 so it reliably propagates into acc2
	php

	; acc2 = P1_hi + P2_hi + P3_lo + carry
	plp
	lda td_tmp32_3
	adc td_tmp32_5
	adc td_prod_lo
	sta td_tmp32_2
	; acc3 = P3_hi + carry
	lda td_prod_hi
	adc #$00
	sta td_tmp32_3
	; No debug printing here in production image.

	; Now perform arithmetic right shift by 14 on the 32-bit acc (td_tmp32_3:2:1:0)
	ldx #$0E
shr14_loop:
    ; set carry = sign bit of acc3
    lda td_tmp32_3
    and #$80
    beq clr_c
    sec
    jmp do_ror
clr_c:
    clc
do_ror:
    ; ROR must start at most-significant byte (acc3) and propagate down to acc0
    ror td_tmp32_3
    ror td_tmp32_2
    ror td_tmp32_1
    ror td_tmp32_0
    dex
    bne shr14_loop


	; Return lower 16 bits (acc0:acc1) as result into td_tmp_lo:td_tmp_hi
	lda td_tmp32_0
	sta td_tmp_lo
	lda td_tmp32_1
	sta td_tmp_hi
	; No result prints here in production image
	ldy td_saved_y
	pla
	tax
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
	; print bin index (compact)
	txa
	jsr printsafebyte
	; reset per-bin energy
	lda #$00
	sta td_bin_power_lo
	lda #$00
	sta td_bin_power_hi
	; init prevs
	lda #$00
	sta td_s_prev_lo
	sta td_s_prev_hi
	sta td_s_prev2_lo
	sta td_s_prev2_hi
	; process 64 samples
	ldy #$00
@sample_loop2:
	lda (ptr),y
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
	; store per-bin energy
	lda td_tmp_lo
	sta td_bin_power_lo
	lda td_tmp_hi
	sta td_bin_power_hi
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
