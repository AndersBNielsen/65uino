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
lda #$A5
jsr serial_tx
lda #$A5
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
	stx xtmp      ; save X in global ZP temp
	sty td_saved_y ; save Y in new workspace temp (avoid clobbering td_b)
	jsr serial_tx ; send A
	ldx xtmp      ; restore X
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
; removed td_coeff_q07 and td_sin_q07 - use td_coeff_lo/td_coeff_hi
td_tmp: .res 1
xtmp:   .res 1
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
td_coeff_lo: .res 1
td_coeff_hi: .res 1
.segment "RODATA"
; Using Q1.14 coeffs and safe zero-page temps
.proc td_asr7_16
	; Arithmetic right shift by 7 on td_prod_hi:td_prod_lo
	; Input: td_prod_hi:td_prod_lo
	; Output: td_prod_hi:td_prod_lo shifted arithmetically by 7
	ldx #$07
@loop:
	lda td_prod_hi
	and #$80
	beq @clc
	sec
	bne @do
@clc:
	clc
@do:
	ror td_prod_hi
	ror td_prod_lo
	dex
	bne @loop
	rts
.endproc

.proc td_asr8_16
	; Arithmetic right shift by 8 on td_prod_hi:td_prod_lo
	ldx #$08
@loop8:
	lda td_prod_hi
	and #$80
	beq @clc8
	sec
	bne @do8
@clc8:
	clc
@do8:
	ror td_prod_hi
	ror td_prod_lo
	dex
	bne @loop8
	rts
.endproc

; Q1.14 coefficients (signed 16-bit, little endian) for bins [2,5,7,9,12,14,16,18]
; Store Q1.14 coeffs split into low/high byte arrays for indexed access
coeff_q14_lo:
	.byte $D5,$45,$41,$8E,$7C,$45,$00,$BB
coeff_q14_hi:
	.byte $3E,$35,$2D,$23,$0C,$06,$00,$F9
sin_q14_lo:
	.byte $7C,$8E,$41,$45,$D5,$96,$00,$96
sin_q14_hi:
	.byte $0C,$23,$2D,$35,$3E,$3F,$40,$3F

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
	;         td_coeff_hi:td_coeff_lo = coeff (16-bit signed) — we reuse td_prod_hi/lo to pass coeff
	; Output: td_tmp_hi:td_tmp_lo = result (16-bit signed)
	; Uses: td_prod_lo/hi as temps, td_tmp32_0..3 for partial accumulation
	; Load operands
	; preserve Y across this helper (caller uses Y heavily)
	sty td_saved_y
	lda td_tmp_lo
	sta td_a         ; val_lo
	lda td_tmp_hi
	sta td_b         ; val_hi
	; load coeff from safe temps (do not clobber td_coeff_q07/td_sin_q07)
	lda td_coeff_lo
	sta td_tmp32_0   ; coeff_lo
	lda td_coeff_hi
	sta td_tmp32_1   ; coeff_hi
	; P0 = coeff_lo * val_lo
	lda td_tmp32_0
	sta td_a
	lda td_tmp_lo
	sta td_b
	jsr mul8_signed
	lda td_prod_lo
	sta td_tmp32_2   ; acc_lo
	lda td_prod_hi
	sta td_tmp32_3   ; acc_hi
	; P1 = coeff_hi * val_lo (goes to high bytes)
	lda td_tmp32_1
	sta td_a
	lda td_tmp_lo
	sta td_b
	jsr mul8_signed
	clc
	lda td_tmp32_3
	adc td_prod_lo
	sta td_tmp32_3
	; P2 = coeff_lo * val_hi
	lda td_tmp32_0
	sta td_a
	lda td_tmp_hi
	sta td_b
	jsr mul8_signed
	clc
	lda td_tmp32_3
	adc td_prod_lo
	sta td_tmp32_3
	; After adding cross terms, apply >>14: do two >>7 shifts sequentially on acc_hi:acc_lo
	; Move acc into td_prod_hi:td_prod_lo
	lda td_tmp32_2
	sta td_prod_lo
	lda td_tmp32_3
	sta td_prod_hi
	jsr td_asr7_16
	jsr td_asr7_16
	lda td_prod_lo
	sta td_tmp_lo
	lda td_prod_hi
	sta td_tmp_hi
	ldy td_saved_y
	rts
.endproc

; 16-bit Q1.14 t computation: t = (coeff_q14 * (s_prev<<1)) >> 14
.proc td_compute_t_q14
	; Inputs: td_s_prev_lo/hi, coeff_q14 in table
	; Output: td_tmp_hi:td_tmp_lo = t
	; Build S2 = s_prev << 1
	asl td_s_prev_lo
	rol td_s_prev_hi
	; Load coeff_q14 for current X (lo/hi)
	lda coeff_q14_lo,x
	sta td_coeff_lo
	lda coeff_q14_hi,x
	sta td_coeff_hi
	; td_tmp = S2
	lda td_s_prev_lo
	sta td_tmp_lo
	lda td_s_prev_hi
	sta td_tmp_hi
	jsr td_mul_q14_shift14
	rts
.endproc

; 16-bit Q1.14 recurrence update and final energy (re/im)
.proc detect_tone_q14_16bit
	; entry marker for debugging
	lda #$44
	jsr dbg_putc
	lda #$08
	sta ptr+1
	ldx #$00
	stx ptr
	stx td_max_bin
	stx td_power_lo
	stx td_power_hi
@bin_loop:
	; bin entry marker
	lda #$62 ; 'b'
	jsr dbg_putc
	; clear per-bin max power
	lda #$00
	sta td_power_lo
	lda #$00
	sta td_power_hi
	; init 16-bit states
	lda #$00
	sta td_s_prev_lo
	sta td_s_prev_hi
	sta td_tmp32_0      ; s_prev2_lo
	sta td_tmp32_1      ; s_prev2_hi
	; sample loop (64 I samples)
	ldy #$00
@sample_loop:
	lda (ptr),y
	; center to signed: A = (A - 128)
	sec
	sbc #$80
	; arithmetic shift right by 1: set carry = sign bit then ROR
	and #$80
	beq @asr_nosign
	sec
	jmp @asr_do2
@asr_nosign:
	clc
@asr_do2:
	ror
	sta td_sample
	; compute t
	jsr td_compute_t
	; s = x + t - s_prev2 -> store into td_tmp_lo/hi first
	clc
	lda td_sample
	adc td_tmp_lo
	sec
	sbc td_tmp32_0
	sta td_tmp_lo
	lda #$00
	adc td_tmp_hi
	sbc td_tmp32_1
	sta td_tmp_hi
	; rotate prevs: s_prev2 = old s_prev; s_prev = s (from td_tmp)
	lda td_s_prev_lo
	sta td_tmp32_0
	lda td_s_prev_hi
	sta td_tmp32_1
	lda td_tmp_lo
	sta td_s_prev_lo
	lda td_tmp_hi
	sta td_s_prev_hi
	; next
	iny
	iny
	cpy #$80
	bne @sample_loop
	; sample-loop exit marker
	lda #$73 ; 's'
	jsr dbg_putc
	; final re/im and energy
	; re = s_prev - (coeff_q14 * s_prev2)>>14
	lda coeff_q14_lo,x
	sta td_coeff_lo
	lda coeff_q14_hi,x
	sta td_coeff_hi
	lda td_tmp32_0
	sta td_tmp_lo
	lda td_tmp32_1
	sta td_tmp_hi
	jsr td_mul_q14_shift14
	; td_tmp = (coeff*s_prev2)>>14
	; re = s_prev - td_tmp
	sec
	lda td_s_prev_lo
	sbc td_tmp_lo
	sta td_prod_lo    ; reuse as re_lo
	lda td_s_prev_hi
	sbc td_tmp_hi
	sta td_prod_hi    ; re_hi
	; im = (sin_q14 * s_prev2)>>14
	lda sin_q14_lo,x
	sta td_coeff_lo
	lda sin_q14_hi,x
	sta td_coeff_hi
	lda td_tmp32_0
	sta td_tmp_lo
	lda td_tmp32_1
	sta td_tmp_hi
	jsr td_mul_q14_shift14
	; td_tmp = im
	; E = re^2 + im^2 (truncate to 16-bit)
	; re^2
	lda td_prod_lo
	sta td_a
	lda td_prod_lo
	sta td_b
	jsr mul8_signed
	lda td_prod_hi
	sta td_power_hi    ; reuse as temp
	lda td_prod_lo
	sta td_power_lo
	; im^2 add
	lda td_tmp_lo
	sta td_a
	lda td_tmp_lo
	sta td_b
	jsr mul8_signed
	clc
	lda td_power_lo
	adc td_prod_lo
	sta td_tmp_lo
	lda td_power_hi
	adc td_prod_hi
	sta td_tmp_hi
	; select max
	jsr td_select_max
	inx
	cpx #$08
	beq :+
	jmp @bin_loop
	:
	; print summary b:<bin> p:<power> newline
	lda #$62
	jsr dbg_putc
	lda #$3A
	jsr dbg_putc
	lda td_max_bin
	jsr printsafebyte
	lda #$20
	jsr dbg_putc
	lda #$70
	jsr dbg_putc
	lda #$3A
	jsr dbg_putc
	lda td_power_hi
	jsr printsafebyte
	lda td_power_lo
	jsr printsafebyte
	lda #$0A
	jsr dbg_putc
	; exit marker
	lda #$45
	jsr dbg_putc
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

; Print A as two hex characters while preserving X and Y
.proc print_byte_preserve
	; save X and Y into zero-page temps
	txa
	sta td_tmp       ; save X
	tya
	sta td_saved_y   ; save Y
	jsr printsafebyte
	lda td_tmp
	tax
	lda td_saved_y
	tay
	rts
.endproc

.proc td_debug_print_bin
	lda #'<'
	jsr dbg_putc
	rts
.endproc

.proc td_select_max
	; Compare td_tmp (E) with td_power, update max and index
	lda td_tmp_hi
	cmp td_power_hi
	bcc @done
	bne @set
	lda td_tmp_lo
	cmp td_power_lo
	bcc @done
@set:
	lda td_tmp_lo
	sta td_power_lo
	lda td_tmp_hi
	sta td_power_hi
	txa
	sta td_max_bin
@done:
	rts
.endproc
