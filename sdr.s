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

	; Tone detection stub callable after capture
	jsr detect_tone

rts

; ---------------------------------------------
; ramout
; Iterate pages and banks, dump bytes via serial.
; Mirrors the traversal used in readadc.
; ---------------------------------------------
ramout:
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
bne ramout ; BRA
ramoutdone:
rts

; ---------------------------------------------
; Tone detection stub (kept in ROM; no segment changes)
; ---------------------------------------------
.segment "WORKSPACE"
; Zero-page workspace for tone detection (fits within $50-$7F window)
td_s_prev_lo: .res 1
td_s_prev_hi: .res 1
td_s_prev2_lo: .res 1
td_s_prev2_hi: .res 1
td_power_lo: .res 1
td_power_hi: .res 1
td_coeff_lo: .res 1
td_coeff_hi: .res 1
td_tmp_lo: .res 1
td_tmp_hi: .res 1
td_sample: .res 1
td_max_bin: .res 1
td_prod_lo: .res 1
td_prod_hi: .res 1
td_a:       .res 1
td_b:       .res 1

.segment "RODATA"
; Coefficients: 2*cos(2*pi*k/N) in Q1.7 for N=64, k=0..7
coeff_lo:
	.byte $00,$FB,$EC,$D5,$B5,$8E,$62,$32
coeff_hi:
	.byte $02,$01,$01,$01,$01,$01,$00,$00

; Sine coefficients sin(2*pi*k/64) in Q1.7 for k=0..7
sin_lo:
	.byte $00,$0C,$27,$49,$6E,$8F,$B3,$D0
sin_hi:
	.byte $00,$00,$00,$00,$00,$00,$00,$00

; General purpose 8x8 -> 16 multiply: td_a * td_b = td_prod_hi:td_prod_lo
.proc mul8
	lda #$00
	sta td_prod_lo
	sta td_prod_hi
	lda td_b
	beq mul8_done
	tax                ; X = td_b (unsigned count)
mul8_loop:
	clc
	lda td_prod_lo
	adc td_a           ; add multiplicand each iteration
	sta td_prod_lo
	bcc :+
	inc td_prod_hi
:
	dex
	bne mul8_loop
mul8_done:
	rts
.endproc

; Goertzel detector: analyzes first 128 I-channel samples, 8 bins, reports max bin index
.proc detect_tone
	; init ptr to $0800 page
	lda #$08
	sta ptr+1
	lda #$00
	sta ptr          ; ensure low byte = 0 (start of page)
	lda #$00
	sta td_max_bin
	lda #$00
	sta td_power_lo
	sta td_power_hi
	ldx #$00 ; bin index 0..7
bin_loop:
	; reset states
	lda #$00
	sta td_s_prev_lo
	sta td_s_prev_hi
	sta td_s_prev2_lo
	sta td_s_prev2_hi
	; load coeff
	lda coeff_lo,x
	sta td_coeff_lo
	lda coeff_hi,x
	sta td_coeff_hi
	; sample loop over 64
	ldy #$00
sample_loop:
	lda (ptr),y      ; I sample
	sec
	sbc #$80         ; center to signed
	sta td_sample
	; tmp = (coeff * s_prev) >> 7  (signed arithmetic)
	; low*low
	lda td_coeff_lo
	sta td_a
	lda td_s_prev_lo
	sta td_b
	jsr mul8
	; td_prod_lo/hi holds coeff_lo * prev_lo
	; add cross term contributions into high byte: (coeff_hi*prev_lo + coeff_lo*prev_hi) << 8
	lda td_coeff_hi
	beq no_ch1
	sta td_a
	lda td_s_prev_lo
	sta td_b
	jsr mul8            ; td_prod_hi:lo = coeff_hi * prev_lo
	lda td_prod_lo
	clc
	adc td_prod_hi      ; accumulate into high with simple add
	sta td_prod_hi
no_ch1:
	lda td_coeff_lo
	beq no_ch2
	sta td_a
	lda td_s_prev_hi
	sta td_b
	jsr mul8            ; td_prod_hi:lo = coeff_lo * prev_hi
	lda td_prod_lo
	clc
	adc td_prod_hi
	sta td_prod_hi
no_ch2:
	; arithmetic shift >>7 keeping sign from s_prev
	lda td_s_prev_hi
	and #$80
	sta td_tmp_hi       ; sign mask
	lda td_prod_hi
	lsr
	lsr
	lsr
	ora td_tmp_hi
	sta td_tmp_hi
	lda td_prod_lo
	ror td_tmp_hi
	lsr
	lsr
	lsr
	sta td_tmp_lo
	; s = sample + tmp - s_prev2
	lda td_sample
	clc
	adc td_tmp_lo
	sec
	sbc td_s_prev2_lo
	sta td_prod_lo   ; reuse as s_lo
	lda #$00
	adc td_tmp_hi
	sbc td_s_prev2_hi
	sta td_prod_hi   ; reuse as s_hi
	; rotate states
	lda td_s_prev_lo
	sta td_s_prev2_lo
	lda td_s_prev_hi
	sta td_s_prev2_hi
	lda td_prod_lo
	sta td_s_prev_lo
	lda td_prod_hi
	sta td_s_prev_hi
	; next sample
	iny
	cpy #$40
	bne sample_loop_tr
	jmp sample_done
sample_loop_tr:
	jmp sample_loop
sample_done:
	; power ≈ s_prev^2 + s_prev2^2 - coeff*s_prev*s_prev2
	; s_prev_lo * s_prev_lo
	lda td_s_prev_lo
	sta td_a
	lda td_s_prev_lo
	sta td_b
	jsr mul8
	lda td_prod_lo
	sta td_tmp_lo
	lda td_prod_hi
	sta td_tmp_hi
	; add s_prev2_lo^2
	lda td_s_prev2_lo
	sta td_a
	lda td_s_prev2_lo
	sta td_b
	jsr mul8
	clc
	lda td_tmp_lo
	adc td_prod_lo
	sta td_tmp_lo
	lda td_tmp_hi
	adc td_prod_hi
	sta td_tmp_hi
	; subtract coeff*s_prev_lo*s_prev2_lo (approx)
	lda td_s_prev_lo
	sta td_a
	lda td_s_prev2_lo
	sta td_b
	jsr mul8
	; multiply by coeff_lo (approx scale)
	lda td_coeff_lo
	sta td_a
	lda td_prod_lo
	sta td_b
	jsr mul8
	; tmp = tmp - prod
	sec
	lda td_tmp_lo
	sbc td_prod_lo
	sta td_prod_lo
	lda td_tmp_hi
	sbc td_prod_hi
	sta td_prod_hi
	; compare with max power
	lda td_prod_hi
	cmp td_power_hi
	bcc next_bin
	bne set_new
	lda td_prod_lo
	cmp td_power_lo
	bcc next_bin
set_new:
	lda td_prod_lo
	sta td_power_lo
	lda td_prod_hi
	sta td_power_hi
	txa
	sta td_max_bin
next_bin:
	inx
	cpx #$08
	beq bins_done
	jmp bin_loop
bins_done:
	; Determine sign: sign(Im) ~ sign(s_prev2) for k=1..7 (sin>0)
	lda td_s_prev2_hi
	bmi negfreq
	lda #'+'
	bne printsign
negfreq:
	lda #'-'
printsign:
	jsr serial_tx
	; then print bin index
	lda td_max_bin
	jsr serial_tx
	lda #$0A
	jsr serial_tx
	rts
.endproc

