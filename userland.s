;This is userland - it's where the ROM bootloader puts code received via serial
;It's very convenient to test out new code in userland before comitting it to ROM
;Use assemble.sh with SERIAL=1 to send userland to the 65uino.
;The more userland space you use, the less stack available. 100bytes should be ok. 
;Zero-page aliases should be defined in abn6507rom.s

.SEGMENT "USERLAND"
.org $0e ; Just to make listing.txt match
userland:


lda DDRA
ora #$80
sta DDRA ; Clock output


lda DRA ; Clear stale ADC data
ora #$82
sta DRA  ; High clock
and #$7f
sta DRA  ; 
;ora #$80
;sta DRA  ; High clock
;and #$7f
;sta DRA  ; 

jsr readadcstart

; print start marker

lda #'>'
jsr dbg_putc

jsr detect_tone_q14_16bit
lda td_max_bin

jsr printsafebyte
lda #$0A
jsr dbg_putc
;ldx #$5A

	;jsr printsafebyte
	;jsr td_init_bin_state
;	jsr td_sample_loop
	;jsr td_done_samples

;txa
;jsr printsafebyte

; print end marker
;jsr dbg_putc
; Halt here so the test runs only once and we can observe output


lda #$02
sta DRA            ; Turn off clock, Serial TX high
ldy #0
jsr ramout


; stop here after one run so serial output can be observed
hang:
	jmp hang

; Test mul8: multiply 6 * 7 and print 16-bit result via serial in hex.
; Uses ROM routines: serial_tx, serialbyte, bytetoa, hextoa
; (start_mul8_test removed to fit userland)
; Simple 8x8->16 multiply: A=multiplicand, X=multiplier -> A=prod_lo, Y=prod_hi
; Using mul8 from sdr.s
