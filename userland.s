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

/*
lda DRA ; Clear stale ADC data
ora #$82
sta DRA  ; High clock
and #$7f
sta DRA  ; 
ora #$80
sta DRA  ; High clock
and #$7f
sta DRA  ; 
ora #$80
sta DRA  ; High clock
and #$7f
sta DRA  ;
*/
jsr readadcstart

;jsr ramout ; Assumes bank is 0 and serial tx is high 

lda #'U'
jsr serial_tx
jsr detect_tone

; Invoke the test once, then loop.

lda #'E'
jsr serial_tx
lda #$0A
jsr serial_tx

jmp userland

; Test mul8: multiply 6 * 7 and print 16-bit result via serial in hex.
; Uses ROM routines: serial_tx, serialbyte, bytetoa, hextoa
/*
.proc start_mul8_test
	lda #10
	sta td_a    ; multiplicand for ROM mul8
	lda #30
	sta td_b    ; multiplier for ROM mul8
	jsr mul8    ; result in td_prod_lo/td_prod_hi
	lda td_prod_hi
	jsr serialbyte
	lda td_prod_lo
	jsr serialbyte
	lda #$0A    ; newline
	jsr serial_tx
	rts
.endproc
*/
; Simple 8x8->16 multiply: A=multiplicand, X=multiplier -> A=prod_lo, Y=prod_hi
; Using mul8 from sdr.s
