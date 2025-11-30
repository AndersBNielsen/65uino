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
ora #$80
sta DRA  ; High clock
and #$7f
sta DRA  ; 
ora #$80
sta DRA  ; High clock
and #$7f
sta DRA  ;

jsr readadcstart
jsr ramout ; Assumes bank is 0 and serial tx is high 

jmp userland ; BRA