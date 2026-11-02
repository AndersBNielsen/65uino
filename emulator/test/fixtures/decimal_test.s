; Verify decimal mode behavior
; Written by Bruce Clark.  This code is public domain.
; see http://www.6502.org/tutorials/decimal_mode.html
;
; Ported to ca65 syntax for the 65uino emulator test suite, taken from the
; copy distributed with Klaus Dormann's 6502_65C02_functional_tests
; (6502_decimal_test.a65). Configuration: NMOS 6502 (cputype = 0), invalid
; BCD operands allowed, and ALL flags (N, V, Z, C) plus the accumulator are
; checked. The test ends in "jmp *" at DONE; ERROR ($0B) = 0 means pass.

.setcpu "6502"
.segment "ZP"
.org 0
N1:     .res 1
N2:     .res 1
HA:     .res 1
HNVZC:  .res 1
DA:     .res 1
DNVZC:  .res 1
AR:     .res 1
NF:     .res 1
VF:     .res 1
ZF:     .res 1
CF:     .res 1
ERROR:  .res 1
N1L:    .res 1
N1H:    .res 1
N2L:    .res 1
N2H:    .res 2

.segment "CODE"
.org $0200
TEST:   ldy #1
        sty ERROR
        lda #0
        sta N1
        sta N2
LOOP1:  lda N2
        and #$0F
        sta N2L
        lda N2
        and #$F0
        sta N2H
        ora #$0F
        sta N2H+1
LOOP2:  lda N1
        and #$0F
        sta N1L
        lda N1
        and #$F0
        sta N1H
        jsr ADD
        jsr A6502
        jsr COMPARE
        bne DONE
        jsr SUB
        jsr S6502
        jsr COMPARE
        bne DONE
NEXT1:  inc N1
        bne LOOP2
NEXT2:  inc N2
        bne LOOP1
        dey
        bpl LOOP1
        lda #0
        sta ERROR
DONE:   jmp DONE

ADD:    sed
        cpy #1
        lda N1
        adc N2
        sta DA
        php
        pla
        sta DNVZC
        cld
        cpy #1
        lda N1
        adc N2
        sta HA
        php
        pla
        sta HNVZC
        cpy #1
        lda N1L
        adc N2L
        cmp #$0A
        ldx #0
        bcc A1
        inx
        adc #5
        and #$0F
        sec
A1:     ora N1H
        adc N2H,x
        php
        bcs A2
        cmp #$A0
        bcc A3
A2:     adc #$5F
        sec
A3:     sta AR
        php
        pla
        sta CF
        pla
        sta VF
        rts

SUB:    sed
        cpy #1
        lda N1
        sbc N2
        sta DA
        php
        pla
        sta DNVZC
        cld
        cpy #1
        lda N1
        sbc N2
        sta HA
        php
        pla
        sta HNVZC
        rts

SUB1:   cpy #1
        lda N1L
        sbc N2L
        ldx #0
        bcs S11
        inx
        sbc #5
        and #$0F
        clc
S11:    ora N1H
        sbc N2H,x
        bcs S12
        sbc #$5F
S12:    sta AR
        rts

COMPARE:
        lda DA
        cmp AR
        bne C1
        lda DNVZC
        eor NF
        and #$80
        bne C1
        lda DNVZC
        eor VF
        and #$40
        bne C1
        lda DNVZC
        eor ZF
        and #2
        bne C1
        lda DNVZC
        eor CF
        and #1
C1:     rts

A6502:  lda VF
        sta NF
        lda HNVZC
        sta ZF
        rts

S6502:  jsr SUB1
        lda HNVZC
        sta NF
        sta VF
        sta ZF
        sta CF
        rts
