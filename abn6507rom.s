.feature string_escapes ; Allow c-style string escapes when using ca65
.feature org_per_seg
.feature c_comments

RIOT = $80

DRA     = RIOT + $00 ;DRA ('A' side data register)
DDRA    = RIOT + $01 ;DDRA ('A' side data direction register)
DRB     = RIOT + $02 ;DRB ('B' side data register)
DDRB    = RIOT + $03 ;('B' side data direction register)
READTDI = RIOT + $04 ;Read timer (disable interrupt)

WEDGC   = RIOT + $04 ;Write edge-detect control (negative edge-detect,disable interrupt)
RRIFR   = RIOT + $05 ;Read interrupt flag register (bit 7 = timer, bit 6 PA7 edge-detect) Clear PA7 flag
A7PEDI  = RIOT + $05 ;Write edge-detect control (positive edge-detect,disable interrupt)
A7NEEI  = RIOT + $06 ;Write edge-detect control (negative edge-detect, enable interrupt)
A7PEEI  = RIOT + $07 ;Write edge-detect control (positive edge-detect enable interrupt)

READTEI = RIOT + $0C ;Read timer (enable interrupt)
WTD1DI  = RIOT + $14 ; Write timer (divide by 1, disable interrupt)
WTD8DI  = RIOT + $15 ;Write timer (divide by 8, disable interrupt)
WTD64DI = RIOT + $16 ;Write timer (divide by 64, disable interrupt)
WTD1KDI = RIOT + $17 ;Write timer (divide by 1024, disable interrupt)

WTD1EI  = RIOT + $1C ;Write timer (divide by 1, enable interrupt)
WTD8EI  = RIOT + $1D ;Write timer (divide by 8, enable interrupt)
WTD64EI = RIOT + $1E ;Write timer (divide by 64, enable interrupt)
WTD1KEI = RIOT + $1F ;Write timer (divide by 1024, enable interrupt)

.segment "RODATA"
.org $F000 ; Not strictly needed with CA65 but shows correct address in listing.txt
  nmi:
  irq:
  reset:
          cld ; Because you never know
          sei ; Probably not needed with the 6507
          ;Set stack pointer and clear RAM
          ldx #$7f
          txs
          lda #0
  clearzp:
          sta $00,x
          dex
          bne clearzp
          sta $00,x

          lda #$FF
          sta DDRB ; Set whole B register to output
          ; Reset state of DDRA is $00. 
main:
inc DRB
lda #244
bit DRA
bpl quartersecond
sta WTD64DI ; 244*64 = 15616 ~= 16ms
bne wait ; BRA
quartersecond:
sta WTD1KDI ; 244 * 1024 = 249856 ~= quarter second

wait:
lda READTDI
bne wait ; Loop until timer runs out

jmp main ; loop

.segment "VECTORS6502"
.ORG $fffa
.word nmi,reset,irq
.reloc
