; Written by Anders Nielsen, 2023-2024
; License: https://creativecommons.org/licenses/by-nc/4.0/legalcode

; Relatively Universal ROM Programmer 6502 Firmware

erasing:.asciiz "\nErasing...\n"
m27c512rom:.asciiz "M27C512"
w27c512rom:.asciiz "W27C512" 
foundwithid:.asciiz " found with ID: "
unrecognized:.asciiz "Unrecognized ROM"
blankcheckstr:.asciiz "Performing blank check... "
romblankstr: .asciiz "ROM blank. \n"
cloning: .asciiz "Cloning internal ROM.. "
verifying: .asciiz "Verifying.. \n"
romnotblankstr: .asciiz "First mismatch found at addr: $"
bytesverifiedstr: .asciiz " bytes verified OK"
pressbutton: .asciiz "Press USR button "
toexit: .asciiz "to exit calibration."
finished: .asciiz "Finished."

runprogrammer:
lda userland+1 ; Command
cmp #1
beq sendromtoserial
cmp #2
beq burnromfromserial
cmp #3 
beq eraserom
;cmp #4 
jmp calibrateVEP

stoppage = userland ; Steal a byte of RAM

sendromtoserial:
jsr ssd1306_clear
print verifying
lda #<userland+1  ;Executed from ROM we can use all of userland for buffer (faster)
sta fifobufferpnt
lda #>userland+1
sta fifobufferpnt+1
lda userland+2  ; Buffer length
sta buffer_length
lda userland+3  ; Addr LSB
sta romaddr
lda userland+4  ; MSB
sta romaddr+1
lda userland+5  ; Stoppage - stop reading ROM when this rolls over. 
sta stoppage

fetchromandsendserial:
jsr rom_fillbuf
ldy #$7f
jsr serial_wbuf
lda romaddr+1
cmp #$ff
bne notlastread
lda #0
sta stoppage
lda romaddr+1
notlastread:
cmp stoppage
bne fetchromandsendserial
    lda romaddr
    jsr latchlsb        ; Just to reflect the actual next address that would've been read
jmp finish

eraserom:
vendorid = userland+2 
serialid = userland+3
jsr ssd1306_clear
jsr idanderaserom
jmp finish

burnromfromserial:
;Enable VPE
lda #<userland+1  ;Executed from ROM we can use all of userland for buffer (faster)
sta fifobufferpnt
lda #>userland+1
sta fifobufferpnt+1
lda userland+2  ; Buffer length
sta buffer_length
lda userland+3
sta stoppage

lda #BITMASK_VPE_TO_VPP
jsr latchctrl
lda #BITMASK_REG_DISABLE | BITMASK_VPE_TO_VPP
jsr latchctrl
lda #$ff ; Max delay
jsr delay_long ; Should probably check with a scope how long it actually takes to charge the regulator output cap.
lda #0
sta romaddr
sta romaddr+1
lda #2 
sta DRA
sta DDRA
ldy #$7f
fetchandburnrom:
jsr serial_rbuf ; Takes a bit value for delay_short in Y
LDA #BITMASK_REG_DISABLE | BITMASK_VPE_TO_VPP | BITMASK_VPE_ENABLE ; This should be fixed to match the programming profile
JSR latchctrl  ; Latch the updated control signals
jsr rom_writebuf ; This needs to be fixed to allow for different profiles
LDA #BITMASK_REG_DISABLE | BITMASK_VPE_TO_VPP ; This should be fixed to match the programming profile
JSR latchctrl  ; Latch the updated control signals
lda romaddr+1
cmp #$ff
bne notlast
lda #0
sta stoppage
lda romaddr+1
notlast:
ldy #$7f
cmp stoppage
bne fetchandburnrom
lda #0 
jsr latchctrl
jsr delay_long
jsr serial_rbuf ; Let's tell sender we've finished with this
jmp finish

calibrateVEP:
jsr ssd1306_clear
print pressbutton
print toexit
lda #$0A
jsr ssd1306_sendchar
lda #BITMASK_REG_DISABLE
jsr latchctrl
waithere:
bit DRB
bvs waithere
finish:
print finished
lda #0
sta mode
jsr latchctrl
lda #2 ; Fix serial
sta DDRA
jmp main

;-----------------------------------------------------
; Function: serial_rbuf
; Description: Reads data from serial communication
;              and fills a buffer in RAM.
; Parameters: None
; Returns: None
;-----------------------------------------------------

serial_rbuf:
    lda #$02            ; Set bit 1 for serial TX (Output), clear bit 0 for serial RX (Input)
    sta DDRA            
    sta DRA ; Make sure 
    tya
    jsr delay_short     ; Delay long enough for a serial stop condition (high)
    lda #$AA            ; Better pattern than XON
    jsr serial_tx       ; Tell host we're ready to receive
    lda #244
    sta WTD1KDI ; 244 * 1024 = 249856 ~= quarter second
rxbufloop:
    lda READTDI         ; Timeout - hope we don't need to hang around longer
    beq rxbufdone   
    lda DRA             ; Read port register
    and #$01            ; Mask A to extract bit 0 (serial RX)
    bne rxbufloop       ; If bit is not clear, continue waiting - needs to be fixed with a timeout
    ldy #0              ; Clear y register
rxloop:
    lda DRA             ; Read port register
    and #$01            ; Mask to extract bit 0 (serial RX)
    bne rxloop       ; If bit is not clear, continue waiting
    jsr serial_rx       ; Read byte from serial receiver
    sta (fifobufferpnt),y   ; Store byte in buffer at index Y
    iny                 ; Increment index Y
    cpy buffer_length   ; Check if buffer is full
    bne rxloop          ; If not, continue reading
    rts
rxbufdone:
    lda #$1f 
    sta romaddr+1       ; Indicate we're done even if timeout
    rts                 ; Return from subroutine


;-----------------------------------------------------
; Function: rom_fillbuf
; Description: Reads data from an external ROM chip
;              and fills a buffer in RAM.
; Parameters: None
; Returns: None
;-----------------------------------------------------
rom_fillbuf:
    lda romaddr+1       ; Load high byte of ROM address into A
    jsr latchmsb        ; Call subroutine to set up ROM chip with MSB
    lda romaddr         ; Load low byte of ROM address into A
    jsr latchlsb       ; Call subroutine to set up ROM chip with LSB
    ldy #0              ; Clear Y register
fillbufloop:
    jsr readrom        ; Call subroutine to read byte from ROM returned in X
    txa
    sta (fifobufferpnt),y    ; Store byte in fifobuffer at index Y
    inc romaddr         ; Increment ROM address
    bne samepage        ; Branch if low byte did not overflow
    inc romaddr+1       ; Increment high byte if low byte overflowed
    lda romaddr+1       ; Load high byte of ROM address again    
    jsr latchmsb        ; Call subroutine to set up ROM chip with MSB again
samepage:
    iny                 ; Increment index Y
    cpy buffer_length   ; Check if all bytes have been read
    bne fillbufloop     ; If not, loop back to read next byte
    rts                 ; Return from subroutine

;-----------------------------------------------------
; Function: rom_writebuf
; Description: Reads data from buffer
;              and writes to ROM.
; Parameters: Assumes romaddr contains first ROM address
; Returns: None
;-----------------------------------------------------
rom_writebuf:          
    lda romaddr+1       ; Load high byte of ROM address into A
    jsr latchmsb        ; Call subroutine to set up ROM chip with MSB
    lda romaddr         ; Load low byte of ROM address into A
    jsr latchlsb       ; Call subroutine to set up ROM chip with LSB
    ldy #0              ; Clear Y register
writebufloop:
    LDA #BITMASK_REG_DISABLE | BITMASK_VPE_TO_VPP | BITMASK_VPE_ENABLE ; This should be fixed to match the programming profile
    sta DRA
    JSR latchctrl2  ; Latch the updated control signals
    lda (fifobufferpnt),y    ; Store byte in fifobuffer at index Y    
    ; Write ROM
    sta DRA
    jsr writerom2
    LDA #BITMASK_REG_DISABLE | BITMASK_VPE_TO_VPP ; This should be fixed to match the programming profile
    sta DRA
    jsr latchctrl2
    inc romaddr         ; Increment ROM address
    bne samepage2        ; Branch if low byte did not overflow
    inc romaddr+1       ; Increment high byte if low byte overflowed
    lda romaddr+1       ; Load high byte of ROM address again
    jsr latchmsb        ; Call subroutine to set up ROM chip with MSB again
samepage2:
    lda romaddr
    jsr latchlsb
    iny                 ; Increment index Y
    cpy buffer_length   ; Check if all bytes have been read
    bne writebufloop     ; If not, loop back to read next byte
    rts                 ; Return from subroutine

;-----------------------------------------------------

;-----------------------------------------------------
; Function: serial_wbuf
; Description: Sends data from a buffer serially.
; Parameters: None
; Returns: None
;-----------------------------------------------------
serial_wbuf:
    lda #$02            ; Set bit 1 for serial TX (Output)
    sta DDRA            ; Store to configure port direction
    sta DRA
    tya 
    jsr delay_short     ; Might need this to let the line go inactive long enough for receiver to sync start of byte
    lda #$AA            ; Start of frame
    jsr serial_tx    
    ldy #0              ; Clear Y register
txbufloop:
    lda (fifobufferpnt),y  ; Load byte from buffer at index Y
    jsr serial_tx       ; Send byte serially
    iny                 ; Increment index Y
    cpy buffer_length            ; Check if all bytes have been sent
    bne txbufloop       ; If not, loop back to send next byte
buftxd:
    rts                 ; Return from subroutine

;-----------------------------------------------
; Function: idanderaserom
; Description: ID's and erases an EEPROM
; Inputs: None
; Outputs: None
;-----------------------------------------------
idanderaserom:
    jsr identifyrom          ; Returns ROM ID in romaddr(+1)
    lda romaddr
    cmp vendorid            ; Check if it's a Winbond ROM
    bne badrom               ; Branch if not Winbond
    lda romaddr+1
    cmp serialid                   ; Check if it's a W27C512 ROM
    bne badrom               ; Branch if not W27C512
    ; If Winbond W27C512 detected, proceed with cloning
    print erasing           ; Print macro in macros.s
    jsr erasew27c512        ; First, erase the ROM
rts
badrom:
lda #$0A
jsr ssd1306_sendchar
print unrecognized
rts

;-----------------------------------------------
; Function: clonetow27c512
; Description: Cloning routine for W27C512 EEPROM
; Inputs: None
; Outputs: None
;-----------------------------------------------
clonetow27c512:
    lda #%01000000           ; Indicator cloning started
    jsr latchctrl

    jsr identifyrom          ; Returns ROM ID in romaddr(+1)
    lda romaddr
    cmp #$DA                ; Check if it's a Winbond ROM
    bne notwb               ; Branch if not Winbond
    lda romaddr+1
    cmp #8                   ; Check if it's a W27C512 ROM
    bne notwb               ; Branch if not W27C512

    ; If Winbond W27C512 detected, proceed with cloning

wipe:
    print erasing           ; Print macro in macros.s
    jsr erasew27c512        ; First, erase the ROM

;Second entry point
clonetow27c512nowipe:
    lda #0
    sta longdelay           ; Must be initialized to either 0 for 100us ROMs or number of ms for slow programming ROMs

    print cloning
    jsr cloneROM

    print verifying
    jsr clonecheck          ; Verify

    lda #%01000000
    jsr latchctrl

    lda #'$'
    jsr ssd1306_sendchar   ; Print "$" as the start of the address marker

    lda romaddr+1
    jsr printbyte           ; Print address of first mismatch ($1000 if OK)
    lda romaddr
    jsr printbyte
    print bytesverifiedstr

    rts

notwb:
    print unrecognized
    print foundwithid
    rts

;-----------------------------------------------
; Function: identifyrom
; Description: Identify ROM type
; Inputs: None
; Outputs: None
;-----------------------------------------------
identifyrom:
    jsr getromid
    stx romaddr             ; Store ROM ID temporarily
    sty romaddr+1

    cpx #$DA                ; Check if it's a Winbond ROM
    beq winbond
    cpx #$20                ; Check if it's an ST ROM
    beq st
    jmp fail                ; Jump to fail if unrecognized

winbond:
    cpy #$08
    bne fail                ; Jump to fail if not W27C512
    print w27c512rom        ; Print ROM type
    jmp nofail              ; Jump to nofail

st:
    cpy #$3D
    bne fail                ; Jump to fail if not M27C512
    print m27c512rom        ; Print ROM type
    jmp nofail              ; Jump to nofail

fail:
    print unrecognized
nofail:
    print foundwithid
    lda romaddr
    jsr printbyte           ; Print ROM ID
    lda romaddr+1
    jsr printbyte
    rts

;Returns manufacturer ID in X and device ID in Y
getromid:
lda #BITMASK_REG_DISABLE
jsr latchctrl
/*
lda romprofile
and #ID_A9_VPP_MSK
beq enablevppduringid
*/

lda #BITMASK_VPE_TO_VPP | BITMASK_REG_DISABLE | BITMASK_A9_VPP_ENABLE
jsr latchctrl
lda #$50
jsr delay_long ; Stabilize VPP
ldy #0
sty DRA
lda DRB
ora #BITMASK_RLSBLE | BITMASK_RMSBLE 
sta DRB
and #~BITMASK_RMSBLE & ~BITMASK_RLSBLE & ~BITMASK_ROM_CE & $FF ; &$FF to make inversions 8 bits
sta DRB
sty DDRA ; DRA input
and .lobyte(~BITMASK_ROM_OE) ; .lobyte() is an alternative to & $FF
sta DRB
ldx DRA
ora #BITMASK_ROM_OE
sta DRB
ldy #$ff
sty DDRA
ldy #$01
sty DRA
ora #BITMASK_RLSBLE
sta DRB
and #~BITMASK_RLSBLE &$FF
sta DRB
ldy #0
sty DDRA
and #~BITMASK_ROM_OE&$FF
sta DRB
ldy DRA
ora #BITMASK_ROM_OE | BITMASK_ROM_CE
sta DRB
lda #BITMASK_REG_DISABLE
jsr latchctrl
rts

clonecheck:
;Init
lda #0
sta romaddr
sta DRA
tay
tax

lda #$10 ; ROM is mapped at $1000
sta romaddr+1

lda #$FF
sta DDRA
LDA DRB
ORA #BITMASK_RMSBLE  | BITMASK_RLSBLE ; Latch 0 into address registers
STA DRB
AND #~BITMASK_RMSBLE & $ff& ~BITMASK_RLSBLE ; Clear LE and output
STA DRB

tya
sta DDRA ; A input
LDA DRB
AND #~BITMASK_ROM_CE&$FF & ~BITMASK_ROM_OE&$FF;Output ROM data
sta DRB
lda DRA
cmp (romaddr),y
bne clonecheckdone
clc ; Let's make sure

checknextaddress:
lda DRB
ora #BITMASK_ROM_OE
sta DRB
iny
bne nexta
inx
beq clonecheckdone
clc ; Wonder if this is needed
lda romaddr+1
adc #1
sta romaddr+1
cmp #$20
beq clonecheckdone
stx DRA
jsr latchmsb2 ; Could unwrap this but only a little bit faster
nexta:
sty DRA
LDA #$FF
sta DDRA
; Set RLSBLE (bit 2) to latch the lower byte
LDA DRB
ORA #BITMASK_RLSBLE
STA DRB
AND #~BITMASK_RLSBLE &$FF&$FF
STA DRB
lda #0
sta DDRA
LDA DRB
AND #~BITMASK_ROM_OE&$FF;Output ROM data
sta DRB
lda DRA
cmp (romaddr),y
beq checknextaddress ; Bytes match, continue testing
clonecheckdone:
sty romaddr
stx romaddr+1
ldx DRA ; Return byte found in X 
lda DRB
ora #BITMASK_ROM_OE | BITMASK_ROM_CE
sta DRB
rts

;This snippet checks if a ROM is blank
checkblank:
print verifying
jsr blankcheck ; Verify erasure
cpx #$ff ; Check byte value 
bne notblank
print romblankstr

notblank:
lda romaddr+1
jsr printbyte ; Print address of first mismatch ($1000 if OK)
lda romaddr
jsr printbyte
print bytesverifiedstr
rts

blankcheck:
;Init
lda #0
sta romaddr
sta romaddr+1
sta DRA
tay
tax
jsr latchctrl

lda #$FF
sta DDRA
LDA DRB
ORA #BITMASK_RMSBLE  | BITMASK_RLSBLE ; Latch 0 into address registers
STA DRB
AND #~BITMASK_RMSBLE & $ff& ~BITMASK_RLSBLE ; Clear LE and output
STA DRB

tya ; Clear A
sta DDRA ; A input
LDA DRB
AND #~BITMASK_ROM_CE&$FF & ~BITMASK_ROM_OE&$FF ;Output ROM data
sta DRB
lda DRA
eor #$ff
bne checkdone

nextaddress:
lda DRB
ora #BITMASK_ROM_OE
sta DRB
inx
bne next
iny
beq checkdone
sty DRA
jsr latchmsb2 ; Could unwrap this but only a little bit faster
next:
stx DRA
LDA #$FF
sta DDRA
; Set RLSBLE (bit 2) to latch the lower byte
LDA DRB
ORA #BITMASK_RLSBLE
STA DRB
AND #~BITMASK_RLSBLE &$FF
STA DRB
lda #0
sta DDRA
LDA DRB
AND #~BITMASK_ROM_OE&$FF;Output ROM data
sta DRB
lda DRA
eor #$ff
beq nextaddress
checkdone:
stx romaddr
sty romaddr+1
ldx DRA ; Return byte found in X 
lda DRB
ora #BITMASK_ROM_OE | BITMASK_ROM_CE
sta DRB
rts

cloneROM:
LDA #BITMASK_REG_DISABLE
JSR latchctrl  ; Latch the updated control signals
LDA #BITMASK_REG_DISABLE | BITMASK_VPE_ENABLE | BITMASK_VPE_TO_VPP
JSR latchctrl  ; Latch the updated control signals
lda #$ff 
sta DDRA
jsr delay_short ; ~2ms

clonerom2:
lda #0 ; Address latches to 0
sta romaddr ; Zero LSB 
sta DRA
tay
LDA DRB
ORA #BITMASK_RMSBLE 
STA DRB
AND #$ff & ~BITMASK_RMSBLE
STA DRB

lda #$10

nextpage:
sta romaddr+1
sec 
sbc #$10 ; ROM is mapped offset $1000
sta DRA
LDA DRB
ORA #BITMASK_RMSBLE 
STA DRB
AND #$ff & ~BITMASK_RMSBLE
STA DRB
pageloop:
sty DRA ; Latch next address
LDA DRB
ORA #BITMASK_RLSBLE
STA DRB
AND #~BITMASK_RLSBLE & $FF
STA DRB

lda (romaddr),Y
sta DRA
lda DRB
AND #~BITMASK_ROM_CE&$FF ; Programming pulse start
sta DRB
lda longdelay
beq defaultpulse
jsr delay_long
clc
bcc pulsed
defaultpulse:
lda #11
jsr delay_short
pulsed:
lda DRB
ORA #BITMASK_ROM_CE ; Programming pulse end
sta DRB
iny
bne pageloop
lda romaddr+1
adc #0 ; C is set
cmp #$20
bne nextpage

lda #0
JSR latchctrl  ; Latch the updated control signals
rts

writerom: ; Expects byte to burn in A, ROM address in $XY
; Returns: Nothing, but leaves voltages ON!(!)
pha 
LDA #BITMASK_REG_DISABLE
JSR latchctrl  ; Latch the updated control signals
LDA #BITMASK_REG_DISABLE | BITMASK_VPE_ENABLE | BITMASK_VPE_TO_VPP ; Probably want to make this part separate or dynamic. 
JSR latchctrl  ; Latch the updated control signals
lda #$ff
sta DDRA
jsr delay_long
pla

writeaddress:
;Expects byte to burn in A, ROM address in $XY, voltages set for burning
; Returns nothing
; Destroys X, Y, A and ctrl register.
pha
txa
jsr latchmsb
sty DRA
jsr latchlsb2
pla
sta DRA
;Falls through to writerom

writerom2: ; Expects address and voltages to be set and DDRA = $FF
lda DRB
AND #~BITMASK_ROM_CE&$FF
sta DRB

lda #11 ; Hardcoded to W27C512. Must fix. 
jsr delay_short

lda DRB
ORA #BITMASK_ROM_CE
sta DRB

 ; Leaves CTRL register with voltages on!
rts

erasew27c512: ; Assumes DDRA is $FF (output)
lda #0
sta DRA
lda DRB ; Make sure CE & OE not already LOW and clear address latches
ora #BITMASK_ROM_CE|BITMASK_ROM_OE | BITMASK_RLSBLE | BITMASK_RMSBLE 
sta DRB
and #~BITMASK_RLSBLE & ~BITMASK_RMSBLE & $ff; Latch 0 
sta DRB
; Set CTRL_REGISTER to enable the regulator and set VPE/14V on A9
LDA #BITMASK_REG_DISABLE
jsr latchctrl
lda #50 ; ~50ms
jsr delay_long ; Stabilize VPE
LDA #BITMASK_REG_DISABLE | BITMASK_VPE_ENABLE | BITMASK_A9_VPP_ENABLE
JSR latchctrl  ; Latch the updated control signals
lda #50; ~50ms
jsr delay_long ; Stabilize VPE

lda #$ff ; Set DRA to OUTPUT
sta DDRA
sta DRA

lda DRB
AND #~BITMASK_ROM_CE&$FF
sta DRB

lda #106
jsr delay_long

lda DRB
ORA #BITMASK_ROM_CE
sta DRB

lda #BITMASK_REG_DISABLE
jsr latchctrl ; Disable high voltage regulator outputs - leave voltage reg ON
rts

;Routines
latchctrl:
  STA DRA            ; Store the low byte in DRA
  ; Set DDRA to OUTPUT
  LDA #$FF
  sta DDRA
latchctrl2:
  ; Set CTRL_LE (bit 2) to latch the lower byte
  LDA DRB
  ORA #BITMASK_CTRL_LE
  STA DRB

  ; Clear CTRL_LE (bit 2) to release the latch
  ;LDA DRB ; Don't need to reload
  AND #~BITMASK_CTRL_LE &$FF
  STA DRB
rts

latchlsb:
  STA DRA            ; Store the low byte in DRA
  ; Set DDRA to OUTPUT
  LDA #$FF
  sta DDRA
  ; Set RLSBLE (bit 2) to latch the lower byte
latchlsb2:
  LDA DRB
  ORA #BITMASK_RLSBLE
  STA DRB

  ; Clear RLSBLE (bit 2) to release the latch
  ;LDA DRB ; Don't need to reload
  AND #~BITMASK_RLSBLE & $FF
  STA DRB
rts

latchmsb:
sta DRA
latchmsb2:
lda #$FF
sta DDRA
; Set RMSBLE (bit 3) to latch the higher byte
LDA DRB
ORA #BITMASK_RMSBLE 
STA DRB

; Clear RMSBLE (bit 3) to release the latch
;LDA DRB ; Don't need to reload
AND #$ff & ~BITMASK_RMSBLE
STA DRB
rts

readrom:
  ; Takes ROM address to read in romaddr(+1)
  ; Returns data in X
  lda romaddr+1
  sta DRA
  jsr latchmsb2
readrom2:  
  LDA romaddr        ; Load the low byte of the address
  sta DRA
  jsr latchlsb2
  ; Set DRA to input (all bits of DDRA = 0)
  LDX #$00
  STX DDRA

  ; Set ROM_CE (bit 7) and ROM_OE (bit 4) to enable the ROM chip
  ;LDA DRB ; Don't need to reload
  AND #~BITMASK_ROM_CE& $FF & ~BITMASK_ROM_OE&$FF
  STA DRB

  ; Load the byte from the ROM into the accumulator
  LDX DRA            ; Assuming the ROM is connected to DRA

  ; Set ROM_CE (bit 7) and ROM_OE (bit 4) to disable the ROM chip
  LDA DRB
  ORA #BITMASK_ROM_CE | BITMASK_ROM_OE
  STA DRB
rts

