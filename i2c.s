; Written by Anders Nielsen, 2023-2025
; License: https://creativecommons.org/licenses/by-nc/4.0/legalcode

SCL     = 1 ; DRB0 bitmask
SCL_INV = $FE ; Inverted for easy clear bit
SDA     = 2 ; DRB1 bitmask
SDA_INV = $FD

; i2c routines for the 65uino

i2c_start:
  lda I2CADDR
  rol ; Shift in carry
  sta outb ; Save addr + r/w bit

  lda #SCL_INV
  and DDRB
  sta DDRB ; Start with SCL as input HIGH - that way we can inc/dec from here

  lda #SDA ; Ensure SDA is output low before SCL is LOW
  ora DDRB
  sta DDRB
  lda #SDA_INV
  and DRB
  sta DRB

  lda #SCL_INV ; Ensure SCL is low when it turns to output
  and DRB
  sta DRB
  inc DDRB ; Set to output by incrementing the direction register == OUT, LOW

  ; Fall through to send address + RW bit
  ; After a start condition we always send the address byte so we don't need to RTS+JSR again here

i2cbyteout: ; Clears outb
  lda #SDA_INV ; In case this is a data byte we set SDA LOW
  and DRB
  sta DRB
  ldx #8
  bne first ; BRA - skip INC since first time already out, low
I2Cbyteloop:
  inc DDRB ; SCL out, low
first:
  asl outb ; MSB to carry
  bcc seti2cbit0 ; If bit was low
  lda DDRB       ; else set it high
  and #SDA_INV
  sta DDRB
  bcs wasone ; BRA doesn't exist on 6507
seti2cbit0:
  lda DDRB
  ora #SDA
  sta DDRB
  wasone:
  dec DDRB
  dex
  bne I2Cbyteloop

  inc DDRB

  lda DDRB ; Set SDA to INPUT (HIGH)
  and #SDA_INV
  sta DDRB

  dec DDRB ; Clock high
  lda DRB  ; Check ACK bit
  sec
  and #SDA
  bne nack
  clc ; Clear carry on ACK
  nack:
  inc DDRB ; SCL low
  rts

i2cbytein:
  ; Assume SCL is low from address byte
  lda DDRB  ; SDA, input
  and #SDA_INV
  sta DDRB
  lda #0
  sta inb
  ldx #8
i2cbyteinloop:
  clc
  dec DDRB ; SCL HIGH
  lda DRB ; Let's read after SCL goes high
  and #SDA
  beq got0
  sec
  got0:
  rol inb ; Shift bit into the input byte
  inc DDRB ; SCL LOW
  dex
  bne i2cbyteinloop

  lda tflags  ; Check the ACK/NACK flag in tflags
  and #$08    ; Mask to isolate the ACK/NACK bit
  beq send_ack  ; If the bit is 0, send ACK

  ; Send NACK == SDA high
  lda DDRB
  and #SDA_INV
  sta DDRB
  bne finish_bit

send_ack:
  ; Send ACK == SDA low
  lda DDRB
  ora #SDA
  sta DDRB

finish_bit:
  dec DDRB  ; SCL HIGH
  inc DDRB  ; SCL LOW
  rts

i2c_stop:
  lda DDRB ; SDA low
  ora #SDA
  sta DDRB
  dec DDRB ; SCL HIGH
  lda DDRB ; Set SDA high after SCL == Stop condition
  and #SDA_INV
  sta DDRB
  rts

i2c_scan:
    lda #$08  ; Start scanning from address 0x08
    sta scan_addr

i2c_scan_loop:
    clc  ; Clear carry to indicate WRITE (RW = 0)
    lda scan_addr
    sta I2CADDR  ; Set the I2C address to test
    jsr i2c_start  ; Send start condition with address
    bcs no_ack  ; If carry is set, no ACK received, skip

    lda #$3C  ; Load the print address
    sta I2CADDR  ; Set I2CADDR to 0x3C before printing
    lda scan_addr  ; Load detected address
    jsr printbyte  ; Print found device
    lda #' '
    jsr ssd1306_sendchar

no_ack:
    inc scan_addr  ; Move to the next address
    lda scan_addr
    cmp #$78  ; Stop at 0x77
    bne i2c_scan_loop
    rts

; Input: A = register address to read
; I2CADDR must be set
i2c_read_reg:
    pha              ; Save register address
    ; === Send register address (write) ===
    clc
    jsr i2c_start

    pla 
    sta outb             ; Store register address in outb
    jsr i2cbyteout
    jsr i2c_stop

    ; === Read from register ===
    sec
    jsr i2c_start

    lda tflags
    ora #$08       ; Set bit 3 = send NACK
    sta tflags
    jsr i2cbytein         ; Result in 'inb'

    jsr i2c_stop
    rts

;Assumes I2CADDR is set to the device address
;Assumes reg is in A, val in Y
i2c_write_register:
    pha               ; Save register address
    clc
    jsr i2c_start     ; send start + address

    pla               ; Restore register address
    sta outb
    jsr i2cbyteout    ; send register address

    tya               ; value to write
    sta outb
    jsr i2cbyteout    ; send value

    jsr i2c_stop      ; finish transmission
    rts

