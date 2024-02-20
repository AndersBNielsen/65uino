; Written by Anders Nielsen, 2023-2024
; License: https://creativecommons.org/licenses/by-nc/4.0/legalcode

.macro print stringlabel
    ldy #<stringlabel
    ldx #>stringlabel
    jsr ssd1306_prints
.endmacro
