#!/bin/zsh
ca65 -vvv --cpu 6502 -l  build/listing.txt -o  build/abn6507rom.o abn6507rom.s
ld65 -o build/abn6507rom.bin -C memmap.cfg "./build/abn6507rom.o" #"./build/crom.o" "./build/userland.o"
minipro -s -p "W27C512@DIP28" -w  build/abn6507rom.bin
