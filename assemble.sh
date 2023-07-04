#!/bin/zsh
/opt/homebrew/bin/ca65 -vvv --cpu 6502 -l  build/listing.txt -o  build/abn6507rom.o abn6507rom.s
/opt/homebrew/bin/ld65 -o build/abn6507rom.bin -C memmap.cfg "./build/abn6507rom.o" #"./build/crom.o" "./build/userland.o"
#minipro -s -p "W27C512@DIP28" -w  build/abn6507rom.bin
/opt/homebrew/bin/minipro -s -p "SST39SF010A" -w build/abn6507rom.bin 
