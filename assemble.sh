#!/bin/bash
/opt/homebrew/bin/ca65 -vvv --cpu 6502 -l  build/listing.txt -o  build/abn6507rom.o abn6507rom.s
/opt/homebrew/bin/ld65 -o build/abn6507rom.bin -C memmap.cfg "./build/abn6507rom.o" #"./build/crom.o" "./build/userland.o"
#/opt/homebrew/bin/minipro -s -p "W27C512@DIP28" -w  build/abn6507rom.bin
SERIAL=0 #0 to program ROM using minipro - 1 to send userland code via serial. Use send_binary to upload ROM to programmer via serial
baudrate=9600
if [[ $SERIAL -eq 0 ]]; then
minipro -s -p "SST39SF010A" -w build/abn6507rom.bin #Note the ROM model in this line (-s ignores ROM size differs from file)
else
serial="/dev/cu.usb*" #macos
eval serial=$serial
bytes=$(stat -c %s build/userland.bin 2>/dev/null || stat -f %z build/userland.bin)
xxd build/userland.bin
echo "Uploading $bytes bytes..."
python3 send_userland.py $serial $baudrate build/userland.bin #Script requires python and pyserial module: pip3 install pyserial
fi
