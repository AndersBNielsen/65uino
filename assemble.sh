#!/bin/bash
/opt/homebrew/bin/ca65 -vvv --cpu 6502 -l  build/listing.txt -o  build/abn6507rom.o abn6507rom.s
/opt/homebrew/bin/ld65 -o build/abn6507rom.bin -C memmap.cfg "./build/abn6507rom.o" #"./build/crom.o" "./build/userland.o"
#/opt/homebrew/bin/minipro -s -p "W27C512@DIP28" -w  build/abn6507rom.bin
SERIAL=1
if [[ $SERIAL -eq 0 ]]; then
minipro -s -p "SST39SF010A" -w build/abn6507rom.bin
else
serial="/dev/cu.usbs*" #macos
cat < $serial & echo -n $'\x01' > $serial
pid=$!
sleep 0.5
kill $pid
wait $pid 2>/dev/null #silence!
cat < $serial & cat build/userland.bin > $serial
pid=$!
sleep 0.5
kill $pid
wait $pid 2>/dev/null #silence!
fi
