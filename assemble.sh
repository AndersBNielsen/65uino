#!/bin/bash
/opt/homebrew/bin/ca65 -vvv --cpu 6502 -l  build/listing.txt -o  build/abn6507rom.o abn6507rom.s
/opt/homebrew/bin/ld65 -o build/abn6507rom.bin -C memmap.cfg "./build/abn6507rom.o" #"./build/crom.o" "./build/userland.o"
#/opt/homebrew/bin/minipro -s -p "W27C512@DIP28" -w  build/abn6507rom.bin
SERIAL=0
baudrate=4800
if [[ $SERIAL -eq 0 ]]; then
minipro -s -p "SST39SF010A" -w build/abn6507rom.bin
else
serial="/dev/cu.usbs*" #macos
eval serial=$serial
cat -v < $serial & #Keep serial alive
pid=$! #Save for later
stty -f $serial $baudrate
echo -n $'\x01' > $serial #Send SOH to get 65uino ready to receive
sleep 0.1 #Wait for timeout
cat build/userland.bin > $serial
kill $pid #Terminate serial
wait $pid 2>/dev/null #silence!
fi
