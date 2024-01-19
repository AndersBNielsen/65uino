#!/bin/bash
/opt/homebrew/bin/ca65 -vvv --cpu 6502 -l  build/listing.txt -o  build/abn6507rom.o abn6507rom.s
/opt/homebrew/bin/ld65 -o build/abn6507rom.bin -C memmap.cfg "./build/abn6507rom.o" #"./build/crom.o" "./build/userland.o"
/opt/homebrew/bin/minipro -s -p "W27C512@DIP28" -w  build/abn6507rom.bin
SERIAL=1
baudrate=9600
if [[ $SERIAL -eq 0 ]]; then
minipro -s -p "SST39SF010A" -w build/abn6507rom.bin
else
serial="/dev/cu.usbs*" #macos
eval serial=$serial
cat -v < $serial & #Keep serial alive
pid=$! #Save for laterdd
stty -f $serial $baudrate #This can be buggy on macOS - if it doesn't work then this and the cat command above in a separate terminal window
echo -n $'\x01' > $serial #Send SOH to get 65uino ready to receive
sleep 1 #Wait for timeout
stty -f $serial $baudrate && cat build/userland.bin > $serial
kill $pid
wait $pid 2>/dev/null #silence!
fi
