#!/usr/bin/env bash
set -euo pipefail

# Controls:
#   SERIAL=1     → build and send/burn over serial
#   SERIAL=0     → build only (default)
#   DRY=1        → build only, do not send (overrides SERIAL)
#   QUIET=1      → reduce log output

SERIAL=${SERIAL:-0}
DRY=${DRY:-0}
QUIET=${QUIET:-0}

log() {
	if [[ "$QUIET" != "1" ]]; then
		echo "$@"
	fi
}

log "SERIAL=${SERIAL} DRY=${DRY} QUIET=${QUIET}"

# Ensure build directory exists
mkdir -p build

# Assemble ROM
log "Assembling..."
ca65 -o build/abn6507rom.o abn6507rom.s
ld65 -o build/abn6507rom.bin -C memmap.cfg build/abn6507rom.o
log "ROM built: build/abn6507rom.bin"

# Skip sending if DRY=1
if [[ "$DRY" == "1" ]]; then
	log "Dry run: skipping serial send/burn."
	exit 0
fi

#!/bin/bash
/opt/homebrew/bin/ca65 -vvv --cpu 6502 -l  build/listing.txt -o  build/abn6507rom.o abn6507rom.s
/opt/homebrew/bin/ld65 -o build/abn6507rom.bin -C memmap.cfg "./build/abn6507rom.o" #"./build/crom.o" "./build/userland.o"
#/opt/homebrew/bin/minipro -s -p "W27C512@DIP28" -w  build/abn6507rom.bin
# Respect existing SERIAL if set externally; default to 1
if [[ -z "${SERIAL}" ]]; then
	SERIAL=1 # 0 to program ROM using firestarter; 1 to send userland via serial
fi
echo "SERIAL=${SERIAL}"
baudrate=4800
CHIP=w27c512

if [[ $SERIAL -eq 0 ]]; then
firestarter write $CHIP build/abn6507rom.bin
#minipro -s -p "$CHIP" -w build/abn6507rom.bin #Note the ROM model in this line (-s ignores ROM size differs from file)
#bash Programmer/burnromwithReUnROMP.sh #Uncomment this to try to burn ROM with "Relatively Universal ROM Programmer" 6502 PoC code
else
serial="/dev/cu.usbserial-110*" #macos
#serial="/dev/cu.usbserial-11230"
eval serial=$serial
bytes=$(stat -c %s build/userland.bin 2>/dev/null || stat -f %z build/userland.bin)
xxd build/userland.bin #Hex printout of the code
echo "Uploading $bytes bytes..." #Should be <100 bytes or a stack overflow is imminent
python send_userland.py $serial $baudrate build/userland.bin #Script requires python and pyserial module: pip3 install pyserial
fi