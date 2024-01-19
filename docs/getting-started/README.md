# Getting Started with 65uino

<!-- TOC start  -->

- [Welcome to 65uino! ](#welcome-to-65uino)
- [Building the Board](#building-the-board)
- [The Software](#the-software)
   * [Devlelopment Toolchain](#devlelopment-toolchain)
   * [The 65uino BIOS](#the-65uino-bios)
   * [Build your ROMs](#build-your-roms)
- [65uino Architecture](#65uino-architecture)
   * [65uino Memory Map](#65uino-memory-map)
- [Writing your code](#writing-your-code)

<!-- TOC end -->

<!-- TOC --><a name="welcome-to-65uino"></a>
## Welcome to 65uino!

Before you start getting wrapped into writing 6502 assembly to conquer the world, you'll need to get soldering, set up the board and burn some software in the EEPROM.

If you’re new to the 65uino, you might want to [check out the videos](https://www.youtube.com/playlist?list=PL9Njj9WL8poFsM4C6Gi8V5FRoidOOL0Fv) first where you will see how the 65uino came to be, and how it works. You'll find more details in the [HackaDay Project page](https://hackaday.io/project/190260-65uino)

![65uino](https://cdn.hackaday.io/images/8644651680943366680.jpeg)

## Building the Board

This part assumes you already have the PCB, for ordering instrucctions check the "Getting the PCB" section in the [README](https://github.com/AndersBNielsen/65uino/blob/main/README.md) in the root of this repo.

You will need the 4 core ICs: CPU, RAM/IO and ROM. Be green, get em while they're used!

- 6507 Microprocessor Yes, the one in the Atari 2600!!
- 6532 RIOT
- 24 or 28 pin ROM W27C512 or 28C16 will do
- 74HC04 Inverter IC (DIP-14) or a SN74LVC1G14 single gate inverter (SOT23-5)

I recommend you use sockets for the ICs, you'll need: 1 x 40-pin socket, 2 x 28-pin sockets. And the Pin Headers:

- 2 x PinHeader Female Socket 2.54mm / 8 pins
- 1 x PinHeader Female Socket 2.54mm / 6 pins
- 1 x PinHeader Female Socket 2.54mm / 10 pins
- 1 x PinHeader Female Socket 2.54mm / 10 pins

If you did not to get the populated board, the full [BOM is in the repo](https://github.com/AndersBNielsen/65uino/blob/main/hardware/65uino-bom.csv)

Remember that you only need one type of Oscilator, either the 4 pin SMD can or a standard crystal with the capacitors. Check the schematic for more info.

Now let's start soldering!!

Always start from the shortest to the tallest components. You do not need to solder the barrel jack for power if you are going to use only the USB port for power.

## The Software

Before you can play with the board you need to build the ROM. For this the first order of business is to setup the development toolchain in your computer. All the software is available in multiple platfrom so it does not matter if you use Mac, Linux or Windows.

### Devlelopment Toolchain

The assembler and linker of choice is [cc65](https://cc65.github.io/doc/) a complete cross development package for 65(C)02 systems.

For Windows and Linux: download and install [cc65 from their main site](https://cc65.github.io/)
For Mac: you can use MacPorts `sudo port install cc65` or Homebrew `brew install cc65`

Be sure you add the cc65 `bin` folder to your path, it will make your life a lot easier.

To burn the rom you will need an EEPROM programmer. The programmer of choice is the venerable TL866II+.
The software of choice here will be [minipro by David Griffith](https://gitlab.com/DavidGriffith/minipro/) if you are on Linux or Mac. This is a great open source program for controlling the TL866xx series of chip programmers.
If you use Windows you will have to use the native (GUI) Xgecu software that came with you TL866xx. Xgecu does not have a command line option so you will have to burn manually.

### The 65uino BIOS

In the 65uino the BIOS implements:

- [I2C bit banged](https://www.youtube.com/watch?v=i7q0P9-wszM&list=PL9Njj9WL8poFsM4C6Gi8V5FRoidOOL0Fv&index=12&t=13s) in Port B of the RIOT. SCL is in pin A5 and SDA in pin A4
- RS232 TTL Serial with XON/OFF flow control, bitbanged in Port A of the RIOT. RX is pin D0 and TX pin D1. 
- [Libraries for driving a SSD1306 0.98 in OLED](https://www.youtube.com/watch?v=x6xsTXY7OtI&list=PL9Njj9WL8poFsM4C6Gi8V5FRoidOOL0Fv&index=11&t=744s) with an example implementation that will act as a serial echo console
- [A serial bootloader](https://www.youtube.com/watch?v=nOmQd3y3pDw&list=PL9Njj9WL8poFsM4C6Gi8V5FRoidOOL0Fv&index=10) to load your own binaries without flashing the ROM.

The BIOS source is in `abn6507rom.s` and the font data lives in `95char5x7font.s`.

The serial port is TTL level so you will need to use an TTL to USB FTDI [board](https://www.amazon.com/gp/product/B07WX2DSVB/) or cable to connect the 65uino to your computer.
The BIOS is set for the serial to work at 4800bps, 8 bits, no parity, 1 stop bit with XON flow control. You can test it form the command prompt with `stty` or with a terminal program like [CoolTerm](https://freeware.the-meiers.org/) which works for all platfroms.

### Build your ROMs

The `assemble.sh` bash script contains all the steps needed to build and burn the BIOS on Mac or Linux. You will need to check and see if the paths match your installation and change them accordinly. I recommed you add your cc65 \bin folder to your path and then just delete the path from the script.

Below is a quick walkthrough of the `assemble.sh` build script.

```bash
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
```

Let's break this down.

```bash
/opt/homebrew/bin/ca65 -vvv --cpu 6502 -l  build/listing.txt -o  build/abn6507rom.o abn6507rom.s
/opt/homebrew/bin/ld65 -o build/abn6507rom.bin -C memmap.cfg "./build/abn6507rom.o"
```

The scripts assumes cc65 is located in `/opt/homebrew/bin/`, the first 2 lines run the *ca65* assembler and then the *ld65* linker. You should not need to change anything else in this lines.
If you want to just build the rom files, you can run these 2 commands directly. This generates 2 separate files:

- `abn6507.bin` > this is the BIOS files to burn in the EEPROM
- `userland.bin` > this is your own program to load though the bootloader

The variable SERIAL is forced to 0 in line 5 `SERIAL=0` this force the script to execute line 8:

```bash
minipro -s -p "SST39SF010A" -w build/abn6507rom.bin
```

This line will use `minipro` to burn the BIOS ROM and finish the script.
It assumes you are using a *SST39SF010A* NOR Flash and that *minipro* is in your PATH. If you are using the more common 27C512 EEPROM change th e`-p` parameter to *"W27C512@DIP28"*.

If you want to use this script to transfer your own code, you need to change line 5 and set SERIAL to something other than 0, e.g. `SERIAL=1`.
Your code should line in the *USERLAND* segment in `anb6507rom.s`

When SERIAL is not 0 this portion of the script will execute that will enable you to load your own code via serial.

```bash
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
```

This section is tested and works in MacOS, only the `serial="/dev/cu.usbs*"` line may need modification for Linux.

It initiates `stty` in the USB 2 serial port at 4800 bauds.
Then to set the 65uino board into serial bootloader mode, line 15 sends a [SOH](https://www.ascii-code.com/character/%E2%90%81) ASCII charater which correspond to HEX 01.
After that it just sends the `userland.bin` though the serial terminal and kills the process.

Then your code will be loaded into RAM and ready to run in the 65uino.

## 65uino Architecture

To start writing let's begin underatnding the 65uino architecture. The hardware is based on the MOS Technology 6507 chip (made famous by the Atari 2600), this chip is a MOS 6502 die in a 28 pin DIP package. The reduction of pins, 40 to 28, means that it lacks A13-A15 address lines (which limits it`s address space to 8K) an unfortunatelly also /INT and /NMI.
Besides interrupt handling, all other aspects of 6502 programming and interfacing can be learn with this board.

For I/O and RAM we are using a [6532 RIOT](https://www.youtube.com/watch?v=Fo5bwoBWVhU&list=PL9Njj9WL8poFsM4C6Gi8V5FRoidOOL0Fv&index=17) also made famouns by the [Atari 2600](https://en.wikipedia.org/wiki/Atari_2600). 
The RIOT provides 128 bytes of static RAM (yes those are BYTES), two bidirectional 8-bit digital input/output ports, and a programmable interval timer. Since the 6507 does not have INT pins the timer. Then we use a standard EEPROM as ROM.

### 65uino Memory Map

| Address | Description |
| ----------- | ----------- |
| $0000-007f | RAM |
| $0080-00ff | RIOT Registers |
| $1000-$1FFF | ROM |

Given that the 6507 can only access addresses using the low 13 bits of an address (8kb), bits 14, 15, and 16 are totally ignored. This means that the first 8Kb memory segment is mirrored 8 times in the 6502 full addresable space.

Because of this and the fact that the 6502 reset vectors live at $FFFA to $FFFF. For consistency with other architectures is better to picture the ROM in the last mirror image at $F000-$FFFF.

## Writing your code

The cc65 environment is already configured for you in the project. The `memmap.cfg` file has the memory map and defines 4 segments:

- ZEROPAGE: Self explanatory, this is the RIOT RAM and registers starting at $0000 and 256 bytes in size
- USERLAND: Reserved for your code starts at $000F and 91 bytes in size
- RODATA: This is the 4kb ROM segment starting at $F000
- VECTORS6502: This starts at $FFFA (ROM) and is used for defining the reset vectors

You have to use the ***USERLAND*** segment to write yur code. You have 91 bytes of RAM for your code that can be loaded through the serial bootloader.
This segments is compiled and linked to `build/userland.bin`.
