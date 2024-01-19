# Getting Started with 65uino

Welcome to 65uino! 
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

### Build the 65uino BIOS

In the 65uino the BIOS implements:

- [I2C bit banged](https://www.youtube.com/watch?v=i7q0P9-wszM&list=PL9Njj9WL8poFsM4C6Gi8V5FRoidOOL0Fv&index=12&t=13s) in Port B of the RIOT. SCL is in pin A5 and SDA in pin A4
- RS232 TTL Serial with XON/OFF flow control, bitbanged in Port A of the RIOT. RX is pin D0 and TX pin D1.
- [Libraries for driving a SSD1306 0.98 in OLED](https://www.youtube.com/watch?v=x6xsTXY7OtI&list=PL9Njj9WL8poFsM4C6Gi8V5FRoidOOL0Fv&index=11&t=744s) with an example implementation that will act as a serial echo console
- [A serial bootloader](https://www.youtube.com/watch?v=nOmQd3y3pDw&list=PL9Njj9WL8poFsM4C6Gi8V5FRoidOOL0Fv&index=10) to load your own binaries without flashing the ROM.

