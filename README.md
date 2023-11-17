# Welcome to the 65uino project

It's a green 6502 based learning platform in a familiar form factor and a contestant in the 2023 Hackaday Prize. By taking IC's out of the waste stream and into useful products we encourage recycling and reuse.
It's a fun way to learn assembly programming and a tool you can use in your workshop for a lot more than you expect.

![65uino](https://cdn.hackaday.io/images/8644651680943366680.jpeg)

In this repository you will find KiCAD hardware files, gerbers and 6502 ASM source files for the 65uino.
The 65uino is based on the Single Breadboard Computer - https://youtu.be/s3t2QMukBRs

You'll find more details at https://hackaday.io/project/190260-65uino

Prerequisites:

Install cc65 (and minipro if using a tl866II+ to program a ROM) using your favorite package manager
Run assemble.sh to assemble the source file
(If you use Windows you probably want to use WSL for this^)

The hardware schematic was created using KiCAD 6

## Getting a PCB
This project is kindly sponsored by JLCPCB. They offer cheap, professional looking PCBs and super fast delivery.

Step 1: Get the gerber file zip package from the /hardware folder
[hardware/65uinogerbers20231023-nomousebites.zip](https://github.com/AndersBNielsen/65uino/blob/main/hardware/65uinogerbers20231023-nomousebites.zip) for instance.

Step 2: Upload to JLCPCB https://jlcpcb.com


![](https://github.com/AndersBNielsen/65uino/blob/main/images/upload.png?raw=true)

Step 3: Pick your color, surface finish and order.
![https://github.com/AndersBNielsen/65uino/blob/main/images/settings.png?raw=true](https://github.com/AndersBNielsen/65uino/blob/main/images/settings.png?raw=true)


![https://github.com/AndersBNielsen/65uino/blob/main/images/save.png?raw=true](https://github.com/AndersBNielsen/65uino/blob/main/images/save.png?raw=true)

You can use these affilite links to get a board for $2 and also get $54 worth of New User Coupons at: https://jlcpcb.com/?from=Anders

And in case you also want to order a 3D-printed case you can use this link. 
How to Get a $7 3D Printing Coupon: https://3d.jlcpcb.com/?from=Anders3DP
