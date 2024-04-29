#!/bin/bash
python3 send_command.py /dev/cu.usbserial-UUT1 9600 03 DA 08  # 03 = erase, DA = Winbond, 08 = W27C512
# After sending data there's a timeout before loading happens, so we need a delay
sleep 1 
python3 send_binary.py /dev/cu.usbserial-UUT1 9600 build/abn6507rom.bin 64 
sleep 1
#Read back the ROM we just programmed to check if it's good  
python3 read_binary.py /dev/cu.usbserial-UUT1 9600 verify.bin 4096 64 
diff -s build/abn6507rom.bin verify.bin