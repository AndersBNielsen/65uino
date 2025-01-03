import sys
import time
import serial

def send_file(serial_port, baud_rate, file_path):
    try:
        # Open serial port
        ser = serial.Serial(serial_port, baud_rate, timeout=0)
         # 65uino Rev 1 delay
        time.sleep(0.8);
        # Send Start of Header (SOH) byte (hex 0x01)
        ser.write(b'\x01')
        # Send binary file
        with open(file_path, 'rb') as file:
            file_data = file.read()

        # Wait for 65uino to get ready for data
        time.sleep(0.1)

        ser.write(file_data)

        time.sleep(0.1) #Closing too fast empties TX buffer without sending 

        print("File sent successfully.")

    finally:
        # Close the serial port
        ser.close()

if __name__ == "__main__":
    # Check if correct number of command line arguments is provided
    if len(sys.argv) != 4:
        print("Usage: python send_userland.py <serial_port> <baud_rate> <file_path>")
        sys.exit(1)

    # Extract command line arguments
    serial_port = sys.argv[1]
    baud_rate = int(sys.argv[2])
    file_path = sys.argv[3]

    # Call the function to send the file
    send_file(serial_port, baud_rate, file_path)
