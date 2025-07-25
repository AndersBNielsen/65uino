import sys
import time
import serial

def send_file_and_listen(serial_port, baud_rate, file_path):
    try:
        # Open serial port with non-blocking read
        ser = serial.Serial(serial_port, baud_rate, timeout=0)

        # 65uino Rev 1 delay
        time.sleep(2)

        # Send Start of Header (SOH) byte (hex 0x01)
        ser.write(b'\x01')

        # Send binary file
        with open(file_path, 'rb') as file:
            file_data = file.read()

        time.sleep(0.5)  # Wait for target device to prepare

        ser.write(file_data)

        time.sleep(0.5)  # Prevent early TX buffer flush

        print("File sent successfully. Listening for output...\nPress Ctrl+C to exit.\n")

        # Now continuously listen for incoming data
        while True:
            if ser.in_waiting:
                output = ser.read(ser.in_waiting).decode(errors='replace')
                if output:
                    print(output, end='', flush=True)
            else:
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python send_userland.py <serial_port> <baud_rate> <file_path>")
        sys.exit(1)

    serial_port = sys.argv[1]
    baud_rate = int(sys.argv[2])
    file_path = sys.argv[3]

    send_file_and_listen(serial_port, baud_rate, file_path)
