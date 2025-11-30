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

        # Open a file to save incoming binary data and also accumulate chunks
        with open("received_output.bin", "ab") as output_file:
            chunk_buf = bytearray()
            chunk_count = 0
            CHUNK_SIZE = 4096  # bytes before invoking analyzer
            CHUNK_TIMEOUT = 0.1  # seconds to wait before flushing partial chunk
            last_data_time = time.time()

            # Now continuously listen for incoming data
            while True:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    if data:
                        output_file.write(data)
                        output_file.flush()

                        chunk_buf.extend(data)
                        last_data_time = time.time()

                        # If we've reached (or exceeded) chunk size, write and analyze
                        while len(chunk_buf) >= CHUNK_SIZE:
                            to_write = bytes(chunk_buf[:CHUNK_SIZE])
                            # Save chunk to a numbered file
                            chunk_filename = f"received_chunk_{chunk_count}.bin"
                            with open(chunk_filename, 'wb') as cf:
                                cf.write(to_write)
                            # Call analyzer on this chunk (non-blocking stdout/stderr passthru)
                            try:
                                import subprocess, sys as _sys
                                _sys.stdout.write(f"\n[Analyzer] Processing {chunk_filename}\n")
                                _sys.stdout.flush()
                                subprocess.run([_sys.executable, 'tools/analyze_adc.py', '--file', chunk_filename, '--iq', '--sample-rate', '27000', '--peaks', '5'], check=False)
                            except Exception as e:
                                print(f"Analyzer call failed: {e}", file=sys.stderr)
                            # remove written bytes from buffer
                            del chunk_buf[:CHUNK_SIZE]
                            chunk_count += 1
                else:
                    # If no data for a short while, flush a partial chunk
                    now = time.time()
                    if chunk_buf and (now - last_data_time) >= CHUNK_TIMEOUT:
                        # flush what's available (even if smaller than CHUNK_SIZE)
                        to_write = bytes(chunk_buf)
                        chunk_filename = f"received_chunk_{chunk_count}.bin"
                        with open(chunk_filename, 'wb') as cf:
                            cf.write(to_write)
                        try:
                            import subprocess, sys as _sys
                            _sys.stdout.write(f"\n[Analyzer] Processing {chunk_filename} (partial {len(to_write)} bytes)\n")
                            _sys.stdout.flush()
                            subprocess.run([_sys.executable, 'tools/analyze_adc.py', '--file', chunk_filename, '--iq', '--sample-rate', '27000', '--peaks', '5'], check=False)
                        except Exception as e:
                            print(f"Analyzer call failed: {e}", file=sys.stderr)
                        chunk_count += 1
                        chunk_buf.clear()
                    else:
                        time.sleep(0.01)

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
