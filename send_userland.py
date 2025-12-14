import sys
import time
import serial
import subprocess
from pathlib import Path

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
            CHUNK_TIMEOUT = 0.05  # seconds of inactivity to flush partial chunk
            last_data_time = time.time()
            capture_active = False
            last_byte = None

            def print_to_terminal(b: bytes):
                # Print raw text; fallback to hex for non-printables
                try:
                    txt = b.decode('ascii', errors='replace')
                    sys.stdout.write(txt)
                    sys.stdout.flush()
                except Exception:
                    sys.stdout.write(b.hex() + "\n")
                    sys.stdout.flush()

            # Now continuously listen for incoming data
            while True:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    if data:
                        output_file.write(data)
                        output_file.flush()
                        last_data_time = time.time()

                        # Stream parser: look for preamble 0xA5A5 to start capture
                        i = 0
                        while i < len(data):
                            byte = data[i]
                            if not capture_active:
                                # Detect possible preambles: AB AB or A5 A5
                                if last_byte == 0xAB and byte == 0xAB:
                                    sys.stdout.write("[Preamble] 0xAB 0xAB detected, starting capture\n")
                                    sys.stdout.flush()
                                    capture_active = True
                                    chunk_buf.clear()
                                    last_data_time = time.time()
                                elif last_byte == 0xA5 and byte == 0xA5:
                                    sys.stdout.write("[Preamble] 0xA5 0xA5 detected, starting capture\n")
                                    sys.stdout.flush()
                                    capture_active = True
                                    chunk_buf.clear()
                                    last_data_time = time.time()
                                else:
                                    # Print preamble-less data to terminal
                                    print_to_terminal(bytes([byte]))
                                last_byte = byte
                                i += 1
                                continue

                            # capture_active: push bytes into chunk buffer
                            chunk_buf.append(byte)
                            last_byte = byte
                            i += 1

                        # If we've reached (or exceeded) chunk size, write and analyze
                        while capture_active and len(chunk_buf) >= CHUNK_SIZE:
                            to_write = bytes(chunk_buf[:CHUNK_SIZE])
                            # Save chunk to a numbered file
                            chunk_filename = f"received_chunk_{chunk_count}.bin"
                            with open(chunk_filename, 'wb') as cf:
                                cf.write(to_write)
                            # Run validator to report bin powers and max
                            try:
                                res = subprocess.run([
                                    sys.executable,
                                    str(Path(__file__).parent / 'tools' / 'validate_goertzel.py'),
                                    str(chunk_filename)
                                ], capture_output=True, text=True)
                                if res.returncode == 0:
                                    print(res.stdout.rstrip())
                                else:
                                    print(res.stderr.rstrip())
                            except Exception as e:
                                print(f"Validator error: {e}")
                            # Call analyzer on this chunk (non-blocking stdout/stderr passthru)
                            try:
                                import subprocess as _subp
                                sys.stdout.write(f"\n[Analyzer] Processing {chunk_filename}\n")
                                sys.stdout.flush()
                                _subp.run([sys.executable, 'tools/analyze_adc.py', '--file', chunk_filename, '--iq', '--sample-rate', '27778', '--peaks', '5'], check=False)
                            except Exception as e:
                                print(f"Analyzer call failed: {e}", file=sys.stderr)
                            # remove written bytes from buffer
                            del chunk_buf[:CHUNK_SIZE]
                            chunk_count += 1
                            # After finishing a chunk, stop capture and return to preamble-wait
                            capture_active = False
                            last_byte = None
                            # Inform user and continue printing debug data while waiting
                            sys.stdout.write(f"\n[Capture] Finished chunk {chunk_count-1}, waiting for preamble 0xABAB\n")
                            sys.stdout.flush()
                else:
                    # Idle: sleep briefly; flush partial chunk if capture_active and timed out
                    if capture_active and CHUNK_TIMEOUT is not None and (time.time() - last_data_time) > CHUNK_TIMEOUT and len(chunk_buf) > 0:
                        # flush partial chunk
                        to_write = bytes(chunk_buf)
                        chunk_filename = f"received_chunk_{chunk_count}.bin"
                        with open(chunk_filename, 'wb') as cf:
                            cf.write(to_write)
                        # Run validator
                        try:
                            res = subprocess.run([
                                sys.executable,
                                str(Path(__file__).parent / 'tools' / 'validate_goertzel.py'),
                                str(chunk_filename)
                            ], capture_output=True, text=True)
                            if res.returncode == 0:
                                print(res.stdout.rstrip())
                            else:
                                print(res.stderr.rstrip())
                        except Exception as e:
                            print(f"Validator error: {e}")
                        # Analyzer call
                        try:
                            import subprocess as _subp
                            sys.stdout.write(f"\n[Analyzer] Processing {chunk_filename} (partial flush)\n")
                            sys.stdout.flush()
                            _subp.run([sys.executable, 'tools/analyze_adc.py', '--file', chunk_filename, '--iq', '--sample-rate', '27778', '--peaks', '5'], check=False)
                        except Exception as e:
                            print(f"Analyzer call failed: {e}", file=sys.stderr)
                        del chunk_buf[:]
                        chunk_count += 1
                        capture_active = False
                        last_byte = None
                        sys.stdout.write(f"[Capture] Flushed partial chunk {chunk_count-1} after timeout, waiting for preamble 0xABAB\n")
                        sys.stdout.flush()
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
