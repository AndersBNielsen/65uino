# Note: This script generates a ROM file with a specific bit cleared.
# Configuration
rom_size = 2048               # 2KB ROM
byte_offset = 0x7f0           # Choose the byte offset (0 to 2047)
bit_to_clear = 7              # Choose the bit to clear (0 = LSB, 7 = MSB)
output_file = 'tms2516_rom.bin'

# Create ROM filled with 0xFF
rom = bytearray([0xFF] * rom_size)

# Clear the specified bit (set to 0)
rom[byte_offset] &= ~(1 << bit_to_clear)

# Write to file
with open(output_file, 'wb') as f:
    f.write(rom)

print(f"ROM generated: {output_file}")
print(f"Cleared bit {bit_to_clear} at offset 0x{byte_offset:X}")
