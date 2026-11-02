// Simple I2C slave devices found on (or plugged into) the 65uino.

/**
 * 24C64-style serial EEPROM (U10 on the 65uino Rev 1.1, a 24C64 in SOT-23-5;
 * this package has no address pins, so it answers at 0x50).
 * 8 KiB, two address bytes, 32-byte pages, sequential reads. Writes are
 * committed instantly at STOP (the real chip needs ~5 ms and NACKs meanwhile).
 */
export class EEPROM24C64 {
  constructor(address = 0x50, size = 8192, pageSize = 32) {
    this.address = address;
    this.size = size;
    this.pageSize = pageSize;
    this.mem = new Uint8Array(size).fill(0xFF);
    this.ptr = 0;
    this.addrBytes = 0;
    this.pendingWrite = [];
    this.onChange = null;
  }
  start(read) {
    this.reading = read;
    this.addrBytes = read ? 2 : 0;
    this.pendingWrite = [];
    return true;
  }
  write(b) {
    if (this.addrBytes === 0) { this.ptr = (b << 8) & (this.size - 1); this.addrBytes = 1; return true; }
    if (this.addrBytes === 1) { this.ptr = (this.ptr | b) & (this.size - 1); this.addrBytes = 2; this.writePtr = this.ptr; return true; }
    this.pendingWrite.push([this.writePtr, b]);
    // After one page, later bytes overwrite earlier addresses on real EEPROM.
    // Keep only the final page so a ROM cannot grow this queue without STOP.
    if (this.pendingWrite.length > this.pageSize) this.pendingWrite.shift();
    // Page write: address wraps within the current page
    const page = this.writePtr & ~(this.pageSize - 1);
    this.writePtr = page | ((this.writePtr + 1) & (this.pageSize - 1));
    return true;
  }
  read() {
    const v = this.mem[this.ptr];
    this.ptr = (this.ptr + 1) & (this.size - 1);
    return v;
  }
  stop() {
    if (this.pendingWrite.length) {
      for (const [a, v] of this.pendingWrite) this.mem[a] = v;
      this.ptr = this.writePtr;
      this.pendingWrite = [];
      this.onChange?.(this);
    }
  }
}

/**
 * Generic register-file device: first written byte selects a register, then
 * data bytes auto-increment. Used to stand in for the Si5351 clock generator
 * on Anders' "PhaseLoom" SDR shield (0x60). The BIOS only writes to it.
 */
export class RegisterDevice {
  constructor(address, name = 'device') {
    this.address = address;
    this.name = name;
    this.regs = new Uint8Array(256);
    this.reg = 0; this.first = true;
    this.writeCount = 0;
    this.onChange = null;
  }
  start() { this.first = true; return true; }
  write(b) {
    if (this.first) { this.reg = b; this.first = false; return true; }
    this.regs[this.reg] = b; this.reg = (this.reg + 1) & 0xFF; this.writeCount++;
    this.onChange?.(this);
    return true;
  }
  read() { const v = this.regs[this.reg]; this.reg = (this.reg + 1) & 0xFF; return v; }
  stop() {}
}
