// The 65uino board: 6507 + 6532 RIOT + 4 KiB ROM window + I2C bus (SSD1306
// OLED, 24C64 EEPROM, optional PhaseLoom Si5351) + CH340 serial, wired as in
// hardware/65uino.kicad_sch (Rev 1.1) of github.com/AndersBNielsen/65uino.
//
// Address decode (6507 has A0..A12 only, so everything mirrors every 8 KiB):
//   A12 = 1             -> ROM (W27C512 /OE = NOT A12 via U7, /CE = GND,
//                          ROM A12..A15 tied low by JP2/JP8/JP10+R18, so the
//                          CPU sees chip offset $0000-$0FFF at $1000-$1FFF)
//   A12..A10 = 000      -> 74HC138 /Y0 -> RIOT /CS2 (CS1 = +5V)
//                          A7 = RS: 0 = RAM (A0-A6), 1 = I/O + timer
//                          A8, A9 not decoded -> RAM/IO mirrored, so the
//                          6502 stack page $01xx lands in RIOT RAM.
//   A12..A10 = 001..011 -> /Y1../Y3 go only to the J6 "Chip select" header;
//                          nothing on the board answers (open bus).
//
// RIOT ports (Arduino header names in brackets):
//   PA0 [D0/RX]  serial in from CH340       PB0 [SCL/A5] I2C SCL (5k1 pull-up)
//   PA1 [D1/TX]  serial out to CH340        PB1 [SDA/A4] I2C SDA (5k1 pull-up)
//   PA2..PA7 [D2..D7]  free GPIO            PB2..PB5 [D8..D11] free GPIO
//                                           PB6 [D12] BTN (SW2 to GND, 22k pull-up)
//                                           PB7 [D13] green LED D1 (active low)
// The RIOT /IRQ can be routed to PB6 by bridging solder jumper JP9 (open by
// default); the 6507 itself has no IRQ or NMI pin.
//
// Clock: 8 MHz oscillator -> 74HC161 divider; solder jumpers pick PHI0 =
// 8/4/2/1/0.5 MHz. JP7 (1 MHz) is the one bridged by default.

import { CPU6502 } from './cpu6502.js';
import { RIOT6532 } from './riot6532.js';
import { I2CBus } from './i2c.js';
import { SSD1306 } from './ssd1306.js';
import { EEPROM24C64, RegisterDevice } from './i2cdevices.js';
import { UartToBoard, UartFromBoard } from './uart.js';

export const CLOCK_OPTIONS = [500000, 1000000, 2000000, 4000000, 8000000];
export const DEFAULT_BAUD = 4800;

export const PIN_INFO = [
  { id: 'PA0', port: 'A', bit: 0, header: 'D0', fn: 'RX (serial in)' },
  { id: 'PA1', port: 'A', bit: 1, header: 'D1', fn: 'TX (serial out)' },
  { id: 'PA2', port: 'A', bit: 2, header: 'D2', fn: '' },
  { id: 'PA3', port: 'A', bit: 3, header: 'D3', fn: '' },
  { id: 'PA4', port: 'A', bit: 4, header: 'D4', fn: '' },
  { id: 'PA5', port: 'A', bit: 5, header: 'D5', fn: '' },
  { id: 'PA6', port: 'A', bit: 6, header: 'D6', fn: '' },
  { id: 'PA7', port: 'A', bit: 7, header: 'D7', fn: 'edge detect' },
  { id: 'PB0', port: 'B', bit: 0, header: 'A5/SCL', fn: 'I2C SCL' },
  { id: 'PB1', port: 'B', bit: 1, header: 'A4/SDA', fn: 'I2C SDA' },
  { id: 'PB2', port: 'B', bit: 2, header: 'D8', fn: '' },
  { id: 'PB3', port: 'B', bit: 3, header: 'D9', fn: '' },
  { id: 'PB4', port: 'B', bit: 4, header: 'D10', fn: '' },
  { id: 'PB5', port: 'B', bit: 5, header: 'D11', fn: '' },
  { id: 'PB6', port: 'B', bit: 6, header: 'D12', fn: 'BTN (SW2)' },
  { id: 'PB7', port: 'B', bit: 7, header: 'D13', fn: 'LED D1' },
];

export class Board65uino {
  constructor() {
    this.clockHz = 1000000;
    this.baud = DEFAULT_BAUD;
    this.rom = new Uint8Array(4096).fill(0xFF);
    this.lastBus = 0;
    this.button = false;     // SW2 "BTN" held
    this.jp9 = false;        // RIOT /IRQ -> PB6
    this.extA = 0xFF;        // externally applied levels on input pins (1 = high / pulled up)
    this.extB = 0xFF;
    this.serialOut = [];     // bytes received from the board's TX pin
    this.onSerialByte = null;

    this.cpu = new CPU6502(this);
    const now = () => this.cpu.cycles;
    this.uartIn = new UartToBoard(now);
    this.uartOut = new UartFromBoard(now, (b, err) => {
      this.serialOut.push(b);
      if (this.serialOut.length > 8192) this.serialOut.shift();
      this.onSerialByte?.(b, err);
    });
    this.i2c = new I2CBus();
    this.oled = this.i2c.attach(new SSD1306(0x3C));
    this.eeprom = this.i2c.attach(new EEPROM24C64(0x50));
    this.phaseloom = this.i2c.attach(new RegisterDevice(0x60, 'Si5351 (PhaseLoom)'));
    this.phaseloom.enabled = false;

    // pin activity accounting (for LED brightness / indicators)
    this.pinLevels = new Uint8Array(16).fill(1);
    this.pinHigh = new Float64Array(16);
    this.activityStart = 0;
    this.lastActivity = 0;

    this.riot = new RIOT6532(now, {
      readA: () => (this.extA & 0xFE) | (this.uartIn.level() & (this.extA & 1)),
      readB: () => {
        let v = (this.extB & 0x3C) | 0x80 | this.i2c.scl | (this.i2c.sda << 1);
        let pb6 = (this.extB >> 6) & 1;
        if (this.button) pb6 = 0;
        if (this.jp9 && this.riot.irq) pb6 = 0;
        return v | (pb6 << 6);
      },
      onPortChange: (port) => this.portChanged(port),
    });
    this.setTiming();
  }

  setTiming() {
    this.uartIn.setTiming(this.clockHz, this.baud);
    this.uartOut.setTiming(this.clockHz, this.baud);
  }
  setClock(hz) { this.clockHz = hz; this.setTiming(); }
  setBaud(b) { this.baud = b; this.setTiming(); }

  // ---- system bus (13-bit 6507 address space) ----
  read(addr) {
    const a = addr & 0x1FFF;
    let v;
    if (a & 0x1000) v = this.rom[a & 0x0FFF];
    else if ((a & 0x0C00) === 0) v = this.riot.read(a & 0xFF);
    else v = this.lastBus; // J6 chip-select header area: open bus
    this.lastBus = v;
    return v;
  }
  write(addr, v) {
    const a = addr & 0x1FFF;
    this.lastBus = v;
    if (a & 0x1000) return; // EPROM: no write strobe
    if ((a & 0x0C00) === 0) this.riot.write(a & 0xFF, v);
  }
  /** Side-effect-free read for debuggers and disassembly. */
  peek(addr) {
    const a = addr & 0x1FFF;
    if (a & 0x1000) return this.rom[a & 0x0FFF];
    if ((a & 0x0C00) === 0) return this.riot.peek(a & 0xFF);
    return 0xFF;
  }

  // ---- pins ----
  portChanged(port) {
    const r = this.riot;
    if (!r) return; // during RIOT construction
    this.accountActivity();
    if (port === 'B') {
      const scl = (r.ddrb & 1) ? (r.orb & 1) : 1;
      const sda = (r.ddrb & 2) ? ((r.orb >> 1) & 1) : 1;
      this.i2c.setMaster(scl, sda);
    } else {
      const tx = (r.ddra & 2) ? ((r.ora >> 1) & 1) : 1;
      this.uartOut.setLevel(tx);
    }
    this.refreshPinLevels();
  }
  refreshPinLevels() {
    const a = this.riot.pinLevelA(), b = this.riot.pinLevelB();
    for (let i = 0; i < 8; i++) { this.pinLevels[i] = (a >> i) & 1; this.pinLevels[8 + i] = (b >> i) & 1; }
  }
  accountActivity() {
    const now = this.cpu.cycles, dt = now - this.lastActivity;
    if (dt > 0) {
      for (let i = 0; i < 16; i++) if (this.pinLevels[i]) this.pinHigh[i] += dt;
      this.lastActivity = now;
    }
  }
  /** Fraction of time each pin was high since the last call (0..1). */
  takeActivity() {
    this.refreshPinLevels();
    this.accountActivity();
    const total = this.cpu.cycles - this.activityStart;
    const out = new Float32Array(16);
    for (let i = 0; i < 16; i++) {
      out[i] = total > 0 ? this.pinHigh[i] / total : this.pinLevels[i];
      this.pinHigh[i] = 0;
    }
    this.activityStart = this.cpu.cycles;
    return out;
  }
  setButton(down) {
    this.button = !!down;
    this.accountActivity(); this.refreshPinLevels();
  }
  /** Drive an input pin from outside: level 1 = high/pulled up, 0 = low. */
  setExternal(port, bit, level) {
    this.accountActivity();
    if (port === 'A') this.extA = level ? this.extA | (1 << bit) : this.extA & ~(1 << bit);
    else this.extB = level ? this.extB | (1 << bit) : this.extB & ~(1 << bit);
    if (port === 'A') this.riot.checkPA7();
    this.refreshPinLevels();
  }
  setPhaseLoom(on) { this.phaseloom.enabled = !!on; }

  // ---- ROM / reset ----
  /**
   * Load a ROM image. 4 KiB images map 1:1. Smaller power-of-two images
   * (e.g. a 2 KiB 28C16) mirror. Larger images (a full 27C512 dump) are
   * windowed like the board's default jumpers do: chip offset $0000-$0FFF.
   */
  loadRom(bytes, offset = 0) {
    const src = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
    const rom = new Uint8Array(4096);
    if (src.length >= 4096) rom.set(src.subarray(offset, offset + 4096));
    else if (src.length > 0) for (let i = 0; i < 4096; i++) rom[i] = src[i % src.length];
    this.rom = rom;
  }

  /** Press the RST button: /RES goes to the 6507 and the 6532. */
  reset() {
    this.riot.reset();
    this.cpu.reset();
    this.i2c.setMaster(1, 1);
  }

  /** Full power cycle: OLED, RAM and everything else start from scratch. */
  powerOn() {
    this.cpu.cycles = 0;
    this.activityStart = this.lastActivity = 0;
    this.pinHigh.fill(0);
    this.riot.lastCycle = 0;
    this.oled.powerOn();
    this.uartIn.clear();
    this.serialOut = [];
    this.cpu.a = this.cpu.x = this.cpu.y = 0; this.cpu.s = 0xFD;
    this.reset();
  }

  /** Run for `cycles` CPU cycles (or until a breakpoint). Returns 'bp' on breakpoint. */
  run(cycles, breakpoints = null) {
    const cpu = this.cpu;
    const target = cpu.cycles + cycles;
    if (breakpoints && breakpoints.size) {
      let first = true;
      while (cpu.cycles < target) {
        if (!first && breakpoints.has(cpu.pc)) return 'bp';
        first = false;
        cpu.step();
      }
    } else {
      while (cpu.cycles < target) cpu.step();
    }
    this.uartOut.poll();
    return 'ok';
  }

  /** Execute exactly one instruction. */
  stepInstruction() { this.cpu.step(); this.uartOut.poll(); }

  /** Send bytes to the board's serial RX, like a terminal would. */
  serialSend(bytes, delaySeconds = 0) {
    this.uartIn.send(bytes, Math.round(delaySeconds * this.clockHz));
  }

  /**
   * Upload a userland binary the way send_userland.py does: SOH (0x01), wait
   * 0.5 s, then the raw bytes. The BIOS loads it to $000E and runs it.
   */
  uploadUserland(bytes) {
    this.uartIn.send([0x01]);
    this.uartIn.send(bytes, Math.round(0.5 * this.clockHz) + this.uartIn.bitCycles * 10);
  }

  framebuffer(out) { return this.oled.render(out); }
}
