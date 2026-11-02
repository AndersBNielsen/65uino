// Bit-level asynchronous serial (8N1) model for the CH340G USB-UART on the
// 65uino. The 6532 bit-bangs the serial port: PA0 = RX (input, from the
// CH340 TXD through R9) and PA1 = TX (output, to the CH340 RXD through R10).
// The BIOS is built for 4800 baud (BAUDRATE = 4800 in abn6507rom.s).
//
// Everything is driven by the CPU cycle counter, so a bit lasts
// clockHz / baud cycles of *emulated* time and the firmware's cycle-counted
// delay loops see exactly the edges they would see on the real board.

export class UartToBoard {
  /** @param {() => number} now cycle counter */
  constructor(now) {
    this.now = now;
    this.frames = [];       // { start, byte }
    this.lastEnd = 0;
    this.bitCycles = 1e6 / 4800;
  }
  setTiming(clockHz, baud) { this.bitCycles = clockHz / baud; }
  get pending() { return this.frames.length; }
  clear() { this.frames = []; this.lastEnd = 0; }
  /** Queue bytes. `delayCycles` inserts idle time before the first byte. */
  send(bytes, delayCycles = 0, gapCycles = 0) {
    let t = Math.max(this.now() + delayCycles, this.lastEnd);
    for (const b of bytes) {
      this.frames.push({ start: t, byte: b & 0xFF });
      t += 10 * this.bitCycles + gapCycles;
    }
    this.lastEnd = t;
  }
  /** Line level at the current cycle (1 = idle/mark). */
  level() {
    const now = this.now();
    const f = this.frames;
    while (f.length && now >= f[0].start + 10 * this.bitCycles) f.shift();
    if (!f.length || now < f[0].start) return 1;
    const bit = Math.floor((now - f[0].start) / this.bitCycles);
    if (bit === 0) return 0;               // start bit
    if (bit <= 8) return (f[0].byte >> (bit - 1)) & 1; // data, LSB first
    return 1;                               // stop bit
  }
}

export class UartFromBoard {
  constructor(now, onByte) {
    this.now = now;
    this.onByte = onByte;
    this.bitCycles = 1e6 / 4800;
    this.lineLevel = 1;
    this.edges = [];        // [cycle, level] since the start bit
    this.frameStart = -1;
    this.framingErrors = 0;
  }
  setTiming(clockHz, baud) { this.bitCycles = clockHz / baud; }
  /** Report the TX pin level (called on every port A change). */
  setLevel(level) {
    this.poll();
    if (level === this.lineLevel) return;
    const t = this.now();
    this.lineLevel = level;
    if (this.frameStart < 0) {
      if (level === 0) { this.frameStart = t; this.edges = [[t, 0]]; }
      return;
    }
    this.edges.push([t, level]);
  }
  levelAt(t) {
    let l = 1;
    for (const [c, v] of this.edges) { if (c <= t) l = v; else break; }
    return l;
  }
  /** Decode any frame whose stop bit has been reached. */
  poll() {
    if (this.frameStart < 0) return;
    const now = this.now();
    const stopSample = this.frameStart + 9.5 * this.bitCycles;
    if (now < stopSample) return;
    let byte = 0;
    for (let i = 0; i < 8; i++) {
      byte |= this.levelAt(this.frameStart + (1.5 + i) * this.bitCycles) << i;
    }
    const stop = this.levelAt(stopSample);
    if (!stop) this.framingErrors++;
    // A new start bit may already have begun after the stop-bit sample point.
    const next = this.edges.findIndex(([c, v]) => c > stopSample && v === 0);
    if (next >= 0) { this.edges = this.edges.slice(next); this.frameStart = this.edges[0][0]; }
    else { this.edges = []; this.frameStart = -1; }
    this.onByte?.(byte, !stop);
    if (this.frameStart >= 0) this.poll();
  }
}
