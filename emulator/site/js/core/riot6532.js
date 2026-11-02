// MOS 6532 RIOT (RAM-I/O-Timer), written from scratch for the 65uino emulator.
//
// - 128 bytes of static RAM (selected when RS = 0)
// - Two 8-bit I/O ports with data-direction registers
// - 8-bit interval timer with /1, /8, /64, /1024 prescaler, interrupt flag and
//   the "after zero, count down every cycle" behaviour
// - PA7 edge detector (positive or negative edge) with its own flag
//
// Register decode (RS = 1), A0..A4, per the 6532 datasheet:
//   A2=0          : A1A0 = 00 ORA/IRA, 01 DDRA, 10 ORB/IRB, 11 DDRB
//   A2=1, write   : A4=1 -> timer write, A1A0 = prescaler, A3 = timer IRQ enable
//                   A4=0 -> edge-detect control, A0 = positive edge, A1 = PA7 IRQ enable
//   A2=1, read    : A0=0 -> read timer, A3 = timer IRQ enable
//                   A0=1 -> read interrupt flags (bit7 timer, bit6 PA7), clears PA7 flag
//
// Timer model: identical in behaviour to the well-tested model in the Stella
// Atari 2600 emulator (which uses the same chip). Writing N loads the counter
// and the counter is decremented one cycle later and then every `prescale`
// cycles. When it passes zero the interrupt flag is set and it keeps counting
// down once per cycle. Reading or writing the timer clears the timer flag (and
// so returns the counter to the programmed prescaler).
//
// Port reads: port A returns the actual pin levels; port B returns the output
// register for pins set as outputs and the pin level for inputs (the PB
// outputs are push-pull buffered on the 6532).
//
// The RIOT does not keep its own clock: `now()` returns the current CPU cycle
// and the timer state is brought up to date lazily on every access.

const SHIFTS = [0, 3, 6, 10];
export const TIMER_FLAG = 0x80, PA7_FLAG = 0x40;

export class RIOT6532 {
  /**
   * @param {() => number} now  current cycle count
   * @param {object} pins  { readA(): number, readB(): number, onPortChange(port) }
   *   readA/readB return the external pin levels (1 = high/pulled up) seen when
   *   the pin is an input; called lazily on reads.
   */
  constructor(now, pins) {
    this.now = now;
    this.pins = pins;
    this.ram = new Uint8Array(128);
    for (let i = 0; i < 128; i++) this.ram[i] = (i * 73 + 41) & 0xFF; // power-on garbage
    this.timer = 0xFF;
    this.shift = 10;
    this.divider = 1024;
    this.subTimer = 0;
    this.lastCycle = 0;
    this.wrappedThisCycle = false;
    this.flags = 0;
    this.resetRegs();
  }

  resetRegs() {
    // /RES clears the I/O registers and interrupt enables; the timer keeps running.
    this.ora = 0; this.ddra = 0; this.orb = 0; this.ddrb = 0;
    this.timerIrqEnable = false;
    this.pa7IrqEnable = false;
    this.pa7Positive = false;
    this.lastPA7 = 1;
    this.pins.onPortChange?.('A');
    this.pins.onPortChange?.('B');
  }

  reset() { this.update(); this.resetRegs(); this.lastPA7 = this.pinLevelA() >> 7; }

  // ---- timer ----
  update() {
    const now = this.now();
    let cycles = now - this.lastCycle;
    if (cycles <= 0) return;
    this.lastCycle = now;
    this.wrappedThisCycle = false;
    const sub = this.subTimer;
    this.subTimer = (cycles + sub) % this.divider;
    if ((this.flags & TIMER_FLAG) === 0) {
      const ticks = Math.floor((cycles + sub) / this.divider);
      if (ticks > this.timer) {
        cycles -= (this.timer + 1) * this.divider - sub;
        this.wrappedThisCycle = cycles === 0;
        this.timer = 0xFF;
        this.flags |= TIMER_FLAG;
      } else {
        this.timer -= ticks;
        cycles = 0;
      }
    }
    if (cycles > 0) {
      // After the flag is set the timer counts down every cycle.
      this.timer = (this.timer - cycles) & 0xFF;
    }
  }

  writeTimer(value, interval, irqEnable) {
    this.update();
    this.shift = SHIFTS[interval];
    this.divider = 1 << this.shift;
    this.subTimer = this.divider - 1;
    this.timer = value & 0xFF;
    this.flags &= ~TIMER_FLAG;
    this.timerIrqEnable = irqEnable;
    this.wrappedThisCycle = false;
  }

  readTimer(irqEnable) {
    this.update();
    if (!this.wrappedThisCycle) this.flags &= ~TIMER_FLAG;
    this.timerIrqEnable = irqEnable;
    return this.timer;
  }

  readFlags() {
    this.update();
    const r = this.flags;
    this.flags &= ~PA7_FLAG;
    return r;
  }

  /** True while the (open-drain, active-low) /IRQ output is asserted. */
  get irq() {
    this.update();
    return ((this.flags & TIMER_FLAG) !== 0 && this.timerIrqEnable) ||
      ((this.flags & PA7_FLAG) !== 0 && this.pa7IrqEnable);
  }

  // ---- ports ----
  /** Pin levels of port A (outputs drive, inputs see the outside world). */
  pinLevelA() {
    const ext = this.pins.readA();
    return ((this.ora & this.ddra) | (ext & ~this.ddra)) & 0xFF;
  }
  /** Pin levels of port B. */
  pinLevelB() {
    const ext = this.pins.readB();
    return ((this.orb & this.ddrb) | (ext & ~this.ddrb)) & 0xFF;
  }
  /** Call whenever PA7 may have changed (output write or external input). */
  checkPA7() {
    const lvl = (this.pinLevelA() >> 7) & 1;
    if (lvl !== this.lastPA7) {
      if ((lvl === 1 && this.pa7Positive) || (lvl === 0 && !this.pa7Positive)) {
        this.update();
        this.flags |= PA7_FLAG;
      }
      this.lastPA7 = lvl;
    }
  }

  /** Read with A0..A7 already decoded as RIOT; addr bit 7 = RS. */
  read(addr) {
    if ((addr & 0x80) === 0) return this.ram[addr & 0x7F];
    if ((addr & 0x04) === 0) {
      switch (addr & 0x03) {
        case 0: return this.pinLevelA();
        case 1: return this.ddra;
        case 2: { const ext = this.pins.readB(); return ((this.orb & this.ddrb) | (ext & ~this.ddrb)) & 0xFF; }
        case 3: return this.ddrb;
      }
    }
    if ((addr & 0x01) === 0) return this.readTimer((addr & 0x08) !== 0);
    return this.readFlags();
  }

  /** Side-effect-free read for debuggers. */
  peek(addr) {
    if ((addr & 0x80) === 0) return this.ram[addr & 0x7F];
    if ((addr & 0x04) === 0) {
      switch (addr & 0x03) {
        case 0: return this.pinLevelA();
        case 1: return this.ddra;
        case 2: return this.pinLevelB();
        case 3: return this.ddrb;
      }
    }
    this.update();
    return (addr & 1) ? this.flags : this.timer;
  }

  write(addr, v) {
    if ((addr & 0x80) === 0) { this.ram[addr & 0x7F] = v; return; }
    if ((addr & 0x04) === 0) {
      switch (addr & 0x03) {
        case 0: this.ora = v; this.pins.onPortChange?.('A'); this.checkPA7(); return;
        case 1: this.ddra = v; this.pins.onPortChange?.('A'); this.checkPA7(); return;
        case 2: this.orb = v; this.pins.onPortChange?.('B'); return;
        case 3: this.ddrb = v; this.pins.onPortChange?.('B'); return;
      }
    }
    if (addr & 0x10) { this.writeTimer(v, addr & 0x03, (addr & 0x08) !== 0); return; }
    // Edge-detect control
    this.update();
    this.pa7Positive = (addr & 0x01) !== 0;
    this.pa7IrqEnable = (addr & 0x02) !== 0;
  }

  getState() {
    this.update();
    return { ram: this.ram, ora: this.ora, ddra: this.ddra, orb: this.orb, ddrb: this.ddrb,
      timer: this.timer, prescaler: this.divider, flags: this.flags,
      timerIrqEnable: this.timerIrqEnable, pa7IrqEnable: this.pa7IrqEnable,
      pa7Positive: this.pa7Positive, irq: this.irq };
  }
}
