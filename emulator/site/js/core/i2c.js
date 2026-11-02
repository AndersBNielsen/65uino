// Bit-level I2C bus model, slave side.
//
// The 65uino bit-bangs I2C on RIOT port B: PB0 = SCL, PB1 = SDA. Both lines
// are open-drain-ish: the firmware keeps the output register bits at 0 and
// toggles the *data-direction* bits (DDR=1 pulls the line low, DDR=0 lets the
// 5k1 pull-up take it high). The master never actively drives a line high.
//
// This module watches the two wired-AND lines, detects START / repeated START
// / STOP, shifts bits on SCL rising edges, generates ACK/NACK and drives read
// data on SDA on behalf of the addressed slave device.
//
// A device implements:
//   address            7-bit address
//   start(read)        addressed; return true to ACK
//   write(byte)        received a byte; return true to ACK
//   read()             return next byte to send
//   stop()             STOP or repeated START seen

const IDLE = 0, ADDR = 1, RX = 2, ACK_OUT = 3, TX = 4, ACK_IN = 5, IGNORE = 6;

export class I2CBus {
  constructor() {
    this.devices = [];
    this.scl = 1; this.sda = 1;       // current wired-AND line levels
    this.masterScl = 1; this.masterSda = 1;
    this.slaveSda = 1;               // what the active slave drives (1 = released)
    this.state = IDLE;
    this.bits = 0; this.shift = 0;
    this.device = null; this.reading = false;
    this.txByte = 0;
    this.log = null;                 // optional array of decoded events
    this.stats = { starts: 0, stops: 0, bytes: 0, nacks: 0 };
  }

  attach(dev) { this.devices.push(dev); return dev; }
  detach(dev) { this.devices = this.devices.filter((d) => d !== dev); }

  /** Called by the board whenever the master's pin drive may have changed. */
  setMaster(scl, sda) {
    if (scl === this.masterScl && sda === this.masterSda) return;
    const sclChanged = scl !== this.masterScl, sdaChanged = sda !== this.masterSda;
    if (sclChanged && sdaChanged) {
      // Both moved in one register write: assume data changes while SCL is low.
      if (scl === 0) { this.masterScl = 0; this.recompute(); this.masterSda = sda; this.recompute(); }
      else { this.masterSda = sda; this.recompute(); this.masterScl = 1; this.recompute(); }
      return;
    }
    this.masterScl = scl; this.masterSda = sda;
    this.recompute();
  }

  recompute() {
    const scl = this.masterScl;
    const sda = this.masterSda & this.slaveSda;
    const prevScl = this.scl, prevSda = this.sda;
    this.scl = scl; this.sda = sda;
    if (scl === prevScl) {
      if (scl === 1 && sda !== prevSda) {
        if (sda === 0) this.onStart(); else this.onStop();
      }
      return;
    }
    if (scl === 1) this.onRise(); else this.onFall();
  }

  setSlaveSda(v) {
    if (v === this.slaveSda) return;
    this.slaveSda = v;
    // Slaves only change SDA while SCL is low, so this never forms START/STOP.
    this.sda = this.masterSda & this.slaveSda;
  }

  onStart() {
    if (this.device) this.device.stop?.();
    this.device = null;
    this.state = ADDR; this.bits = 0; this.shift = 0;
    this.stats.starts++;
    this.log?.push({ t: 'start' });
  }

  onStop() {
    if (this.device) this.device.stop?.();
    this.device = null;
    this.state = IDLE;
    this.setSlaveSda(1);
    this.stats.stops++;
    this.log?.push({ t: 'stop' });
  }

  onRise() {
    switch (this.state) {
      case ADDR: case RX:
        this.shift = ((this.shift << 1) | this.sda) & 0xFF;
        this.bits++;
        break;
      case TX:
        this.bits++;
        break;
      case ACK_IN:
        this.masterAck = this.sda === 0;
        break;
    }
  }

  onFall() {
    switch (this.state) {
      case ADDR:
        if (this.bits < 8) return;
        {
          const addr = this.shift >> 1, read = (this.shift & 1) === 1;
          const dev = this.devices.find((d) => d.address === addr && d.enabled !== false);
          let ack = false;
          if (dev) ack = dev.start(read) !== false;
          this.log?.push({ t: 'addr', addr, read, ack });
          if (ack) {
            this.device = dev; this.reading = read;
            this.setSlaveSda(0);
            this.state = ACK_OUT;
          } else {
            this.stats.nacks++;
            this.device = null;
            this.state = IGNORE;
          }
        }
        return;
      case RX:
        if (this.bits < 8) return;
        {
          const ack = this.device.write(this.shift) !== false;
          this.stats.bytes++;
          this.log?.push({ t: 'byte', v: this.shift, ack });
          if (ack) { this.setSlaveSda(0); this.state = ACK_OUT; }
          else { this.stats.nacks++; this.state = IGNORE; }
        }
        return;
      case ACK_OUT:
        // End of the ACK clock: release SDA, move on to data.
        this.setSlaveSda(1);
        this.bits = 0; this.shift = 0;
        if (this.reading) this.loadTx(); else this.state = RX;
        return;
      case TX:
        if (this.bits < 8) { this.setSlaveSda((this.txByte >> (7 - this.bits)) & 1); return; }
        this.setSlaveSda(1);
        this.state = ACK_IN; this.masterAck = false;
        return;
      case ACK_IN:
        if (this.masterAck) { this.bits = 0; this.loadTx(); }
        else this.state = IGNORE;
        return;
    }
  }

  loadTx() {
    this.txByte = this.device.read() & 0xFF;
    this.stats.bytes++;
    this.log?.push({ t: 'read', v: this.txByte });
    this.state = TX; this.bits = 0;
    this.setSlaveSda((this.txByte >> 7) & 1);
  }
}
