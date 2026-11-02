// SSD1306 128x64 OLED controller model (I2C interface), from the datasheet.
//
// I2C protocol: after the address byte the master sends a control byte
// (Co = bit 7, D/C# = bit 6). With Co = 0 every following byte in the
// transfer is a command (D/C# = 0) or GDDRAM data (D/C# = 1). With Co = 1
// exactly one data/command byte follows, then another control byte.
//
// Commands implemented (all of the fundamental, addressing and hardware
// configuration commands, parameters are always consumed correctly):
//   81 contrast, A4/A5 entire display on, A6/A7 normal/inverse, AE/AF off/on,
//   00-0F/10-1F column start (page mode), 20 addressing mode, 21 column range,
//   22 page range, B0-B7 page start, 40-7F display start line, A0/A1 segment
//   remap, A8 multiplex ratio, C0/C8 COM scan direction, D3 display offset,
//   DA COM pins, D5 clock, D9 pre-charge, DB VCOMH, 8D charge pump, E3 NOP.
// Parsed but not rendered: continuous hardware scrolling (26/27/29/2A/2E/2F/
// A3) and fade/zoom (23/D6). Undocumented command bytes are ignored.

const PARAMS = {
  0x81: 1, 0x20: 1, 0x21: 2, 0x22: 2, 0xA8: 1, 0xD3: 1, 0xDA: 1, 0xD5: 1, 0xD9: 1, 0xDB: 1,
  0x8D: 1, 0x26: 6, 0x27: 6, 0x29: 5, 0x2A: 5, 0xA3: 2, 0x23: 1, 0xD6: 1,
};

export class SSD1306 {
  constructor(address = 0x3C) {
    this.address = address;
    this.gram = new Uint8Array(1024); // 8 pages x 128 columns, LSB = top pixel
    this.powerOn();
    this.commandLog = [];
    this.commandCounts = new Map();
    this.writes = 0;
    this.version = 0; // bumps whenever something visible may have changed
  }

  powerOn() {
    for (let i = 0; i < this.gram.length; i++) this.gram[i] = 0;
    // Reset values from the datasheet
    this.contrast = 0x7F;
    this.entireOn = false;
    this.inverse = false;
    this.displayOn = false;
    this.addrMode = 2;            // page addressing
    this.colStart = 0; this.colEnd = 127;
    this.pageStart = 0; this.pageEnd = 7;
    this.col = 0; this.page = 0;
    this.pageModeColStart = 0;
    this.startLine = 0;
    this.segRemap = false;
    this.mux = 63;
    this.comRemap = false;
    this.offset = 0;
    this.comPins = 0x12;
    this.clockDiv = 0x80;
    this.precharge = 0x22;
    this.vcomh = 0x20;
    this.chargePump = false;
    this.scrollActive = false;
    this.pending = null; this.params = [];
    this.expectControl = true; this.co = false; this.dc = false;
    this.version++;
  }

  // ---- I2C device interface ----
  start(read) { this.expectControl = true; this.isRead = read; return true; }
  stop() { this.expectControl = true; }
  read() { return this.displayOn ? 0x00 : 0x40; } // status byte (bit 6 = display off)
  write(byte) {
    if (this.expectControl) {
      this.co = (byte & 0x80) !== 0;
      this.dc = (byte & 0x40) !== 0;
      this.expectControl = false;
      return true;
    }
    if (this.dc) this.data(byte); else this.command(byte);
    if (this.co) this.expectControl = true;
    return true;
  }

  // ---- GDDRAM writes ----
  data(byte) {
    this.gram[this.page * 128 + this.col] = byte;
    this.writes++;
    this.version++;
    switch (this.addrMode) {
      case 0: // horizontal
        if (this.col >= this.colEnd) {
          this.col = this.colStart;
          this.page = this.page >= this.pageEnd ? this.pageStart : (this.page + 1) & 7;
        } else this.col = (this.col + 1) & 127;
        break;
      case 1: // vertical
        if (this.page >= this.pageEnd) {
          this.page = this.pageStart;
          this.col = this.col >= this.colEnd ? this.colStart : (this.col + 1) & 127;
        } else this.page = (this.page + 1) & 7;
        break;
      default: // page addressing: column wraps within the page
        this.col = this.col >= 127 ? this.pageModeColStart : this.col + 1;
    }
  }

  // ---- commands ----
  command(byte) {
    if (this.pending !== null) {
      this.params.push(byte);
      if (this.params.length >= PARAMS[this.pending]) {
        const cmd = this.pending; this.pending = null;
        this.execute(cmd, this.params);
      }
      return;
    }
    if (byte in PARAMS) { this.pending = byte; this.params = []; return; }
    this.execute(byte, []);
  }

  execute(cmd, p) {
    this.commandCounts.set(cmd, (this.commandCounts.get(cmd) || 0) + 1);
    if (this.commandLog.length < 4096) this.commandLog.push(p.length ? [cmd, ...p] : [cmd]);
    this.version++;
    if (cmd <= 0x0F) { this.pageModeColStart = (this.pageModeColStart & 0xF0) | cmd; if (this.addrMode === 2) this.col = this.pageModeColStart; return; }
    if (cmd <= 0x1F) { this.pageModeColStart = ((cmd & 0x07) << 4) | (this.pageModeColStart & 0x0F); if (this.addrMode === 2) this.col = this.pageModeColStart; return; }
    if (cmd >= 0x40 && cmd <= 0x7F) { this.startLine = cmd & 0x3F; return; }
    if (cmd >= 0xB0 && cmd <= 0xB7) { if (this.addrMode === 2) this.page = cmd & 7; return; }
    switch (cmd) {
      case 0x81: this.contrast = p[0]; return;
      case 0xA4: this.entireOn = false; return;
      case 0xA5: this.entireOn = true; return;
      case 0xA6: this.inverse = false; return;
      case 0xA7: this.inverse = true; return;
      case 0xAE: this.displayOn = false; return;
      case 0xAF: this.displayOn = true; return;
      case 0x20: this.addrMode = p[0] & 3; if (this.addrMode === 3) this.addrMode = 2; return;
      case 0x21: this.colStart = p[0] & 127; this.colEnd = p[1] & 127; this.col = this.colStart; return;
      case 0x22: this.pageStart = p[0] & 7; this.pageEnd = p[1] & 7; this.page = this.pageStart; return;
      case 0xA0: this.segRemap = false; return;
      case 0xA1: this.segRemap = true; return;
      case 0xA8: if ((p[0] & 0x3F) >= 15) this.mux = p[0] & 0x3F; return;
      case 0xC0: this.comRemap = false; return;
      case 0xC8: this.comRemap = true; return;
      case 0xD3: this.offset = p[0] & 0x3F; return;
      case 0xDA: this.comPins = p[0]; return;
      case 0xD5: this.clockDiv = p[0]; return;
      case 0xD9: this.precharge = p[0]; return;
      case 0xDB: this.vcomh = p[0]; return;
      case 0x8D: this.chargePump = (p[0] & 0x04) !== 0; return;
      case 0x2E: this.scrollActive = false; return;
      case 0x2F: this.scrollActive = true; return;
      default: return; // NOP / scroll setup / undocumented
    }
  }

  /** Is the panel lit at all? */
  get lit() { return this.displayOn && this.chargePump; }

  /**
   * Render what the glass shows into a 128x64 Uint8Array (1 = lit).
   * Orientation: the 65uino uses the common 0.96" module mounted with the
   * header at the top; with A1 (segment remap) and C8 (COM remap) the image is
   * upright, which is what the BIOS programs.
   */
  render(out = new Uint8Array(128 * 64)) {
    if (!this.lit) { out.fill(0); return out; }
    const alt = (this.comPins & 0x10) !== 0, lrRemap = (this.comPins & 0x20) !== 0;
    for (let r = 0; r < 64; r++) {
      // k = row-counter index shown on physical row r. With C8 the module is
      // upright (k = r); with C0 it is upside down.
      let k = this.comRemap ? r : 63 - r;
      // The glass is wired for "alternative COM pin configuration" (DA 12h);
      // other settings interleave/swap the halves like real panels do.
      if (!alt) k = (k & 1) ? 32 + (k >> 1) : (k >> 1);
      if (lrRemap) k = (k + 32) & 63;
      const base = r * 128;
      if (k > this.mux) { out.fill(0, base, base + 128); continue; }
      // Display start line and display offset both shift which RAM row is shown.
      const ramRow = (k + this.startLine + this.offset) & 63;
      const page = ramRow >> 3, bit = 1 << (ramRow & 7);
      for (let x = 0; x < 128; x++) {
        const col = this.segRemap ? x : 127 - x;
        let px = (this.gram[page * 128 + col] & bit) ? 1 : 0;
        if (this.inverse) px ^= 1;
        if (this.entireOn) px = 1;
        out[base + x] = px;
      }
    }
    return out;
  }

  /** Debug helper: GDDRAM as ASCII art in RAM orientation. */
  gramAscii() {
    let s = '';
    for (let y = 0; y < 64; y++) {
      for (let x = 0; x < 128; x++) s += (this.gram[(y >> 3) * 128 + x] >> (y & 7)) & 1 ? '#' : '.';
      s += '\n';
    }
    return s;
  }
}
