// NMOS 6502 / 6507 CPU core, written from scratch for the 65uino emulator.
//
// Every bus access (including the "dummy" reads and writes the real NMOS 6502
// performs) is one call to bus.read()/bus.write() and costs exactly one clock
// cycle, so instruction timing, page-crossing penalties and read-modify-write
// double writes come out the way the silicon does them. `cycles` counts PHI2
// cycles since power-on.
//
// Implemented:
//  - all 151 documented opcodes, NMOS decimal mode (incl. N/V/Z quirks)
//  - the common "illegal" NMOS opcodes (LAX, SAX, DCP, ISC, SLO, RLA, SRE,
//    RRA, ANC, ALR, ARR, SBX, multi-byte NOPs, $EB SBC) and the unstable ones
//    (ANE, LXA, SHA, SHX, SHY, TAS, LAS) using their usual approximations
//  - JAM/KIL opcodes halt the CPU until reset (the 65uino BIOS uses $F2 on
//    purpose for "Airplane mode")
//  - IRQ/NMI inputs (the 6507 has no pins for them, but the core supports them)

export const FLAG_C = 0x01, FLAG_Z = 0x02, FLAG_I = 0x04, FLAG_D = 0x08,
  FLAG_B = 0x10, FLAG_U = 0x20, FLAG_V = 0x40, FLAG_N = 0x80;

// Addressing modes
export const IMP = 0, ACC = 1, IMM = 2, ZP = 3, ZPX = 4, ZPY = 5, ABS = 6, ABX = 7,
  ABY = 8, IND = 9, IZX = 10, IZY = 11, REL = 12;
export const MODE_BYTES = [1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 2, 2, 2];

// Instruction kinds
const K_READ = 0, K_WRITE = 1, K_RMW = 2, K_IMPL = 3, K_BRANCH = 4, K_SPECIAL = 5, K_JAM = 6;

// prettier-ignore
const TABLE_SRC = [
  // 0x00
  'BRK imp','ORA izx','JAM imp','SLO izx','NOP zp','ORA zp','ASL zp','SLO zp',
  'PHP imp','ORA imm','ASL acc','ANC imm','NOP abs','ORA abs','ASL abs','SLO abs',
  'BPL rel','ORA izy','JAM imp','SLO izy','NOP zpx','ORA zpx','ASL zpx','SLO zpx',
  'CLC imp','ORA aby','NOP imp','SLO aby','NOP abx','ORA abx','ASL abx','SLO abx',
  // 0x20
  'JSR abs','AND izx','JAM imp','RLA izx','BIT zp','AND zp','ROL zp','RLA zp',
  'PLP imp','AND imm','ROL acc','ANC imm','BIT abs','AND abs','ROL abs','RLA abs',
  'BMI rel','AND izy','JAM imp','RLA izy','NOP zpx','AND zpx','ROL zpx','RLA zpx',
  'SEC imp','AND aby','NOP imp','RLA aby','NOP abx','AND abx','ROL abx','RLA abx',
  // 0x40
  'RTI imp','EOR izx','JAM imp','SRE izx','NOP zp','EOR zp','LSR zp','SRE zp',
  'PHA imp','EOR imm','LSR acc','ALR imm','JMP abs','EOR abs','LSR abs','SRE abs',
  'BVC rel','EOR izy','JAM imp','SRE izy','NOP zpx','EOR zpx','LSR zpx','SRE zpx',
  'CLI imp','EOR aby','NOP imp','SRE aby','NOP abx','EOR abx','LSR abx','SRE abx',
  // 0x60
  'RTS imp','ADC izx','JAM imp','RRA izx','NOP zp','ADC zp','ROR zp','RRA zp',
  'PLA imp','ADC imm','ROR acc','ARR imm','JMP ind','ADC abs','ROR abs','RRA abs',
  'BVS rel','ADC izy','JAM imp','RRA izy','NOP zpx','ADC zpx','ROR zpx','RRA zpx',
  'SEI imp','ADC aby','NOP imp','RRA aby','NOP abx','ADC abx','ROR abx','RRA abx',
  // 0x80
  'NOP imm','STA izx','NOP imm','SAX izx','STY zp','STA zp','STX zp','SAX zp',
  'DEY imp','NOP imm','TXA imp','ANE imm','STY abs','STA abs','STX abs','SAX abs',
  'BCC rel','STA izy','JAM imp','SHA izy','STY zpx','STA zpx','STX zpy','SAX zpy',
  'TYA imp','STA aby','TXS imp','TAS aby','SHY abx','STA abx','SHX aby','SHA aby',
  // 0xA0
  'LDY imm','LDA izx','LDX imm','LAX izx','LDY zp','LDA zp','LDX zp','LAX zp',
  'TAY imp','LDA imm','TAX imp','LXA imm','LDY abs','LDA abs','LDX abs','LAX abs',
  'BCS rel','LDA izy','JAM imp','LAX izy','LDY zpx','LDA zpx','LDX zpy','LAX zpy',
  'CLV imp','LDA aby','TSX imp','LAS aby','LDY abx','LDA abx','LDX aby','LAX aby',
  // 0xC0
  'CPY imm','CMP izx','NOP imm','DCP izx','CPY zp','CMP zp','DEC zp','DCP zp',
  'INY imp','CMP imm','DEX imp','SBX imm','CPY abs','CMP abs','DEC abs','DCP abs',
  'BNE rel','CMP izy','JAM imp','DCP izy','NOP zpx','CMP zpx','DEC zpx','DCP zpx',
  'CLD imp','CMP aby','NOP imp','DCP aby','NOP abx','CMP abx','DEC abx','DCP abx',
  // 0xE0
  'CPX imm','SBC izx','NOP imm','ISC izx','CPX zp','SBC zp','INC zp','ISC zp',
  'INX imp','SBC imm','NOP imp','SBC imm','CPX abs','SBC abs','INC abs','ISC abs',
  'BEQ rel','SBC izy','JAM imp','ISC izy','NOP zpx','SBC zpx','INC zpx','ISC zpx',
  'SED imp','SBC aby','NOP imp','ISC aby','NOP abx','SBC abx','INC abx','ISC abx',
];

const MODE_NAMES = { imp: IMP, acc: ACC, imm: IMM, zp: ZP, zpx: ZPX, zpy: ZPY, abs: ABS,
  abx: ABX, aby: ABY, ind: IND, izx: IZX, izy: IZY, rel: REL };

const READ_OPS = new Set(['LDA', 'LDX', 'LDY', 'ADC', 'SBC', 'AND', 'ORA', 'EOR', 'CMP', 'CPX',
  'CPY', 'BIT', 'LAX', 'NOP', 'ANC', 'ALR', 'ARR', 'ANE', 'LXA', 'SBX', 'LAS']);
const WRITE_OPS = new Set(['STA', 'STX', 'STY', 'SAX', 'SHA', 'SHX', 'SHY', 'TAS']);
const RMW_OPS = new Set(['ASL', 'LSR', 'ROL', 'ROR', 'INC', 'DEC', 'SLO', 'RLA', 'SRE', 'RRA',
  'DCP', 'ISC']);
const BRANCH_OPS = new Set(['BPL', 'BMI', 'BVC', 'BVS', 'BCC', 'BCS', 'BNE', 'BEQ']);
const SPECIAL_OPS = new Set(['BRK', 'JSR', 'RTS', 'RTI', 'JMP', 'PHA', 'PHP', 'PLA', 'PLP']);
export const DOCUMENTED = new Set(['ADC', 'AND', 'ASL', 'BCC', 'BCS', 'BEQ', 'BIT', 'BMI', 'BNE',
  'BPL', 'BRK', 'BVC', 'BVS', 'CLC', 'CLD', 'CLI', 'CLV', 'CMP', 'CPX', 'CPY', 'DEC', 'DEX', 'DEY',
  'EOR', 'INC', 'INX', 'INY', 'JMP', 'JSR', 'LDA', 'LDX', 'LDY', 'LSR', 'NOP', 'ORA', 'PHA', 'PHP',
  'PLA', 'PLP', 'ROL', 'ROR', 'RTI', 'RTS', 'SBC', 'SEC', 'SED', 'SEI', 'STA', 'STX', 'STY', 'TAX',
  'TAY', 'TSX', 'TXA', 'TXS', 'TYA']);

/** Opcode table: { name, mode, bytes, kind, documented } for each of the 256 opcodes. */
export const OPCODES = TABLE_SRC.map((src, op) => {
  const [name, modeName] = src.split(' ');
  const mode = MODE_NAMES[modeName];
  let kind;
  if (name === 'JAM') kind = K_JAM;
  else if (BRANCH_OPS.has(name)) kind = K_BRANCH;
  else if (SPECIAL_OPS.has(name)) kind = K_SPECIAL;
  else if (READ_OPS.has(name) && mode !== IMP) kind = K_READ;
  else if (WRITE_OPS.has(name)) kind = K_WRITE;
  else if (RMW_OPS.has(name)) kind = K_RMW;
  else kind = K_IMPL;
  const documented = DOCUMENTED.has(name) && !(name === 'NOP' && op !== 0xEA) && op !== 0xEB;
  return { op, name, mode, bytes: MODE_BYTES[mode], kind, documented };
});

// Numeric ids for fast dispatch
const NAME_IDS = {};
OPCODES.forEach((o) => { if (!(o.name in NAME_IDS)) NAME_IDS[o.name] = Object.keys(NAME_IDS).length; });
const OP_NAME_ID = Uint8Array.from(OPCODES, (o) => NAME_IDS[o.name]);
const OP_MODE = Uint8Array.from(OPCODES, (o) => o.mode);
const OP_KIND = Uint8Array.from(OPCODES, (o) => o.kind);
const N = NAME_IDS;

export class CPU6502 {
  /**
   * @param {{read:(addr:number)=>number, write:(addr:number,value:number)=>void}} bus
   */
  constructor(bus) {
    this.bus = bus;
    this.a = 0; this.x = 0; this.y = 0; this.s = 0xFD; this.pc = 0;
    // Flags kept separately for speed
    this.fC = 0; this.fZ = 0; this.fI = 1; this.fD = 0; this.fV = 0; this.fN = 0;
    this.cycles = 0;
    this.jammed = false;
    this.irqLine = false;      // level-sensitive, active when true
    this.nmiPending = false;   // edge latched
    this.lastOpcodePC = 0;
    this.instructions = 0;
  }

  get p() {
    return (this.fN << 7) | (this.fV << 6) | FLAG_U | FLAG_B | (this.fD << 3) | (this.fI << 2) |
      (this.fZ << 1) | this.fC;
  }
  set p(v) {
    this.fN = (v >> 7) & 1; this.fV = (v >> 6) & 1; this.fD = (v >> 3) & 1;
    this.fI = (v >> 2) & 1; this.fZ = (v >> 1) & 1; this.fC = v & 1;
  }

  rd(addr) { this.cycles++; return this.bus.read(addr & 0xFFFF); }
  wr(addr, v) { this.cycles++; this.bus.write(addr & 0xFFFF, v & 0xFF); }
  fetch() { const v = this.rd(this.pc); this.pc = (this.pc + 1) & 0xFFFF; return v; }
  push(v) { this.wr(0x100 | this.s, v); this.s = (this.s - 1) & 0xFF; }
  pull() { this.s = (this.s + 1) & 0xFF; return this.rd(0x100 | this.s); }
  setNZ(v) { this.fZ = v === 0 ? 1 : 0; this.fN = (v >> 7) & 1; }

  /** Hardware reset: 7 cycles, reads the vector at $FFFC. */
  reset() {
    this.jammed = false;
    this.rd(this.pc); this.rd(this.pc);
    this.rd(0x100 | this.s); this.s = (this.s - 1) & 0xFF;
    this.rd(0x100 | this.s); this.s = (this.s - 1) & 0xFF;
    this.rd(0x100 | this.s); this.s = (this.s - 1) & 0xFF;
    this.fI = 1;
    const lo = this.rd(0xFFFC), hi = this.rd(0xFFFD);
    this.pc = lo | (hi << 8);
  }

  interrupt(vector, isBrk) {
    if (isBrk) { this.fetch(); } else { this.rd(this.pc); this.rd(this.pc); }
    this.push(this.pc >> 8); this.push(this.pc & 0xFF);
    this.push(isBrk ? this.p : (this.p & ~FLAG_B));
    this.fI = 1;
    const lo = this.rd(vector), hi = this.rd(vector + 1);
    this.pc = lo | (hi << 8);
  }

  // ---- effective address calculation with the exact NMOS bus cycles ----
  eaRead(mode) {
    switch (mode) {
      case IMM: { const a = this.pc; this.pc = (this.pc + 1) & 0xFFFF; return a; }
      case ZP: return this.fetch();
      case ZPX: { const b = this.fetch(); this.rd(b); return (b + this.x) & 0xFF; }
      case ZPY: { const b = this.fetch(); this.rd(b); return (b + this.y) & 0xFF; }
      case ABS: { const lo = this.fetch(); return lo | (this.fetch() << 8); }
      case ABX: return this.indexedRead(this.x);
      case ABY: return this.indexedRead(this.y);
      case IZX: {
        const b = this.fetch(); this.rd(b);
        const p = (b + this.x) & 0xFF;
        const lo = this.rd(p); const hi = this.rd((p + 1) & 0xFF);
        return lo | (hi << 8);
      }
      case IZY: {
        const b = this.fetch();
        const lo = this.rd(b); const hi = this.rd((b + 1) & 0xFF);
        const base = lo | (hi << 8); const a = (base + this.y) & 0xFFFF;
        if ((base ^ a) & 0xFF00) this.rd((base & 0xFF00) | (a & 0xFF));
        return a;
      }
    }
    throw new Error('bad read mode ' + mode);
  }
  indexedRead(idx) {
    const lo = this.fetch(); const hi = this.fetch();
    const base = lo | (hi << 8); const a = (base + idx) & 0xFFFF;
    if ((base ^ a) & 0xFF00) this.rd((base & 0xFF00) | (a & 0xFF));
    return a;
  }
  // For stores and RMW the indexed modes always do the "unfixed" dummy read.
  // Returns the address; this.ehi holds the unfixed high byte+1 for SHx ops.
  eaWrite(mode) {
    switch (mode) {
      case ZP: return this.fetch();
      case ZPX: { const b = this.fetch(); this.rd(b); return (b + this.x) & 0xFF; }
      case ZPY: { const b = this.fetch(); this.rd(b); return (b + this.y) & 0xFF; }
      case ABS: { const lo = this.fetch(); return lo | (this.fetch() << 8); }
      case ABX: return this.indexedWrite(this.fetch(), this.fetch(), this.x);
      case ABY: return this.indexedWrite(this.fetch(), this.fetch(), this.y);
      case IZX: {
        const b = this.fetch(); this.rd(b);
        const p = (b + this.x) & 0xFF;
        const lo = this.rd(p); const hi = this.rd((p + 1) & 0xFF);
        return lo | (hi << 8);
      }
      case IZY: {
        const b = this.fetch();
        const lo = this.rd(b); const hi = this.rd((b + 1) & 0xFF);
        return this.indexedWrite(lo, hi, this.y);
      }
    }
    throw new Error('bad write mode ' + mode);
  }
  indexedWrite(lo, hi, idx) {
    const base = lo | (hi << 8); const a = (base + idx) & 0xFFFF;
    this.rd((base & 0xFF00) | (a & 0xFF));
    this.ehi = (hi + 1) & 0xFF;
    this.crossed = ((base ^ a) & 0xFF00) !== 0;
    return a;
  }

  // ---- ALU ----
  adc(v) {
    const c = this.fC;
    if (!this.fD) {
      const t = this.a + v + c;
      this.fV = (~(this.a ^ v) & (this.a ^ t) & 0x80) ? 1 : 0;
      this.fC = t > 0xFF ? 1 : 0;
      this.a = t & 0xFF; this.setNZ(this.a);
      return;
    }
    // NMOS decimal mode (N, V from the intermediate result, Z from binary sum)
    let t = (this.a & 0x0F) + (v & 0x0F) + c;
    if (t > 0x09) t += 0x06;
    if (t <= 0x0F) t = (t & 0x0F) + (this.a & 0xF0) + (v & 0xF0);
    else t = (t & 0x0F) + (this.a & 0xF0) + (v & 0xF0) + 0x10;
    this.fZ = ((this.a + v + c) & 0xFF) === 0 ? 1 : 0;
    this.fN = (t >> 7) & 1;
    this.fV = (((this.a ^ t) & 0x80) && !((this.a ^ v) & 0x80)) ? 1 : 0;
    if ((t & 0x1F0) > 0x90) t += 0x60;
    this.fC = (t & 0xFF0) > 0xF0 ? 1 : 0;
    this.a = t & 0xFF;
  }
  sbc(v) {
    const borrow = this.fC ? 0 : 1;
    const t = this.a - v - borrow;
    if (this.fD) {
      let ta = (this.a & 0x0F) - (v & 0x0F) - borrow;
      if (ta & 0x10) ta = ((ta - 6) & 0x0F) | ((this.a & 0xF0) - (v & 0xF0) - 0x10);
      else ta = (ta & 0x0F) | ((this.a & 0xF0) - (v & 0xF0));
      if (ta & 0x100) ta -= 0x60;
      this.fC = t >= 0 ? 1 : 0;
      this.setNZ(t & 0xFF);
      this.fV = (((this.a ^ t) & 0x80) && ((this.a ^ v) & 0x80)) ? 1 : 0;
      this.a = ta & 0xFF;
      return;
    }
    this.fC = t >= 0 ? 1 : 0;
    this.fV = (((this.a ^ t) & 0x80) && ((this.a ^ v) & 0x80)) ? 1 : 0;
    this.a = t & 0xFF; this.setNZ(this.a);
  }
  cmp(r, v) { const t = r - v; this.fC = t >= 0 ? 1 : 0; this.setNZ(t & 0xFF); }

  rmw(id, v) {
    switch (id) {
      case N.ASL: this.fC = v >> 7; v = (v << 1) & 0xFF; this.setNZ(v); return v;
      case N.LSR: this.fC = v & 1; v >>= 1; this.setNZ(v); return v;
      case N.ROL: { const c = this.fC; this.fC = v >> 7; v = ((v << 1) | c) & 0xFF; this.setNZ(v); return v; }
      case N.ROR: { const c = this.fC; this.fC = v & 1; v = (v >> 1) | (c << 7); this.setNZ(v); return v; }
      case N.INC: v = (v + 1) & 0xFF; this.setNZ(v); return v;
      case N.DEC: v = (v - 1) & 0xFF; this.setNZ(v); return v;
      case N.SLO: this.fC = v >> 7; v = (v << 1) & 0xFF; this.a |= v; this.setNZ(this.a); return v;
      case N.RLA: { const c = this.fC; this.fC = v >> 7; v = ((v << 1) | c) & 0xFF; this.a &= v; this.setNZ(this.a); return v; }
      case N.SRE: this.fC = v & 1; v >>= 1; this.a ^= v; this.setNZ(this.a); return v;
      case N.RRA: { const c = this.fC; this.fC = v & 1; v = (v >> 1) | (c << 7); this.adc(v); return v; }
      case N.DCP: v = (v - 1) & 0xFF; this.cmp(this.a, v); return v;
      case N.ISC: v = (v + 1) & 0xFF; this.sbc(v); return v;
    }
    throw new Error('bad rmw');
  }

  read(id, v) {
    switch (id) {
      case N.LDA: this.a = v; this.setNZ(v); return;
      case N.LDX: this.x = v; this.setNZ(v); return;
      case N.LDY: this.y = v; this.setNZ(v); return;
      case N.LAX: this.a = this.x = v; this.setNZ(v); return;
      case N.ADC: this.adc(v); return;
      case N.SBC: this.sbc(v); return;
      case N.AND: this.a &= v; this.setNZ(this.a); return;
      case N.ORA: this.a |= v; this.setNZ(this.a); return;
      case N.EOR: this.a ^= v; this.setNZ(this.a); return;
      case N.CMP: this.cmp(this.a, v); return;
      case N.CPX: this.cmp(this.x, v); return;
      case N.CPY: this.cmp(this.y, v); return;
      case N.BIT: this.fZ = (this.a & v) === 0 ? 1 : 0; this.fN = (v >> 7) & 1; this.fV = (v >> 6) & 1; return;
      case N.NOP: return;
      case N.ANC: this.a &= v; this.setNZ(this.a); this.fC = this.fN; return;
      case N.ALR: this.a &= v; this.fC = this.a & 1; this.a >>= 1; this.setNZ(this.a); return;
      case N.ARR: {
        const t = this.a & v;
        if (!this.fD) {
          this.a = (t >> 1) | (this.fC << 7); this.setNZ(this.a);
          this.fC = (this.a >> 6) & 1; this.fV = ((this.a >> 6) ^ (this.a >> 5)) & 1;
        } else {
          let r = (t >> 1) | (this.fC << 7);
          this.fN = this.fC; this.fZ = r === 0 ? 1 : 0;
          this.fV = ((t ^ r) >> 6) & 1;
          if (((t & 0x0F) + (t & 0x01)) > 5) r = (r & 0xF0) | ((r + 6) & 0x0F);
          if (((t & 0xF0) + (t & 0x10)) > 0x50) { r = (r + 0x60) & 0xFF; this.fC = 1; } else this.fC = 0;
          this.a = r;
        }
        return;
      }
      case N.ANE: this.a = (this.a | 0xEE) & this.x & v; this.setNZ(this.a); return;
      case N.LXA: this.a = this.x = (this.a | 0xEE) & v; this.setNZ(this.a); return;
      case N.SBX: { const t = (this.a & this.x) - v; this.fC = t >= 0 ? 1 : 0; this.x = t & 0xFF; this.setNZ(this.x); return; }
      case N.LAS: this.a = this.x = this.s = v & this.s; this.setNZ(this.a); return;
    }
    throw new Error('bad read op');
  }

  store(id, a, mode) {
    switch (id) {
      case N.STA: this.wr(a, this.a); return;
      case N.STX: this.wr(a, this.x); return;
      case N.STY: this.wr(a, this.y); return;
      case N.SAX: this.wr(a, this.a & this.x); return;
      // Unstable "SHx" stores: value ANDed with (high byte of base + 1); on a
      // page crossing the high byte of the target is replaced by the value.
      case N.SHA: return this.shStore(a, this.a & this.x);
      case N.SHX: return this.shStore(a, this.x);
      case N.SHY: return this.shStore(a, this.y);
      case N.TAS: this.s = this.a & this.x; return this.shStore(a, this.s);
    }
    throw new Error('bad store');
  }
  shStore(a, r) {
    const v = r & this.ehi;
    if (this.crossed) a = (v << 8) | (a & 0xFF);
    this.wr(a, v);
  }

  implied(id) {
    switch (id) {
      case N.CLC: this.fC = 0; return; case N.SEC: this.fC = 1; return;
      case N.CLI: this.fI = 0; return; case N.SEI: this.fI = 1; return;
      case N.CLD: this.fD = 0; return; case N.SED: this.fD = 1; return;
      case N.CLV: this.fV = 0; return;
      case N.TAX: this.x = this.a; this.setNZ(this.x); return;
      case N.TAY: this.y = this.a; this.setNZ(this.y); return;
      case N.TXA: this.a = this.x; this.setNZ(this.a); return;
      case N.TYA: this.a = this.y; this.setNZ(this.a); return;
      case N.TSX: this.x = this.s; this.setNZ(this.x); return;
      case N.TXS: this.s = this.x; return;
      case N.INX: this.x = (this.x + 1) & 0xFF; this.setNZ(this.x); return;
      case N.INY: this.y = (this.y + 1) & 0xFF; this.setNZ(this.y); return;
      case N.DEX: this.x = (this.x - 1) & 0xFF; this.setNZ(this.x); return;
      case N.DEY: this.y = (this.y - 1) & 0xFF; this.setNZ(this.y); return;
      case N.NOP: return;
    }
    throw new Error('bad implied op');
  }

  branchTaken(id) {
    switch (id) {
      case N.BPL: return !this.fN; case N.BMI: return !!this.fN;
      case N.BVC: return !this.fV; case N.BVS: return !!this.fV;
      case N.BCC: return !this.fC; case N.BCS: return !!this.fC;
      case N.BNE: return !this.fZ; case N.BEQ: return !!this.fZ;
    }
    return false;
  }

  /** Execute one instruction (or one stalled cycle when jammed). */
  step() {
    if (this.jammed) { this.rd(0xFFFF); return; }
    if (this.nmiPending) { this.nmiPending = false; this.interrupt(0xFFFA, false); return; }
    if (this.irqLine && !this.fI) { this.interrupt(0xFFFE, false); return; }
    this.lastOpcodePC = this.pc;
    this.instructions++;
    const op = this.fetch();
    const id = OP_NAME_ID[op], mode = OP_MODE[op];
    switch (OP_KIND[op]) {
      case K_READ: { const a = this.eaRead(mode); this.read(id, this.rd(a)); return; }
      case K_WRITE: { const a = this.eaWrite(mode); this.store(id, a, mode); return; }
      case K_RMW: {
        if (mode === ACC) { this.rd(this.pc); this.a = this.rmw(id, this.a); return; }
        const a = this.eaWrite(mode);
        const v = this.rd(a);
        this.wr(a, v);            // NMOS dummy write of the unmodified value
        this.wr(a, this.rmw(id, v));
        return;
      }
      case K_IMPL: this.rd(this.pc); this.implied(id); return;
      case K_BRANCH: {
        const off = this.fetch();
        if (!this.branchTaken(id)) return;
        this.rd(this.pc);
        const target = (this.pc + ((off ^ 0x80) - 0x80)) & 0xFFFF;
        if ((target ^ this.pc) & 0xFF00) this.rd((this.pc & 0xFF00) | (target & 0xFF));
        this.pc = target;
        return;
      }
      case K_SPECIAL: this.special(id, mode); return;
      case K_JAM: this.rd(this.pc); this.jammed = true; this.pc = (this.pc - 1) & 0xFFFF; return;
    }
  }

  special(id, mode) {
    switch (id) {
      case N.BRK: this.interrupt(0xFFFE, true); return;
      case N.JSR: {
        const lo = this.fetch();
        this.rd(0x100 | this.s);
        this.push(this.pc >> 8); this.push(this.pc & 0xFF);
        const hi = this.fetch();
        this.pc = lo | (hi << 8);
        return;
      }
      case N.RTS: {
        this.rd(this.pc); this.rd(0x100 | this.s);
        const lo = this.pull(); const hi = this.pull();
        this.pc = lo | (hi << 8);
        this.rd(this.pc); this.pc = (this.pc + 1) & 0xFFFF;
        return;
      }
      case N.RTI: {
        this.rd(this.pc); this.rd(0x100 | this.s);
        this.p = this.pull();
        const lo = this.pull(); const hi = this.pull();
        this.pc = lo | (hi << 8);
        return;
      }
      case N.JMP: {
        const lo = this.fetch(); const hi = this.fetch();
        if (mode === ABS) { this.pc = lo | (hi << 8); return; }
        // JMP (ind) with the NMOS page-wrap bug
        const ptr = lo | (hi << 8);
        const tl = this.rd(ptr); const th = this.rd((ptr & 0xFF00) | ((ptr + 1) & 0xFF));
        this.pc = tl | (th << 8);
        return;
      }
      case N.PHA: this.rd(this.pc); this.push(this.a); return;
      case N.PHP: this.rd(this.pc); this.push(this.p); return;
      case N.PLA: this.rd(this.pc); this.rd(0x100 | this.s); this.a = this.pull(); this.setNZ(this.a); return;
      case N.PLP: this.rd(this.pc); this.rd(0x100 | this.s); this.p = this.pull(); return;
    }
  }

  getState() {
    return { a: this.a, x: this.x, y: this.y, s: this.s, pc: this.pc, p: this.p, cycles: this.cycles,
      jammed: this.jammed };
  }
}
