// 6502 disassembler using the CPU's opcode table.
import { OPCODES, IMP, ACC, IMM, ZP, ZPX, ZPY, ABS, ABX, ABY, IND, IZX, IZY, REL } from './cpu6502.js';

const hex2 = (v) => v.toString(16).toUpperCase().padStart(2, '0');
const hex4 = (v) => v.toString(16).toUpperCase().padStart(4, '0');
export { hex2, hex4 };

/**
 * Disassemble one instruction.
 * @param {(addr:number)=>number} peek side-effect-free memory read
 * @param {number} pc
 * @param {Map<number,string>} [labels] address -> name (13-bit addresses for the 6507)
 */
export function disassemble(peek, pc, labels) {
  const op = peek(pc);
  const info = OPCODES[op];
  const b1 = peek((pc + 1) & 0xFFFF), b2 = peek((pc + 2) & 0xFFFF);
  const w = b1 | (b2 << 8);
  const lab = (a, digits) => {
    const n = labels && (labels.get(a) ?? labels.get(a & 0x1FFF));
    return n ? n : '$' + (digits === 2 ? hex2(a) : hex4(a));
  };
  let operand = '';
  switch (info.mode) {
    case IMP: break;
    case ACC: operand = 'A'; break;
    case IMM: operand = '#$' + hex2(b1); break;
    case ZP: operand = lab(b1, 2); break;
    case ZPX: operand = lab(b1, 2) + ',X'; break;
    case ZPY: operand = lab(b1, 2) + ',Y'; break;
    case ABS: operand = lab(w, 4); break;
    case ABX: operand = lab(w, 4) + ',X'; break;
    case ABY: operand = lab(w, 4) + ',Y'; break;
    case IND: operand = '(' + lab(w, 4) + ')'; break;
    case IZX: operand = '(' + lab(b1, 2) + ',X)'; break;
    case IZY: operand = '(' + lab(b1, 2) + '),Y'; break;
    case REL: operand = lab((pc + 2 + ((b1 ^ 0x80) - 0x80)) & 0xFFFF, 4); break;
  }
  const bytes = [op, b1, b2].slice(0, info.bytes);
  const text = (info.documented ? info.name : info.name.toLowerCase() + '*') + (operand ? ' ' + operand : '');
  return { pc, bytes, len: info.bytes, text, label: labels?.get(pc & 0x1FFF) || labels?.get(pc) || null };
}

/** Parse an ld65 -Ln label file ("al 001F2D .name"). */
export function parseLd65Labels(text) {
  const m = new Map();
  for (const line of text.split(/\r?\n/)) {
    const r = /^al\s+([0-9A-Fa-f]+)\s+\.(\S+)/.exec(line);
    if (!r) continue;
    const addr = parseInt(r[1], 16);
    if (addr <= 0x1FFF && !m.has(addr)) m.set(addr, r[2]);
  }
  return m;
}

/** Names for the 65uino's RIOT registers (as used in abn6507rom.s). */
export const RIOT_LABELS = new Map([
  [0x80, 'DRA'], [0x81, 'DDRA'], [0x82, 'DRB'], [0x83, 'DDRB'], [0x84, 'READTDI'], [0x85, 'RRIFR'],
  [0x86, 'A7NEEI'], [0x87, 'A7PEEI'], [0x8C, 'READTEI'], [0x94, 'WTD1DI'], [0x95, 'WTD8DI'],
  [0x96, 'WTD64DI'], [0x97, 'WTD1KDI'], [0x9C, 'WTD1EI'], [0x9D, 'WTD8EI'], [0x9E, 'WTD64EI'],
  [0x9F, 'WTD1KEI'],
]);
