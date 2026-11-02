// CPU tests: Klaus Dormann's 6502 functional test, Bruce Clark's decimal mode
// test (all flags), and bus-cycle level timing checks.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { CPU6502, OPCODES } from '../site/js/core/cpu6502.js';
import { ROOT } from './helpers.js';

// Klaus Dormann's suite is GPL-3.0, so it is downloaded at test time rather
// than redistributed. Pinned to a commit and verified by SHA-256.
const DORMANN_COMMIT = '7954e2dbb49c469ea286070bf46cdd71aeb29e4b';
const DORMANN_URL = `https://raw.githubusercontent.com/Klaus2m5/6502_65C02_functional_tests/${DORMANN_COMMIT}/bin_files/6502_functional_test.bin`;
const DORMANN_SHA256 = 'fa12bfc761e6f9057e4cc01a665a7b800ff01ae91f598af1e39a1201d01953fd';

async function dormannBinary() {
  const file = path.join(ROOT, 'test/.cache/6502_functional_test.bin');
  if (!fs.existsSync(file)) {
    const res = await fetch(DORMANN_URL);
    if (!res.ok) throw new Error('download failed: ' + res.status);
    fs.mkdirSync(path.dirname(file), { recursive: true });
    fs.writeFileSync(file, Buffer.from(await res.arrayBuffer()));
  }
  const data = fs.readFileSync(file);
  const { createHash } = await import('node:crypto');
  assert.equal(createHash('sha256').update(data).digest('hex'), DORMANN_SHA256, 'unexpected test binary');
  return new Uint8Array(data);
}

function flatCpu(mem) {
  return new CPU6502({ read: (a) => mem[a], write: (a, v) => { mem[a] = v; } });
}

function runUntilTrap(cpu, maxCycles) {
  for (;;) {
    const pc = cpu.pc;
    cpu.step();
    if (cpu.pc === pc) return pc;
    if (cpu.cycles > maxCycles) throw new Error('timeout at ' + cpu.pc.toString(16));
  }
}

test('Klaus Dormann 6502_functional_test passes', async (t) => {
  let mem;
  try { mem = await dormannBinary(); } catch (e) { t.skip('cannot fetch Dormann test binary: ' + e.message); return; }
  const cpu = flatCpu(mem);
  cpu.pc = 0x0400;
  const trap = runUntilTrap(cpu, 200e6);
  assert.equal(trap.toString(16), '3469', `trapped at $${trap.toString(16)} in test ${mem[0x200].toString(16)}`);
  // 96,241,364 cycles to reach the success trap + 3 for executing the trap once
  assert.equal(cpu.cycles, 96241367, 'total cycle count');
});

test('Bruce Clark decimal mode test passes with N, V, Z and C checked', () => {
  const mem = new Uint8Array(65536);
  mem.set(fs.readFileSync(path.join(ROOT, 'test/fixtures/decimal_test.bin')), 0x200);
  const cpu = flatCpu(mem);
  cpu.pc = 0x200;
  runUntilTrap(cpu, 200e6);
  assert.equal(mem[0x0B], 0, `ERROR flag set at N1=${mem[0]} N2=${mem[1]} Y=${cpu.y}`);
});

/** Assemble bytes at $0200 and return per-instruction cycle counts + bus log. */
function timing(bytes, setup = () => {}) {
  const mem = new Uint8Array(65536);
  mem.set(bytes, 0x200);
  const log = [];
  const cpu = new CPU6502({
    read: (a) => { log.push(['R', a]); return mem[a]; },
    write: (a, v) => { log.push(['W', a, v]); mem[a] = v; },
  });
  cpu.pc = 0x200;
  setup(cpu, mem);
  return { cpu, mem, log, step() { const c0 = cpu.cycles; log.length = 0; cpu.step(); return cpu.cycles - c0; } };
}

test('documented opcode set is complete (151 opcodes)', () => {
  assert.equal(OPCODES.filter((o) => o.documented).length, 151);
});

test('cycle counts incl. page crossing and branches', () => {
  // LDA abs,X without and with page crossing
  let t = timing([0xBD, 0x00, 0x30, 0xBD, 0xFF, 0x30], (c) => { c.x = 1; });
  assert.equal(t.step(), 4);
  assert.equal(t.step(), 5);
  // STA abs,X is always 5
  t = timing([0x9D, 0x00, 0x30], (c) => { c.x = 1; });
  assert.equal(t.step(), 5);
  // LDA (zp),Y with page crossing = 6
  t = timing([0xB1, 0x10], (c, m) => { c.y = 0xFF; m[0x10] = 0x80; m[0x11] = 0x30; });
  assert.equal(t.step(), 6);
  // Branches: not taken 2, taken 3, taken across a page 4
  t = timing([0xD0, 0x02], (c) => { c.fZ = 1; }); assert.equal(t.step(), 2);
  t = timing([0xD0, 0x02], (c) => { c.fZ = 0; }); assert.equal(t.step(), 3);
  t = timing([0xD0, 0x80], (c) => { c.fZ = 0; }); assert.equal(t.step(), 4);
  // JSR 6, RTS 6, BRK 7, JMP (ind) 5
  t = timing([0x20, 0x00, 0x03]); assert.equal(t.step(), 6);
  t = timing([0x00]); assert.equal(t.step(), 7);
  t = timing([0x6C, 0x00, 0x03]); assert.equal(t.step(), 5);
  // INC zp,X = 6, INC abs,X = 7
  t = timing([0xF6, 0x10]); assert.equal(t.step(), 6);
  t = timing([0xFE, 0x00, 0x30]); assert.equal(t.step(), 7);
});

test('read-modify-write does the NMOS double write (old value, then new)', () => {
  // INC $83 - this is how the BIOS toggles SCL via DDRB
  const t = timing([0xE6, 0x83], (c, m) => { m[0x83] = 0x80; });
  t.step();
  const writes = t.log.filter((e) => e[0] === 'W');
  assert.deepEqual(writes, [['W', 0x83, 0x80], ['W', 0x83, 0x81]]);
});

test('JMP ($xxFF) page-wrap bug and JAM halts the CPU', () => {
  const t = timing([0x6C, 0xFF, 0x30, 0x02], (c, m) => { m[0x30FF] = 0x34; m[0x3000] = 0x12; m[0x3100] = 0x99; });
  t.step();
  assert.equal(t.cpu.pc, 0x1234);
  const j = timing([0xF2]);
  j.step(); assert.equal(j.cpu.jammed, true);
  const pc = j.cpu.pc; j.step(); j.step();
  assert.equal(j.cpu.pc, pc);
});
