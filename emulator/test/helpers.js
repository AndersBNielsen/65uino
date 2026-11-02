import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { Board65uino } from '../site/js/core/board.js';

export const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
export const rom = (name) => fs.readFileSync(path.join(ROOT, 'site/roms', name));

export function bootBoard(romName = '65uino-bios.bin') {
  const b = new Board65uino();
  b.loadRom(rom(romName));
  b.powerOn();
  return b;
}

/** Run the board for `ms` milliseconds of emulated time. */
export const runMs = (b, ms) => b.run(Math.round((ms * b.clockHz) / 1000));

export function press(b, holdMs, afterMs = 400) {
  b.setButton(true); runMs(b, holdMs);
  b.setButton(false); runMs(b, afterMs);
}

/** Framebuffer (what the glass shows) as 64 lines of '#' and '.'. */
export function fbAscii(b) {
  const fb = b.framebuffer();
  const lines = [];
  for (let y = 0; y < 64; y++) {
    let s = '';
    for (let x = 0; x < 128; x++) s += fb[y * 128 + x] ? '#' : '.';
    lines.push(s);
  }
  return lines.join('\n') + '\n';
}

/**
 * Compare against test/snapshots/<name>.txt. Set UPDATE_SNAPSHOTS=1 to
 * (re)write the snapshot files.
 */
export function matchSnapshot(name, actual) {
  const file = path.join(ROOT, 'test/snapshots', name + '.txt');
  if (process.env.UPDATE_SNAPSHOTS || !fs.existsSync(file)) {
    fs.mkdirSync(path.dirname(file), { recursive: true });
    fs.writeFileSync(file, actual);
    if (!process.env.UPDATE_SNAPSHOTS) console.log(`  wrote new snapshot ${name}`);
    return;
  }
  assert.equal(actual, fs.readFileSync(file, 'utf8'), `framebuffer differs from snapshot ${name}`);
}

/** Read text from the OLED by matching 8x8 cells against the BIOS font. */
export function readScreenText(b) {
  const romBytes = b.rom;
  // locate the 5 font column tables via the labels file
  const labels = fs.readFileSync(path.join(ROOT, 'site/roms/65uino-bios.labels.txt'), 'utf8');
  const addr = (n) => parseInt(new RegExp(`al ([0-9A-F]+) \\.${n}\\b`).exec(labels)[1], 16) & 0xFFF;
  const cols = [1, 2, 3, 4, 5].map((i) => addr('fontc' + i));
  const glyphs = new Map();
  for (let c = 0x20; c < 0x7F; c++) {
    const g = [0, ...cols.map((t) => romBytes[t + c - 0x20]), 0, 0].join(',');
    if (!glyphs.has(g)) glyphs.set(g, String.fromCharCode(c));
    const inv = [0, ...cols.map((t) => romBytes[t + c - 0x20]), 0, 0].map((v, i) => (i === 0 ? 0xFF : v ^ 0xFF));
    inv[6] = 0xFF; inv[7] = 0xFF; inv[0] = 0xFF;
    if (!glyphs.has(inv.join(','))) glyphs.set(inv.join(','), String.fromCharCode(c));
  }
  const lines = [];
  for (let p = 0; p < 8; p++) {
    let s = '';
    for (let cx = 0; cx < 16; cx++) {
      const cell = [];
      for (let i = 0; i < 8; i++) cell.push(b.oled.gram[p * 128 + cx * 8 + i]);
      s += glyphs.get(cell.join(',')) ?? '?';
    }
    lines.push(s.trimEnd());
  }
  return lines;
}
