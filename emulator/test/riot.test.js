// 6532 RIOT: timer prescalers, wrap/flag behaviour, PA7 edge detect, ports,
// and the 65uino address decoding/mirroring.
import test from 'node:test';
import assert from 'node:assert/strict';
import { RIOT6532, TIMER_FLAG, PA7_FLAG } from '../site/js/core/riot6532.js';
import { Board65uino } from '../site/js/core/board.js';

function riotAt(ext = { a: 0xFF, b: 0xFF }) {
  const clock = { now: 0 };
  const r = new RIOT6532(() => clock.now, { readA: () => ext.a, readB: () => ext.b });
  return { r, clock, ext };
}
const TIMER_WRITE = [0x94, 0x95, 0x96, 0x97]; // /1 /8 /64 /1024, IRQ disabled

for (const [i, div] of [1, 8, 64, 1024].entries()) {
  test(`timer /${div}: counts N..0 then wraps to $FF and sets the flag`, () => {
    const N = 5;
    const at = (n) => { const { r, clock } = riotAt(); r.write(TIMER_WRITE[i], N); clock.now = n; return r; };
    assert.equal(at(1).read(0x84), N - 1, 'decremented one cycle after the write');
    if (div > 1) assert.equal(at(div).read(0x84), N - 1, 'still N-1 until a full prescale period');
    assert.equal(at(div + 1).read(0x84), N - 2);
    assert.equal(at(N * div).read(0x84), 0, 'reaches zero after N periods');
    let r = at(N * div);
    assert.equal(r.read(0x85) & TIMER_FLAG, 0, 'no flag while showing 0');
    r = at(N * div + 1);
    assert.equal(r.read(0x85) & TIMER_FLAG, TIMER_FLAG, 'flag set on wrap');
    r = at(N * div + 1);
    assert.equal(r.peek(0x84), 0xFF, 'wraps to $FF');
    r = at(N * div + 11);
    assert.equal(r.peek(0x84), 0xFF - 10, 'after the wrap it counts once per cycle');
  });
}

test('reading the timer clears the flag and restores the prescaler', () => {
  const { r, clock } = riotAt();
  r.write(0x96, 2); // /64
  clock.now = 2 * 64 + 1 + 20;      // 20 cycles after the wrap
  assert.equal(r.read(0x85) & TIMER_FLAG, TIMER_FLAG);
  assert.equal(r.read(0x84), 0xFF - 20);
  assert.equal(r.read(0x85) & TIMER_FLAG, 0, 'flag cleared by the timer read');
  clock.now += 64;
  assert.equal(r.read(0x84), 0xFF - 21, 'back to one tick per 64 cycles');
});

test('timer interrupt output follows the enable bit (A3)', () => {
  const { r, clock } = riotAt();
  r.write(0x1C | 0x80, 3); // WTD1EI: /1, IRQ enabled
  clock.now = 3;
  assert.equal(r.irq, false);
  clock.now = 4;
  assert.equal(r.irq, true);
  r.read(0x8C); // a read in the very cycle of the wrap does not clear the flag
  assert.equal(r.irq, true);
  clock.now = 5;
  r.read(0x8C); // a later read clears it
  assert.equal(r.irq, false);
  r.write(0x94, 1); clock.now += 3; // disabled
  assert.equal(r.irq, false);
});

test('PA7 edge detect: negative and positive, flag cleared by reading the flag register', () => {
  const ext = { a: 0xFF, b: 0xFF };
  const { r } = riotAt(ext);
  r.reset();
  r.write(0x84, 0);            // negative edge, IRQ off
  ext.a = 0x7F; r.checkPA7();  // falling edge
  assert.equal(r.read(0x85) & PA7_FLAG, PA7_FLAG);
  assert.equal(r.read(0x85) & PA7_FLAG, 0, 'cleared by the read');
  ext.a = 0xFF; r.checkPA7();  // rising edge ignored
  assert.equal(r.read(0x85) & PA7_FLAG, 0);
  r.write(0x87, 0);            // positive edge, IRQ on
  ext.a = 0x7F; r.checkPA7();
  ext.a = 0xFF; r.checkPA7();
  assert.equal(r.irq, true);
  assert.equal(r.read(0x85) & PA7_FLAG, PA7_FLAG);
  assert.equal(r.irq, false);
});

test('ports: A reads pins, B reads the output latch for outputs', () => {
  const ext = { a: 0x0F, b: 0x00 };
  const { r } = riotAt(ext);
  r.write(0x81, 0xF0); r.write(0x80, 0x5A);
  assert.equal(r.read(0x80), 0x5F);
  r.write(0x83, 0xF0); r.write(0x82, 0xA5);
  assert.equal(r.read(0x82), 0xA0);
  assert.equal(r.read(0x81), 0xF0);
  assert.equal(r.read(0x83), 0xF0);
});

test('65uino decode: 13-bit mirroring, RAM mirrors (stack page), ROM window, open bus', () => {
  const b = new Board65uino();
  const rom = new Uint8Array(4096); rom[0] = 0x11; rom[0xFFC] = 0x22;
  b.loadRom(rom);
  assert.equal(b.read(0x1000), 0x11);
  assert.equal(b.read(0xF000), 0x11, 'ROM mirrors at $F000');
  assert.equal(b.read(0xFFFC), 0x22, 'reset vector visible at $FFFC');
  b.write(0x0010, 0x42);
  for (const a of [0x0110, 0x0210, 0x0310, 0x2010, 0xE010]) assert.equal(b.read(a), 0x42, a.toString(16));
  b.write(0x017F, 0x99);
  assert.equal(b.read(0x007F), 0x99, 'stack at $017F is RIOT RAM $7F');
  b.write(0x0181, 0x0F);
  assert.equal(b.riot.ddra, 0x0F, 'I/O mirrored at $0180');
  b.read(0x1000);
  assert.equal(b.read(0x0400), 0x11, 'J6 chip-select area returns the last bus value');
});
