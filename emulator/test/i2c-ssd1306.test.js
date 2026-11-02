// I2C slave decoder -> SSD1306 / 24C64 models, driven bit by bit like the
// 65uino BIOS does (open-drain lines, data changes while SCL is low).
import test from 'node:test';
import assert from 'node:assert/strict';
import { I2CBus } from '../site/js/core/i2c.js';
import { SSD1306 } from '../site/js/core/ssd1306.js';
import { EEPROM24C64 } from '../site/js/core/i2cdevices.js';

class Master {
  constructor(bus) { this.bus = bus; this.scl = 1; this.sda = 1; }
  set(scl, sda) { this.scl = scl; this.sda = sda; this.bus.setMaster(scl, sda); }
  start() { this.set(1, 1); this.set(1, 0); this.set(0, 0); }
  stop() { this.set(0, 0); this.set(1, 0); this.set(1, 1); }
  /** returns true on ACK */
  byte(v) {
    for (let i = 7; i >= 0; i--) { this.set(0, (v >> i) & 1); this.set(1, (v >> i) & 1); this.set(0, (v >> i) & 1); }
    this.set(0, 1); this.set(1, 1);
    const ack = this.bus.sda === 0;
    this.set(0, 1);
    return ack;
  }
  readByte(ack) {
    let v = 0;
    this.set(0, 1);
    for (let i = 0; i < 8; i++) { this.set(1, 1); v = (v << 1) | this.bus.sda; this.set(0, 1); }
    this.set(0, ack ? 0 : 1); this.set(1, ack ? 0 : 1); this.set(0, ack ? 0 : 1); this.set(0, 1);
    return v;
  }
  cmds(addr, ...bytes) { this.start(); const r = [this.byte(addr << 1), this.byte(0x00), ...bytes.map((b) => this.byte(b))]; this.stop(); return r; }
  data(addr, ...bytes) { this.start(); this.byte(addr << 1); this.byte(0x40); bytes.forEach((b) => this.byte(b)); this.stop(); }
}

function setup() {
  const bus = new I2CBus();
  const oled = bus.attach(new SSD1306(0x3C));
  const eeprom = bus.attach(new EEPROM24C64(0x50));
  return { bus, oled, eeprom, m: new Master(bus) };
}

test('ACK for present devices, NACK for absent ones', () => {
  const { m } = setup();
  m.start(); assert.equal(m.byte(0x3C << 1), true); m.stop();
  m.start(); assert.equal(m.byte(0x60 << 1), false); m.stop();
  m.start(); assert.equal(m.byte(0x50 << 1), true); m.stop();
});

test('horizontal addressing with column/page window wraps correctly', () => {
  const { m, oled } = setup();
  m.cmds(0x3C, 0x20, 0x00, 0x21, 10, 12, 0x22, 2, 3);
  m.data(0x3C, 1, 2, 3, 4, 5, 6, 7);
  // 7th byte wrapped back to page 2, column 10
  assert.deepEqual([...oled.gram.subarray(2 * 128 + 10, 2 * 128 + 14)], [7, 2, 3, 0]);
  assert.deepEqual([...oled.gram.subarray(3 * 128 + 10, 3 * 128 + 14)], [4, 5, 6, 0]);
  assert.equal(oled.gram[2 * 128 + 9], 0);
  assert.equal(oled.page, 2); assert.equal(oled.col, 11);
});

test('vertical and page addressing modes', () => {
  const { m, oled } = setup();
  m.cmds(0x3C, 0x20, 0x01, 0x21, 0, 127, 0x22, 0, 1);
  m.data(0x3C, 0xA1, 0xA2, 0xA3);
  assert.equal(oled.gram[0], 0xA1); assert.equal(oled.gram[128], 0xA2); assert.equal(oled.gram[1], 0xA3);
  m.cmds(0x3C, 0x20, 0x02, 0xB5, 0x07, 0x12);  // page 5, column $27
  m.data(0x3C, 0x55);
  assert.equal(oled.gram[5 * 128 + 0x27], 0x55);
});

test('Co=1 single-byte control mode alternates control/data bytes', () => {
  const { m, oled } = setup();
  m.start(); m.byte(0x78);
  m.byte(0x80); m.byte(0xAF);   // Co=1, command: display on
  m.byte(0xC0); m.byte(0x3C);   // Co=1, data
  m.byte(0x80); m.byte(0xA7);   // Co=1, command: inverse
  m.stop();
  assert.equal(oled.displayOn, true);
  assert.equal(oled.inverse, true);
  assert.equal(oled.gram[0], 0x3C);
});

test('render: display on/off, charge pump, invert, entire-on, remap, start line and offset', () => {
  const { m, oled } = setup();
  m.cmds(0x3C, 0x20, 0x00, 0x21, 0, 127, 0x22, 0, 7);
  m.data(0x3C, 0x01);         // pixel at RAM row 0, column 0
  let fb = oled.render();
  assert.equal(fb.reduce((a, b) => a + b, 0), 0, 'display off after power-on');
  m.cmds(0x3C, 0x8D, 0x14, 0xAF, 0xA1, 0xC8);
  fb = oled.render();
  assert.equal(fb[0], 1, 'upright: top-left');
  m.cmds(0x3C, 0xA0); fb = oled.render(); assert.equal(fb[127], 1, 'A0 mirrors horizontally');
  m.cmds(0x3C, 0xC0); fb = oled.render(); assert.equal(fb[63 * 128 + 127], 1, 'C0 flips vertically');
  m.cmds(0x3C, 0xA1, 0xC8, 0x7F); fb = oled.render(); assert.equal(fb[1 * 128], 1, 'start line 63 moves image down 1px');
  m.cmds(0x3C, 0x40, 0xD3, 0x08); fb = oled.render(); assert.equal(fb[56 * 128], 1, 'offset 8 scrolls up 8 rows');
  m.cmds(0x3C, 0xD3, 0x00, 0xA7); fb = oled.render(); assert.equal(fb[0], 0); assert.equal(fb[1], 1, 'inverse');
  m.cmds(0x3C, 0xA6, 0xA5); fb = oled.render(); assert.equal(fb.reduce((a, b) => a + b, 0), 8192, 'entire display on');
  m.cmds(0x3C, 0xA4, 0x81, 0x10); assert.equal(oled.contrast, 0x10);
  m.cmds(0x3C, 0xAE); fb = oled.render(); assert.equal(fb.reduce((a, b) => a + b, 0), 0, 'display off');
});

test('multi-byte command parameters are consumed (scroll setup does not leak)', () => {
  const { m, oled } = setup();
  m.cmds(0x3C, 0x26, 0x00, 0x00, 0x07, 0x07, 0x00, 0xFF, 0xAF);
  assert.equal(oled.displayOn, true);
  assert.equal(oled.startLine, 0, '$7F etc. were parameters, not start-line commands');
});

test('24C64 EEPROM: page write and sequential read through the bus', () => {
  const { m, eeprom } = setup();
  m.start(); m.byte(0xA0); m.byte(0x01); m.byte(0x23); m.byte(0xDE); m.byte(0xAD); m.byte(0xBE); m.stop();
  assert.deepEqual([...eeprom.mem.subarray(0x123, 0x126)], [0xDE, 0xAD, 0xBE]);
  m.start(); m.byte(0xA0); m.byte(0x01); m.byte(0x23);
  m.start(); m.byte(0xA1);    // repeated start, read
  const got = [m.readByte(true), m.readByte(true), m.readByte(false)];
  m.stop();
  assert.deepEqual(got, [0xDE, 0xAD, 0xBE]);
});
