// End-to-end: boot the real 65uino BIOS (built from AndersBNielsen/65uino with
// cc65) on the emulated board and check what the OLED shows.
import test from 'node:test';
import assert from 'node:assert/strict';
import { bootBoard, runMs, press, fbAscii, matchSnapshot, readScreenText, rom } from './helpers.js';

const MENU = ['Flappy Larus', 'Run Userland $0E', 'Code Monitor', 'I2C Scanner', 'Terminal',
  'Airplane mode', 'St. motor ctrl'];

test('BIOS boots to the main menu with "Flappy Larus" selected', () => {
  const b = bootBoard();
  runMs(b, 1500);
  assert.equal(b.oled.lit, true, 'display initialised and on');
  assert.deepEqual(readScreenText(b).slice(0, 7), MENU);
  // selected item is drawn inverted: page 0 is mostly lit
  const page0 = b.oled.gram.subarray(0, 128);
  assert.ok(page0.filter((v) => v === 0xFF).length > 20, 'first item inverted');
  matchSnapshot('bios-menu', fbAscii(b));
});

test('BIOS blinks the green LED on PB7 (~4 Hz toggle)', () => {
  const b = bootBoard();
  runMs(b, 1500);
  let toggles = 0, last = b.pinLevels[15];
  for (let i = 0; i < 100; i++) { runMs(b, 10); if (b.pinLevels[15] !== last) { toggles++; last = b.pinLevels[15]; } }
  assert.ok(toggles >= 3 && toggles <= 5, `LED toggled ${toggles} times in 1 s`);
});

test('short BTN press moves the menu selection', () => {
  const b = bootBoard();
  runMs(b, 1500);
  const before = fbAscii(b);
  press(b, 60);
  assert.notEqual(fbAscii(b), before);
  assert.ok(b.oled.gram.subarray(0, 128).filter((v) => v === 0xFF).length < 10, 'item 0 no longer inverted');
  assert.ok(b.oled.gram.subarray(128, 256).filter((v) => v === 0xFF).length > 20, 'item 1 inverted');
  matchSnapshot('bios-menu-item2', fbAscii(b));
});

test('long BTN press on "Flappy Larus" starts the game', () => {
  const b = bootBoard();
  runMs(b, 1500);
  press(b, 600, 1500);
  const text = readScreenText(b).join('\n');
  assert.ok(!text.includes('Code Monitor'), 'menu gone');
  assert.ok(text.includes('E'), 'the larus ("E") is on screen');
  matchSnapshot('flappy-larus-start', fbAscii(b));
});

test('serial text (4800 baud, bit-banged on PA0) is printed on the OLED', () => {
  const b = bootBoard();
  runMs(b, 1500);
  b.serialSend([...Buffer.from('Hello 65uino!')]);
  runMs(b, 300);
  assert.equal(readScreenText(b)[7], 'Hello 65uino!');
});

test('I2C scanner finds the OLED (3C) and the on-board EEPROM (50)', () => {
  const b = bootBoard();
  runMs(b, 1500);
  for (let i = 0; i < 3; i++) press(b, 60);
  press(b, 600, 2000);
  const text = readScreenText(b).join('');
  assert.ok(text.startsWith('Scanning i2c busFound: 3C 50'), text);
});

test('userland upload via serial (send_userland.py protocol) loads and runs', () => {
  const b = bootBoard();
  runMs(b, 1500);
  const ul = rom('65uino-userland.bin');
  b.uploadUserland(ul);
  runMs(b, 1200);
  assert.equal(readScreenText(b).slice(0, 2).join(' '), `Loaded ${ul.length.toString(16).toUpperCase()} bytes of data.`);
  assert.deepEqual([...b.riot.ram.subarray(0x0E, 0x0E + ul.length)], [...ul]);
  b.setPhaseLoom(true);
  runMs(b, 1500);
  assert.ok(b.phaseloom.writeCount > 20, 'userland talked to the Si5351 at 0x60');
});

test('with a PhaseLoom (Si5351 @ 0x60) attached the BIOS enters SDR mode', () => {
  const b = bootBoard();
  b.setPhaseLoom(true);
  b.powerOn();
  runMs(b, 1000);
  assert.ok(b.phaseloom.writeCount > 20);
  assert.equal(b.riot.ddra, 0xF2, 'PA4-PA7 LEDs + TX as outputs');
  assert.equal(b.oled.lit, false, 'display untouched in SDR mode');
});

test('hello demo ROM draws its screen', () => {
  const b = bootBoard('hello-demo.bin');
  runMs(b, 1100);
  matchSnapshot('hello-demo', fbAscii(b));
  press(b, 100, 100);
  assert.equal(b.oled.inverse, true, 'BTN inverts the display');
});
