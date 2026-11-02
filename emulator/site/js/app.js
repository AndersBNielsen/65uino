import { Board65uino, PIN_INFO } from './core/board.js';
import { disassemble, hex2, hex4, parseLd65Labels, RIOT_LABELS } from './core/disasm.js';

const $ = (id) => document.getElementById(id);
const board = new Board65uino();
const canvas = $('oled');
const ctx = canvas.getContext('2d', { alpha: false });
const pixels = ctx.createImageData(128, 64);
const framebuffer = new Uint8Array(128 * 64);
const breakpoints = new Set();
const firmware = new Map();
const bundledRoms = new Set(['65uino-bios.bin', 'hello-demo.bin']);
const MAX_LOCAL_ROM_BYTES = 65536;
const MAX_FRAME_CYCLES = 100000;
const labelsByRom = new Map();
let labels = new Map(RIOT_LABELS);
let currentRom = '65uino-bios.bin';
let running = false;
let lastFrame = 0;
let lastOledVersion = -1;
let serialText = '';
let buttonDown = false;

function status(message, kind = '') {
  $('status').textContent = message;
  $('status').className = 'status ' + kind;
}

function setRunning(value) {
  running = value;
  $('playPause').textContent = running ? 'Pause' : 'Run';
  status(running ? 'Running' : 'Paused', running ? '' : 'paused');
  lastFrame = performance.now();
}

function drawOled(force = false) {
  if (!force && board.oled.version === lastOledVersion) return;
  lastOledVersion = board.oled.version;
  board.framebuffer(framebuffer);
  for (let i = 0; i < framebuffer.length; i++) {
    const p = i * 4, on = framebuffer[i] !== 0;
    pixels.data[p] = on ? 169 : 5;
    pixels.data[p + 1] = on ? 232 : 19;
    pixels.data[p + 2] = on ? 245 : 31;
    pixels.data[p + 3] = 255;
  }
  ctx.putImageData(pixels, 0, 0);
}

function pinDirection(info) {
  return ((info.port === 'A' ? board.riot.ddra : board.riot.ddrb) >> info.bit) & 1;
}

function pinExternalHigh(info) {
  return ((info.port === 'A' ? board.extA : board.extB) >> info.bit) & 1;
}

function pinClick(info) {
  if (pinDirection(info) || ['PB0', 'PB1', 'PB6'].includes(info.id)) return;
  board.setExternal(info.port, info.bit, !pinExternalHigh(info));
  refresh();
}

function buildPins() {
  for (const info of PIN_INFO) {
    const rail = info.port === 'A' ? $('leftPins') : $('rightPins');
    const pin = document.createElement('button');
    pin.type = 'button';
    pin.className = 'pin';
    pin.dataset.pin = info.id;
    pin.title = `${info.id} · ${info.header}${info.fn ? ' · ' + info.fn : ''}`;
    pin.setAttribute('aria-label', pin.title);
    // The photo's right header has D8–D13, then GND, 5V, SDA, SCL.
    // Leave the two power sockets visible and place the I²C controls last.
    if (info.port === 'B') pin.style.gridColumn = String(info.bit < 2 ? 10 - info.bit : info.bit - 1);
    const hole = document.createElement('span');
    hole.className = 'pin-hole';
    const name = document.createElement('span');
    name.className = 'pin-name';
    name.textContent = info.header;
    pin.append(hole, name);
    pin.addEventListener('click', () => pinClick(info));
    rail.append(pin);

    const item = document.createElement('div');
    item.className = 'gpio-item';
    item.dataset.pin = info.id;
    const light = document.createElement('i'); light.className = 'light';
    const pinId = document.createElement('b'); pinId.textContent = info.id;
    const fn = document.createElement('span'); fn.className = 'fn'; fn.textContent = info.fn || info.header;
    const control = document.createElement('button'); control.type = 'button';
    control.setAttribute('aria-label', `Toggle ${info.id} input`); control.textContent = 'H';
    control.addEventListener('click', () => pinClick(info));
    item.append(light, pinId, fn, control);
    $('gpioList').append(item);
  }
}

function refreshPins() {
  board.refreshPinLevels();
  for (const info of PIN_INFO) {
    const i = info.port === 'A' ? info.bit : info.bit + 8;
    const high = board.pinLevels[i] !== 0;
    const output = !!pinDirection(info);
    const locked = output || ['PB0', 'PB1', 'PB6'].includes(info.id);
    const rail = document.querySelector(`.pin-rail [data-pin="${info.id}"]`);
    rail.className = `pin ${high ? 'high' : 'low'} ${output ? 'output' : ''}`;
    rail.disabled = locked;
    rail.setAttribute('aria-pressed', !pinExternalHigh(info));
    const row = document.querySelector(`.gpio-item[data-pin="${info.id}"]`);
    row.className = `gpio-item ${high ? 'high' : 'low'} ${output ? 'output' : ''}`;
    const control = row.querySelector('button');
    control.disabled = locked;
    control.textContent = high ? 'H' : 'L';
    control.title = locked ? (output ? 'Driven by firmware' : 'Dedicated on-board connection') : 'Click to pull input low or release it';
  }
  $('led').classList.toggle('on', board.pinLevels[15] === 0 && pinDirection(PIN_INFO[15]));
}

function refreshDebug() {
  const c = board.cpu;
  $('cycleCount').textContent = `${c.cycles.toLocaleString()} cycles`;
  const values = [['PC', hex4(c.pc)], ['A', hex2(c.a)], ['X', hex2(c.x)],
    ['Y', hex2(c.y)], ['SP', hex2(c.s)], ['P', hex2(c.p)]];
  const regNodes = values.map(([key, value]) => {
    const span = document.createElement('span');
    const strong = document.createElement('b'); strong.textContent = `$${value}`;
    span.append(`${key} `, strong);
    return span;
  });
  $('registers').replaceChildren(...regNodes);
  let pc = c.pc;
  const lines = [];
  for (let i = 0; i < 7; i++) {
    const d = disassemble((addr) => board.peek(addr), pc, labels);
    const bytes = d.bytes.map(hex2).join(' ').padEnd(8);
    lines.push(`${i ? ' ' : '▶'} ${hex4(pc)}  ${bytes} ${d.text}${d.label ? `  ; ${d.label}` : ''}`);
    pc = (pc + d.len) & 0xFFFF;
  }
  $('disassembly').textContent = lines.join('\n');
}

function refresh(force = false) {
  drawOled(force);
  refreshPins();
  refreshDebug();
}

function appendSerial(byte, error) {
  const c = byte === 10 ? '\n' : byte === 13 ? '' : byte === 9 ? '\t' :
    (byte >= 32 && byte < 127 ? String.fromCharCode(byte) : `\\x${hex2(byte)}`);
  serialText = (serialText + (error ? '�' : c)).slice(-8000);
  $('terminal').textContent = serialText;
  $('terminal').scrollTop = $('terminal').scrollHeight;
}

async function loadPreset(name) {
  try {
    if (!bundledRoms.has(name)) throw new Error('Unsupported firmware selection');
    if (!firmware.has(name)) {
      const response = await fetch(`./roms/${name}`);
      if (!response.ok) throw new Error(`ROM request returned ${response.status}`);
      firmware.set(name, new Uint8Array(await response.arrayBuffer()));
    }
    currentRom = name;
    board.loadRom(firmware.get(name));
    board.powerOn();
    board.setButton(false);
    serialText = '';
    $('terminal').textContent = '';
    labels = new Map([...RIOT_LABELS, ...(labelsByRom.get(name) || [])]);
    setRunning(true);
    refresh(true);
  } catch (error) {
    setRunning(false);
    status(`Could not load firmware: ${error.message}`, 'error');
  }
}

async function loadLocalRom(file) {
  if (!file) return;
  if (file.size < 1 || file.size > MAX_LOCAL_ROM_BYTES) {
    status('Choose a raw ROM image of 1–65,536 bytes', 'error');
    return;
  }
  try {
    // File.arrayBuffer reads the visitor's selection locally. No network request
    // or server-side upload is made; the bytes are interpreted by Board65uino.
    const bytes = new Uint8Array(await file.arrayBuffer());
    if (bytes.length !== file.size) throw new Error('File changed while reading');
    board.loadRom(bytes);
    board.powerOn();
    board.setButton(false);
    currentRom = 'local';
    $('preset').selectedIndex = -1;
    serialText = '';
    $('terminal').textContent = '';
    labels = new Map(RIOT_LABELS);
    setRunning(true);
    status(`Running local ROM: ${file.name}`);
    refresh(true);
  } catch (error) {
    status(`Could not read local ROM: ${error.message}`, 'error');
  }
}

function button(value) {
  if (buttonDown === value) return;
  buttonDown = value;
  board.setButton(value);
  $('boardButton').classList.toggle('pressed', value);
  $('boardButtonMobile').classList.toggle('pressed', value);
}

function frame(now) {
  if (running) {
    // Cap a late frame so switching tabs does not freeze the page to catch up.
    const elapsed = Math.min(50, Math.max(0, now - lastFrame));
    const cycles = Math.min(MAX_FRAME_CYCLES, Math.round(elapsed * board.clockHz * Number($('speed').value) / 1000));
    if (cycles > 0) {
      const result = board.run(cycles, breakpoints);
      if (result === 'bp') { setRunning(false); status(`Breakpoint at $${hex4(board.cpu.pc)}`, 'paused'); }
    }
    refresh();
  }
  lastFrame = now;
  requestAnimationFrame(frame);
}

function wireEvents() {
  $('playPause').addEventListener('click', () => setRunning(!running));
  $('step').addEventListener('click', () => { setRunning(false); board.stepInstruction(); refresh(); });
  $('reset').addEventListener('click', () => { board.reset(); setRunning(true); refresh(true); });
  for (const id of ['boardReset', 'boardResetMobile']) {
    $(id).addEventListener('click', () => { board.reset(); setRunning(true); refresh(true); });
  }
  $('power').addEventListener('click', () => { board.powerOn(); setRunning(true); refresh(true); });
  $('preset').addEventListener('change', (e) => loadPreset(e.target.value));
  $('romFile').addEventListener('change', (e) => {
    loadLocalRom(e.target.files?.[0]);
    e.target.value = '';
  });
  $('clock').addEventListener('change', (e) => { board.setClock(Number(e.target.value)); refresh(); });
  $('baud').addEventListener('change', (e) => board.setBaud(Number(e.target.value)));
  $('phaseloom').addEventListener('change', (e) => { board.setPhaseLoom(e.target.checked); board.powerOn(); setRunning(true); refresh(true); });
  for (const id of ['boardButton', 'boardButtonMobile']) {
    $(id).addEventListener('pointerdown', (e) => { e.preventDefault(); e.currentTarget.setPointerCapture(e.pointerId); button(true); });
    $(id).addEventListener('pointerup', () => button(false));
    $(id).addEventListener('pointercancel', () => button(false));
    $(id).addEventListener('lostpointercapture', () => button(false));
    $(id).addEventListener('keydown', (e) => { if (e.code === 'Space' || e.code === 'Enter') { e.preventDefault(); button(true); } });
    $(id).addEventListener('keyup', (e) => { if (e.code === 'Space' || e.code === 'Enter') { e.preventDefault(); button(false); } });
  }
  window.addEventListener('blur', () => button(false));
  $('uploadUserland').addEventListener('click', async () => {
    if (currentRom !== '65uino-bios.bin') { status('Select the original BIOS before uploading userland', 'paused'); return; }
    const response = await fetch('./roms/65uino-userland.bin');
    if (!response.ok) { status('Sample userland is unavailable', 'error'); return; }
    board.uploadUserland(new Uint8Array(await response.arrayBuffer()));
    setRunning(true);
    status('Uploading sample userland…');
  });
  $('terminalForm').addEventListener('submit', (e) => {
    e.preventDefault();
    const input = $('terminalInput');
    if (input.value) board.serialSend([...new TextEncoder().encode(input.value + '\r')]);
    input.value = '';
    setRunning(true);
  });
  $('clearTerminal').addEventListener('click', () => { serialText = ''; $('terminal').textContent = ''; });
  $('setBreakpoint').addEventListener('click', () => {
    const value = $('breakpoint').value.trim().replace(/^\$/, '');
    if (!/^[0-9a-fA-F]{1,4}$/.test(value)) { $('breakpointStatus').textContent = 'Enter a 1–4 digit hex address'; return; }
    const addr = parseInt(value, 16);
    breakpoints.clear(); breakpoints.add(addr);
    $('breakpointStatus').textContent = `$${hex4(addr)} armed`;
  });
  $('clearBreakpoint').addEventListener('click', () => { breakpoints.clear(); $('breakpointStatus').textContent = 'No breakpoint'; });
}

async function start() {
  buildPins();
  wireEvents();
  board.onSerialByte = appendSerial;
  try {
    const [meta, biosLabels, demoLabels] = await Promise.all([
      fetch('./roms/firmware.json').then((r) => r.json()),
      fetch('./roms/65uino-bios.labels.txt').then((r) => r.text()),
      fetch('./roms/hello-demo.labels.txt').then((r) => r.text()),
    ]);
    labelsByRom.set('65uino-bios.bin', parseLd65Labels(biosLabels));
    labelsByRom.set('hello-demo.bin', parseLd65Labels(demoLabels));
    $('firmwareVersion').textContent = `BIOS ${meta.commit.slice(0, 8)}`;
  } catch { $('firmwareVersion').textContent = 'Firmware metadata unavailable'; }
  await loadPreset(currentRom);
  requestAnimationFrame(frame);
}

start();
