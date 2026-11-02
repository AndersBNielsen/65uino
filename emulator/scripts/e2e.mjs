// Smoke-test the actual browser interface. Requires `npm install` and a local
// Chrome/Chromium installation; PLAYWRIGHT_CHROME can override the executable.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { setTimeout as delay } from 'node:timers/promises';
import { chromium } from 'playwright';

const host = 'http://127.0.0.1:18065/';
const server = spawn('python3', ['-m', 'http.server', '18065', '--directory', 'site'], { stdio: 'ignore' });
let browser;
try {
  let ready = false;
  for (let i = 0; i < 30; i++) {
    try { if ((await fetch(host)).ok) { ready = true; break; } } catch { /* server starting */ }
    await delay(100);
  }
  assert.ok(ready, 'local static server did not start');
  browser = await chromium.launch({ headless: true, executablePath: process.env.PLAYWRIGHT_CHROME || undefined });
  const page = await browser.newPage({ viewport: { width: 1280, height: 900 } });
  const pageErrors = [];
  const requests = [];
  page.on('pageerror', (error) => pageErrors.push(error.message));
  page.on('request', (request) => requests.push(request.url()));
  await page.goto(host, { waitUntil: 'domcontentloaded' });
  await page.getByRole('button', { name: 'Pause' }).waitFor();
  assert.equal(await page.locator('input[type="file"]').count(), 1, 'local ROM picker must be available');
  assert.ok(await page.locator('.board-photo').evaluate((img) => img.naturalWidth > 1000), 'mounted-board photo must load');
  // Independent socket locations measured from the 2407 px board photograph.
  // Catch a rail that drifts or wraps into a second row at either viewport size.
  const checkHeaderAlignment = async () => {
    const expected = { PA0: 346, PA7: 859, PB2: 959, PB7: 1320, PB1: 1540, PB0: 1615 };
    const points = await page.evaluate(() => {
      const board = document.querySelector('#board').getBoundingClientRect();
      return Object.fromEntries([...document.querySelectorAll('.pin-rail .pin')].map((pin) => {
        const hole = pin.querySelector('.pin-hole').getBoundingClientRect();
        return [pin.dataset.pin, {
          x: (hole.x + hole.width / 2 - board.x) / board.width,
          y: (hole.y + hole.height / 2 - board.y) / board.height,
        }];
      }));
    });
    for (const [pin, photoX] of Object.entries(expected)) {
      assert.ok(Math.abs(points[pin].x - (photoX - 195) / 2070) < 0.012, `${pin} misses its photographed socket`);
      assert.ok(Math.abs(points[pin].y - (1840 - 453) / 1459) < 0.012, `${pin} is on the wrong header row`);
    }
  };
  await checkHeaderAlignment();
  await page.waitForTimeout(1500);
  const oledHash = () => page.locator('#oled').evaluate((canvas) => {
    let hash = 0;
    const pixels = canvas.getContext('2d').getImageData(0, 0, 128, 64).data;
    for (let i = 0; i < pixels.length; i += 4) hash = (hash * 31 + pixels[i]) >>> 0;
    return hash;
  });
  const menu = await oledHash();
  assert.notEqual(menu, 0, 'OLED must show firmware output');

  const button = page.locator('#boardButton');
  await button.scrollIntoViewIfNeeded();
  const box = await button.boundingBox();
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.down();
  await page.waitForTimeout(140);
  await page.mouse.up();
  await page.waitForTimeout(500);
  assert.notEqual(await oledHash(), menu, 'BTN must change the BIOS menu display');

  await page.locator('.gpio-item[data-pin="PA2"] button').click();
  assert.equal(await page.locator('.gpio-item[data-pin="PA2"] button').innerText(), 'L');
  await page.locator('#preset').selectOption('hello-demo.bin');
  await page.waitForTimeout(1200);
  assert.notEqual(await oledHash(), menu, 'demo ROM must draw a distinct screen');

  const localRom = Buffer.alloc(4096, 0xEA);
  localRom.set([0x4C, 0x00, 0x10], 0); // JMP $1000, safely running in emulated CPU
  localRom.set([0x00, 0x10], 0xFFC); // 6507 reset vector
  const requestCount = requests.length;
  await page.locator('#romFile').setInputFiles({ name: 'local-test.bin', mimeType: 'application/octet-stream', buffer: localRom });
  await page.getByText('Running local ROM: local-test.bin').waitFor();
  await page.waitForTimeout(250);
  assert.equal(requests.length, requestCount, 'opening a local ROM must make no network request');
  assert.equal(await page.locator('#preset').inputValue(), '', 'local ROM must not appear as a bundled preset');
  await page.locator('#romFile').setInputFiles({ name: 'too-large.bin', mimeType: 'application/octet-stream', buffer: Buffer.alloc(65537) });
  await page.getByText('Choose a raw ROM image of 1–65,536 bytes').waitFor();
  assert.equal(requests.length, requestCount, 'rejected ROM must make no network request');
  await page.locator('#preset').selectOption('hello-demo.bin');
  await page.getByRole('button', { name: 'Pause' }).waitFor();

  await page.getByRole('button', { name: 'Pause' }).click();
  const before = await page.locator('#cycleCount').innerText();
  await page.getByRole('button', { name: 'Step', exact: true }).click();
  assert.notEqual(await page.locator('#cycleCount').innerText(), before, 'Step must execute an instruction');
  await page.setViewportSize({ width: 390, height: 844 });
  assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false, 'mobile layout overflows');
  await checkHeaderAlignment();
  await page.locator('#preset').selectOption('65uino-bios.bin');
  await page.waitForTimeout(1500);
  const mobileMenu = await oledHash();
  const mobileButton = page.locator('#boardButtonMobile');
  assert.ok(await mobileButton.isVisible(), 'large mobile BTN control must be visible');
  const mobileBox = await mobileButton.boundingBox();
  await page.mouse.move(mobileBox.x + mobileBox.width / 2, mobileBox.y + mobileBox.height / 2);
  await page.mouse.down();
  await page.waitForTimeout(140);
  await page.mouse.up();
  await page.waitForTimeout(500);
  assert.notEqual(await oledHash(), mobileMenu, 'mobile BTN must change the BIOS display');
  assert.ok(requests.every((url) => url.startsWith(host)), 'page must fetch only same-origin assets');
  assert.deepEqual(pageErrors, []);
  console.log('Browser smoke test passed: mounted photo, boot, OLED, buttons, GPIO, bundled/local ROMs, no local-file network request, size limit, step and mobile layout');
} finally {
  if (browser) await browser.close();
  server.kill();
}
