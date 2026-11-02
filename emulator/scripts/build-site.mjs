import fs from 'node:fs/promises';
import path from 'node:path';
import crypto from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const src = path.join(root, 'site');
const dest = path.join(root, 'dist');
const meta = JSON.parse(await fs.readFile(path.join(src, 'roms/firmware.json'), 'utf8'));
const rom = await fs.readFile(path.join(src, 'roms', meta.bios.file));
const digest = crypto.createHash('sha256').update(rom).digest('hex');
if (rom.length !== meta.bios.size || digest !== meta.bios.sha256) {
  throw new Error('BIOS does not match firmware.json; rebuild firmware before packaging');
}
for (const file of ['.htaccess', 'index.html', 'style.css', 'js/app.js', 'js/core/board.js', 'roms/hello-demo.bin', 'assets/65uino-board.jpeg']) {
  await fs.access(path.join(src, file));
}
await fs.rm(dest, { recursive: true, force: true });
await fs.cp(src, dest, { recursive: true });
// Ship the precise committed sources alongside the browser app, with no .git,
// ignored caches, local test downloads, credentials or node_modules.
const gitPrefix = execFileSync('git', ['rev-parse', '--show-prefix'], { cwd: root, encoding: 'utf8' }).trim().replace(/\/$/, '');
const sourceTree = gitPrefix ? `HEAD:${gitPrefix}` : 'HEAD';
execFileSync('git', ['archive', '--format=zip',
  `--output=${path.join(dest, '65uino-emulator-source.zip')}`,
  '--prefix=65uino-emulator/', sourceTree], { cwd: root });
console.log(`Built ${dest} (BIOS ${meta.commit.slice(0, 8)}, SHA-256 verified)`);
