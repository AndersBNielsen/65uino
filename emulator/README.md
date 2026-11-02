# 65uino browser emulator

A self-contained browser emulator for [Anders Nielsen's 65uino](https://github.com/AndersBNielsen/65uino),
the 6507 learning board. It boots the real BIOS and models the 6532 RIOT,
bit-banged I²C, SSD1306 128×64 OLED, 24C64 EEPROM, pushbutton, GPIO, LED,
CH340-style serial connection and optional PhaseLoom/Si5351 I²C presence.

## Run locally

No runtime dependencies, bundler, server-side code or account are required.
Use a local HTTP server because browser ES modules and ROM fetches cannot be
loaded from `file://` in most browsers:

```sh
python3 -m http.server 8065 --directory site
```

Open `http://localhost:8065/`. The entire emulator runs in your browser; local
ROM selections are read in the browser and never sent to a server. `npm run build` copies the
site to `dist/` and verifies the firmware checksum; deploy the contents of
`dist/` as ordinary static files. The build also includes a ZIP of the precise
committed source tree at `dist/65uino-emulator-source.zip` for public download.
Commit changes before building a release so the source ZIP matches the page.

## Use

- Click or hold **BTN** to navigate/select in the BIOS. **RST** resets the CPU
  and RIOT; **Power cycle** resets the OLED and serial state too.
- The board view uses Anders' populated-board photograph as a fixed backdrop;
  the live OLED pixels, BTN/RST and header GPIO controls are aligned over it.
  Phones also have larger BTN/RST controls just below the image.
- Choose either bundled, source-attributed ROM or open your own raw 6507 ROM
  image (`.bin`/`.rom`, 1–65,536 bytes) from your computer. It stays in your
  browser. The 6507 sees a 4 KiB ROM window at `$1000–$1FFF`; shorter images
  repeat, and larger images use their first 4 KiB, matching the board jumpers.
- **Run bundled userland** sends the included BIOS userland image over
  emulated serial, entirely inside the browser. The terminal sends typed text
  to the emulated board at the selected baud rate.
- GPIO header dots and the GPIO list show live pin levels. Click a free input
  to pull it low or release it; outputs and the on-board I²C/BTN lines are
  controlled by the emulated board. The LED follows PB7.
- **Step**, **Run/Pause**, and a hexadecimal breakpoint let you inspect CPU
  registers and disassembly. CPU-clock and speed selectors are independent:
  clock changes the simulated hardware oscillator; speed changes playback.
- The optional PhaseLoom switch attaches a presence/register stub at I²C
  address `$60` and power-cycles the board. It is sufficient to exercise the
  firmware's SDR path, not a full Si5351 signal-generation model.

## Build firmware / tests

The checked-in BIOS image was assembled from 65uino commit
`2dcd9826e19dc98c5d480c1f56d852400a396110`. To rebuild it, install
[cc65](https://cc65.github.io/) and run `npm run build:firmware`. To rebuild
the example ROM, run `npm run build:examples`.

```sh
npm test           # 6502 functional/decimal tests and board/firmware tests
npm run build      # produce verified dist/ static site
npm run e2e        # browser smoke test (requires Playwright and Chrome)
```

`npm test` downloads Klaus Dormann's GPL-3.0 functional test from a pinned
revision when the cache is empty; the file is not distributed here. The other
tests include real BIOS boot/menu/button/serial/I²C/userland paths and display
snapshots. `npm run e2e` uses a browser installed on the local machine and the
optional `playwright` package; see the script for environment variables.

## Fidelity / limitations

The CPU implements all 151 documented NMOS 6502 opcodes, bus-cycle timing and
common undocumented opcodes. The RIOT timer and I/O, serial bit timing,
open-drain I²C bus and SSD1306 commands used by the firmware are modelled.
Unstable undocumented opcodes are approximations. The optional Si5351 and
off-board GPIO attachments are not full analog/electrical simulations.
Browsers may throttle background tabs, so wall-clock emulation speed is best
effort; CPU-cycle counters remain deterministic.

## Public-page security boundary

The published page has a local file chooser but no upload endpoint, server-side
emulator process, or dynamic ROM URL. It fetches only its same-origin, bundled
assets. The file's bytes are passed to the JavaScript 6507 interpreter via the
browser File API; they are not transmitted or run as native code. The file size
is checked before reading and emulation is capped at 100,000 cycles per frame.
The dedicated directory's `.htaccess` denies non-GET/HEAD requests and directory
listing, and applies a restrictive Content Security Policy. The same policy is
also in the page as a local-preview fallback. The CPU interpreter does not use
JavaScript `eval` or `Function`; serial text is rendered with `textContent`.
As with any website, this is not a claim that an arbitrary visitor's own
browser or the surrounding shop is invulnerable.

## License

Emulator code is MIT. The BIOS, example assembly/ROM and upstream-derived font
remain under the 65uino project's CC BY-NC 4.0 license. See [LICENSE.md](LICENSE.md)
for the complete breakdown. The included firmware is source-attributed and
independently replaceable with your own binary.
