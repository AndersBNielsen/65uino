# Licensing

The emulator JavaScript, browser interface, build scripts and original tests in
this repository are Copyright (c) 2026 Anders Nielsen and contributors,
licensed under the MIT License:

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Separately licensed material

- `site/roms/65uino-bios.bin`, `site/roms/65uino-userland.bin`, and the BIOS
  label file are built from the [65uino firmware](https://github.com/AndersBNielsen/65uino)
  by Anders Nielsen and retain its [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)
  license. Source revision and checksum are in `site/roms/firmware.json`.
- `examples/hello/` and `site/roms/hello-demo.bin` are CC BY-NC 4.0. The
  demo font is derived from the 65uino firmware font; see `examples/hello/font.s`.
- `site/assets/65uino-board.jpeg` is the populated-board photo from the
  [65uino project](https://github.com/AndersBNielsen/65uino/blob/main/images/65uinoRev1Populated.jpeg)
  by Anders Nielsen and retains the project's CC BY-NC 4.0 license.
- `test/fixtures/decimal_test.s` is Bruce Clark's public-domain decimal-mode
  test, ported to ca65 syntax. The generated `.bin` has the same status.
- Klaus Dormann's GPL-3.0 functional test is not included. The test runner
  downloads a pinned copy into ignored `test/.cache/` and checks its SHA-256.
