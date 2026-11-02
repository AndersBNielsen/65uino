#!/usr/bin/env bash
# Build the real 65uino BIOS from Anders Nielsen's repository with cc65 and
# copy the results into site/roms/ for the emulator.
#
#   scripts/build-firmware.sh            # uses/updates .cache/65uino-src (main)
#   FIRMWARE_SRC=~/src/65uino scripts/build-firmware.sh   # use an existing checkout
#   FIRMWARE_REF=<commit|branch> scripts/build-firmware.sh
#
# Requires ca65/ld65 (brew install cc65, apt install cc65, or build from
# https://github.com/cc65/cc65). Same commands as the repo's assemble.sh, plus
# -g/-Ln so the emulator's disassembler can show the BIOS labels.
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$(pwd)
REPO_URL=${FIRMWARE_REPO:-https://github.com/AndersBNielsen/65uino}
REF=${FIRMWARE_REF:-main}
SRC=${FIRMWARE_SRC:-$ROOT/.cache/65uino-src}

command -v ca65 >/dev/null || { echo "ca65 not found - install cc65 first" >&2; exit 1; }

if [[ -z "${FIRMWARE_SRC:-}" ]]; then
  if [[ -d "$SRC/.git" ]]; then
    git -C "$SRC" fetch -q origin
  else
    mkdir -p "$(dirname "$SRC")"
    git clone -q "$REPO_URL" "$SRC"
  fi
  git -C "$SRC" checkout -q "$REF"
  git -C "$SRC" reset -q --hard "origin/$REF" 2>/dev/null || true
fi

COMMIT=$(git -C "$SRC" rev-parse HEAD)
BUILD=$(mktemp -d)
mkdir -p "$BUILD/build"
# memmap.cfg writes userland.bin to build/userland.bin relative to the cwd
( cd "$SRC" && ca65 -g --cpu 6502 -l "$BUILD/listing.txt" -o "$BUILD/abn6507rom.o" abn6507rom.s \
  && cd "$BUILD" && ld65 -o "$BUILD/abn6507rom.bin" -C "$SRC/memmap.cfg" -Ln "$BUILD/labels.txt" "$BUILD/abn6507rom.o" )

OUT=$ROOT/site/roms
mkdir -p "$OUT"
cp "$BUILD/abn6507rom.bin" "$OUT/65uino-bios.bin"
cp "$BUILD/build/userland.bin" "$OUT/65uino-userland.bin"
cp "$BUILD/labels.txt" "$OUT/65uino-bios.labels.txt"
SIZE=$(wc -c < "$OUT/65uino-bios.bin" | tr -d ' ')
SHA=$(shasum -a 256 "$OUT/65uino-bios.bin" | cut -d' ' -f1)
CA65=$(ca65 --version 2>&1 | head -1)
cat > "$OUT/firmware.json" <<EOF
{
  "source": "$REPO_URL",
  "commit": "$COMMIT",
  "assembler": "$CA65",
  "bios": { "file": "65uino-bios.bin", "size": $SIZE, "sha256": "$SHA" },
  "userland": { "file": "65uino-userland.bin" },
  "labels": "65uino-bios.labels.txt",
  "license": "CC BY-NC 4.0 (https://creativecommons.org/licenses/by-nc/4.0/) - firmware by Anders Nielsen"
}
EOF
rm -rf "$BUILD"
echo "Built 65uino BIOS from $COMMIT ($SIZE bytes, sha256 $SHA)"
