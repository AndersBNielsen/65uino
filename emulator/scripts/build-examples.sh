#!/usr/bin/env bash
# Assemble the example ROM(s) in examples/ into site/roms/ (needs cc65).
set -euo pipefail
cd "$(dirname "$0")/../examples/hello"
TMP=$(mktemp -d)
ca65 -g hello.s -o "$TMP/hello.o"
ld65 -C hello.cfg -o ../../site/roms/hello-demo.bin -Ln ../../site/roms/hello-demo.labels.txt "$TMP/hello.o"
rm -rf "$TMP"
echo "Built site/roms/hello-demo.bin"
