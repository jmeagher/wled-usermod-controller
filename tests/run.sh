#!/usr/bin/env bash
# Compile the desktop tests with warnings as errors and ASan+UBSan, then run them.
#   test_logic: ctl_logic.h and the JD9853 init table (no dependencies)
#   test_json:  ctl_json.h against ArduinoJson 6.18.1, the version WLED bundles. The single-header release is
#               downloaded once into build/ and checked against its SHA-256 (or set ARDUINOJSON_H to a local copy).
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build
cxx="${CXX:-c++}"
flags=(-std=c++17 -Wall -Wextra -Werror -O1 -g "-fsanitize=address,undefined" -fno-sanitize-recover=undefined)

"$cxx" "${flags[@]}" tests/test_logic.cpp -o build/test_logic
build/test_logic

aj_version=6.18.1
aj_sha256=e886c0c99e719e02c341c8783f499287ab68c74f949987b1cdc8c06523bcb5ad
aj="${ARDUINOJSON_H:-build/arduinojson/ArduinoJson-v$aj_version.h}"
sha256() { if command -v sha256sum >/dev/null; then sha256sum "$1"; else shasum -a 256 "$1"; fi | cut -d' ' -f1; }
if [ -z "${ARDUINOJSON_H:-}" ] && { [ ! -f "$aj" ] || [ "$(sha256 "$aj")" != "$aj_sha256" ]; }; then
  mkdir -p "$(dirname "$aj")"
  curl -fsSL --retry 3 -o "$aj.tmp" \
    "https://github.com/bblanchon/ArduinoJson/releases/download/v$aj_version/ArduinoJson-v$aj_version.h"
  [ "$(sha256 "$aj.tmp")" = "$aj_sha256" ] || { echo "ArduinoJson download: checksum mismatch" >&2; rm -f "$aj.tmp"; exit 1; }
  mv "$aj.tmp" "$aj"
fi
[ -f "$aj" ] || { echo "ArduinoJson header not found: $aj" >&2; exit 1; }
# The library is included as a system header: its own code is not ours to keep warning-free.
mkdir -p build/aj-include
cp "$aj" "build/aj-include/ArduinoJson-v$aj_version.h"
"$cxx" "${flags[@]}" -isystem build/aj-include tests/test_json.cpp -o build/test_json
build/test_json
