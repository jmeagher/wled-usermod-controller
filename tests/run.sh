#!/usr/bin/env bash
# Compile the desktop logic tests with warnings as errors and ASan+UBSan, then run them.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build
${CXX:-c++} -std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=undefined \
  tests/test_logic.cpp -o build/test_logic
build/test_logic
