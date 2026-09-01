#!/usr/bin/env bash
# Run the host test suite for dreamosc: Catch2 unit tests over the DSP core.
# (The former Python golden regression is retired — the reference lives in
# archive/ and the synthesis has diverged to canonical PaulXStretch.)
# Exits non-zero on failure, so it can gate a commit / CI.
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "== Vendored sources match their pinned upstream SHAs =="
"$TEST_DIR/../../vendor/fetch.sh" --check

echo
echo "== Catch2 unit tests (DSP core + vendored FFT) =="
c++ -std=c++17 -O1 -I"$TEST_DIR/.." -I"$TEST_DIR" \
    "$TEST_DIR/test_stretch_core.cpp" "$TEST_DIR/test_shy_fft.cpp" \
    -o "$TEST_DIR/unit_tests"
"$TEST_DIR/unit_tests"

echo
echo "== all host tests passed =="
