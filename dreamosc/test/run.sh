#!/usr/bin/env bash
# Run the full host test suite for dreamosc:
#   1. Catch2 unit tests over the DSP core (stretch_core.h)
#   2. Golden regression: C++ core vs the Python reference (stretchseq.py)
# Exits non-zero if either layer fails, so it can gate a commit / CI.
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_DIR="$(cd "$TEST_DIR/../host" && pwd)"

echo "== 1/2  Catch2 unit tests =="
c++ -std=c++17 -O1 -I"$TEST_DIR/.." -I"$TEST_DIR" \
    "$TEST_DIR/test_stretch_core.cpp" -o "$TEST_DIR/unit_tests"
"$TEST_DIR/unit_tests"

echo
echo "== 2/2  Golden regression (C++ vs Python) =="
PY="$HOST_DIR/.venv/bin/python"
if [[ ! -x "$PY" ]]; then
  echo "!! host/.venv not found — create it: python3 -m venv host/.venv && host/.venv/bin/pip install numpy" >&2
  exit 1
fi
"$PY" "$HOST_DIR/regression.py"

echo
echo "== all host tests passed =="
