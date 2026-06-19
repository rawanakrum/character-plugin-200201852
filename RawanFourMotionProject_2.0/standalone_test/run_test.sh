#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
g++ -std=c++17 -O2 RawanStandaloneMotionTest.cpp -o RawanStandaloneMotionTest
./RawanStandaloneMotionTest
