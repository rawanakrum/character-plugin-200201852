#!/usr/bin/env bash
set -euo pipefail
mkdir -p build
g++ -std=c++17 -I include tests/standalone_test.cpp src/StudentController.cpp -o build/standalone_test
./build/standalone_test
