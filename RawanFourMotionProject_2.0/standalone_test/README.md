# Standalone Sanity Test

This folder contains a dependency-free Linux/Windows sanity test for the closed-library motion model.

It checks that the model can run for 1000 ticks and that all generated joint quaternions are finite and approximately unit length.

## Linux / macOS

```bash
./run_test.sh
```

Expected output:

```text
Linux standalone sanity test:
SDK version = 0x00010000
PASS: 1000 ticks, all quaternions finite & unit-ish
```

## Windows Developer Command Prompt

```bat
run_test.bat
```
