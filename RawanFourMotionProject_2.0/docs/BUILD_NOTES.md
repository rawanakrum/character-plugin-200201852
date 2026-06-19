# Build Notes

## Plugin DLL

The plugin source depends on the N8RO / ASTSIM SDK headers and runtime. The Visual Studio plugin project is included under:

```text
visual_studio/RawanFourMotionPlugin.vcxproj
```

Before building, configure the environment variable:

```text
N8RO_SDK_DIR
```

to point to the local N8RO / ASTSIM SDK folder that contains `include/`, `src/`, and/or `lib/` folders.

## Standalone test

The standalone test does not need N8RO. It simulates the closed-library model for 1000 ticks, converts all 10 joint-angle outputs into quaternions, and checks that every quaternion is finite and approximately unit length.

Linux / macOS:

```bash
cd standalone_test
./run_test.sh
```

Windows Developer Command Prompt:

```bat
cd standalone_test
run_test.bat
```

Expected output:

```text
Linux standalone sanity test:
SDK version = 0x00010000
PASS: 1000 ticks, all quaternions finite & unit-ish
```

