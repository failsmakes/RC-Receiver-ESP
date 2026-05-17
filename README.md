# RC Receiver ESP

ESP-based RC receiver project with support for:
- `nodemcuv2` (ESP8266)
- `esp32dev` (ESP32)

## Project structure

- `platformio.ini` — build configuration and environments
- `src/` — main receiver sources
- `include/` — shared project headers
- `lib/bluepad32/` — vendored Bluepad32 library for ESP32 PS3/PS4 support

## Vendored Bluepad32

This project now includes the full Bluepad32 library under `lib/bluepad32`, including its `btstack` submodule.

Do not remove `lib/bluepad32`; it is required for ESP32 PS3/PS4 controller support.

## Build requirements

- PlatformIO
- Python environment for PlatformIO

On Windows, if PlatformIO build fails due to `intelhex`, install it into the PlatformIO penv if needed:

```powershell
python -m pip install intelhex
```

## Build commands

From the project root:

```powershell
pio run -e nodemcuv2
pio run -e esp32dev
```

## ESP32 PS3/PS4 pairing

1. Build and flash the `esp32dev` firmware.
2. Power on the ESP32 and put the PS3/PS4 controller in pairing mode.
   - PS3 controller: hold `PS` + `Share` until the light blinks.
   - PS4 controller: hold `PS` + `Share` until the light flashes rapidly.
3. Wait for the controller LED to stabilize; the controller should connect automatically.
4. If pairing fails, reset the ESP32 and retry.

## Notes

- `platformio.ini` uses `default_envs` to choose default boards.
- `config.h` contains board-specific pin mappings and settings.
- `receiver.cpp` includes the ESP32-specific Bluepad32 path and PS3/PS4 controller handling.

## Status

- `nodemcuv2` build: verified successful
- `esp32dev` build: verified successful
