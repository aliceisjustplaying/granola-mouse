# Granola Wand 🪄

Point a Waveshare ESP32-S3 Touch AMOLED 1.8" at your laptop and it becomes an
air mouse: wave to move the cursor, press the BOOT button to click and drag.

Built at a hackathon. The device streams its IMU (~220 Hz) over USB serial; a
host-side Python script tracks orientation (gravity-anchored AHRS), casts a
pointing ray at the screen, and drives the macOS cursor — "recentered inertial
pointing."

## Hardware

Waveshare ESP32-S3-Touch-AMOLED-1.8 (V2: CO5300 panel, QMI8658 IMU). Two units
supported, identified on-screen by badge (🟠 A / 🔵 B) and resolved by chip MAC
— see `devices.conf` and `./scripts/device list`.

## Quick start

```sh
# prereqs: eim with ESP-IDF v6.0.2, uv
./scripts/build && ./scripts/build flash B   # build + flash firmware
uv run proto/wand_mac/wand_proto.py --port $(./scripts/device port B)
```

1. Place the device flat on the desk, hands off → **Enter** (3 s gyro bias)
2. Pick it up flat like a remote, aim at screen center → press **`r`**
3. Wave. **BOOT** = left click; hold it to drag. **`r`** re-centers anytime;
   **`q`** quits.

Tuning: `--sens 1.5` scales cursor speed; `GYRO_DEADBAND_DPS` (top of
`wand_proto.py`) trades drift-freeze against slow-motion sensitivity.

## What's on the device

ArUco marker (legacy calibration target), device badge, and live IMU telemetry
bars — 3 green gyro axes (red at clipping) + 3 cyan accel axes.

## Development

- `--guided` runs an app-coached test protocol with timed steps; `--record`
  saves annotated raw captures; `--replay FILE` re-runs any capture offline.
- `proto/logs/` holds hardware recordings used as replay fixtures.
- `AGENTS.md` has contributor ground rules; `ARCHITECTURE.md` records design
  decisions; `DEMO.md` is the demo script.

## License

MIT
