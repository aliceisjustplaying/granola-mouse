# Architecture — "Granola Mouse" 🥣🖱️

Waveshare ESP32-S3 Touch AMOLED 1.8" (V2 rev) as a BLE HID air mouse.
Gyro drives the cursor, touch screen provides buttons/controls, echolocation is
a stretch goal for surface-proximity sensing. See `HARDWARE_NOTES.md` for pins
and reuse receipts.

## Big picture

```
                ┌─────────────────────────────────────────────────┐
                │                    app_main                     │
                │        (task startup + central event queue)     │
                └─────────────────────────────────────────────────┘
                     │               │                │
        ┌────────────┴───┐   ┌───────┴────────┐   ┌───┴────────────┐
        │  imu task      │   │  ui task       │   │  ble task      │
        │  (200 Hz)      │   │  (~30 Hz)      │   │  (NimBLE host) │
        └────────────────┘   └────────────────┘   └────────────────┘
             │                    │      │              ▲
             ▼                    │      │              │ drain @ conn interval
      ┌──────────────┐            │      │      ┌───────┴────────┐
      │ pointer      │  dx/dy     │      │      │ report         │
      │ engine       ├────────────┼──────┼─────▶│ accumulator    │
      │ (fusion,     │            │      │      │ (atomic dx/dy/ │
      │  deadzone,   │  buttons/wheel    │      │  buttons/wheel)│
      │  curves)     │◀───────────┘      │      └────────────────┘
      └──────────────┘   touch regions   ▼
             ▲                     ┌───────────┐
      ┌──────┴───────┐             │ display   │
      │ qmi8658      │             │ (status,  │
      │ driver (I2C) │             │ settings) │
      └──────────────┘             └───────────┘

  drivers (bottom): board_bus (I2C0) · display (CO5300) · touch (CST820)
                    power (AXP2101) · buttons (GPIO0 + PMU) · audio (ES8311)★
  ★ = stretch
```

## Modules

### Drivers (`main/board/`) — mostly lifted from tinydraw

| Module | Source | Notes |
|---|---|---|
| `board_bus` | **extracted** from `co5300_panel_transport.cpp:127` | Owns the single `i2c_master_bus_handle_t` (I2C0, SDA 15 / SCL 14). Everything else takes the handle. tinydraw buried this in the panel transport; we hoist it because the IMU needs the bus too. |
| `display` | copy `co5300_panel_transport.{h,cpp}` + a trimmed `physical_display` | Keep init + `push_rect`; drop tinydraw's toolbar compositor / world-canvas plumbing. |
| `touch` | copy `physical_touch.{h,cpp}` | CST816S driver, INT GPIO 21. |
| `power` | copy `power_manager.{h,cpp}` | Battery %, charging, 4 s-hold shutdown. |
| `buttons` | pattern from `hardware_app.cpp:36` | BOOT (GPIO 0) short/long press. |
| `imu` | **new**: `qmi8658.{h,cpp}` | I2C addr 0x6A/0x6B. Config: gyro ±512 dps, accel ±4 g, ~200 Hz ODR. Crib register map from Waveshare BSP. |
| `audio` ★ | **new**: ES8311 + I2S full-duplex | Only if echolocation happens. |

### Services (`main/`) — all new

**`ble/hid_mouse`** — NimBLE HOGP device.
- Standard boot-mouse report map (buttons ×3, X, Y, wheel), based on ESP-IDF
  `ble_hid_device_demo` / `esp_hidd`.
- Owns advertising, bonding, reconnection. Exposes:
  `hid_mouse_init()`, `hid_mouse_connected()`, and the **report accumulator**.
- Report accumulator = atomics for pending dx/dy/wheel + button state. Producers
  add deltas any time; the BLE sender drains & sends once per connection
  interval (~11 ms). Natural coalescing, no queues to overflow, latest-wins —
  same spirit as tinydraw's `enqueue_latest`.

**`pointer/pointer_engine`** — pure logic, no hardware (unit-testable on host).
- Input: timestamped gyro/accel samples. Output: dx/dy floats + fractional
  carry (so slow movements aren't truncated to zero).
- Pipeline: gyro bias calibration (auto, when at rest) → deadzone → low-pass →
  sensitivity curve (slow = precise, fast = flick) → axis mapping
  (yaw→X, pitch→Y, like LG Magic Remote).
- Mute flag (cursor freeze while user touches the settings UI, or via button).

**`ui/control_surface`** — screen + touch as the mouse's "body".
- Layout: big left/right click zones, scroll strip on the edge, status bar
  (BLE state, battery %), settings toggle (sensitivity, recenter).
- Touch events route to: button state → accumulator; scroll strip → wheel;
  settings taps → local UI only (pointer muted while in settings).
- Render straight RGB565 rects via `display.push_rect` — no framework needed.

**`echo/echo_ranger`** ★ — stretch.
- Emit 18–22 kHz chirp on speaker, capture mic via I2S, cross-correlate for
  1D proximity. Realistic scope: **lift-off detection** (auto-mute pointer when
  the device is picked up/set down), not 2D positioning.
- Isolated behind one interface (`echo_ranger_proximity()`); nothing else
  depends on it existing.

## Tasks & priorities

| Task | Prio | Rate | Job |
|---|---|---|---|
| `imu_task` | high | 200 Hz | read QMI8658 → pointer_engine → accumulator |
| NimBLE host | high (stack-managed) | conn interval | drain accumulator → HID input report |
| `ui_task` | low | ~30 Hz render, touch on INT | touch regions, status, settings |
| `power_poll` | low | 1 Hz | battery status → UI (tinydraw pattern) |
| `echo_task` ★ | mid | ~10 Hz pings | proximity → pointer mute |

Single event queue for control events (buttons, power, connection changes);
hot path (motion → BLE) bypasses it via the accumulator.

## Repo layout

```
granola_esp32_hackathon/
├── CMakeLists.txt
├── sdkconfig.defaults        # base from tinydraw + CONFIG_BT_ENABLED, NimBLE
├── partitions.csv            # simple: app + nvs (bonding keys live in NVS)
├── scripts/
│   ├── bootstrap-idf         # copied from tinydraw (eim, ESP-IDF v6.0.2)
│   └── build                 # thin idf.py wrapper: build / flash PORT / monitor
└── main/
    ├── app_main.cpp
    ├── board/                # board_bus, display, touch, power, buttons, imu
    ├── ble/hid_mouse.{h,cpp}
    ├── pointer/pointer_engine.{h,cpp}
    ├── ui/control_surface.{h,cpp}
    └── echo/echo_ranger.{h,cpp}      # ★ stretch
```

## SCOPE CUT (decision 2026-08-30, late): audio + camera dropped

Time-boxed descope. DROPPED: (1) TABLE-ACOUSTIC / all audio experiments —
transport code stays committed as archive, no further work; (2) webcam ArUco
calibration — AIR mode is "recentered inertial pointing": desk bias → flip →
full recenter (r / touch), assumed distance + user sensitivity instead of
solvePnP. Remaining scope: BLE HID mouse slice, AIR mode (no camera),
TABLE-IMU (guided capture, stroke-based).

## TABLE mode: two independent implementations (decision 2026-08-30)

Table tracking ships as two side-by-side modes — neither replaces the other:

- **table-imu**: ZUPT stroke dead reckoning (proven on real captures:
  ~10 cm slides recovered at 5–10%; continuous drags — e.g. a 3.2 s circle —
  blow up as t² physics predicts). Enhancements (planar constraint, vibration
  motion gating, micro-rest ZUPT) must preserve baseline stroke behavior or
  stay opt-in.
- **table-acoustic**: CAT-lite (MobiCom '16) — the MacBook's stereo speakers
  emit inaudible ~19/20 kHz tones as a spatially separated beacon pair; the
  device mic (ES8311) streams to the Mac, which tracks per-tone phase → 2D
  relative position without error accumulation, fused with IMU. Separate
  firmware variant (GRANOLA_PROTO_AUDIO) and separate host script.

The mode manager treats AIR / TABLE-IMU / TABLE-ACOUSTIC as distinct modes
with per-mode axis conventions (device is held differently in each).

## Key decisions

1. **NimBLE, not Bluedroid** — smaller RAM/flash; we only need peripheral +
   HOGP. Report map borrowed from the Bluedroid demo works fine on NimBLE.
2. **Accumulator over queue** for motion — mouse deltas are additive; coalescing
   is correct by construction and BLE backpressure can't stall the IMU task.
3. **pointer_engine is hardware-free** — tune deadzone/curves with recorded
   sample dumps on the host instead of reflash-and-wave loops.
4. **Touch = buttons** — capacitive taps beat gyro-click (clicking physically
   moves the device; we also suppress motion for ~50 ms after a click, a
   standard air-mouse trick).
5. **Echolocation is opt-in** — 1 mic ⇒ 1D range only; scoped to lift-off
   detection so the core product never depends on it.

## Milestones

1. **It's alive**: skeleton builds; BLE pairs as "Granola Mouse"; cursor jiggles
   on a timer. (No sensors yet — proves the whole BLE path.)
2. **Motion**: QMI8658 driver + raw gyro→cursor. First real air-mouse moment.
3. **Clicks**: touch zones → left/right click + scroll strip.
4. **Feel**: bias calibration, deadzone, sensitivity curve, click suppression.
5. **Polish**: status UI, battery, settings, power management.
6. ★ **Echo**: chirp lift-off detection → auto-mute.
