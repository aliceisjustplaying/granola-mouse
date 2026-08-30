# Hardware & reuse notes (from ~/src/a/tinydraw)

Board: Waveshare ESP32-S3 Touch AMOLED 1.8" — **V2 revision** (CO5300 display +
CST820 touch, per tinydraw `DEVELOPING.md:155`).
Wiki: https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8

## Onboard peripherals

| Part | Chip | Bus / pins | Used by tinydraw? |
|---|---|---|---|
| MCU | ESP32-S3, 240 MHz, 8 MiB octal PSRAM, 16 MiB flash | — | yes |
| Display | CO5300 AMOLED 368×448 | QSPI: SCLK 11, D0–D3 4/5/6/7, CS 12, TE 13 (~40 MHz effective) | yes (`co5300_panel_transport.cpp`) |
| Touch | CST820 (CST816S-compatible driver) | I2C0, INT GPIO 21 | yes (`physical_touch.cpp`) |
| I2C bus | shared | I2C_NUM_0: SDA 15, SCL 14 @400 kHz | yes (created in `co5300_panel_transport.cpp:127`) |
| IMU | **QMI8658** 6-axis (gyro + accel) | I2C0 (addr 0x6A/0x6B) | **no — we write this** |
| Audio | **ES8311 codec + MEMS mic + speaker amp** | I2S + I2C0 | **no — we write this** |
| PMU / battery | AXP2101 @ 0x34 | I2C0 | yes (`power_manager.cpp`) |
| RTC | PCF85063 | I2C0 | yes (`rtc_clock.cpp`, likely not needed for us) |
| Buttons | BOOT = GPIO 0; PMU power button (short/long press) | — | yes (`hardware_app.cpp:36`, `power_manager.cpp`) |
| Radio | Wi-Fi 2.4 GHz + **Bluetooth 5 LE only** (no BT Classic) | — | no — BT not enabled in their sdkconfig |

## Reusable from tinydraw (esp32/main/)

- `co5300_panel_transport.{h,cpp}` + `physical_display.{h,cpp}` — full AMOLED
  bring-up, DMA push, also creates the shared I2C bus.
- `physical_touch.{h,cpp}` — touch events for on-screen controls.
- `power_manager.{h,cpp}` — battery %, charging state, 4s-hold shutdown config.
- Build infra: `scripts/bootstrap-idf` (eim-managed **ESP-IDF v6.0.2**),
  `scripts/esp32` wrapper, `sdkconfig.defaults` (240 MHz, octal PSRAM, perf opt).
- BOOT-button long/short press pattern in `hardware_app.cpp`.
- Their drawing core/journal/USB export: not needed for a mouse.

## New work

1. **BLE HID mouse (HOGP)** — ESP32-S3 is BLE-only, so it must be a BLE HID
   mouse (works fine on macOS/Win/Linux/Android/iPadOS). Base it on ESP-IDF's
   `esp_hid_device` example (esp_hid component or NimBLE). Needs
   `CONFIG_BT_ENABLED` etc. — tinydraw's sdkconfig has no BT at all.
2. **QMI8658 driver** — small I2C driver, or pull Waveshare's BSP component
   `waveshare/esp32_s3_touch_amoled_1_8` which bundles one.
3. **Gyro → cursor mapping** — air-mouse style: gyro yaw/pitch angular rate →
   dx/dy with deadzone + low-pass (like LG Magic Remote / Wii). Accel for tilt
   correction if drift annoys us.
4. **Screen/touch UI** — sensitivity, left/right click buttons, scroll strip,
   battery display.
5. **Echolocation (stretch)** — ES8311 speaker emits ultrasonic chirp
   (~18–22 kHz), mic captures echo; single mic ⇒ 1D range / doppler only
   (FingerIO / LLAP-style). Realistic hackathon use: surface-proximity /
   lift-off detection, not full 2D positioning.

## Gotchas

- Everything hangs off one I2C bus (SDA 15 / SCL 14) — one
  `i2c_master_bus_handle_t` must be shared (tinydraw creates it in the panel
  transport; worth extracting into its own module).
- GPIO 0 (BOOT) doubles as the ROM-flasher strap; fine as an input button.
- On battery, enter ROM flasher: power off → hold BOOT → short-press power.
- V1 vs V2 board rev matters for display/touch drivers — check the back label;
  tinydraw code is V2 (CO5300/CST820).
