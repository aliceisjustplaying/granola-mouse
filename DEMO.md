# MVP demo script — AIR wand (wired)

## Setup (before audience)
- Device B on USB, generous cable slack held in the device hand
- Rehearse once; pick a `--sens` value that feels right (start 1.0; 1.5 if sluggish)
- Terminal ready with the command below; font big

## The demo
```sh
uv run proto/wand_mac/wand_proto.py --port $(./scripts/device port B) --sens 1.0
```
1. Device flat on desk, hands off → **Enter** → "calibrating, 3 seconds"
2. Pick up, hold flat like a remote, aim at screen center → **`r`**
   → "and now it's a wand" 🪄
3. Wave: circles, figure-eights, point at UI elements
4. **Hold BOOT** → drag something → release. ("And it clicks.")
5. If the cursor wanders: aim center, tap **`r`** — say "recalibrating" like
   it's a feature (it is — motion pointers all do this; the product does it
   with a thumb tap on the touchscreen)
6. **`q`** to end

## Talking points
- Gyro-based inertial pointing, 220 Hz, gravity-anchored vertical
- "Recentered inertial pointing" — yaw has no compass; one keypress re-anchors
- BLE HID + table stroke mode exist in the codebase (parked for time)

## Failure modes & saves
- Cursor pinned at an edge → tap `r`
- Serial drop (cable yank) → script auto-reconnects; keep waving
- Total chaos → q, relaunch, 15 seconds to back-in-business
