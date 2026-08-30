# Test protocol — round 3 (AIR track + TABLE track)

Run both tracks. Save and return the full log files from `proto/logs/`.
Before starting: give yourself generous USB cable slack (hold some slack in
your device hand). All commands run from the repo root.

---

## Track A: AIR (wand pointing)

### Command
```sh
uv run proto/wand_mac/wand_proto.py --debug 2>&1 | tee proto/logs/air1.log
```

### Steps
1. **Desk bias**: place the device flat on the desk, screen up, hands off.
   Press **Enter**. Wait ~3 s.
   - Expect: `GYRO BIAS` around (6, 5.6, 0.3) dps and per-axis std < 1 dps.
   - Bad: a "moving" warning → redo (hands off the desk too).
2. **Calibrate**: pick the device up and hold it **USB connector pointing
   down, screen facing the webcam**, ~50 cm away. (This is the one unique
   pose those two constraints allow — the screen will be in landscape; that
   is expected and fine.) Hold steady, wait for the marker-detected overlay
   in the preview, then press **spacebar**.
   (Do not worry about which way the marker "looks" — the `r` recenter in
   step 3 absorbs orientation offsets.)
   - Expect: a `CALIBRATION position_mm=[..]` line; the z value ≈ your real
     distance in mm (sanity: ~-400 to -600).
3. **Flip + recenter**: flip the device flat into your hand, screen up, held
   like a remote with the long axis pointing at the **center** of your laptop
   screen (a short edge faces the screen). **Note whether the USB connector
   is on your left or your right and tell us** — both work for this test, and
   your sweep logs will tell us which grip to standardize. Hold it there and
   press **r**.
   - Expect: `# recentered` and the cursor jumping to ~screen center.
4. **Yaw sweep**: slowly rotate left ↔ right (like turning a key), ~10 s.
   - Expect: cursor tracks left/right, y roughly steady.
5. **Pitch sweep**: slowly tilt up ↕ down, ~10 s.
   - Expect: cursor tracks up/down, x roughly steady.
   - Note WHICH direction the cursor moves when you tilt up.
6. **Corner tour**: point at each screen corner and hold ~2 s each, in this
   order: top-left → top-right → bottom-right → bottom-left.
   - Expect: cursor reaches each corner region.
7. **Drift rest**: set the device down flat on the desk, hands off,
   for a full **30 seconds**. Do not touch it.
   - This produces our first real `drift=` measurement (HUD, °/min).
8. **Free wave**: pick it back up, press **r** while aiming at screen center,
   then wave freely for ~15 s like a wand. Try a brisk flick or two.
   - Watch for `CLIP!` in the HUD and any `# serial lost, reconnecting...`.
9. Press **q** to quit (prints SUMMARY).

### Report back
- Full `proto/logs/air1.log`
- Your words: shakiness vs last round? Does the cursor go where you point
  right after `r`? Did it feel "off" again by the end of step 8?

---

## Track B: TABLE (desk sliding capture)

### Command 1 — record
```sh
uv run proto/table_zupt/table_zupt.py --capture proto/logs/table1.csv
```
Device flat on the desk, screen up, hand resting on it like a mouse.
Perform this exact sequence (approximate distances are fine — but tell us
what they actually were; using a mousepad edge as a ruler helps):

1. Still 3 s
2. Slide **right ~10 cm**, briskly (~half a second), then still 2 s
3. Slide **back left ~10 cm**, still 2 s
4. Slide **away from you ~10 cm**, still 2 s
5. Slide **toward you ~10 cm**, still 2 s
6. Draw a **circle, ~10 cm across**, one smooth motion, still 3 s
7. **Ctrl-C** to stop the capture

Keep the device FLAT — normal mousing pressure, no rocking, no lifting.

### Command 2 — analyze
```sh
uv run proto/table_zupt/table_zupt.py --analyze proto/logs/table1.csv 2>&1 | tee proto/logs/table1_analysis.log
```

### Report back
- Full `proto/logs/table1_analysis.log`
- The generated PNG path plot (path next to the CSV) — does the drawn shape
  resemble what you actually did?
- The real distances you slid, if you measured them.

---

## Known issues going in (don't be surprised)
- A constant ~90° azimuth offset exists until you press `r` (marker renders
  rotated on the AMOLED — known firmware bug, fix pending).
- Filter tuning is provisional: if the cursor still shakes, that's a finding,
  not a failure — we'll lower MIN_CUTOFF.
- TABLE mode is a physics feasibility probe, not a feature yet. Sloppy
  reconstruction is a legitimate result.
