# Test protocol — round 4 (AIR, controlled conditions)

One motion at a time. After each numbered step there's an EXPECT line — if
reality differs, note what actually happened; that difference is the data.
Return the full `proto/logs/air2.log` plus your notes.

Cable slack reminder: hold some USB slack in your device hand.

## Command
```sh
uv run proto/wand_mac/wand_proto.py --debug 2>&1 | tee proto/logs/air2.log
```

## Steps

1. **Desk bias**: device flat on desk, screen up, hands off → press **Enter**,
   wait ~3 s.
   - EXPECT: `GYRO BIAS` line, per-axis std < 1 dps, no movement warning.
2. **Calibrate**: hold it **USB connector down, screen facing the webcam**
   (~50 cm; screen will be landscape — correct). Steady → **spacebar**.
   - EXPECT: `CALIBRATION position_mm=[..]` with z ≈ your distance in mm.
3. **Flip + recenter**: flip flat into your hand like a remote, long axis
   aimed at **screen center**. Press **r**.
   - EXPECT: `# !!! RECENTERED !!!` in the terminal AND cursor jumps to
     ~screen center. If you don't see the message, press r again — the log
     must contain it.
4. **Yaw check (single axis)**: SLOWLY rotate left ~15°, back to center,
   right ~15°, back. Nothing else — no tilting. ~10 s total.
   - EXPECT: cursor moves LEFT / center / RIGHT / center, height steady.
   - NOTE for us: when you rotated LEFT, which way did the cursor go?
5. **Pitch check (single axis)**: SLOWLY tilt up ~15°, back, down ~15°, back.
   No left/right. ~10 s.
   - EXPECT: cursor moves UP / center / DOWN / center, horizontal steady.
   - NOTE: when you tilted UP, which way did the cursor go?
   - If either check moves the WRONG SINGLE axis direction, continue anyway
     (sign fix for us). If motion is DIAGONAL or axes are SWAPPED: quit (q)
     and redo from step 1 with:
     `uv run proto/wand_mac/wand_proto.py --debug --mount-roll -90 2>&1 | tee proto/logs/air2b.log`
6. **Hold test**: aim at screen center, press **r**, then hold as still as
   you can for **10 s**.
   - EXPECT: cursor stays within a small area around center.
   - NOTE: roughly how far did it wander (a coin? a fist? half the screen?)
     and was it jittering or gliding?
7. **Corner tour**: point at each corner, hold ~2 s: top-left → top-right →
   bottom-right → bottom-left.
   - EXPECT: cursor reaches each corner region without fighting you.
8. **Drift measurement** (the one we're still missing): set the device flat
   on the desk, hands off, **60 seconds**, do not touch it.
   - EXPECT: HUD switches to REST, a `# bias updated [..]` may appear, and
     `drift=` shows a real number. This is our headline metric.
9. **Free wave**: pick up, aim center, press **r**, wave freely ~15 s with
   one brisk flick.
   - EXPECT: maybe a brief SATURATION!/CLIP! that CLEARS within ~2 s.
10. Press **q**.
    - EXPECT: SUMMARY line with `recenters=` ≥ 2.

## Report back
- Full `proto/logs/air2.log` (and air2b.log if you did the -90 rerun)
- Answers to the NOTE questions in steps 4, 5, 6
- One-line verdict: closer to "wand" or closer to "ouija board"?

## Not in this round
- TABLE round 2 comes after the phase-1 improvements land (planar constraint,
  vibration gating, cursor simulation).
- Audio/acoustic experiments need a reflash — separate session.
