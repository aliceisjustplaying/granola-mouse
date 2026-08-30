# Project rules (all agents)

## File reading — context safety
- NEVER read large files wholesale (logs, CSVs, .bin captures, session .jsonl,
  build output). Reading a big file at once overloads your context window and
  kills the session before compaction can save you.
- Use `head`/`tail`/`grep`/`rg`, line offsets+limits, or a short python script
  that prints SUMMARY STATISTICS instead of contents.
- Binary captures (.bin, PCM): only ever analyze with scripts; never dump.
- proto/logs/ and /tmp captures are radioactive: sample, don't slurp.

## Git
- Do NOT run `git commit` — the parent session owns git.

## Hardware / serial ports
- Never open a /dev/cu.* port unless your task explicitly assigns you a device.
- Device labels: resolve ports via `./scripts/device port A` (audio test unit)
  or `./scripts/device port B` (wand unit). Never hardcode usbmodem numbers.

## Scope
- Touch only the files/directories your task assigns. Other agents often work
  in parallel in this repo.
