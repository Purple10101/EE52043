# Gripper

Single-pinion, twin-rack parallel gripper driven by a TowerPro SG90, mounted on the EduSCARA Z-shaft
with the jaws 35 mm toward the elbow. Two variants share one print set:

- `SG90/` - plain gripper. Design notes: `SG90/v1.1 SG90 design notes.md`.
- `SG90 + Camera/` - adds a slide-in cradle for a XIAO ESP32S3 Sense looking straight down on the
  Z-shaft axis. Design notes: `SG90 + Camera/v1.1 SG90 + Camera design notes.md`.
- `Print/` - print-oriented STLs, `.3mf` plates, fit-test coupons, and the
  **print and assembly guide** (`Print/v1.1 print and assembly guide.md`): slicer settings,
  hardware list and assembly order.

Fusion cloud: project *Josh Robot Gripper* / folder *Gripper v1.1*. The EduSCARA + Gripper arm-scene
files are only there (too big for the repo).

![SG90 gripper](<SG90/Gripper v1.1 animation.gif>)

## Revision log

| Version | Date | Status |
|---|---|---|
| v1.1 | 2026-10-15 | Designed and checked in CAD (interference, printability, arm clearance). Not yet printed. Copied in from `Robot Resources\gripper\v1.1`. |
