# Camera tower

Fixed overhead mount for the Arducam IMX708 USB camera (B030401, legs removed), at 200 mm lens
height, aimed at the plate centre from the corner of the rig.

- `build_camera_tower.py` - **the source.** A Fusion API script that builds the tower in the active
  design. Everything is in the rig scene frame (mm, z up, origin at the robot base centre). The
  dimensions and requirements are the constants at the top.
- `print/` - exports from the script: `camera_tower_base.stl`, `camera_tower_head.stl` and the
  `camera_tower_assembly.step`. Re-export and commit these with every script change.
- `arm_sweep.py`, `arm_pose.py` - sweep the EduSCARA through its joint limits in the Fusion scene
  and check for collisions with the tower. `arm_pose_originals.json` stores the arm's original
  occurrence transforms so `arm_pose.restore()` can put them back.
- `collision_map.png`, `arm_safe_range.gif`, `arm_sweep_elbow45.gif` - sweep results. The individual
  frames are written to `sweep_frames/`, which is git-ignored.

## Parts

| Part | Notes |
|---|---|
| Base (print) | 45 x 45 x 5 foot on the rig plate holes, 75 mm socket for the extrusion |
| 2020 extrusion | 167 mm; slides +-25 mm in the socket to adjust height |
| Camera head (print) | Sleeve over the extrusion top (4 x M5 into T-nuts) and the camera pocket at the fixed aim angle. The camera slides in from the upper end. |

## Revision log

| Version | Date | Status |
|---|---|---|
| v1 | 2026-10-14 | Designed and collision-checked in CAD. Not yet printed. |
