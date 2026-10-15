# CAD

Printed parts for the rig, one folder per part.

- `gripper/` - SG90 rack-and-pinion gripper, plain and camera versions (Fusion `.f3d`).
- `camera_tower/` - overhead camera mount, built by a Fusion API script.

## Conventions

- **Git is the versioning.** Revise files in place, don't add `v2/` folders. Tag the commit that
  each physical print came from (`gripper-v1.1`, `camera-tower-v1`) so the geometry of any part on
  the bench can be found again.
- **Commit the source and its exports together**: the `.f3d` or build script, plus the STEP / STL /
  `.3mf` made from it, so they never drift apart.
- **Each part has a README** with its print settings, hardware and a revision log (what was printed,
  when, and what needed changing).
- **Keep big files out.** Full EduSCARA + part scenes (about 11 MB each) stay in Fusion cloud;
  rendered sweep frames and G-code are git-ignored.
