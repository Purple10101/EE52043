"""Dry run of the cycle's front end: snapshot, detect, map to the robot frame, check reach, and print
the move each resistor would get. Nothing is sent to the arm.

    python -m robot_control.task.plan_picks --live
    python -m robot_control.task.plan_picks "../Robot Resources/robot_offline/data/reference/2026-10-29"

For each frame it prints one line per detected body (robot-frame centre, yaw, size, joints or the
reason it can't be reached) and the ScaraController call that would put the open gripper above it,
and writes to --out (default logs/plan_picks/): <id>_plan.png, the top-down view with the reach band,
outlines and numbers (green reachable, red not), and <id>_plan.json. --live also saves the raw frame
there, so the run can be replayed.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from robot_control.config import (ArmSettings, GripperSettings, KeepOutMap, RigSettings, VisionSettings,
                                  WorkspaceSettings)
from robot_control.kinematics import KeepOut, Kinematics
from robot_control.vision.rig import robot_to_scene

OK_COLOUR, BAD_COLOUR, BAND_COLOUR = (60, 200, 60), (40, 40, 230), (255, 160, 0)


def kinematics():
    """Kinematics with the arm's limits and every configured keep-out zone."""
    return Kinematics(ArmSettings.from_config(), [KeepOut(m) for m in KeepOutMap.from_config()])


def plan(detections, kin, gripper, workspace):
    """(detection, IKResult) per body, reachable ones first, then by score."""
    if any(gripper.tool_offset):
        raise NotImplementedError(f"{gripper.name}: off-axis tool offsets are not handled yet")
    out = []
    for d in detections:
        jaws = d.yaw + 90.0                     # jaws close across the body
        result = kin.ik(*d.centre, jaws - gripper.jaw_axis_at_zero_deg, workspace.hover_z, gripper.symmetric)
        out.append((d, result))
    out.sort(key=lambda dr: (not dr[1].ok, not dr[0].plausible, -dr[0].score))
    return out


def report(planned, workspace):
    lines = [f"{'#':>2} {'score':>5}  {'robot x, y (mm)':>16} {'yaw':>6}  {'L x W (mm)':>11}  "
             f"{'base':>6} {'elbow':>6} {'wrist':>6}  result"]
    for i, (d, r) in enumerate(planned, 1):
        x, y = (v * 1000 for v in d.centre)
        size = f"{d.length * 1000:4.1f} x {d.width * 1000:3.1f}" + ("" if d.plausible else "?")
        joints = f"{r.joints.j0:6.1f} {r.joints.j1:6.1f} {r.joints.j2:6.1f}" if r.ok else " " * 20
        result = (f"move_coordinates({d.centre[0]:.4f}, {d.centre[1]:.4f}, {r.tool_angle:.1f}, "
                  f"{workspace.hover_z:.3f}, {workspace.move_time:.1f})" if r.ok else f"SKIP: {r.reason}")
        lines.append(f"{i:>2} {d.score:5.2f}  {x:7.1f}, {y:7.1f} {d.yaw:6.1f}  {size:>11}  {joints}  {result}")
    return "\n".join(lines)


def text(img, s, org, colour, scale=0.5):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 1, cv2.LINE_AA)


def draw_plan(img, planned, cal, kin, rig, from_rect=lambda p: p, text_scale=0.8):
    """The arm's reach band and each body, numbered as in the report (green reachable, red not, the
    first reachable one ringed: the next pick), on img in place. from_rect: rectified px (N × 2) to
    img px, for drawing on a view other than the top-down one."""
    from robot_offline.pre import rectify
    st = cal.settings
    w, h = st.size
    fixed = lambda p: np.round(np.asarray(from_rect(p), float).reshape(-1, 2) * 16).astype(np.int32)
    angles = np.radians(np.linspace(-180, 180, 1441))
    for r in kin.reach():
        ring = rectify.mm_to_rect(robot_to_scene(np.stack([r * np.sin(angles), r * np.cos(angles)], 1), rig), st)
        inside = (ring[:, 0] >= 0) & (ring[:, 0] <= w) & (ring[:, 1] >= 0) & (ring[:, 1] <= h)
        # Only the parts over the rectified area: mapped to the camera view, points far off it go wild.
        breaks = np.flatnonzero(np.diff(inside.astype(int))) + 1
        for run in np.split(np.arange(len(ring)), breaks):
            if inside[run[0]] and len(run) > 1:
                cv2.polylines(img, [fixed(ring[run])], False, BAND_COLOUR, 2, cv2.LINE_AA, 4)
    next_pick = next((i for i, (_, r) in enumerate(planned) if r.ok), None)
    for i, (d, r) in enumerate(planned):
        colour = OK_COLOUR if r.ok else BAD_COLOUR
        outline = rectify.to_rectified(d.outline_px, cal)
        cv2.polylines(img, [fixed(outline)], True, colour, 2, cv2.LINE_AA, 4)
        centre = fixed(rectify.mm_to_rect(robot_to_scene([d.centre], rig), st))[0]
        cv2.circle(img, tuple(int(v) for v in centre), 3 * 16, colour, -1, cv2.LINE_AA, 4)
        if i == next_pick:
            cv2.circle(img, tuple(int(v) for v in centre), 22 * 16, (255, 255, 255), 2, cv2.LINE_AA, 4)
        x, y = np.round(np.asarray(from_rect(outline), float).reshape(-1, 2).max(0) + (4, 0)).astype(int)
        text(img, str(i + 1), (int(x), int(y)), colour, text_scale)
    return img


def overlay(image, planned, cal, kin, rig):
    """The top-down view with the arm's reach band and each body numbered as in the report."""
    return draw_plan(image.copy(), planned, cal, kin, rig)


def to_json(snapshot_id, planned, cal):
    return {"snapshot": snapshot_id, "camera_mm": list(cal.fit["camera_mm"]) if cal.fit else None, "bodies": [
        {"score": d.score, "plausible": d.plausible, "centre_m": list(d.centre), "yaw_deg": d.yaw,
         "length_m": d.length, "width_m": d.width, "outline_robot_m": d.outline_robot.round(6).tolist(),
         "outline_px": d.outline_px.round(1).tolist(), "reachable": r.ok, "reason": r.reason,
         "tool_angle_deg": r.tool_angle, "joints": None if not r.ok else
         {"j0": r.joints.j0, "j1": r.joints.j1, "j2": r.joints.j2, "z": r.joints.z}}
        for d, r in planned]}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("frames", nargs="?", help="a saved raw frame, or a folder of them")
    src.add_argument("--live", action="store_true", help="one snapshot from the tower camera")
    p.add_argument("--calibration", help="a saved rectify calibration (.json) to start from, instead of "
                                         "finding the plate from scratch")
    p.add_argument("--conf", type=float, help="confidence threshold (default: vision.toml)")
    p.add_argument("--out", default="logs/plan_picks", help="folder for the overlays, plans and live frames")
    args = p.parse_args(argv)

    from robot_offline.pre import rectify
    from robot_control.vision.camera import LiveCamera, ReplayCamera
    from robot_control.vision.detector import Detector

    vision = VisionSettings.from_config()
    if args.conf is not None:
        vision = VisionSettings(vision.weights, args.conf)
    rig, workspace, gripper = RigSettings.from_config(), WorkspaceSettings.from_config(), GripperSettings.from_config()
    kin = kinematics()
    detector = Detector(vision, rig, workspace)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    camera = LiveCamera() if args.live else ReplayCamera(args.frames)
    frames = [camera.grab()] if args.live else camera.frames()
    start = rectify.Calibration.load(args.calibration) if args.calibration else None
    try:
        for snapshot_id, frame in frames:
            if args.live:
                cv2.imwrite(str(out_dir / f"{snapshot_id}.png"), frame)
            t = time.time()
            try:
                cal = detector.calibrate(frame, start)
            except rectify.PlateNotFound as e:
                print(f"\n{snapshot_id}: plate not found ({e})")
                continue
            detections, balanced = detector.detect(frame, cal, snapshot_id)
            planned = plan(detections, kin, gripper, workspace)
            reachable = sum(r.ok for _, r in planned)
            print(f"\n{snapshot_id}: {len(planned)} bodies, {reachable} reachable ({time.time() - t:.1f} s)")
            if planned:
                print(report(planned, workspace))
            cv2.imwrite(str(out_dir / f"{snapshot_id}_plan.png"), overlay(balanced, planned, cal, kin, rig))
            (out_dir / f"{snapshot_id}_plan.json").write_text(json.dumps(to_json(snapshot_id, planned, cal), indent=1))
    finally:
        camera.close()
    print(f"\nOverlays and plans in {out_dir}")


if __name__ == "__main__":
    sys.exit(main())
