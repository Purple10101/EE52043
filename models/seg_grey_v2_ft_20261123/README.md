# seg_grey_v2_ft_20261123

`seg_grey_v2` fine-tuned on real frames from the rig, 2026-11-23. The live segmentation model in both repos
from that day (EE52043 `robot_control/config/vision.toml`; robot_offline `config.SEG_WEIGHTS`, the default of
every tool). Same class (`resistor_body`), input (greyscale) and tiling (1024, overlap 128) as `seg_grey_v2`.

## Why

seg_grey_v2 was trained on synthetic frames only. On the rig it missed some plain, sharp resistors outright: on
2026-11-23 one by the crosshair on the "/" diagonal scored 0.00 straight and 0.19 turned a quarter turn, and
turned in software in 15 deg steps it flipped between about 0 and 0.9 (under 0.5 at 6 of 12 angles): the model was
on a knife-edge for it, not weak at one angle. Running the model two ways round and averaging
(`robot_control/vision/orientations.py`) helped a little but couldn't find a body missed both ways.

## Training

| | |
|---|---|
| Start | `models/seg_grey_v2/best.pt` |
| Train | `synth_v2` (800 synthetic frames) + real session `2026-11-23_a` frames 0001-0015, the real tiles listed 13 times so real is about 1 in 5 |
| Val | `synth_v2_val` (100) + `2026-11-23_a` frames 0016-0020 |
| Real labels | 20 frames, 6 resistors each, moved and turned between frames: seg_grey_v2's outlines (both ways round, checked on a contact sheet) plus 2 bodies it missed, outlined by colour against the plate. In the manifest as session `2026-11-23_a` (`label_source` "model" / "auto") |
| Run | 10 epochs, AdamW lr0 0.0002, warmup 0.5, batch 4, mosaic off for the last 2, rotation 180 and flips, grey. About 40 min on the RTX 3060 laptop GPU (slowed by memory spilling to system RAM; about 20 min without) |
| Run folder | robot_offline `runs/seg_grey_v2_ft_20261123` (its `handoff.json` says class `object`, rules 2: copied from dataset.toml, not what was trained; this folder's is corrected) |

## Scores

Resistor bodies at conf 0.5, found / false / missed. "Two-way" is the model run as it is and turned a quarter
turn, scores averaged.

| | seg_grey_v2 one way | seg_grey_v2 two-way | this model (one way = two-way) |
|---|---|---|---|
| Test sessions (`dataset.toml` test_sessions, 241 labelled bodies, never trained on) | 231 / 1 / 10 | 233 / 0 / 8 | **238 / 3 / 3** |
| Held out: `2026-11-23_a` 0016-0020 (30 bodies) | 27 / 0 / 3 | 29 / 0 / 1 | **30 / 0 / 0** |
| The crosshair body, picture turned 0-165 deg in 15 deg steps | under 0.5 at 6 of 12 (0.00-0.90) | | **0 of 12 (0.83-0.90)** |

The crosshair body is the same resistor as in the training frames, but that picture wasn't trained on.

The "false" detections are other objects, not made-up resistors: at epoch 3 the four were two screw heads and a
shiny metal lump (in two frames), in `2026-10-23_b` 0013-0015, where only resistors are labelled. seg_grey_v2
ignored them. So this model may outline screws and similar small parts on the plate as bodies; whatever handles
non-resistors downstream has to catch them.

Two-way averaging makes no difference with this model (every count above is the same either way); it can be
turned off to halve inference time (`COMBINE` and the callers of `predict_two_ways`).

## Limits

One session of real data: the kit's blue resistors on this plate, this lighting and camera. It helps most where
the rig looks like that; it isn't evidence about other parts or lighting. The larger fix is labelling the
sort-run frames (`data/real_frames/` in EE52043: about 125 frames, inventory and pre-labels) and training on
synthetic plus all of it.

## Switching back

EE52043: `weights = "models/seg_grey_v2/best.pt"` in `robot_control/config/vision.toml`, and `SEG_WEIGHTS` in
`robot_offline/config.py`. robot_offline: `SEG_WEIGHTS` in `src/robot_offline/config.py`.
