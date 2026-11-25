# LEAP cube-turn SAC: current handoff (2026-10-11)

**Superseded by `sac_current_status.md` (2026-10-12).** The v2 geometry/observation repairs, stage-0 resets, new SAC and residual experiments, repeatability limits, and pending wooden-support measurements are recorded there. The old reward issue discussed below has been repaired; it is not an outstanding task.

No motor or camera device was opened during this offline work. No new policy is approved for motor control.

## Requested whole-cube order

Starting green-up/orange-front, the intended orientation order is red-up/green-front (first vertical flip), red-up/yellow-front (right face toward camera), then white-up/red-front (second vertical flip). `sac_ordered_cube_goal_result.json` checks the corresponding face-ID geometry. The present SAC training task covers **only the first vertical flip**.

## Evidence from the physical pass

In the October 11 05:15 sequence, the cube stayed green-up. Relative to mount ID41, stage 2 to stage 4 produced 35.4 degrees of total rotation, **-8.8 degrees of signed forward tip** and +34.0 degrees of lateral turn. The wrong turn is real cube motion, not simply a moving camera or hand mount. The user identified the physical obstruction as the rear-protruding second motor housing catching the **external wooden support** under the hand; that support is not modeled in MuJoCo. See `sac_relative_cube_motion_result.json` and `sac_sim_status.md`.

## Model and training result

An October 11 same-session stage-2 pose was reconstructed from ID41 and two cube markers. A nearby reset offset changes two-finger contact sharply over only 0.625 mm, while the marker-bridge estimates differ by 1.49 mm. A friction/offset fit can reproduce the observed wrong-turn angles on that *same* pass, but predicts 23.7 mm cube travel versus 11.1 mm measured. That is not independent validation.

`sac_forward_task_env.py` encodes the signed first-flip reward; `sac_stage2_start_env.py` samples seven nearby offsets. The 2,000-step and 20,000-step SAC experiments both had **0/7 directional successes** in independent deterministic simulation replay. The 20,000-step policy also had 0/7 correct red-up/green-front face pairs, and commonly displaced the cube by more than 110 mm. Its checkpoint stays offline. See `sac_forward_stage2_20k/directional_eval_result.json`.

## Ruler photo received; next offline work

The user supplied the requested ruler-and-side photo and then provided its original path, `C:/Users/김준서/Documents/카카오톡 받은 파일/KakaoTalk_20261011_192435215.jpg`. Do not repeat the old photo request. The 4000x3000 original is now copied into `support_ruler_20261011/source.jpg`; SHA256 and the pixel transform are in `support_ruler_20261011/analysis.json`. `rectify_support_ruler.py` rotates original pixels clockwise by approximately 20.35 degrees using the ruler-label baseline. Inspect `support_ruler_20261011/support_detail_preview.jpg` for the leveled close-up. This is deterministic image-roll correction, **not complete perspective or 3-D calibration**. It shows the hand base, rear motor housings, support plate, wooden support edge and ruler more clearly. Together with the user's description, it documents the external-support obstruction layout. The oblique viewpoint, cable occlusion and uncertain depth alignment of the ruler still prevent a defensible precise millimeter clearance. No exact clearance or calibrated wooden-support collision geometry has yet been derived.

The subsequent code review also found that `sac_forward_task_env.py` projects the initial front-face normal onto the initial top plane to compute yaw. For a pure forward tip, that projected vector reverses after 90 degrees, so 89 to 91 degrees can jump from zero to 180 degrees of apparent yaw despite no lateral turn. Existing quarter-turn tests exercise a separate offline gate and do not cover this reward boundary. Correct and test this before further training. The 0/7 rollout result does not isolate contact mismatch as its cause: reward defects, limited training and the stage-2-versus-stage-0 starting-pose discrepancy remain competing explanations. The seven evaluation seeds also do not ensure seven distinct sampled reset offsets.

No further user capture is required to repair these software issues. Subsequent support modeling must retain uncertainty where the new photo does not resolve dimensions; physical transfer still requires independent replay validation.
