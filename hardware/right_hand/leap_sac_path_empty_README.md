# SAC-derived path: empty hand actuator check

**Latest result: revision4 also stopped at ID12 (2026-10-15 00:58).** Do not repeat
the same motion to troubleshoot. See `sac_id12_speed_comparison_result.json` and
the latest `sac_current_status.md`: power-OFF visual inspection of ID12 linkage/
cable is next; saturation was not observed in the recorded PWM/current samples.

**CURRENT: Revision 4.** Use [revision4 instructions](leap_sac_path_revision4_README.md).
Same launcher, EMPTY hand, 238 interpolated goals at50ms (11.9s nominal), active
PWM/velocity logged. Current450mA, PID and fault thresholds unchanged. 31 focused
tests and device-free check passed. Sections below are HISTORICAL; their revision
numbers, target counts and timing do not describe the current launcher.

## Revision 3: distinguish retracting targets

The revision-2 ID15 stop used a 0.25s window in which actual position stayed at
2714..2716 but the target was returning from 2784 to 2767. Absolute target change
incorrectly counted this as advancing load. The early progress guard now requires
monotonic movement AWAY from measured position in the sign of current error.
Targets returning toward the joint remain subject to all existing hard health
checks. Physical contact cause remains unknown; this is a logic fix, not proof
of unobstructed hardware. Path, current, PID and threshold values are unchanged.
23 tests pass including both actual archived events. Same command; verify
`Revision 3`, empty hand only, terminal SPACE once, automatic OFF at completion.

## Revision 2 after the ID16 plateau

ID16's new start and path goals are capped at 3238 ticks, below the observed
3253..3254 plateau. This is a provisional avoidance bound, not a certified
mechanical limit. Initial preload/root relief retain measured current angles.
Preparation raises roots, prepares thumb IDs1..4 first, then other fingers and
holds. The original simultaneous preparation failed the native tracking audit;
the ordered version followed by the complete empty path passes at both 30/40ms
preparation interpolation from the recorded preflight. Maximum preparation error
is 0.055rad. Actual starting posture and actuator dynamics may differ.

The path progress guard stops if six samples over 0.2..0.8s show <=2 ticks actual
motion while goals advance >=16 ticks, lag >=40 ticks and current >=80mA. It
detects the archived plateau before the old 0.18rad trip. Existing trips remain.
Nineteen relevant tests pass, including archived stall detection and shutdown.

Three user photos show distal links ahead of the support and wires around joints;
they do not prove the cause of the earlier stall. With motor power OFF, check
that loose wires have slack and are outside moving gaps without pulling their
connectors or forcing joints. Then run one EMPTY revision-2 trial. If a stall or
undervoltage repeats, return the log rather than retrying unchanged.

This tests the complete refined forward-flip joint path, after the already-tested
root relief and revised start. It does **not** run the live SAC actor and must not
be run with a cube. It measures actual tracking and control timing before further
policy integration. No camera, ID40, or ID41 is required.

1. Remove the cube. Secure the base and clear fingers/cables from the support.
2. Close other motor programs. Connect motor power and USB.
3. Run `leap_sac_path_empty.cmd`, then press SPACE once in its terminal.
4. Preparation runs, then 119 recorded goals at nominal 50ms (5.95 seconds,
   including 1.5s final hold). Torque turns OFF automatically. Q/Esc/Ctrl+C stops.
5. Return the terminal output, including whether any housing hit the glass/wood.

Uses existing 450mA and sequence PID settings. Fresh full-health reads precede
each subsequent target; slow reads extend the period rather than causing a
catch-up burst. Existing voltage handling and 0.18rad tracking trip remain active.
The shared runner turns torque OFF and restores settings on completion or error.
Result: `leap_sac_path_empty_result.json` plus a timestamped archive.

Source is a refinement of one successful simulated SAC rollout: achieved joint
position plus at most 0.07rad of its requested tracking offset. This is a fixed
path diagnostic, **not a new trained SAC policy or physical flip success**.
Quantized goals passed five empty and five cube starts in native MuJoCo and MJX
separately; modeled support penetration was zero, tracking below 0.118rad. Hardware
actuator dynamics, registration and occlusion behavior are still to be validated.

`sac_forward_empty_trace.json` is an earlier rejected raw-target experiment, not
this launcher input. The launcher pins `sac_reachable_empty_path_id16capped.json` and the
independent validation files by SHA256 and defaults to check-only without
`--execute`. The same empty-path launcher uses revision 2; the live actor is unchanged.
