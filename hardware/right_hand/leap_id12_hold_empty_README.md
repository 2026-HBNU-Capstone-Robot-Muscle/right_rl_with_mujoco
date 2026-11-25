# ID12 fixed-goal diagnostic v1 — EMPTY only

The user reports no visible cable snag or plastic/linkage contact. This is recorded
as an observation; internal resistance and transient contact have not been ruled
out. Full- and half-speed runs both fail at goal3350/actual3227 or3232. At the
half-speed fault PWM24%, current210mA, voltage4.4V; no sampled output saturation.

This diagnostic distinguishes transient lag from residual error at a fixed goal.
`leap_id12_hold_empty.cmd` uses the same checked preparation, followed by ONLY the
first40 half-speed goals (2s), ending at ID12 goal3290. That target was previously
sent with about0.123rad lag, before the3350 failure. All16 goals then stay fixed
for1s while fresh full health/PWM/velocity samples are read. No goal writes,
gain/current change, extended recovery wait, or subsequent path during/after hold.
The shared finally block turns torque OFF and restores settings. Any fault exits
immediately through that same cleanup. Other fingers move during preparation and
the prefix: this is not a single-motor exercise.

User: remove cube, secure base, close other motor programs, connect USB and motor
power, launch once, terminal SPACE once. No camera or placement step. Return the
console log. Latest result is `leap_id12_hold_empty_result.json`, timestamped copy
also saved; previous full-path result is preserved.

Recorded diagnostic includes first and final median joint position, error,
current, signed PWM and signed velocity. It does not infer a hard mechanical limit
or claim SAC/flip success. A persisting fixed-goal offset can come from load,
friction, linkage constraints or insufficient controller stiffness; convergence
instead supports a timing/response issue. Neither alone proves the cause.

Native offline audit: measured readonly start through preparation/prefix/1s hold,
plus5 empty reset cases; all support/tracking criteria pass (overall0.1372rad,
prefix/hold~0.1086rad; support penetration0). Model motor gain is3Nm/rad for all
joints, not a calibrated mapping of real P450/P600; self-contact geometry also has
unresolved overlaps in reconstructed measured poses. No physical-clearance or
loaded validation claim. Audit hash is pinned; all source transforms checked.
34 device-free tests passed: endpoint bounds, no hold writes, fixed duration,
converging and stationary observations, cancellation/fault OFF/restoration,
telemetry decoding, mapping and earlier progress-guard regressions.
