# ID12 bounded stiffness comparison — EMPTY only

Baseline completed 2026-10-15 01:34:41: all torque OFF/restored. Fixed target3290,
actual3210 at all13 samples spanning0.922s. Median140mA (max141), PWM17.515%,
minimum ID12 voltage4.6V. Final lag80ticks=0.122718rad. No external contact/snags
reported by user. This distinguishes sustained fixed-goal error from simple
target-rate lag; it does not uniquely identify friction/load/internal contact.

New `leap_id12_stiffness_empty.cmd`: same checked empty preparation and same40
half-speed goals2s, hold1s, OFF. ID12 only: P450->600, I0/D150 unchanged, Goal
Current450->225mA, measured trip550->250mA. Others unchanged. This compares stiffness
without increasing target angles or the current ceiling. Do not infer the
previous140mA was a current-limit saturation. P600 may still yield a residual
offset or meet the225mA limit; inspect telemetry before deciding any next change.

No setting changes while enabled: gains/current configured during OFF before
enable, restored to original saved settings after OFF. During hold only fresh
health reads. Existing voltage/tracking/watchdog/temp/error/cancellation checks
remain. Fault means OFF, not automatic retry/continuation. All fingers move in
preparation/prefix. No cube or live SAC.

Run once with cube removed, base secured, other motor programs closed, USB and
motor power connected. SPACE in terminal once. Result:
`leap_id12_stiffness_empty_result.json` plus timestamped archive. The prior baseline
`leap_id12_hold_empty_result.json` is preserved. Return console log.

Offline native gain sensitivity (model ID12 stiffness scaled600/450, same audited
path) passed support/tracking criteria from measured start and5 resets. It is NOT
a calibrated real motor/current-limit model; model contact simplifications remain.
No claim of physical clearance or validated loaded flipping.53 device-free tests
pass including all tuning writes while OFF, per-ID250mA fault, exact settings
restoration on fault/success, no global PID mutation, invalid profile refusal,
fixed hold timing/no writes, shared voltage handling and path regressions.
