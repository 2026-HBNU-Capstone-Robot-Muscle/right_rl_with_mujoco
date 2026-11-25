# Revision 4 — empty half-speed diagnostic

Latest read-only result (2026-10-15 00:49) found all 16 PWM Limit and Goal PWM
registers at 885. Current Limit and restored Goal Current are 2352mA; these are
OFF-state settings, not the trial's commanded current. Trial configuration remains
450mA, P600/I0/D200 (IDs8/12/16 P450/I0/D150), with restoration after verified OFF.
No low configured PWM cap was found. Active PWM saturation/contact is unknown.

Last revision-3 stop was moving ID12, goal3350 actual3227 current221mA. Its last
0.25s actual advanced20 ticks while targets advanced83 ticks. This is a tracking
fault, not the earlier direction-guard bug. No new permanent ID12 angle cap added.

The same `leap_sac_path_empty.cmd` now runs revision4. Every original path segment
gets its rounded integer midpoint followed by its original endpoint. 238 targets
at 50ms = 11.9s nominal, max step32 ticks (runtime ceiling35), same geometric path,
same ID16 cap3238. Root relief/thumb-first preparation is unchanged. Path deadline
30s accommodates normal read jitter; no catch-up bursts/retries. Existing hard
tracking0.18rad, voltage, current, temperature, cancellation and progress guards
remain. No automatic current increase, torque re-enable or retry.

Present PWM124, Present Velocity128 and Goal PWM100 now decode from the SAME
64..146 packet as health reads; no additional serial transaction. Signed raw values
are saved even on a fault. Manufacturer units: PWM0.113%, velocity0.229RPM.
https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/

Native MuJoCo audit: latest measured OFF posture -> all four preparation phases
-> slowed path without reset, plus five empty reset cases, all pass. Maximum
tracking across preparation/path0.1372rad; path-only0.1078rad; support penetration0.
Audit is hash-pinned. This remains a model check with uncalibrated actuator behavior;
ID16 upper3254 is hypothetical. The retimed path has NOT been validated with a
cube, and is NOT live SAC. Do not copy this timing into loaded motion.

User action: cube removed, base secured, other motor programs closed, USB and motor
power connected. Run SAME launcher; verify Revision4; terminal SPACE once. It
prepares, runs once empty, turns OFF/restores settings. Return console output.
If lag persists despite halved target rate, use synchronized PWM/velocity/current
to select mechanical/actuator investigation, not another arbitrary limit increase.
