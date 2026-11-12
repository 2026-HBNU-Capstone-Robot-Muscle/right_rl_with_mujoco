# First-flip SAC export — offline only

This package contains the newly trained **reference trajectory plus SAC position correction**. It does not open a motor port or camera. The existing `leap_sac.cmd` still uses its previous policy; it has not been switched to this export.

Run the offline Windows parity check:

```powershell
& ".\.runtime\python\python.exe" -B .\sac_reference_numpy.py
```

The exporter `sac_reference_export.py` uses installed JAX/Brax in WSL. It pins checkpoint files, reset, reference and collision-model version. Its fixture consists of 90 consecutive MJX steps at offset index 1 / seed 0, plus 512 random normalized observation inputs. NumPy outputs are compared against `checkpoint.load_policy(..., deterministic=True)` and actual environment command outputs. This is numerical parity, not another hardware or robustness evaluation.

The real-rollout comparison uses original GPU inference (action error ~2.68e-7, goal error ~2.38e-7 rad). Batched random inputs initially differed by ~6.10e-4 with default GPU matmul precision. At JAX `highest` matmul precision the maximum actor difference is ~3.05e-7. The fixture retains default GPU random outputs as a separate diagnostic; they are not silently discarded. No training weights or live settings were changed to obtain this comparison.

## Observation contract

The actor accepts **110 float32 values**. All spatial quantities below must share the same simulation world frame; a camera-frame pose cannot be supplied directly. Quaternions use **w,x,y,z**. Joint order is index, middle, ring, thumb, corresponding to hardware IDs `[5..16,1..4]`; simulation angles are motor radians minus pi.

| Indices | Quantity |
|---|---|
| 0:16 | Current joint angles, rad |
| 16:64 | Three newest-first joint-angle-minus-commanded-goal histories |
| 64:73 | Three newest-first palm-position-minus-cube-position histories, m |
| 73:91 | Three newest-first orientation error histories: row-major rotation matrix of `cube_quat * inverse(goal_quat)`, flattened entries `[3:]` |
| 91:107 | Previous **executed base action**, not the raw SAC residual; initially zero |
| 107 | Full orientation error `2*asin(min(norm(normalize(quat_diff).xyz),1))`, rad |
| 108:110 | `[min(step*0.05/3.6,1), step*0.05 >= 3.6]` |

The trained reset warms all three histories with the starting state. The simulator adds observation noise. Real deployment must use measured inputs with a separately validated camera-to-hand-to-world transform and freshness handling; this module does not invent poses during occlusion or assemble live observations.

## Action contract

Normalize using exported mean/std (no extra clipping), two 256-unit ReLU hidden layers, then tanh of the first 16 logits. At step `k`, reference targets correspond to `(k+1)*0.05` seconds along three 1.2-second linear waypoint segments. Add an absolute `0.05 * actor_action` rad correction; clip to trained joint limits. Convert to base action by clipping `(desired - previous_commanded_goal)/0.2` to [-1,1], then construct the next goal and clip again. The *commanded* goal, not encoder position, is the reference for that delta. `ReferencePolicy.command` returns the goal and executed base action without sending either.

Do not advance phase on a faster host loop, feed raw residuals back as previous base actions, or replay this open loop and label it closed-loop SAC.

## Remaining physical integration

- Empty revised-start movement has already completed in the user's trials; do not repeat it without cause.
- A failed trial recorded 3.6 V, with a large voltage difference across IDs. Adapter output rating and power-entry wiring remain unknown. A later completion does not establish that voltage sag is resolved.
- Live observation construction, marker occlusion handling and sustained 20 Hz sensor/control timing have not been validated for this policy.
- The modeled actuator response is not calibrated to the physical 450 mA/PID configuration.
- First-flip nominal simulation result is 10/10; horizontal and following vertical rotations are not completed. No physical success is claimed by this export.
