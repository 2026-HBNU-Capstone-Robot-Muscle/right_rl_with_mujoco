# New SAC observation adapter — offline integration

`sac_reference_observation.py` builds the new actor's 110 inputs using NumPy.
It opens no camera or serial port and is not connected to a motor launcher.

- Motor ID 1..16 encoder ticks convert to simulation joint order 5..16,1..4,
  radians minus pi. Joint commands must use that same order and units.
- Pose inputs must already use the same validated world coordinate frame;
  quaternions are wxyz. Do not feed the provisional camera registration directly.
- The history is three samples, newest first: measured-minus-commanded joints,
  palm-minus-cube position, then the last two rows of the rotation matrix of
  `cube_quaternion * inverse(goal_quaternion)`.
- Previous action means the **executed base action** returned by
  `ReferencePolicy.command`, not the learned residual output.
- Phase uses completed reference steps at 50 ms per step, reaching the final
  hold at step 72. Camera frames and wall-clock delays do not advance phase.
- Warm with three fresh observations at step zero. Invalid input leaves all
  history untouched. Reset and warm again when the goal/reference frame changes.
- Occlusion, timing, hardware goal acknowledgement and dropout recovery remain
  controller integration work; this adapter does not make stale poses valid.

Validation: six adapter tests and five actor tests pass on Windows. The adapter
round-trips 90 recorded JAX observations (compare after two history-fill samples),
plus explicit motor mapping, history sign/order, noncommuting quaternion order,
quaternion sign equivalence, invalid input and phase-boundary checks. This proves
numeric assembly, not camera accuracy or physical cube rotation.

Run without devices:

```powershell
& ".\.runtime\python\python.exe" -B -m unittest test_sac_reference_observation test_sac_reference_numpy
```
