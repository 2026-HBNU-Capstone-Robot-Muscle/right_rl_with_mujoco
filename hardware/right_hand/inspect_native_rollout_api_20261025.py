import inspect
import mujoco.rollout
print('ROLL_OUT',inspect.signature(mujoco.rollout.Rollout))
print('METHOD',inspect.signature(mujoco.rollout.Rollout.rollout))
print(mujoco.rollout.Rollout.rollout.__doc__[:8000])
