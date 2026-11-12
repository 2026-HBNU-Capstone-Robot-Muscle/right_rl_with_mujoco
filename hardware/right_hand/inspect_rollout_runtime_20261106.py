"""Read installed MuJoCo rollout API and source frame, without devices."""
import inspect,json
import numpy as np,mujoco
from mujoco import rollout
from dynamic_common_grip_search_20261105 import ROOT
print('version',mujoco.__version__)
print('rollout',inspect.signature(rollout.rollout))
print('Rollout',inspect.signature(rollout.Rollout))
print((rollout.rollout.__doc__ or '')[:4500])
z=np.load(ROOT/'countergrip_level/seed0_source1_lift0_normal2.5_n900_resume_lower_staged_freebase/end_state.npz')
print('up',z['up'].tolist())
