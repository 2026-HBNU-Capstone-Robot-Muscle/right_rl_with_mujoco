"""Repair the early divergence before attempting recovery from a bad late grip."""
import numpy as np
import mujoco
from ccd_recovery_env import CCDRecovery
from native_cycle import OUT


class CCDEarlyRecovery(CCDRecovery):
    starts=np.array([600,900,1200,1600,2000,2600,3000,3400])
    probabilities=np.array([.18,.18,.18,.14,.10,.10,.06,.06])

    def __init__(self, seed=5, training=True):
        super().__init__(seed=seed,training=training)
        self.pool=dict(np.load(OUT/'next_cycle/handoff_refinement_20261110/early_CCD50_states/state_pool.npz',allow_pickle=False))
        self.pool_indices=self.pool['indices'].astype(int)

    def _reset_one(self,rng):
        random=np.random.default_rng(int(np.asarray(rng,dtype=np.uint32).astype(np.uint64).sum())+self.seed)
        index=int(random.choice(self.starts,p=self.probabilities))
        pos=int(np.flatnonzero(self.pool_indices==index)[0]);d=mujoco.MjData(self.model)
        for key in self.slices:getattr(d,key)[:]=self.pool[key][pos]
        d.time=float(self.pool['time'][pos]);mujoco.mj_forward(self.model,d)
        return self._pack(d,index,index+self.horizon,0,first=index),self.observe(self.model,d,index)
