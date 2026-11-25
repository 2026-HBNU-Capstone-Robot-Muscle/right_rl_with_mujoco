"""Long repair windows crossing the observed failure, with positive survival reward."""
import numpy as np
from ccd_repair_env import CCDRepair
from learning_env import decode
from cube_task_frame_20261105 import face_up_error_deg


class CCDRecovery(CCDRepair):
    def __init__(self, seed=4, training=True):
        super().__init__(seed=seed, training=training)
        self.horizon=1200

    def _reset_one(self, rng):
        vector, obs=super()._reset_one(rng)
        values=decode(vector).copy()
        # Every start3000/3400/3600 crosses parent failure4118. No successful
        # endpoint is injected, and the final assessment still starts at placement.
        values[85]=min(int(values[84])+self.horizon,len(self.reference))
        return values.view(np.uint32),obs

    def _step_one(self, vector, obs, action):
        state, new_obs, old_reward, done, metrics=super()._step_one(vector,obs,action)
        model,data=self._restore(state)
        B=data.xmat[model.body('cube').id].reshape(3,3)
        tilt=face_up_error_deg(B,5,self.initial['up'])
        # Valid steps have positive reward. A drop must not become attractive
        # merely because it terminates a long sequence of negative tracking costs.
        reward=(.1+np.exp(-float(metrics[1])/20.)+.5*np.exp(-float(metrics[2])/10.)
                +.25*np.exp(-tilt/10.)-.02*float(metrics[3]))
        if metrics[0]:reward=-20.
        return state,new_obs,np.float32(reward),done,metrics
