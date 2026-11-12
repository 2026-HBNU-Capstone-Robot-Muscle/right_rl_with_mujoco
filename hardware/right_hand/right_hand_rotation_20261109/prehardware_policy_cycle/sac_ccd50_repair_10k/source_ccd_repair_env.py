"""SAC repair windows from actual converged-CCD model states, without hardware."""
import numpy as np
import mujoco
from learning_env import ResidualCycle, decode
from native_cycle import OUT, assets, Guard, motor, ticks_to_sim, TICK
from cube_task_frame_20261105 import orientation_error_deg, face_up_error_deg


class CCDRepair(ResidualCycle):
    def __init__(self, seed=0, training=True):
        super().__init__(residual_rad=.01, horizon=512, seed=seed, training=training)
        self.model, self.initial, _, _, self.provenance = assets('active_ccd50')
        self.models = [self.model]
        self.pool = dict(np.load(OUT/'next_cycle/handoff_refinement_20261110/ccd_replay50/state_pool.npz', allow_pickle=False))
        self.pool_indices = self.pool['indices'].astype(int)
        guard = Guard(self.model)
        self.lo, self.hi, self.qlo, self.qhi = guard.lo, guard.hi, guard.qlo, guard.qhi

    def observe(self, model, data, index, builder=None):
        b = self._builder() if builder is None else builder
        measured = (np.rint(data.qpos[:16]/TICK + 2048)-2048)*TICK
        def sample():
            return b.build(measured, data.ctrl,
                data.sensordata[model.sensor_adr[model.sensor('palm_position').id]:][:3],
                data.qpos[-7:-4], data.qpos[-4:], self.goal_quat)
        if builder is None:
            sample(); sample()
        obs = sample()
        return np.r_[obs, index/max(1, len(self.reference)-1),
                     float(index >= len(self.reference)-134)].astype(np.float32)

    def _reset_one(self, rng):
        seed = int(np.asarray(rng, dtype=np.uint32).astype(np.uint64).sum()) + self.seed
        random = np.random.default_rng(seed)
        index = int(random.choice([3000, 3400, 3600], p=[.4, .4, .2]))
        pos = int(np.flatnonzero(self.pool_indices == index)[0])
        d = mujoco.MjData(self.model)
        for key in self.slices:
            getattr(d, key)[:] = self.pool[key][pos]
        d.time = float(self.pool['time'][pos]); mujoco.mj_forward(self.model, d)
        return self._pack(d, index, min(index+self.horizon, 4100), 0, first=index), self.observe(self.model, d, index)

    def _step_one(self, vector, obs, action):
        model, data = self._restore(vector); v = decode(vector)
        index, end, first = int(v[84]), int(v[85]), int(v[88])
        action = np.clip(np.asarray(action, float), -1, 1)
        desired = ticks_to_sim(self.reference[min(index, len(self.reference)-1)]) + self.residual_rad*action
        previous = motor(data.ctrl)
        goal = np.clip(previous + np.clip(motor(desired)-previous, -2, 2), self.lo, self.hi)
        if np.max(abs(goal-previous)) > 2:
            raise AssertionError('Native command projection exceeded two ticks')
        prior = data.ctrl.copy(); data.ctrl[:] = ticks_to_sim(goal)
        guard = Guard(model); guard.gap = float(v[87]); reason = []
        for _ in range(75):
            mujoco.mj_step(model, data); reason = guard.check(data)
            if np.linalg.norm(data.qpos[-7:-4]-self.initial['initial_cube_center']) > .08:
                reason = list(reason) + ['cube_translation_over80mm']
            if reason:
                break
        next_index = index+1
        builder = self._builder(obs); builder.set_last_act(np.clip((data.ctrl-prior)/.2, -1, 1))
        new_obs = self.observe(model, data, next_index, builder)
        B = data.xmat[model.body('cube').id].reshape(3, 3)
        orientation = orientation_error_deg(B, self.reference_rotations[next_index])
        center = float(np.linalg.norm(data.qpos[-7:-4]-self.reference_centers[next_index])*1000)
        goal_error = orientation_error_deg(B, self.reference_rotations[-1])
        tilt = face_up_error_deg(B, 5, self.initial['up'])
        cost = float(np.mean(action**2))
        reward = 1. - .035*orientation - .04*center - .01*tilt - .02*cost
        if reason:
            reward = -20.
        complete = next_index >= end
        metrics = np.asarray([bool(reason), orientation, center, cost, complete and not reason,
                              0., 0., goal_error], np.float32)
        return (self._pack(data, next_index, end, 0, guard.gap, first), new_obs,
                np.float32(reward), np.float32(bool(reason) or complete), metrics)
