"""Health-checked PLACE and REMOVE checkpoints for one slow loaded approach."""
import time
import numpy as np
import sequence_flip_checked as checked
from held_joint_guard import HeldJointGuard


class GraspCapture:
    def __init__(self,camera,keys,*,cancel,sleep=time.sleep,now=time.monotonic):
        self.camera=camera;self.keys=keys;self.cancel=cancel;self.sleep=sleep;self.now=now

    def _observer(self,bus,command,report,guard):
        def observe(phase):
            report['phase']=phase;self.cancel()
            def check_snapshot(state):
                guard.check(state['position'],command,np.arange(16),phase)
                threshold=report.get('contact_release_threshold_ticks')
                if threshold is not None and state['position'][14]>threshold:
                    raise RuntimeError('ID15 moved back outside its certified release; OFF')
            state,_=checked.sample(bus,command,report,phase,snapshot_check=check_snapshot)
            return state
        return observe

    def release(self,bus,command,report,guard):
        observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('ring_released.jpg','release_contact_confirmation',observe)
        report['contact_release_photo']=photo
        report['camera_instruction']='CUBE OUT; check RING TIP clear; SPACE continues opening'
        print('RING RELEASE CHECK: keep cube OUT. Check the ring fingertip is separated from the palm.',flush=True)
        print('Hands clear, SPACE in this TERMINAL confirms what you see and continues EMPTY opening. Q stops.',flush=True)
        self._space('release_contact_confirmation',observe)
        report['contact_release_operator_confirmed']=True
        report['camera_instruction']='EMPTY HAND; opening preparation; Q stops'

    def _photo(self,name,phase,observe):
        began=self.now()
        while True:
            state=observe(phase);photo=self.camera.capture_named(name,after=began+.2)
            if photo is not None:return state,photo
            if self.now()-began>2:raise RuntimeError('No fresh grasp camera frame; OFF')
            self.sleep(.02)

    def _space(self,phase,observe):
        self.keys.arm();began=self.now()
        while True:
            state=observe(phase)
            if self.keys.consume_space():return state
            if self.now()-began>90:raise RuntimeError(phase+' timed out; OFF')
            self.sleep(.02)

    def place(self,bus,command,report,guard):
        self.guard=guard
        observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('open_before_cube.jpg','open_before_cube',observe)
        report['open_before_cube_photo']=photo
        report['open_before_cube_state']={k:v.tolist() for k,v in state.items()}
        report['camera_instruction']='PLACE CUBE; then hands clear + SPACE in TERMINAL'
        print('PLACE CUBE NOW above the palm beside ID40, between the fingers. GREEN up, ORANGE toward camera.',flush=True)
        print('Let it rest naturally; slight tipping toward the finger roots is expected. Do not force it in.',flush=True)
        print('Remove your hands, then SPACE in this TERMINAL records placement and starts the slow approach after 3s.',flush=True)
        print('If it cannot rest without your hand, press Q. ID40 may be hidden; no marker gate.',flush=True)
        self._space('place_cube',observe)
        state,photo=self._photo('cube_before_approach.jpg','place_cube',observe)
        report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),
            placement_goal_ticks=np.asarray(command).tolist(),cube_placement_confirmed=True,cube_present=True,
            registration_verified=False,placement_current_ma=checked.signed_current(state['current']).tolist())
        report['camera_instruction']='CUBE LOADED; hands clear; Q stops'
        for n in range(3,0,-1):
            print(f'Loaded approach in {n}s; hands clear. Q stops.',flush=True)
            began=self.now()
            while self.now()-began<1:
                observe('loaded_countdown');self.sleep(.02)

    def finish(self,bus,command,report):
        if not report.get('cube_placement_confirmed'):raise RuntimeError('Placement was not confirmed; OFF')
        observe=self._observer(bus,command,report,self.guard)
        state,photo=self._photo('cube_after_approach.jpg','grasp_observation',observe)
        report.update(grasp_observation_photo=photo,grasp_position_ticks=state['position'].tolist(),
                      grasp_goal_ticks=np.asarray(command).tolist(),physical_grasp_verified=False,
                      physical_flip_verified=False)
        print('Approach ended. Observe whether the cube stayed between the fingers. No rotation will run.',flush=True)
        report['camera_instruction']='REMOVE CUBE; then SPACE in TERMINAL = OFF'
        print('REMOVE CUBE gently while goals hold, then SPACE in this TERMINAL turns torque OFF. Q stops.',flush=True)
        state=self._space('remove_cube',observe)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},
                      cube_removal_confirmed=True,cube_present=False)
