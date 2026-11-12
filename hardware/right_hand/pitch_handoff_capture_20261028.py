"""Explicit PLACE and REMOVE checkpoints for pitch then transfer observation."""
import numpy as np
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from pitch_handoff_plan_20261028 import pitch_start,OPEN_PATH

class PitchHandoffCapture(NewPairGraspCapture):
    def place(self,bus,command,report,guard):
        if not np.array_equal(command,pitch_start()):raise RuntimeError('Pitch setup endpoint not reached; no PLACE prompt')
        self.guard=guard;bus.grasp_protocol=self
        observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('pitch_setup_empty.jpg','pitch_setup_empty',observe)
        report.update(prepared_empty_photo=photo,prepared_empty_ticks=state['position'].tolist(),
                      camera_instruction='PLACE at the known pitch setup; GREEN up / ORANGE toward WRIST ID41')
        print('PLACE CUBE NOW at the known pitch setup. GREEN up; ORANGE toward WRIST/ID41.',flush=True)
        print('Use the saved successful placement photo. Cube must rest without your hand; do not force it.',flush=True)
        print('Hands clear, SECOND SPACE in TERMINAL saves placement and starts pitch, then slow pose transfer. No horizontal turn.',flush=True)
        self._space('place_cube',observe)
        state,photo=self._photo('cube_before_pitch.jpg','place_cube',observe)
        report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),
                      placement_goal_ticks=np.asarray(command).tolist(),cube_placement_confirmed=True,
                      cube_present=True,registration_verified=False)
        for n in (3,2,1):
            print(f'Known pitch in {n}s; hands clear. Q stops.',flush=True);began=self.now()
            while self.now()-began<1:observe('loaded_countdown');self.sleep(.02)
    def hold_frame(self,bus,command,report,state):
        if not getattr(self,'hold_started',False):
            self.hold_started=True;self.hold_began=self.now()
            report.update(camera_instruction='Pitch hold; slow pose transfer follows; hands clear',
                          pitch_goal_ticks=np.asarray(command).tolist(),pitch_position_ticks=state['position'].tolist())
            print('Pitch targets complete;4.5s hold, then slow transfer to the next grasp setup. Keep hands clear.',flush=True)
        if not report.get('pitch_observation_photo'):
            photo=self.camera.capture_named('cube_pitch_hold.jpg',after=self.hold_began+.2)
            if photo is not None:report['pitch_observation_photo']=photo
    def rotation_frame(self,bus,command,report,state):
        if not getattr(self,'transfer_hold_started',False):
            self.transfer_hold_started=True;self.transfer_hold_began=self.now()
            report.update(camera_instruction='Transfer ended; no yaw follows; removal prompt soon',
                          transfer_goal_ticks=np.asarray(command).tolist(),transfer_position_ticks=state['position'].tolist())
            print('Pose transfer ended. No horizontal turn will run; observe where the cube rests.',flush=True)
        if not report.get('transfer_observation_photo'):
            photo=self.camera.capture_named('cube_transfer_hold.jpg',after=self.transfer_hold_began+.2)
            if photo is not None:report['transfer_observation_photo']=photo
    def finish(self,bus,command,report):
        if not report.get('cube_placement_confirmed'):raise RuntimeError('No explicit cube placement')
        if not np.array_equal(command,np.load(OPEN_PATH)['open_goal']):raise RuntimeError('Transfer endpoint not reached')
        if not hasattr(bus,'pending') or bus.pending:raise RuntimeError('Transfer queue incomplete')
        observe=self._observer(bus,command,report,self.guard)
        state,photo=self._photo('cube_after_transfer.jpg','transfer_before_remove',observe)
        report.update(transfer_final_photo=photo,physical_handoff_verified=False,
                      camera_instruction='REMOVE CUBE; THIRD SPACE = OFF. No next turn.')
        print('REMOVE CUBE gently, then THIRD SPACE in this TERMINAL turns torque OFF. Q also stops.',flush=True)
        state=self._space('remove_cube',observe)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_removal_confirmed=True,cube_present=False)
