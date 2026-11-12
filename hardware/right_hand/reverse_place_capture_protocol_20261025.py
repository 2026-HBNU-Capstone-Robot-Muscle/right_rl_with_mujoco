"""Fresh fixed-goal photos with existing health, drift and terminal-key checks."""
import numpy as np
import sequence_flip_checked as checked
from grasp_place_capture import GraspCapture

class ReversePlacementCapture(GraspCapture):
    def place(self,bus,command,report,guard):
        self.guard=guard;observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('reverse_start_empty.jpg','prepared_empty_reference',observe)
        report.update(prepared_empty_photo=photo,prepared_empty_on_state={k:v.tolist() for k,v in state.items()},
            prepared_empty_ticks=state['position'].tolist(),cube_rotation_requested=False)
        report['camera_instruction']='PLACE on INDEX/MIDDLE flat link tops; GREEN up; ORANGE toward WRIST. SPACE = photo only'
        print('PLACE CUBE on the flat index/middle link tops shown in the guide. GREEN UP, ORANGE toward WRIST (away from fingertips).',flush=True)
        print('Let the cube rest without your hand. Do not force it. If it cannot rest, Q stops.',flush=True)
        print('Hands clear, SECOND SPACE in this TERMINAL saves the photo. Goals stay FIXED; no rotation.',flush=True)
        print('Keep the whole cube visible. ID40 may be hidden; no marker visibility gate after preparation.',flush=True)
        self._space('place_cube',observe)
        state,photo=self._photo('cube_placed.jpg','place_cube',observe)
        report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),
            placement_goal_ticks=np.asarray(command).tolist(),cube_placement_confirmed=True,cube_present=True,
            registration_verified=False,placement_current_ma=checked.signed_current(state['current']).tolist())
        report['camera_instruction']='PHOTO ONLY; goals fixed. Wait for REMOVE CUBE'

    def finish(self,bus,command,report):
        if not report.get('cube_placement_confirmed'):raise RuntimeError('Placement not confirmed; OFF')
        observe=self._observer(bus,command,report,self.guard)
        state,photo=self._photo('cube_fixed_hold.jpg','placed_fixed_observation',observe)
        report.update(placed_hold_photo=photo,placed_hold_state={k:v.tolist() for k,v in state.items()},
            physical_flip_verified=False,physical_grasp_verified=False)
        report['camera_instruction']='REMOVE CUBE FIRST; THIRD SPACE in TERMINAL = OFF'
        print('Photo saved. REMOVE CUBE first, then THIRD SPACE in this TERMINAL confirms removal and turns torque OFF. Q stops.',flush=True)
        state=self._space('remove_cube',observe)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_removal_confirmed=True,cube_present=False)
