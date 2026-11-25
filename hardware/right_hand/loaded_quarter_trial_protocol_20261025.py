"""Three terminal checkpoints for one full experimental loaded reference."""
import numpy as np
import sequence_flip_checked as checked
from grasp_place_capture import GraspCapture

class LoadedQuarterCapture(GraspCapture):
    def place(self,bus,command,report,guard):
        self.guard=guard;observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('empty_before_place.jpg','prepared_empty_reference',observe)
        report.update(prepared_empty_photo=photo,prepared_empty_ticks=state['position'].tolist(),cube_rotation_requested=True)
        report['camera_instruction']='PLACE: green UP, orange toward WRIST/ID41, on flat INDEX/MIDDLE links. SPACE starts full turn.'
        print('PLACE CUBE on the flat INDEX/MIDDLE link tops. GREEN up; ORANGE toward WRIST/ID41, away from fingertips.',flush=True)
        print('Cube must rest without your hand. Hands clear, SECOND SPACE in TERMINAL starts the whole 38s turn after 3s.',flush=True)
        print('This is the FIRST loaded quarter-turn attempt, not a 3deg probe. Q stops. Do not touch Cube/fingers during motion.',flush=True)
        self._space('place_cube',observe)
        state,photo=self._photo('cube_before_turn.jpg','place_cube',observe)
        report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),
            placement_goal_ticks=np.asarray(command).tolist(),cube_placement_confirmed=True,cube_present=True,
            registration_verified=False,placement_current_ma=checked.signed_current(state['current']).tolist())
        report['camera_instruction']='LOADED: whole reference once, hands clear, Q stops'
        for n in range(3,0,-1):
            print(f'Full loaded reference in {n}s; hands clear. Q stops.',flush=True)
            began=self.now()
            while self.now()-began<1:observe('loaded_countdown');self.sleep(.02)

    def finish(self,bus,command,report):
        if not report.get('cube_placement_confirmed'):raise RuntimeError('Placement not confirmed; OFF')
        observe=self._observer(bus,command,report,self.guard)
        state,photo=self._photo('cube_after_turn.jpg','loaded_turn_observation',observe)
        report.update(rotation_observation_photo=photo,rotation_position_ticks=state['position'].tolist(),
            rotation_goal_ticks=np.asarray(command).tolist(),physical_flip_verified=False)
        report['camera_instruction']='REMOVE CUBE FIRST; THIRD SPACE in TERMINAL turns torque OFF'
        print('Full reference ended. Actual cube orientation will be reviewed from saved video.',flush=True)
        print('REMOVE CUBE first. THIRD SPACE in TERMINAL confirms removal and turns torque OFF. Q stops.',flush=True)
        state=self._space('remove_cube',observe)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_removal_confirmed=True,cube_present=False)
