"""Explicit PLACE and REMOVE checkpoints around the EMPTY-verified finish."""
import numpy as np
import sequence_flip_checked as checked
from loaded_quarter_trial_protocol_20261025 import LoadedQuarterCapture
class LoadedQuarterFinishCapture(LoadedQuarterCapture):
    def place(self,bus,command,report,guard):
        self.guard=guard;observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('empty_before_place.jpg','prepared_empty_reference',observe)
        report.update(prepared_empty_photo=photo,prepared_empty_ticks=state['position'].tolist(),cube_rotation_requested=True,
            expected_reference_seconds=38.,expected_new_finish_seconds=3.2,physical_flip_verified=False)
        report['camera_instruction']='PLACE: GREEN up, ORANGE toward WRIST/ID41. Flat INDEX/MIDDLE support. Hands clear, SPACE.'
        print('PLACE CUBE on the flat INDEX/MIDDLE motor-link tops. GREEN UP, ORANGE toward WRIST/ID41.',flush=True)
        print('Let it rest naturally; do not push it into the palm groove or force a fit. Remove your hand.',flush=True)
        print('SECOND SPACE in TERMINAL starts the 38s reference PLUS the new3.2s finish, after a3s countdown.',flush=True)
        print('If the Cube will not rest without your hand, Q stops. Keep hands clear during motion.',flush=True)
        self._space('place_cube',observe)
        state,photo=self._photo('cube_before_turn.jpg','place_cube',observe)
        report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),placement_goal_ticks=np.asarray(command).tolist(),
            cube_placement_confirmed=True,cube_present=True,registration_verified=False,placement_current_ma=checked.signed_current(state['current']).tolist())
        report['camera_instruction']='LOADED: reference + EMPTY-verified root finish once. Hands clear. Q stops.'
        for n in range(3,0,-1):
            print(f'Full reference and new finish in {n}s; hands clear. Q stops.',flush=True);began=self.now()
            while self.now()-began<1:observe('loaded_countdown');self.sleep(.02)
    def finish(self,bus,command,report):
        super().finish(bus,command,report)
        report['reference_and_finish_commanded']=True
