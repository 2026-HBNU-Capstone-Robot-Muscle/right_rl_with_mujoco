"""Explicit EMPTY/PLACE/REMOVE steps for the newly completed full hand path."""
import numpy as np
import sequence_flip_checked as checked
from slow_forward_protocol_20261026 import SlowForwardCapture
class LoadedThumbCapture(SlowForwardCapture):
    def place(self,bus,command,report,guard):
        self.guard=guard;observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('empty_before_place.jpg','prepared_empty_reference',observe)
        report.update(prepared_empty_photo=photo,prepared_empty_ticks=state['position'].tolist(),
            cube_rotation_requested=True,physical_flip_verified=False,expected_loaded_motion_seconds=96.,
            loaded_goal_path_physically_completed_empty=True,physical_loaded_motor_response_verified=False)
        report['cube_placement_window_open']=True
        report['camera_instruction']='PLACE: GREEN UP, ORANGE toward WRIST/ID41. On flat index/middle link tops. Hands clear, SPACE.'
        print('PLACE CUBE NOW on the flat INDEX/MIDDLE motor-link tops, about5mm toward FINGERTIPS from the earlier placement.',flush=True)
        print('GREEN UP, ORANGE toward WRIST/ID41. Let it rest by itself; Q if it cannot. No forcing or hand support.',flush=True)
        print('SECOND SPACE in TERMINAL starts the known turn plus new thumb retreat/root lift after3s. About96s, then REMOVE.',flush=True)
        self._space('place_cube',observe)
        state,photo=self._photo('cube_before_turn.jpg','place_cube',observe)
        report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),
            placement_goal_ticks=np.asarray(command).tolist(),cube_placement_confirmed=True,cube_present=True,
            registration_verified=False,placement_current_ma=checked.signed_current(state['current']).tolist())
        report['camera_instruction']='LOADED turn: hands clear. Q stops. Wait for REMOVE CUBE.'
        for n in range(3,0,-1):
            print(f'Turn in {n}s; hands clear. Q stops.',flush=True);began=self.now()
            while self.now()-began<1:observe('loaded_countdown');self.sleep(.02)
