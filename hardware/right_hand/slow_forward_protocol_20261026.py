"""Same PLACE/REMOVE checkpoints with accurate slow-duration instructions."""
import numpy as np
import sequence_flip_checked as checked
from forward_cube_placement_20261026 import ForwardPlacementCapture,SOURCE
class SlowForwardCapture(ForwardPlacementCapture):
    def place(self,bus,command,report,guard):
        self.guard=guard;observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('empty_before_place.jpg','prepared_empty_reference',observe)
        report.update(prepared_empty_photo=photo,prepared_empty_ticks=state['position'].tolist(),
            cube_rotation_requested=True,physical_flip_verified=False,
            expected_reference_seconds=57.,expected_new_finish_seconds=4.8,
            placement_comparison=dict(previous_source=SOURCE.parent.name,requested_forward_shift_mm=5.,
                shift_is_operator_estimate=True,shift_measured_or_camera_registered=False,
                motor_goal_positions_changed=False,motion_duration_ratio=1.5,
                undervoltage_threshold_changed=False,voltage_improvement_verified=False))
        report['camera_instruction']='PLACE: about5mm toward fingertips. GREEN UP, ORANGE toward WRIST/ID41. Hands clear, SPACE.'
        print('PLACE CUBE about5mm farther toward FINGERTIPS than the earlier placement. GREEN UP, ORANGE toward WRIST/ID41.',flush=True)
        print('Rest it naturally on the flat INDEX/MIDDLE motor-link tops. Q if it cannot rest by itself.',flush=True)
        print('Hands clear, SECOND SPACE starts the slower turn after3s. About62s of turn, then observation.',flush=True)
        self._space('place_cube',observe)
        state,photo=self._photo('cube_before_turn.jpg','place_cube',observe)
        report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),
            placement_goal_ticks=np.asarray(command).tolist(),cube_placement_confirmed=True,cube_present=True,
            registration_verified=False,placement_current_ma=checked.signed_current(state['current']).tolist())
        report['camera_instruction']='LOADED SLOW: hands clear. Existing voltage/current/tracking stops remain. Q stops.'
        for n in range(3,0,-1):
            print(f'Slow turn in {n}s; hands clear. Q stops.',flush=True);began=self.now()
            while self.now()-began<1:observe('loaded_countdown');self.sleep(.02)
