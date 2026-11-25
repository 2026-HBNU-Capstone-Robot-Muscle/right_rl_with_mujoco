"""PLACE/REMOVE observations for a first small cube yaw attempt."""
import numpy as np
from grasp_place_capture import GraspCapture


class SmallRotationCapture(GraspCapture):
    def place(self,bus,command,report,guard):
        report.update(rotation_command_path_planned=True,rotation_command_path_completed=False,
                      physical_rotation_verified=False,policy_used=False,
                      rotation_candidate='bounded_small_yaw_commissioning_v1')
        print('AFTER GRASP: a 4-second SMALL horizontal rotation path will follow automatically. This is not a 90-degree flip.',flush=True)
        print('No live SAC. Observe the cube itself; Q/Esc/Ctrl+C stops.',flush=True)
        return super().place(bus,command,report,guard)

    def finish(self,bus,command,report):
        if not report.get('cube_placement_confirmed'):raise RuntimeError('Placement was not confirmed; OFF')
        observe=self._observer(bus,command,report,self.guard)
        state,photo=self._photo('cube_after_small_rotation.jpg','small_rotation_observation',observe)
        report.update(rotation_observation_photo=photo,rotation_position_ticks=state['position'].tolist(),
            rotation_goal_ticks=np.asarray(command).tolist(),rotation_command_path_completed=True,
            physical_rotation_verified=False,physical_flip_verified=False,physical_grasp_verified=False)
        print('Small yaw commands ended. Compare the FRONT face/edge with the start; the TOP color may stay the same.',flush=True)
        print('REMOVE CUBE gently while goals hold, then SPACE in this TERMINAL turns torque OFF. Q stops.',flush=True)
        report['camera_instruction']='REMOVE CUBE; SPACE in TERMINAL = OFF'
        state=self._space('remove_cube',observe)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},
                      cube_removal_confirmed=True,cube_present=False)
