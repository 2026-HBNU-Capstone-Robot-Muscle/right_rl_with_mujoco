"""PLACE, loaded reorientation observation, and REMOVE before torque OFF."""
import numpy as np
from grasp_place_capture import GraspCapture

class SupportedCubeRotationCapture(GraspCapture):
    def place(self,bus,command,report,guard):
        report.update(rotation_command_path_planned=True,rotation_command_path_completed=False,
            physical_rotation_verified=False,policy_used=False,rotation_candidate='supported_loaded_observation_v1')
        print('AFTER GRASP: the revised 12.8s SMALL reorientation path follows automatically. This is not a 90-degree flip.',flush=True)
        print('Watch the FRONT face/edge; the TOP color may remain unchanged. Q stops.',flush=True)
        return super().place(bus,command,report,guard)

    def finish(self,bus,command,report):
        if not report.get('cube_placement_confirmed'):raise RuntimeError('Placement not confirmed; OFF')
        observe=self._observer(bus,command,report,self.guard)
        state,photo=self._photo('cube_after_supported_rotation.jpg','supported_rotation_observation',observe)
        report.update(rotation_observation_photo=photo,rotation_position_ticks=state['position'].tolist(),
            rotation_goal_ticks=np.asarray(command).tolist(),rotation_command_path_completed=True,
            physical_rotation_verified=False,physical_flip_verified=False,physical_grasp_verified=False)
        report['camera_instruction']='REMOVE CUBE; SPACE in TERMINAL = OFF'
        print('Reorientation commands complete. Observe the cube itself, then REMOVE CUBE gently.',flush=True)
        print('SPACE in this TERMINAL confirms removal and turns torque OFF. Q stops.',flush=True)
        state=self._space('remove_cube',observe)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},
            cube_removal_confirmed=True,cube_present=False)
