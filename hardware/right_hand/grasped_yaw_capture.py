"""Observe a small coordinated turn before the operator removes the cube."""
import numpy as np
from grasp_place_capture import GraspCapture


class GraspedYawCapture(GraspCapture):
    def place(self,bus,command,report,guard):
        report.update(rotation_command_path_planned=True,rotation_command_path_completed=False,
            physical_rotation_verified=False,policy_used=False,
            rotation_candidate='contact_refined_small_yaw_candidate_v1',
            contact_refinement_samples=20,small_turn_samples=70,
            desired_physical_yaw_degrees=3.,live_sac=False,
            robust_policy=False,full_held_out_angle_criterion_passed=False,
            yaw_baseline_phase='stage_4_fixed_grasp_observation',
            removal_motion_is_success_evidence=False)
        print('AFTER GRASP: fixed 3s -> bounded contact refinement 1s -> SMALL turn 3.5s -> fixed 2s -> REMOVE.',flush=True)
        print('Experimental 3deg yaw goal. Simulated range 0.93-3.24deg; physical rotation NOT verified. No live SAC.',flush=True)
        print('Watch the cube FRONT edges. Keep your hands away until REMOVE CUBE. Q stops.',flush=True)
        return super().place(bus,command,report,guard)

    def finish(self,bus,command,report):
        if not report.get('cube_placement_confirmed'):raise RuntimeError('Placement not confirmed; OFF')
        observe=self._observer(bus,command,report,self.guard)
        state,photo=self._photo('cube_after_grasped_yaw.jpg','grasped_yaw_observation',observe)
        report.update(rotation_observation_photo=photo,rotation_position_ticks=state['position'].tolist(),
            rotation_goal_ticks=np.asarray(command).tolist(),rotation_command_path_completed=True,
            physical_rotation_verified=False,physical_flip_verified=False,
            rotation_evidence_cutoff_camera_monotonic=photo['camera_read_monotonic'])
        report['camera_instruction']='REMOVE CUBE; then SPACE in TERMINAL = OFF'
        print('Before-removal observation saved. REMOVE CUBE gently; SPACE confirms removal and turns torque OFF.',flush=True)
        state=self._space('remove_cube',observe)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_removal_confirmed=True,cube_present=False)
