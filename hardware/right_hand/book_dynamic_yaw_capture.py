"""Separate operator acknowledgement from physically verified cube removal."""
from grasp_place_capture import GraspCapture
from book_small_yaw_capture import BookSmallYawCapture


class BookDynamicYawCapture(BookSmallYawCapture):
    def place(self,bus,command,report,guard):
        report.update(rotation_command_path_planned=True,rotation_command_path_completed=False,
            physical_rotation_verified=False,policy_used=False,live_sac=False,robust_policy=False,
            rotation_candidate='book_measured_dynamic_clockwise_candidate_v2',
            contact_refinement_samples=26,small_turn_samples=100,
            desired_physical_yaw_degrees=-3.,motion_direction='clockwise_viewed_from_above',
            yaw_baseline_phase='stage_4_book_grasp_hold',removal_motion_is_success_evidence=False,
            marker_to_model_registration_verified=False,full_three_degree_amplitude_verified=False,
            cube_removal_physically_verified=False,
            cube_removal_confirmation_method='operator_terminal_SPACE_only_not_vision')
        print('AFTER GRASP: fixed 3s -> revised contact/turn 6.3s -> fixed 2s -> REMOVE.',flush=True)
        print('A bounded index-root movement is included. Actual cube angle is NOT verified. No live SAC.',flush=True)
        print('Keep hands clear until REMOVE. REMOVE the cube BEFORE the final SPACE; it switches torque OFF.',flush=True)
        return GraspCapture.place(self,bus,command,report,guard)

    def finish(self,bus,command,report):
        super().finish(bus,command,report)
        # The shared completion flag is retained as an operator acknowledgement,
        # never re-labelled as camera-verified absence or a measured empty hand.
        report.update(cube_removal_operator_confirmed=True,cube_removal_physically_verified=False,
            cube_present=None,cube_presence_after_final_SPACE='unverified_operator_acknowledgement',
            cube_removal_confirmation_method='operator_terminal_SPACE_only_not_vision')
