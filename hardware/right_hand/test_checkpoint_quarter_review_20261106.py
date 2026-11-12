import copy,unittest,numpy as np
from checkpoint_quarter_review_20261106 import quarter_reasons

def evidence():
    return dict(safety_passed=True,completed_goals=400,total_goals=400,final_yaw_from_placement_deg=90.,final_up_tilt_deg=4.,final_center_world_m=[.15,-.03,0.],regrasp_yaw_change_deg=27.,whole_grip_task_passed=False,settled_tail=dict(seconds=4.05,yaw_drift_deg=.1,center_shift_mm=.2,minimum_separating_plane_gap_1ms_mm=1.,opposed_lower_body_contact_fraction=1.,separation_certificate_bad_steps_1ms=0,certificate_unqueried_steps_after_first_failure=0,certificate=dict(queries=4050)))

class QuarterEvidenceTests(unittest.TestCase):
    def test27degree_continuation_can_finish_the_whole90_target(self):self.assertEqual(quarter_reasons(evidence(),[.15,-.03,0],True,.5),[])
    def test_common_grip_success_at63_is_not_quarter_turn_success(self):
        d=evidence();d.update(final_yaw_from_placement_deg=63,whole_grip_task_passed=True);self.assertIn('whole_cube_not_at90',quarter_reasons(d,[.15,-.03,0],True,.5))
    def test_moving_goals_do_not_count_as_a_fixed_hold(self):self.assertIn('last134_goals_not_constant',quarter_reasons(evidence(),[.15,-.03,0],False,.5))
    def test_quiet_tail_elsewhere_does_not_prove_common_position_return(self):
        d=evidence();d['final_center_world_m'][0]+=.003;self.assertIn('common_center_not_restored',quarter_reasons(d,[.15,-.03,0],True,.5))
    def test_missing_proof_or_nonfinite_angle_cannot_pass(self):
        d=evidence();d['settled_tail']['certificate']['queries']=4049;self.assertIn('exact_upper_space_not_proved_at_every_tail_state',quarter_reasons(d,[.15,-.03,0],True,.5));d=evidence();d['final_yaw_from_placement_deg']=np.nan;self.assertTrue(quarter_reasons(d,[.15,-.03,0],True,.5));self.assertTrue(quarter_reasons(None,[.15,-.03,0],True,.5))
if __name__=='__main__':unittest.main()
