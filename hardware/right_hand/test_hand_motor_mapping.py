import unittest
from unittest.mock import patch
import numpy as np
from hand_motor_mapping import ticks_to_sim, sim_to_ticks, hw_to_sim, metadata, reject_legacy_sac_execution

class MappingTests(unittest.TestCase):
    def test_all_fingers_match_explicit_user_order(self):
        # Independent user-supplied tip-to-root groups, expanded into model slots.
        expected={8:0,6:1,7:2,5:3,12:4,10:5,11:6,9:7,
                  16:8,14:9,15:10,13:11,4:12,3:13,2:14,1:15}
        for motor,joint in expected.items():
            ticks=np.full(16,2048);ticks[motor-1]=2304
            q=ticks_to_sim(ticks)
            self.assertEqual(np.flatnonzero(q).tolist(),[joint])
            self.assertEqual(sim_to_ticks(q)[motor-1],2304)

    def test_each_middle_encoder_has_user_confirmed_role(self):
        # Independent physical IDs, not inferred from the implementation's permutation.
        for motor,joint in ((12,4),(10,5),(11,6),(9,7)):
            ticks=np.full(16,2048);ticks[motor-1]=2304
            result=ticks_to_sim(ticks)
            self.assertEqual(np.flatnonzero(result).tolist(),[joint])
            self.assertAlmostEqual(result[joint],np.pi/8)

    def test_command_conversion_places_root_and_tip_on_correct_ids(self):
        q=np.zeros(16);q[4]=np.pi/2;q[7]=np.pi/8
        ticks=sim_to_ticks(q)
        self.assertEqual(ticks[11],3072);self.assertEqual(ticks[8],2304)
        self.assertEqual(ticks[9],2048);self.assertEqual(ticks[10],2048)

    def test_batch_raw_roundtrip_without_clipping_or_wrap(self):
        ticks=np.array([np.arange(16)*111+1000,np.arange(16)*7+1900])
        np.testing.assert_array_equal(sim_to_ticks(ticks_to_sim(ticks)),ticks)

    def test_input_contract_and_no_motion_certification(self):
        for x in (np.zeros(15),np.full(16,np.nan),0):
            with self.assertRaises(ValueError):hw_to_sim(x)
        self.assertFalse(metadata()['trajectory_validation_complete'])
        with self.assertRaisesRegex(RuntimeError,'obsolete middle mapping'):reject_legacy_sac_execution()

    def test_legacy_execution_stops_before_plan_or_bus(self):
        import leap_sac_start_empty as start
        import leap_sac_path_empty as path
        for module,loader in ((start,'target_pose'),(path,'load_diagnostic_path')):
            with patch('sys.argv',['program','--execute']),patch.object(module,loader) as load,patch.object(module.checked,'Bus') as bus:
                with self.assertRaisesRegex(RuntimeError,'obsolete middle mapping'):module.main()
                load.assert_not_called();bus.assert_not_called()

    def test_legacy_loaded_policy_stops_before_camera(self):
        import held_policy_shadow as loaded
        for options in ({'policy_mode':True},{'loaded_policy_mode':True},{'loaded_policy_mode':True,'sac_deployment_mode':True}):
            with patch.object(loaded,'Camera') as camera,patch.object(loaded.shadow,'Policy') as actor:
                self.assertEqual(loaded.main(**options),1)
                camera.assert_not_called();actor.assert_not_called()

if __name__=='__main__':unittest.main()
