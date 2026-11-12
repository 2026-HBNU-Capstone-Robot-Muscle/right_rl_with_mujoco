import unittest
import numpy as np
from sac_sensor_bias import position_bias_offset_mm,position_bias_cases
from sac_reference_observation import ReferenceObservation

class BiasTest(unittest.TestCase):
    def test_offset_matches_independent_observation_builder(self):
        exact=ReferenceObservation();biased=ReferenceObservation();bias=np.array([3.,-8.,15.])
        for i in range(4):
            args=dict(measured_joints=np.zeros(16),commanded_joints=np.ones(16)*.01,
                      palm_position=[.11,0,.03],cube_position=np.array([.10+.001*i,0,.01]),
                      cube_quaternion=[1,0,0,0],goal_quaternion=[1,0,0,0],last_executed_action=np.zeros(16),reference_step=i)
            a=exact.build(**args);args['cube_position']+=bias/1000;b=biased.build(**args)
        np.testing.assert_allclose(b-a,position_bias_offset_mm(bias),atol=1e-7)
        self.assertEqual(np.count_nonzero(position_bias_offset_mm([0,0,0])),0)
    def test_cases_and_invalid_inputs(self):
        cases=position_bias_cases(15);self.assertEqual(len(cases),7)
        np.testing.assert_allclose(np.sum(cases,axis=0),0)
        for bad in ([1,2],[1,2,np.nan]):
            with self.assertRaises(ValueError):position_bias_offset_mm(bad)
        for bad in (0,-1,np.inf,101):
            with self.assertRaises(ValueError):position_bias_cases(bad)

if __name__=='__main__':unittest.main()
