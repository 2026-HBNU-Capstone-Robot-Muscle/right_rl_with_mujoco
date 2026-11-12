"""Correction/source/proof integration tests; never open real device APIs."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import nominal_cycle_plan_20261112 as plan
import nominal_cycle_gate_20261112 as gate
import leap_cycle_nominal_two_turns_20261112 as entry

BACKUP=plan.OUT/'before_index_support_fix_v3_20261112'
class Tests(unittest.TestCase):
    def test_only_small_support_joint_change(self):
        old=np.load(plan.OUT/'rf16_2177_v2/candidate.npz',allow_pickle=False);new=plan.candidate()
        keep=[j for j in range(16) if j!=7]
        self.assertTrue(np.array_equal(new['start'],old['start']))
        self.assertTrue(np.array_equal(new['goals'][:,keep],old['goals'][:,keep]))
        self.assertTrue(np.array_equal(new['goals'][:1800],old['goals'][:1800]))
        delta=new['goals'][:,7]-old['goals'][:,7]
        self.assertEqual(int(delta.min()),0);self.assertEqual(int(delta.max()),8)
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([new['start'],new['goals']]),axis=0))),2)
    def test_independent_full_cycle_and_unchanged_hold_queue(self):
        z=plan.candidate();r=json.loads(plan.SOURCE_REVIEW.read_bytes())
        self.assertEqual(len(z['goals']),5089);self.assertEqual(r['all1ms_samples'],381675)
        self.assertEqual(r['actual_state_resets_after_initial_placement'],0)
        for phase in (plan.FIRST_HOLD,plan.FINAL_HOLD):
            q=z['goals'][z['phases']==phase];self.assertEqual(len(q),54);self.assertTrue(np.all(q==q[0]))
        for h in r['holds']:
            self.assertEqual(h['samples'],4050);self.assertEqual(h['upper_certified_samples'],4050)
            self.assertEqual(h['supported_samples'],4050);self.assertLess(h['target_error_deg'],5)
    def test_current_proof_preserves_limits_and_failed_cases(self):
        p=plan.NominalCyclePlan(plan.REQUEST);r=gate.validate_review(p,plan.PROOF)
        self.assertTrue(plan.scope_approved());self.assertEqual(r['active_motion_limit_s'],500)
        self.assertEqual((r['current_ma'],r['goal_step_ticks'],r['sample_period_s']),(450,2,.075))
        for key in ('physical_fall_fix_confirmed','tracking_bias_sensitivity_passed','all_condition_safety_certified','hand_only_all_passed','physical_two_turn_success'):
            self.assertFalse(r[key])
        self.assertGreater(r['tracking_bias33_first_hold_error_deg'],5)
        self.assertTrue(all(not row['passed'] and row['stop'] for row in r['hand_only_comparisons']))
    def test_stale_v2_proof_rejected(self):
        with self.assertRaises(RuntimeError):gate.validate_review(plan.NominalCyclePlan(plan.REQUEST),BACKUP/'review.json')
    def test_changed_candidate_rejected_before_runtime(self):
        with tempfile.TemporaryDirectory() as folder:
            bad=Path(folder)/'candidate.npz';z=plan.candidate();z['goals'][2200,7]+=1
            np.savez_compressed(bad,start=z['start'],goals=z['goals'])
            with patch.object(plan,'SOURCE',bad),self.assertRaises(RuntimeError):plan.candidate()
    def test_default_and_mocked_execute_never_prompt_again(self):
        approval=plan.APPROVAL.read_bytes()
        with patch('dynamixel_sdk.PortHandler',side_effect=AssertionError('real motor')), \
             patch('camera_stream.CameraStream',side_effect=AssertionError('real camera')), \
             patch('builtins.input',side_effect=AssertionError('repeat approval')),contextlib.redirect_stdout(io.StringIO()) as log:
            with patch('sys.argv',['trial']):self.assertEqual(entry.main(),0)
            self.assertIn('CHECK ONLY',log.getvalue());self.assertIn('v3',log.getvalue())
            with patch('sys.argv',['trial','--execute']),patch.object(entry.runtime,'main',return_value=0) as runtime:
                self.assertEqual(entry.main(),0);self.assertEqual(runtime.call_count,1)
        self.assertEqual(approval,plan.APPROVAL.read_bytes())
    def test_original_guard_and_sources_unchanged(self):
        before=json.loads((BACKUP/'review.json').read_bytes())['code_hashes']
        allowed={name for name in ('nominal_cycle_plan_20261112.py','nominal_cycle_gate_20261112.py','leap_cycle_nominal_two_turns_20261112.py')}
        allowed.update((plan.REQUEST.relative_to(plan.ROOT).as_posix(),plan.MANIFEST.relative_to(plan.ROOT).as_posix()))
        checked=0
        for name,digest in before.items():
            if name in allowed:continue
            self.assertEqual(plan.sha(plan.ROOT/name),digest,name);checked+=1
        self.assertEqual(checked,len(set(before)-allowed))
        frozen=json.loads((plan.ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json').read_bytes())['code_hashes']
        self.assertEqual(len(frozen),170)
        for name,digest in frozen.items():self.assertEqual(plan.sha(plan.ROOT/name),digest,name)
        (plan.SOURCE.parent/'source_preservation_check.json').write_text(json.dumps(dict(
            prior_cycle_bindings_unchanged=checked,frozen_execution_bindings_verified=len(frozen),
            frozen_execution_changes=[],devices_opened=0,remote_writes=0),indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':
    r=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
    summary=dict(tests=r.testsRun,failures=len(r.failures),errors=len(r.errors),passed=r.wasSuccessful(),
                 devices_opened=0,assistant_motor_writes=0,physical_fall_fix_confirmed=False,
                 full_nominal_cycle_model_passed=True,tracking_bias_sensitivity_passed=False)
    (plan.SOURCE.parent/'software_verification.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary));raise SystemExit(not r.wasSuccessful())
