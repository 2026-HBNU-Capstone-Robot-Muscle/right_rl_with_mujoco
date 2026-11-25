"""Exercise the changed checkpoint with the existing fake-bus route/abort tests."""
import copy
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import test_loaded_quarter_finish_20261026 as prior
import forward_cube_placement_20261026 as mode
import loaded_quarter_finish_gate_20261025 as native_gate
from forward_cube_placement_20261026 import (
    ForwardPlacementPlan, ForwardPlacementCapture, ROOT, FIXTURE, REVIEW, require_placement_review)

original_protocol = prior.protocol

def protocol(bus, fail):
    p = original_protocol(bus, fail)
    p.__class__ = ForwardPlacementCapture
    return p

class ForwardPlacementChecks(prior.LoadedFinishChecks):
    def setUp(self):
        self.plan = ForwardPlacementPlan(FIXTURE/'request.json')

    def run_path(self, fail=None):
        with patch.object(prior, 'protocol', protocol):
            return super().run_path(fail)

    def test_shift_is_a_request_and_never_a_measured_success(self):
        r, bus, _, _ = self.run_path()
        self.assertEqual(len(bus.loaded_goals), 824)
        self.assertEqual(r['placement_comparison']['requested_forward_shift_mm'], 5.)
        self.assertFalse(r['placement_comparison']['shift_measured_or_camera_registered'])
        self.assertFalse(r['placement_comparison']['motor_route_changed'])
        self.assertFalse(r['physical_flip_verified'])

    def test_review_cannot_relabel_previous_partial_flip_as_success(self):
        review = json.loads(REVIEW.read_bytes())
        review['review_ready'] = True
        review['mode_code_hashes'] = {n: mode.sha(ROOT/n) for n in mode.FILES}
        wrong = copy.deepcopy(review)
        wrong['prior_physical_flip_completed'] = True
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'review.json'
            # Native proof validation has its own unchanged tests. Here first
            # accept otherwise-valid metadata, then reject the false outcome.
            with patch.object(mode, 'require_offline_review', return_value={}), \
                    patch.object(native_gate, 'validate_proof', return_value={}):
                p.write_text(json.dumps(review), encoding='utf-8')
                self.assertEqual(require_placement_review(p), {})
                p.write_text(json.dumps(wrong), encoding='utf-8')
                with self.assertRaisesRegex(RuntimeError, 'review/source mismatch'):
                    require_placement_review(p)

    def test_new_default_mode_opens_no_devices(self):
        r = subprocess.run([sys.executable, '-X', 'utf8', '-B', str(ROOT/'leap_forward_cube_placement_20261026.py')],
            capture_output=True, text=True, encoding='utf-8', timeout=8)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.', r.stdout)

if __name__ == '__main__':
    unittest.main()
