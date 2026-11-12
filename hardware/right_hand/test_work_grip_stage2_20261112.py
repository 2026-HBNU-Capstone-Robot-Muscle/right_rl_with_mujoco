"""Contract and operator-boundary tests for the new physical diagnostic."""
import contextlib
import io
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

import work_grip_stage2_plan_20261112 as plan
from work_grip_stage2_capture_20261112 import WorkGripStage2Capture


class Stage2Tests(unittest.TestCase):
    def setUp(self):
        self.z = plan.validate_candidate()
        self.first = plan.first_candidate()
        self.manifest = json.loads(plan.MANIFEST.read_bytes())

    def reject(self, change):
        data = {k: v.copy() for k, v in self.z.items()}
        change(data)
        with self.assertRaises((RuntimeError, ValueError)):
            plan.validate_arrays(data, self.first, self.manifest)

    def test_first_real_route_unchanged(self):
        self.assertTrue(np.array_equal(self.z["goals"][:1417], self.first["goals"]))
        self.assertTrue(np.array_equal(self.z["phases"][:1417], self.first["phases"]))

    def test_reject_changed_first_turn(self):
        self.reject(lambda d: d["goals"].__setitem__((100, 0), d["goals"][100, 0] + 1))

    def test_reject_rate_jump(self):
        self.reject(lambda d: d["goals"].__setitem__((1600, 0), d["goals"][1600, 0] + 8))

    def test_reject_native_joint_excess(self):
        self.reject(lambda d: d["goals"].__setitem__((1800, 13), 4000))

    def test_reject_fractional_goal(self):
        def change(d):
            d["goals"] = d["goals"].astype(float)
            d["goals"][1800, 0] += .5
        self.reject(change)

    def test_reject_phase_truncation(self):
        self.reject(lambda d: d.__setitem__("phases", d["phases"][:-1]))

    def test_reject_hidden_open_or_next_turn(self):
        for phase in ("stage_9_loaded_open", "stage_13_next_rotation"):
            self.reject(lambda d, p=phase: d["phases"].__setitem__(1600, p))

    def test_reject_shortened_hold(self):
        def change(d):
            d["goals"] = d["goals"][:-1]
            d["phases"] = d["phases"][:-1]
        self.reject(change)

    def test_plan_preserves_300second_budget(self):
        p = plan.WorkGripStage2Plan(plan.REQUEST)
        rows = p.plan(p.source_position())
        self.assertEqual(p.ON_TIMEOUT, 300)
        self.assertLess((len(rows) + 40) * .075 + 20, 300)
        long = {k: v.copy() for k, v in self.z.items()}
        long["goals"] = np.vstack([long["goals"], np.tile(long["goals"][-1], (5000, 1))])
        long["phases"] = np.r_[long["phases"], [plan.HOLD] * 5000]
        with patch.object(plan, "validate_candidate", return_value=long), self.assertRaises(RuntimeError):
            p.plan(p.source_position())

    def protocol(self):
        capture = WorkGripStage2Capture.__new__(WorkGripStage2Capture)
        capture.guard = object()
        state = {"position": self.z["goals"][-1].copy()}
        calls = []
        capture._observer = lambda *args: lambda phase: state
        capture._photo = lambda *args: (state, {"image": "fake.jpg"})
        capture._space = lambda phase, observe: (calls.append(phase) or state)
        return capture, calls

    def test_reject_incomplete_transition_and_queue(self):
        for report, pending in (({"cube_placement_confirmed": True}, []),
                                ({"cube_placement_confirmed": True, "work_grip_transition_attempted": True}, [1])):
            capture, calls = self.protocol()
            with self.assertRaises(RuntimeError):
                capture.finish(SimpleNamespace(pending=pending), self.z["goals"][-1], report)
            self.assertFalse(calls)

    def test_reject_wrong_final_target(self):
        capture, calls = self.protocol()
        with self.assertRaises(RuntimeError):
            capture.finish(SimpleNamespace(pending=[]), self.z["goals"][-1] + 1,
                           {"cube_placement_confirmed": True, "work_grip_transition_attempted": True})
        self.assertFalse(calls)

    def test_support_confirmation_is_not_removal_or_task_success(self):
        capture, calls = self.protocol()
        report = {"cube_placement_confirmed": True, "work_grip_transition_attempted": True}
        capture.finish(SimpleNamespace(pending=[]), self.z["goals"][-1], report)
        self.assertEqual(calls, ["support_cube_for_off"])
        self.assertTrue(report["cube_support_confirmed_before_off"])
        self.assertFalse(report["cube_removal_confirmed"])
        self.assertFalse(report["physical_work_grip_verified"])
        self.assertFalse(report["physical_upper_clearance_verified"])

    def test_finite_models_include_fail_closed_safety(self):
        import work_grip_stage2_gate_20261112 as gate
        self.assertEqual(gate.models()["finite_cases"], 9)
        self.assertEqual(gate.models()["hand_only_cases"], 3)
        original = gate.json.loads
        def corrupt(blob, *args, **kwargs):
            data = original(blob, *args, **kwargs)
            if data.get("status") == "bounded_work_grip_diagnostic_finite_native_screen":
                data["rows"][0]["worst"]["self_mm"] = .01
            return data
        with patch.object(gate.json, "loads", side_effect=corrupt), self.assertRaises(RuntimeError):
            gate.models()

    def test_default_mode_constructs_no_devices(self):
        import leap_cycle_stage2_work_grip_20261112 as entry
        with patch("sys.argv", ["leap_cycle_stage2_work_grip_20261112.py"]), \
             patch("dynamixel_sdk.PortHandler", side_effect=AssertionError("Unexpected motor access")) as port, \
             patch("camera_stream.CameraStream", side_effect=AssertionError("Unexpected camera access")) as camera, \
             contextlib.redirect_stdout(io.StringIO()) as log:
            code = entry.main()
        self.assertEqual(code, 0)
        self.assertEqual(port.call_count, 0)
        self.assertEqual(camera.call_count, 0)
        self.assertIn("CHECK ONLY", log.getvalue())


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Stage2Tests))
    report = {"tests": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
              "passed": result.wasSuccessful(), "real_devices_opened": False}
    (plan.OUT / "software_verification.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    raise SystemExit(0 if result.wasSuccessful() else 1)
