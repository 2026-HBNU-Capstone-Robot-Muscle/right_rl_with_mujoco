"""Failed checker diagnostics stay separate from motor/proof approval."""
import json
import tempfile
import unittest
from pathlib import Path
from thumb_live_start_gate import screen_failure_details,run_screen

class Tests(unittest.TestCase):
    def test_derived_plan_and_reported_limits_do_not_use_legacy_64(self):
        from supported_yaw_empty_plan import SupportedYawEmptyPlan
        from cube_small_rotation_plan import SmallRotationPlan
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'proof.json'
            for version in (SupportedYawEmptyPlan.PLAN_VERSION,SmallRotationPlan.PLAN_VERSION):
                report=dict(plan_version=version,static_screens=[dict(contact_clear_pip_travel_ticks=80,
                    self_escape_max_unapproved_overlap_mm=0)],dynamic_sensitivity=[dict(
                    self_escape_max_unapproved_overlap_mm=.001)])
                path.write_text(json.dumps(report),encoding='utf-8')
                reason=screen_failure_details(path)
                self.assertNotIn('short release',reason)
                self.assertIn('contact',reason)
                report['static_screens'][0]['contact_clear_pip_travel_ticks']=81
                path.write_text(json.dumps(report),encoding='utf-8')
                self.assertIn('short release',screen_failure_details(path))
            report['plan_version']='future_derived_version'
            report['contact_release_travel_limit_ticks']=80
            report['static_screens'][0]['contact_clear_pip_travel_ticks']=80
            path.write_text(json.dumps(report),encoding='utf-8')
            self.assertNotIn('short release',screen_failure_details(path))

    def test_failed_certificate_explains_the_required_release(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'proof.json'
            path.write_text(json.dumps(dict(static_screens=[dict(contact_clear_pip_travel_ticks=65,
                self_escape_complete=True)],dynamic_sensitivity=[])),encoding='utf-8')
            self.assertIn('short release',screen_failure_details(path))
            path.write_text('invalid',encoding='utf-8')
            self.assertIn('readable diagnostic',screen_failure_details(path))

    def test_nonzero_checker_exit_is_not_hidden_behind_a_generic_message(self):
        class Process:
            returncode=2
            def poll(self):return 2
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);request=folder/'request.json';request.write_text('{}')
            output=folder/'proof.json'
            def popen(*args,**kwargs):
                output.write_text(json.dumps(dict(static_screens=[dict(contact_clear_pip_travel_ticks=65)],
                    dynamic_sensitivity=[])),encoding='utf-8')
                return Process()
            with self.assertRaisesRegex(RuntimeError,'short release.*no motion started'):
                run_screen(request,output,cancel=lambda:None,popen=popen)

if __name__=='__main__':unittest.main()
