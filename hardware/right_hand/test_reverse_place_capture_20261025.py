"""Device-free checks for fixed goals after PLACE and measured casing release."""
import contextlib,copy,io,json,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from reverse_place_capture_plan_20261025 import ReversePlacePlan,ROOT,TARGET,PHOTO_HOLD,BASE_RELEASE,CASING_PAIR
from reverse_place_capture_gate_20261025 import validate_proof
from reverse_place_capture_protocol_20261025 import ReversePlacementCapture
from leap_reverse_place_capture_20261025 import PlacementBus
from leap_book_cube_grasp_trial import BookGraspBus

FIXTURE=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/placement_fixture_v8'

class Checks(unittest.TestCase):
    def setUp(self):
        self.plan=ReversePlacePlan(FIXTURE/'request.json')
        self.proof=json.loads((FIXTURE/'screen.json').read_bytes())
    def rejected(self,change):
        change(self.proof)
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'screen.json';p.write_text(json.dumps(self.proof),encoding='utf-8')
            with self.assertRaises(RuntimeError):validate_proof(self.plan,p)
    def test_complete_fixture(self):
        self.assertTrue(validate_proof(self.plan,FIXTURE/'screen.json')['placement_capture_only'])
    def test_no_motion_after_place_and_native_bounded_path(self):
        raw=self.plan.source_position();rows=self.plan.plan(raw)
        cp=next(i for i,(p,_) in enumerate(rows) if p==PHOTO_HOLD)
        for _,q in rows[cp-1:]:np.testing.assert_array_equal(q,TARGET)
        values=np.vstack([raw]+[q for _,q in rows]);limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
        self.assertLessEqual(abs(np.diff(values,axis=0)).max(),2)
        self.assertTrue(np.all(values>=np.array(limits['lower_ticks'])))
        self.assertTrue(np.all(values<=np.array(limits['upper_ticks'])))
        self.assertLess((len(rows)+40)*.06+2,self.plan.ON_TIMEOUT)
        for p,q in rows:
            if p==BASE_RELEASE:
                mask=~np.isin(np.arange(16),[3,7,11,15]);np.testing.assert_array_equal(q[mask],raw[mask])
                for i in (7,11,15):self.assertGreaterEqual(q[i],raw[i]);self.assertLessEqual(q[i],TARGET[i])
    def test_reject_injected_motion_after_place(self):
        original=self.plan.plan
        def altered(raw):
            rows=original(raw);rows[-1][1][0]+=1;return rows
        self.plan.plan=altered
        with self.assertRaisesRegex(RuntimeError,'post-placement motion'):validate_proof(self.plan,FIXTURE/'screen.json')
    def test_reject_rotation_claim(self):self.rejected(lambda r:r.update(rotation_included=True))
    def test_reject_missing_case(self):self.rejected(lambda r:r['static_screens'].pop())
    def test_reject_missing_cad_housing(self):self.rejected(lambda r:r['model_manifests'][0]['middle_ring_proxies'].pop())
    def test_reject_new_or_deeper_initial_contact(self):
        self.rejected(lambda r:r['dynamic_sensitivity'][0].update(self_escape_max_unapproved_overlap_mm=.001))
    def test_reject_returning_contact(self):
        self.rejected(lambda r:r['dynamic_sensitivity'][0].update(max_self_after_release_mm=.001))
    def test_reject_casing_scope_change(self):
        self.rejected(lambda r:r.update(initial_casing_contact_limit_mm=2.))
    def test_reject_unsupported_initial_uncertainty_bound(self):
        self.rejected(lambda r:r['dynamic_sensitivity'][0]['initial_support_contacts_mm'].update({'rf_bs_visual_support_proxy|support_book':4.9}))
    def test_reject_moving_enable_hold(self):
        self.rejected(lambda r:r['dynamic_sensitivity'][0]['passive_enable_hold_support']['goal_ticks'].__setitem__(0,1900))
    def test_reject_deep_new_passive_contact(self):
        self.rejected(lambda r:r['dynamic_sensitivity'][0]['passive_enable_hold_support']['contact_maximums_mm'].update({'rf_bs_visual_support_proxy|support_book':.16}))
    def test_reject_unmeasured_release_and_allow_one_following_write(self):
        bus=PlacementBus.__new__(PlacementBus);bus.approve(self.plan,FIXTURE/'screen.json')
        bus.report={};bus._goal_writes=2+bus._base_release_writes
        bus.latest_release_state={'position':self.plan.source_position()};bus.latest_release_read_at=time.monotonic()
        with patch.object(BookGraspBus,'write') as write:
            with self.assertRaisesRegex(RuntimeError,'release not measured'):bus.write(116,4,TARGET)
            write.assert_not_called()
            bus.latest_release_state['position'][3]=bus._base_release_threshold
            bus.write(116,4,TARGET);write.assert_called_once()
            self.assertTrue(bus.report['thumb_base_release_measured_verified'])
    def test_no_stale_release_snapshot(self):
        bus=PlacementBus.__new__(PlacementBus);bus.approve(self.plan,FIXTURE/'screen.json')
        bus.report={};bus._goal_writes=2+bus._base_release_writes
        bus.latest_release_state={'position':TARGET.copy()};bus.latest_release_read_at=time.monotonic()-1
        with patch.object(BookGraspBus,'write') as write:
            with self.assertRaisesRegex(RuntimeError,'No fresh'):bus.write(116,4,TARGET)
            write.assert_not_called()
    def test_place_photo_then_remove_without_motor_writes(self):
        state={'position':TARGET.copy(),'current':np.zeros(16,dtype=int)};events=[]
        class Protocol(ReversePlacementCapture):
            def _observer(self,*args):return lambda phase:state
            def _photo(self,name,phase,observe):events.append(('photo',name));return state,name
            def _space(self,phase,observe):events.append(('space',phase));return state
        protocol=Protocol(None,None,cancel=lambda:None);report={}
        with contextlib.redirect_stdout(io.StringIO()):
            protocol.place(object(),TARGET,report,object());protocol.finish(object(),TARGET,report)
        self.assertEqual(events,[('photo','reverse_start_empty.jpg'),('space','place_cube'),
            ('photo','cube_placed.jpg'),('photo','cube_fixed_hold.jpg'),('space','remove_cube')])
        self.assertTrue(report['cube_removal_confirmed']);self.assertFalse(report['cube_present'])
        self.assertFalse(report['physical_flip_verified']);self.assertFalse(report['registration_verified'])

if __name__=='__main__':unittest.main()
