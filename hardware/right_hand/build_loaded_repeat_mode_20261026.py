"""Isolated copies: original successful loaded files remain unchanged."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def rep(s,a,b,n=1):
    if s.count(a)!=n:raise ValueError('Unexpected source block: '+a[:80])
    return s.replace(a,b,n)
def main():
    s=(ROOT/'loaded_thumb_voltage_sample_20261026.py').read_text(encoding='utf-8')
    s=rep(s,"version='LOADED_immediate_second_healthy_voltage_read_v1_20261026'","version='LOADED_precise_read_paced_voltage_v1_20261026'")
    s=rep(s,'nonrecovered_poll_delay_s=.01,first_healthy_followup_delay_s=0.',
        "nonrecovered_minimum_poll_cycle_s=.01,read_work_included_in_poll_cycle=True,first_healthy_followup_delay_s=0.,\n        confirmation_clock='time.perf_counter',clock_epoch_not_mixed_with_enabled_at=True")
    s=rep(s,'began=time.monotonic()\n    event=dict(phase=phase,t=began-report[\'enabled_at\'],',
        "began=time.perf_counter()\n    event=dict(phase=phase,t=time.monotonic()-report['enabled_at'],")
    s=rep(s,"print('Voltage dip: hold last goal;150ms budget. First healthy voltage is re-read immediately.',flush=True)",
        "print('Voltage dip: hold goals;150ms precise budget. Read work counts toward the10ms polling cycle.',flush=True)")
    s=rep(s,'if time.monotonic()-began>=checked.VOLTAGE_CONFIRM_S:break',
        'if time.perf_counter()-began>=checked.VOLTAGE_CONFIRM_S:break\n            read_began=time.perf_counter()')
    s=rep(s,'elapsed=time.monotonic()-began','elapsed=time.perf_counter()-began')
    s=rep(s,'if good==0:time.sleep(.01)',
        'if good==0:\n                remaining=max(0.,.01-(time.perf_counter()-read_began))\n                if remaining:time.sleep(remaining)')
    s=rep(s,"elapsed_ms=(time.monotonic()-began)*1000,error=str(exc)","elapsed_ms=(time.perf_counter()-began)*1000,error=str(exc)")
    s=rep(s,"if not getattr(plan,'SUPERVISED_LOADED_THUMB_MODE',False) or not getattr(plan,'CAPTURE_PLACEMENT',False):",
        "if (not getattr(plan,'PRECISE_READ_PACED_VOLTAGE',False) or not getattr(plan,'SUPERVISED_LOADED_THUMB_MODE',False) or not getattr(plan,'CAPTURE_PLACEMENT',False)):")
    s=s.replace('Continue the old10ms pacing only while voltage has not recovered.',
        'The10ms minimum retry cycle includes the blocking full-health READ.')
    (ROOT/'loaded_repeat_voltage_20261026.py').write_text(s,encoding='utf-8')
    s=(ROOT/'screen_loaded_thumb_rotation_20261026.py').read_text(encoding='utf-8')
    s=rep(s,'from loaded_thumb_rotation_plan_20261026 import LoadedThumbRotationPlan as BookGraspPlan,',
        'from loaded_repeat_plan_20261026 import LoadedRepeatPlan as BookGraspPlan,')
    s=rep(s,'from loaded_thumb_voltage_sample_20261026 import metadata as voltage_metadata',
        'from loaded_repeat_voltage_20261026 import metadata as voltage_metadata')
    s=rep(s,'from camera_gap_native_policy_20261026 import add_stress_holds,expected_holds',
        'from camera_gap_native_policy_20261026 import add_stress_holds,expected_holds\nfrom repeat_support_model_resolution_20261026 import classify as resolve_support,metadata as support_resolution_metadata')
    s=rep(s,'corner_observations=[]','corner_observations=[];support_resolution_observations=[]')
    s=rep(s,"raw_ring_max=[0.];palm_face_maxima={}","support_discarded={};support_resolution_observations.append(support_discarded)\n        raw_ring_max=[0.];palm_face_maxima={}")
    s=rep(s,'support_pairs,self_pairs=inspect_raw(d)',
        'support_pairs,self_pairs=inspect_raw(d)\n            resolved_support=resolve_support(support_pairs)\n            for pair,depth in support_pairs.items():\n                if pair not in resolved_support:support_discarded[pair]=max(support_discarded.get(pair,0.),depth)\n            support_pairs=resolved_support')
    s=rep(s,'r=dict(fresh_voltage_read_policy=voltage_metadata()',
        'r=dict(support_model_resolution=support_resolution_metadata(),support_resolution_discarded_maximums_by_front=support_resolution_observations,fresh_voltage_read_policy=voltage_metadata()')
    (ROOT/'screen_loaded_repeat_20261026.py').write_text(s,encoding='utf-8')
    s=(ROOT/'loaded_thumb_rotation_gate_20261026.py').read_text(encoding='utf-8')
    s=rep(s,"checker=ROOT/'screen_loaded_thumb_rotation_20261026.py'","checker=ROOT/'screen_loaded_repeat_20261026.py'")
    s=rep(s,'from loaded_thumb_rotation_plan_20261026 import TURN_PHASE,','from loaded_repeat_plan_20261026 import TURN_PHASE,')
    s=rep(s,'from loaded_thumb_voltage_sample_20261026 import metadata as voltage_metadata',
        'from loaded_repeat_voltage_20261026 import metadata as voltage_metadata')
    s=rep(s,'from camera_gap_native_policy_20261026 import expected_holds',
        'from camera_gap_native_policy_20261026 import expected_holds\nfrom repeat_support_model_resolution_20261026 import metadata as support_resolution_metadata,accepted_discard as accepted_support_discard')
    s=rep(s,"corner=r.get('return_corner_discarded_maximums_by_front')",
        "support=r.get('support_resolution_discarded_maximums_by_front')\n    if (r.get('support_model_resolution')!=support_resolution_metadata() or not isinstance(support,list) or len(support)!=3\n        or any(not isinstance(v,dict) or any(not accepted_support_discard(pair,depth) for pair,depth in v.items()) for v in support)):\n        raise RuntimeError('Known support-edge resolution evidence mismatch; no writes')\n    corner=r.get('return_corner_discarded_maximums_by_front')")
    (ROOT/'loaded_repeat_gate_20261026.py').write_text(s,encoding='utf-8')
    print('Built isolated repeat sampler/checker/gate. Previous success files unchanged.')
if __name__=='__main__':main()
