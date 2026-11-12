"""Build standalone rate-aware screens; original programs/proofs stay unchanged."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def rep(s,a,b,n=1):
    if s.count(a)!=n:raise ValueError('Unexpected source block: '+a[:90])
    return s.replace(a,b,n)
def main():
    s=(ROOT/'screen_recovered_forward_20261026.py').read_text(encoding='utf-8')
    s=rep(s,'from recovered_forward_plan_20261026 import RecoveredForwardPlan as BookGraspPlan,TURN_PHASE,END_HOLD,FINISH_PHASE,loaded_basis,finish_goals',
        'from slow_forward_plan_20261026 import SlowForwardPlan as BookGraspPlan,TURN_PHASE,END_HOLD,FINISH_PHASE,loaded_basis,finish_goals\nfrom slow_thumb_seam_resolution_20261026 import classify as classify_slow_seam,metadata as slow_seam_metadata\nCYCLE=BookGraspPlan.SAMPLE_PERIOD')
    s=rep(s,'photo_resolution_observations=[]','photo_resolution_observations=[];slow_seam_observations=[]')
    s=rep(s,"m,manifest=build_cad(env,out.with_name('reverse_empty_CAD_scene.xml'),book_projection_mm=front)",
        "m,manifest=build_cad(env,out.with_name('reverse_empty_CAD_scene.xml'),book_projection_mm=front)\n        manifest['native_timestep_before_refinement_s']=float(m.opt.timestep)\n        m.opt.timestep=.005\n        manifest['refined_simulation_timestep_s']=.005")
    s=rep(s,'raw_ring_max=[0.];photo_discarded={}',
        'raw_ring_max=[0.];photo_discarded={};slow_discarded={};slow_seam_observations.append(slow_discarded)')
    s=rep(s,'return support_pairs,resolved',
        "after_seam=classify_slow_seam(resolved)\n            for pair,depth in resolved.items():\n                if pair not in after_seam:slow_discarded[pair]=max(slow_discarded.get(pair,0.),depth)\n            return support_pairs,after_seam")
    s=rep(s,"lift_s=paths[0]['lift_count']*.05","lift_s=paths[0]['lift_count']*CYCLE")
    s=rep(s,"dynamics=[('initial_hold',angles[0])]*10","dynamics=[('initial_hold',angles[0],.5)]")
    s=rep(s,"dynamics.extend([('native_placement_hold',dynamics[-1][1])]*80);inserted=True","dynamics.append(('native_placement_hold',dynamics[-1][1],4.));inserted=True")
    s=rep(s,'dynamics.append((phase,goal))','dynamics.append((phase,goal,CYCLE))')
    s=rep(s,"dynamics.extend([('final_hold',angles[-1])]*20)","dynamics.append(('final_hold',angles[-1],20*CYCLE))")
    s=rep(s,"tip_s=paths[0]['tip_count']*.05","tip_s=paths[0]['tip_count']*CYCLE")
    s=rep(s,"ring_s=paths[0]['ring_count']*.05","ring_s=paths[0]['ring_count']*CYCLE")
    s=rep(s,'for phase,goal in dynamics:\n                d.ctrl[:]=goal\n                for _ in range(round(.05/m.opt.timestep)):',
        'target_elapsed=0.\n            simulated_steps=0\n            for phase,goal,duration in dynamics:\n                d.ctrl[:]=goal\n                target_elapsed+=duration\n                target_steps=round(target_elapsed/m.opt.timestep)\n                segment_steps=target_steps-simulated_steps\n                simulated_steps=target_steps\n                for _ in range(segment_steps):')
    s=rep(s,'r=dict(recorded_interrupted_contact_policy=',
        "r=dict(slow_thumb_seam_resolution=slow_seam_metadata(),slow_thumb_seam_discarded_maximums_by_front=slow_seam_observations,control_cycle_seconds=CYCLE,simulation_timestep_seconds=m.opt.timestep,control_sim_step_rounding='cumulative_nearest_native_step',maximum_control_boundary_rounding_s=m.opt.timestep/2,initial_hold_seconds=.5,native_placement_hold_seconds=4.,native_final_hold_seconds=20*CYCLE,voltage_supply_model_identified=False,recorded_interrupted_contact_policy=")
    (ROOT/'screen_slow_forward_20261026.py').write_text(s,encoding='utf-8')
    g=(ROOT/'recovered_forward_gate_20261026.py').read_text(encoding='utf-8')
    g=rep(g,"checker=ROOT/'screen_recovered_forward_20261026.py'","checker=ROOT/'screen_slow_forward_20261026.py'")
    g=rep(g,'from recovered_forward_plan_20261026 import TURN_PHASE,FINISH_PHASE,END_HOLD,loaded_basis,finish_goals',
        'from slow_forward_plan_20261026 import TURN_PHASE,FINISH_PHASE,END_HOLD,loaded_basis,finish_goals\nfrom slow_thumb_seam_resolution_20261026 import metadata as slow_seam_metadata,PAIR as SLOW_SEAM_PAIR,NUMERICAL_ZERO_MM as SLOW_ZERO_MM')
    g=rep(g,"r=json.loads(Path(path).read_bytes());raw=plan.source_position();rows=plan.plan(raw)",
        "r=json.loads(Path(path).read_bytes());raw=plan.source_position();rows=plan.plan(raw)\n    cycle=plan.SAMPLE_PERIOD\n    if (r.get('control_cycle_seconds')!=cycle or r.get('simulation_timestep_seconds')!=.005 or r.get('control_sim_step_rounding')!='cumulative_nearest_native_step' or not 0<=r.get('maximum_control_boundary_rounding_s',float('inf'))<=.0025+1e-12 or r.get('initial_hold_seconds')!=.5 or r.get('native_placement_hold_seconds')!=4. or r.get('native_final_hold_seconds')!=20*cycle or r.get('voltage_supply_model_identified') is not False):\n        raise RuntimeError('Slow-cycle timing proof mismatch; no writes')\n    seam=r.get('slow_thumb_seam_discarded_maximums_by_front')\n    if r.get('slow_thumb_seam_resolution')!=slow_seam_metadata() or not isinstance(seam,list) or len(seam)!=3 or any(not isinstance(v,dict) or any(p!=SLOW_SEAM_PAIR or not 0<=d<=SLOW_ZERO_MM for p,d in v.items()) for v in seam):\n        raise RuntimeError('Slow seam numerical-resolution evidence mismatch; no writes')")
    g=g.replace('(ring_count+tip_count)*.05','(ring_count+tip_count)*cycle').replace('lift_count*.05','lift_count*cycle')
    (ROOT/'slow_forward_gate_20261026.py').write_text(g,encoding='utf-8')
    runtime=(ROOT/'quarter_current_runtime_20261025.py').read_text(encoding='utf-8')
    runtime=rep(runtime,"print('50ms cycles include camera/read/write work; every goal still requires fresh health reads.',flush=True)",
        "print(f'{plan_type.SAMPLE_PERIOD*1000:g}ms cycles include camera/read/write work; every goal still requires fresh health reads.',flush=True)")
    runtime=rep(runtime,"if __name__=='__main__':raise SystemExit(main())", "if __name__=='__main__':raise SystemExit('Use leap_slow_forward_placement_20261026.py; no devices opened.')")
    (ROOT/'slow_forward_current_runtime_20261026.py').write_text(runtime,encoding='utf-8')
    print('Built slow rate-aware checker, validator and isolated display wrapper.')
if __name__=='__main__':main()
