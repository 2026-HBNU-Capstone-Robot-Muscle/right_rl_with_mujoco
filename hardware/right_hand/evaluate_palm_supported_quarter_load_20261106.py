"""Test actual hypothetical reaction resistance of a palm/index-supported cube.

The old named MF+TH filter remains unchanged and is reported separately. This
functional probe asks whether the same spatial drift and upper-clearance limits
survive external torque. It does not claim spatial force closure or actual
left-hand contact. All numeric dynamic and load criteria remain unchanged.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json
from unittest.mock import patch
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from evaluate_canonical_grip_reaction_20261105 import evaluate,load_fraction
from research_rotation_body_hold_20261104 import restore,angle

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--downward-N',type=float,choices=(0.,1.,2.),default=0.);args=ap.parse_args()
    parent=ROOT/'near_quarter_full_replay';r=json.loads((parent/'review.json').read_bytes());model=R/r['source_model'];assert sha(model)==r['source_model_sha256'];m=mujoco.MjModel.from_binary_path(str(model));source=ROOT/'postturn_thumb_clearance/id1_delta-64/end_state.npz';z=np.load(source);out=ROOT/(f'palm_supported_quarter_load_down{args.downward_N:g}N' if args.downward_N else 'palm_supported_quarter_load');out.mkdir(exist_ok=True);cases=[];step=mujoco.mj_step;body=m.body('cube').id
    for torque in ((0.,-.01,.01) if args.downward_N else (0.,-.003,.003,-.01,.01)):
        maximum_tilt=0.
        def observe(model,data,*step_args,**kwargs):
            nonlocal maximum_tilt
            seconds=float(data.time)-float(z['time']);data.xfrc_applied[body,:3]=-z['up']*args.downward_N*load_fraction(seconds)
            step(model,data,*step_args,**kwargs);B=data.xmat[body].reshape(3,3);maximum_tilt=max(maximum_tilt,float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1)))))
        save=out/f'torque{torque:+g}_end.npz'
        with patch.object(mujoco,'mj_step',observe):v=evaluate(m,z,torque,save)
        d=restore(m,np.load(save));yaw=angle(d.xmat[body].reshape(3,3),z['initial_cube_rotation'],z['up']);why=[reason for reason in v['reasons'] if reason!='opposed_lower_body_contacts']
        if maximum_tilt>=5:why.append('absolute_working_top_tilt')
        if abs(yaw-90)>=5:why.append('absolute_whole_cube_yaw')
        cases.append(dict(dynamic=v,maximum_absolute_up_tilt_deg=maximum_tilt,final_whole_cube_yaw_deg=yaw,functional_reaction_probe_passed=not why,functional_reasons=why,original_MF_TH_contact_filter_passed=v['load_filter_passed'],spatial_force_closure_not_proved=True))
        report=dict(status='functional_load_probe_of_alternative_palm_index_support',source_state=str(source.relative_to(R)),source_state_sha256=sha(source),source_model_sha256=sha(model),cases=cases,hypothetical_downward_force_N=args.downward_N,left_hand_downward_force_capacity_not_verified=True,numeric_safety_drift_and_clearance_limits_unchanged=True,old_MF_TH_grip_filter_not_changed=True,new_functional_probe_does_not_require_prescribed_finger_names=True,hypothetical_external_torque_not_measured_left_hand_load=True,not_a_full_initial_placement_replay_or_articulated_layer_trial=True,hardware_execution_allowed=False,devices_opened=False);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(torque=torque,downward_N=args.downward_N,functional=not why,reasons=why,yaw_drift=v['max_yaw_change_deg'],shift=v['max_center_shift_mm'],up_tilt=maximum_tilt,gap=v['minimum_separating_gap_1ms_mm'])),flush=True)
if __name__=='__main__':main()
