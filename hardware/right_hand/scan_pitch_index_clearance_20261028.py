"""Full offset screen of small IF axial changes, retaining the RF margin."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,mujoco,numpy as np
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from hand_motor_mapping import ticks_to_sim
from book_empty_preparation_plan import offsets
from screen_pitch_yaw_join_20261028 import contact_reader
from pitch_handoff_plan_20261028 import ROOT,OUT,LOADED_PATH,PITCH

def main():
    folder=OUT/'index_clearance_search';folder.mkdir(parents=True,exist_ok=True)
    z=np.load(LOADED_PATH);pc=len(np.load(PITCH)['goals']);lo,hi=limits()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    m,_=build(registry.load('LeapCubeFaceUp',config=cfg),folder/'scene.xml')
    correct_model(m,z['start']);inspect=contact_reader(m);reports=[]
    for delta in (-8,-12,-4):
        a=z['start'].copy();a[5]+=delta;goals=z['goals'].copy();goals[:pc,5]+=delta
        b=goals[pc-1];end=z['goals'][-1]
        goals[pc:pc+600]=np.rint([b+(end-b)*i/600 for i in range(1,601)]).astype(int)
        assert np.max(abs(np.diff(np.vstack([a,goals]),axis=0)))<=2
        assert np.all(goals>=lo) and np.all(goals<=hi)
        d=mujoco.MjData(m);d.qpos[-7:]=[0,0,2,1,0,0,0]
        sp=sup=0.;worst=None;at=None;checked=0;failed=False
        for offset in offsets():
            for i,q in enumerate(np.vstack([a,goals])):
                d.qpos[:16]=ticks_to_sim(q+offset);mujoco.mj_forward(m,d)
                x,y,p,_=inspect(d)
                if x>sp:sp=x;worst=p;at=i
                sup=max(sup,y)
                if sp>1e-6 or sup>.15:failed=True;break
            checked+=1
            if failed:break
        row=dict(index_axial_motor6_delta_ticks=delta,ring_axial_motor14_delta_ticks=8,
                 passed_all67_offsets=not failed and checked==67,checked_offsets=checked,
                 max_self_mm=sp,max_support_mm=sup,worst_pair=worst,worst_index=at)
        if row['passed_all67_offsets']:
            name=f'if6_{delta:+d}_rf14_+8.npz'
            np.savez_compressed(folder/name,start=a,goals=goals,phases=z['phases'],sample_period_s=.075)
            row['candidate']=name
        reports.append(row);print(json.dumps(row),flush=True)
        if row['passed_all67_offsets']:break
    report=dict(status='index_ring_clearance_entire_loaded_route_search',cases=reports,
                support_front_only_nominal=True,loaded_dynamics_not_checked=True,
                hardware_execution_allowed=False,devices_opened=False)
    (folder/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':main()
