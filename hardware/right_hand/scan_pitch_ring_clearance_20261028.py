"""Offline small RF axial corrections; preserve the other15 recorded goals."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
import mujoco,numpy as np
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from hand_motor_mapping import ticks_to_sim
from pitch_handoff_plan_20261028 import ROOT,OUT,LOADED_PATH,PITCH
from screen_pitch_yaw_join_20261028 import contact_reader

def main():
    folder=OUT/'rf_clearance_search';folder.mkdir(parents=True,exist_ok=True)
    source=np.load(LOADED_PATH);start=source['start'];original=source['goals'];phases=source['phases']
    pitch_count=len(np.load(PITCH)['goals']);transfer_count=600
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    m,_=build(registry.load('LeapCubeFaceUp',config=cfg),folder/'scene.xml')
    correct_model(m,start);inspect=contact_reader(m);lo,hi=limits();cases=[]
    for delta in (-12,-8,-4,4,8,12):
        a=start.copy();a[13]+=delta;goals=original.copy();goals[:pitch_count,13]+=delta
        b=goals[pitch_count-1].copy();end=original[-1]
        goals[pitch_count:pitch_count+transfer_count]=np.rint([b+(end-b)*i/transfer_count for i in range(1,transfer_count+1)]).astype(int)
        assert np.max(abs(np.diff(np.vstack([a,goals]),axis=0)))<=2
        assert np.all(goals>=lo) and np.all(goals<=hi)
        d=mujoco.MjData(m);d.qpos[-7:]=[0,0,2,1,0,0,0]
        sp=sup=0.;worst=None;at=None
        for offset in (np.zeros(16,int),np.full(16,4),np.full(16,-4)):
            for i,q in enumerate(np.vstack([a,goals])):
                d.qpos[:16]=ticks_to_sim(q+offset);mujoco.mj_forward(m,d)
                x,y,p,_=inspect(d)
                if x>sp:sp=x;worst=p;at=i
                sup=max(sup,y)
        passed=sp<=1e-6 and sup<=.15
        row=dict(rf_axial_motor14_delta_ticks=delta,passed_extreme_offsets=passed,
                 max_self_mm=sp,max_support_mm=sup,worst_self_pair=worst,worst_index=at)
        if passed:
            name=f'rf14_{delta:+d}.npz';np.savez_compressed(folder/name,start=a,goals=goals,phases=phases,sample_period_s=.075)
            row['candidate']=name
        cases.append(row);print(json.dumps(row),flush=True)
    result=dict(status='recorded_pitch_small_ring_side_clearance_search',cases=cases,
                original_loaded_path_sha256=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest(),
                all_offsets_or_loaded_physics_verified=False,hardware_execution_allowed=False,devices_opened=False)
    (folder/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':main()
