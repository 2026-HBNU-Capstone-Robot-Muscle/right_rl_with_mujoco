"""Hand-only trajectory review. The unused free cube stays far from the hand."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import inspect_for,native
from hand_motor_mapping import ticks_to_sim

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--front',type=int,choices=(-15,-10,-5),required=True);ap.add_argument('--candidate',type=Path,default=ROOT/'recentered_free/i-77_m-16_t-56/from_reference/candidate.npz');ap.add_argument('--matrix-tag',default='full_placement_v2');ap.add_argument('--output-tag',default='empty_path');a=ap.parse_args();source=a.candidate;path=np.load(source);out=ROOT/f'{a.output_tag}/front{a.front}';out.mkdir(parents=True,exist_ok=True);modelpath=ROOT/f'{a.matrix_tag}/front{a.front}/g1_f0.3/model.mjb';cases=[]
    review=json.loads((modelpath.parent.parent/'review.json').read_bytes())
    if review['candidate_sha256']!=hashlib.sha256(source.read_bytes()).hexdigest():raise ValueError('candidate/model source mismatch')
    for scale in (.5,1.,1.5):
        began=time.perf_counter();m=mujoco.MjModel.from_binary_path(str(modelpath));m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;hand,support,check=inspect_for(m);lo,hi,nlo,nhi=native(m);d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(path['start']);d.ctrl[:]=d.qpos[:16];d.qpos[-7:-4]=[2.,2.,2.];d.qvel[:]=0;mujoco.mj_forward(m,d);worst=dict(self_mm=0.,support_mm=0.,tracking_rad=0.,range_rad=0.);stop=None;completed=0
        def probe():
            su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
            return worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6
        for _ in range(2000):
            mujoco.mj_step(m,d)
            if probe():stop='initial hand settling bound';break
        if stop is None:
            for i,goal in enumerate(path['goals']):
                d.ctrl[:]=ticks_to_sim(goal)
                for _ in range(75):
                    mujoco.mj_step(m,d)
                    if probe():stop='unchanged hand contact/range/tracking bound';break
                completed=i+1
                if stop:break
        row=dict(book_front_mm=a.front,gain_scale=scale,passed=stop is None,stop=stop,completed_goals=completed,total_goals=len(path['goals']),worst=worst,elapsed_seconds=time.perf_counter()-began);cases.append(row);report=dict(status='hand_only_canonical_grip_path_review',candidate=str(source.relative_to(R)),candidate_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),cases=cases,all_passed=all(c['passed'] for c in cases),cube_not_loaded_and_started_2m_away=True,all_hand_geometry_and_native_bounds_unchanged=True,current_actual_OFF_recovery_not_reviewed=True,hardware_execution_allowed=False,devices_opened=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(row),flush=True)
if __name__=='__main__':main()
