"""Change grasp contact timing, then replay the same yaw and common grip.

The original recorded source stays untouched. These separate candidates keep
the original goal endpoints, native bounds,2tick interpolation and1ms dynamics.
Changing who contacts first may reproduce the extra grasp-phase yaw seen in
the photographs. No cube state is reset after initial placement.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import native,restore,angle
from canonical_grip_dynamics_20261105 import run
from canonical_grip_task_review_20261105 import task_reasons
from hand_motor_mapping import ticks_to_sim

def retime(old,thumb_power,middle_power):
    ids=np.flatnonzero(old['phases']=='stage_5_loaded_grasp');first,last=int(ids[0]),int(ids[-1]);assert np.array_equal(ids,np.arange(first,last+1));n=len(ids);begin=old['start'] if first==0 else old['goals'][first-1];values=np.vstack([begin,old['goals'][ids]]);f=np.arange(1,n+1)/n;warped=old['goals'][ids].copy()
    for group,power in ((range(4),thumb_power),(range(8,12),middle_power)):
        for j in group:warped[:,j]=np.rint(np.interp(f**power*n,np.arange(n+1),values[:,j])).astype(int)
    assert np.array_equal(warped[-1],old['goals'][last])
    rows=[];previous=begin.copy()
    for target in warped:
        count=max(1,int(np.ceil(max(abs(target-previous))/2)));rows.extend(np.rint(previous+(target-previous)*i/count).astype(int) for i in range(1,count+1));previous=target
    return np.vstack([old['goals'][:first],rows,old['goals'][last+1:]])

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--group',choices=('thumb','middle','separated'),required=True);a=ap.parse_args()
    configs={'thumb':[(.5,1.),(.65,1.),(.8,1.)],'middle':[(1.,.5),(1.,.65),(1.,.8)],'separated':[(.5,2.),(2.,.5),(.75,1.25)]}[a.group]
    out=ROOT/f'grasp_timing_search/{a.group}';out.mkdir(parents=True,exist_ok=True);folder=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3';m=mujoco.MjModel.from_binary_path(str(folder/'model.mjb'));z=np.load(folder/'initial_state.npz');oldsource=R/'right_hand_rotation_20261104/forward_hold/candidate.npz';old=np.load(oldsource);canonical=np.load(ROOT/'tip_squeeze/id1/plus12/candidate.npz');suffix=canonical['goals'][len(old['goals']):];lo,hi,_,_=native(m);cases=[]
    for tp,mp in configs:
        began=time.perf_counter();variant=retime(old,tp,mp);goals=np.vstack([variant,suffix]);caseout=out/f'thumb{tp:g}_middle{mp:g}';caseout.mkdir(exist_ok=True)
        if np.any(goals<lo) or np.any(goals>hi) or np.max(abs(np.diff(np.vstack([old['start'],goals]),axis=0)))>2:raise ValueError('native or step bound')
        np.savez_compressed(caseout/'candidate.npz',start=old['start'],goals=goals)
        d=run(m,z,goals,len(variant),caseout/'end_state.npz',caseout/'trajectory.npz',True);why=task_reasons(d);prefix_yaw=d['forward_yaw_from_placement_deg'];forward_up=None
        if d['completed_goals']>=len(variant):
            trajectory=np.load(caseout/'trajectory.npz');indices=np.flatnonzero(trajectory['samples']==len(variant));assert len(indices)==1;pose=mujoco.MjData(m);pose.qpos[:]=trajectory['qpos'][indices[0]];mujoco.mj_forward(m,pose);B=pose.xmat[m.body('cube').id].reshape(3,3);forward_up=float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1))))
        full90=bool(not why and prefix_yaw is not None and abs(prefix_yaw-90)<5 and abs(d['final_yaw_from_placement_deg']-90)<5)
        row=dict(thumb_progress_power=tp,middle_progress_power=mp,variant_goals=len(variant),source_goal_endpoints_preserved=True,dynamic=d,grip_task_reasons=why,forward_up_tilt_deg=forward_up,whole90_and_common_grip_filter_passed=full90,elapsed_seconds=time.perf_counter()-began);cases.append(row)
        report=dict(status='changed_grasp_contact_timing_then_saved_yaw_then_common_grip',group=a.group,source_path=str(oldsource.relative_to(R)),source_sha256=hashlib.sha256(oldsource.read_bytes()).hexdigest(),cases=cases,book_front_mm=-10.,gain_scale=1.,friction=.3,all_cube_state_resets_only_before_placement=True,original_goal_source_file_not_modified=True,new_candidate_prefix_is_experimental_and_not_the_original_proven_prefix=True,hardware_execution_allowed=False,devices_opened=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(thumb=tp,middle=mp,safety=d['safety_passed'],completed=d['completed_goals'],total=d['total_goals'],forward_yaw=prefix_yaw,forward_up=forward_up,final_yaw=d['final_yaw_from_placement_deg'],grip_reasons=why,target90=full90,elapsed_seconds=row['elapsed_seconds'])),flush=True)
if __name__=='__main__':main()
