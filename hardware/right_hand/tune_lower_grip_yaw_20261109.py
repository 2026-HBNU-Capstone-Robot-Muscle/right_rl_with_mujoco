"""Tune small joint offsets of a free-replayed lower grip, never pinned."""
from pathlib import Path
import json,time,hashlib
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,motor,angle,native
from research_probe_index_handoff_20261109 import ramp
from canonical_grip_dynamics_20261105 import run

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
BASE=ROOT/'right_hand_rotation_20261109/lower_index_refinement/height-6/end.npz'
PARENT=ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction_linux/checkpoint_1042.npz'
OUT=ROOT/'right_hand_rotation_20261109/lower_grip_yaw_tuning'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path('source.mjb',assets={'source.mjb':SOURCE.read_bytes()})
    z=np.load(BASE,allow_pickle=False);d=restore(m,z);start=motor(d.ctrl);parent=restore(m,np.load(PARENT,allow_pickle=False));B=parent.xmat[m.body('cube').id].reshape(3,3).copy();cases=[]
    lo,hi,_,_=native(m)
    for index_rot in (-24,-12,0,12,24):
      for thumb_rot in (-12,0,12):
        goal=start.copy();goal[5]+=index_rot;goal[2]+=thumb_rot
        folder=OUT/f'index{index_rot:+}_thumb{thumb_rot:+}';folder.mkdir(exist_ok=True)
        if np.any(goal<lo) or np.any(goal>hi):
            row={'index_rotation_delta_ticks':index_rot,'thumb_rotation_delta_ticks':thumb_rot,
                 'native_goal_rejected':True,'goal':goal.tolist(),'coarse_passed':False,'dynamic_not_executed':True}
            cases.append(row);(folder/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
            continue
        cached=folder/'review.json'
        if cached.exists():
            row=json.loads(cached.read_bytes())
            if row.get('goal')==goal.tolist() and 'dynamic' in row:
                cases.append(row);continue
        rows=np.vstack([ramp(start,goal),np.tile(goal,(134,1))]);np.savez_compressed(folder/'research_path.npz',start=start,goals=rows)
        dynamic=run(m,z,rows,1,folder/'end.npz',None,False,required_lower_contacts=('if_tip','th_tip'))
        end=restore(m,np.load(folder/'end.npz',allow_pickle=False));cumulative=angle(end.xmat[m.body('cube').id].reshape(3,3),B,z['up'])
        passed=bool(dynamic['whole_grip_task_passed'] and abs(cumulative)<5)
        row={'index_rotation_delta_ticks':index_rot,'thumb_rotation_delta_ticks':thumb_rot,'goal':goal.tolist(),
             'dynamic':dynamic,'cumulative_yaw_from_probe_deg':cumulative,'coarse_passed':passed,
             'static_target_point_fit_was_not_evidence_of_this_free_contact_pose':True,
             'hardware_execution_allowed':False}
        cases.append(row);(folder/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
        report={'status':'free_cube_small_offset_yaw_tuning_after_low_index_contact',
                'source_model_sha256':sha(SOURCE),'source_state_sha256':sha(BASE),'cases':cases,
                'coarse_passed':sum(item['coarse_passed'] for item in cases),'devices_opened':False,
                'hardware_execution_allowed':False,'elapsed_seconds':time.perf_counter()-began}
        (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({'index_rot':index_rot,'thumb_rot':thumb_rot,'safety':dynamic['safety_passed'],
                          'yaw':cumulative,'gap':dynamic['settled_tail']['minimum_upper_gap_sampled_75ms_mm'],
                          'lower_contact':dynamic['settled_tail']['opposed_lower_body_contact_fraction'],'passed':passed}),flush=True)
    report={'status':'free_cube_offset_comparison_complete_with_native_rejections',
            'source_model_sha256':sha(SOURCE),'source_state_sha256':sha(BASE),'cases':cases,
            'dynamic_cases':sum('dynamic' in row for row in cases),'native_rejections':sum(row.get('native_goal_rejected',False) for row in cases),
            'coarse_passed':sum(row['coarse_passed'] for row in cases),'devices_opened':False,'hardware_execution_allowed':False}
    (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({key:value for key,value in report.items() if key!='cases'}),flush=True)


if __name__=='__main__':main()
