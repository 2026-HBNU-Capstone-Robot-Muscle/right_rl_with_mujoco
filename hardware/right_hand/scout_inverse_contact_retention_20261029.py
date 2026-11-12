"""Preserve tip flexion while rotating; finite checkpoint searches only."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
import scout_inverse_regrasp_cycles_20261029 as replay
from propose_inverse_regrasp_ik_20261029 import interp
from loaded_quarter_reentry_20261025 import limits
from new_pair_grasp_plan_20261027 import OPEN_PATH

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/inverse_contact_retention'
CHECK=ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint'

def main():
    OUT.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CHECK/'checkpoint_model.mjb'))
    source=ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/candidate.npz';old=np.load(source)
    yaw=old['goals'][old['phases']=='stage_7_horizontal_yaw']
    grasp=old['goals'][old['phases']=='stage_6_grasp_hold'][-1].copy()
    opened=np.load(OPEN_PATH)['open_goal'].astype(int);lo,hi=limits();reports=[]
    specs=[(250,-128,0),(350,-128,64),(435,-256,125),(250,-256,64),(435,-128,0),(350,-256,125)]
    for root_delta,thumb_delta,pip_delta in specs:
        name=f'root{root_delta}_thumb{thumb_delta}_pip{pip_delta}';folder=OUT/name;folder.mkdir(exist_ok=True)
        previous=grasp.copy();rows=[];phases=[]
        def move(phase,target,hold=0):
            nonlocal previous
            block=interp(previous,target);rows.extend(block);phases.extend([phase]*len(block))
            rows.extend([target.copy() for _ in range(hold)]);phases.extend([phase]*hold);previous=target.copy()
        q=grasp.copy();q[13]-=128;move('ring_clear',q)
        for t,q0 in enumerate(yaw,1):
            blend=t/len(yaw);q=q0.copy();q[13]-=128
            q[11]+=round(root_delta*blend);q[0]+=round(thumb_delta*blend);q[10]+=round(pip_delta*blend)
            move('turn',q)
        move('hold',previous,40)
        held_open=opened.copy();held_open[13]-=128;move('open',held_open);move('restore_ring',opened,20)
        rows=np.asarray(rows);phases=np.asarray(phases);values=np.vstack([grasp,rows])
        valid=bool(np.all(values>=lo) and np.all(values<=hi) and np.max(abs(np.diff(values,axis=0)))<=2)
        row=dict(name=name,native_and_step_bounds=valid,checkpoint='grasp_hold',full_prefix_verified=False,devices_opened=False,hardware_execution_allowed=False)
        if valid:
            result=replay.evaluate(m,rows,phases,checkpoint='grasp_hold_checkpoint.npz');result.pop('approximate_combined_yaw_deg',None)
            row.update(result)
            frontmask=np.isin(old['phases'],['stage_5_loaded_grasp','stage_6_grasp_hold'])
            goals=np.vstack([old['goals'][frontmask],rows])
            phases_out=np.asarray(['stage_8_horizontal_hold' if p=='hold' else 'stage_9_loaded_open' if p in ('open','restore_ring') else 'stage_7_horizontal_yaw' for p in phases])
            z=folder/'candidate.npz';np.savez_compressed(z,start=old['start'],goals=goals,phases=np.r_[old['phases'][frontmask],phases_out],
                empty_preposition=old['empty_preposition'],empty_gap_open=old['empty_gap_open'])
            row.update(candidate=z.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(z.read_bytes()).hexdigest(),
                loaded_goals=len(goals),nominal_motion_with_previous_recovery_s=(len(goals)+1327)*.075)
        reports.append(row);(folder/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
        (OUT/'review.json').write_text(json.dumps(dict(cases=reports,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in row.items() if k!='phases'}),flush=True)
if __name__=='__main__':main()
