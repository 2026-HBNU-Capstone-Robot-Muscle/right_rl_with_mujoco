"""Legal root/tip offsets to level the free lower grip; no hardware handoff."""
from pathlib import Path
import json,itertools,time,hashlib
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,motor,angle,native
from canonical_grip_dynamics_20261105 import run
from research_probe_index_handoff_20261109 import ramp

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
BASE=ROOT/'right_hand_rotation_20261109/lower_grip_contact_tuning/i+0_tm+16_tc+12/end.npz'
PARENT=ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction_linux/checkpoint_1042.npz'
OUT=ROOT/'right_hand_rotation_20261109/level_lower_grip'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path('source.mjb',assets={'source.mjb':SOURCE.read_bytes()})
    z=np.load(BASE,allow_pickle=False);d=restore(m,z);start=motor(d.ctrl);parent=restore(m,np.load(PARENT,allow_pickle=False));B=parent.xmat[m.body('cube').id].reshape(3,3).copy();lo,hi,_,_=native(m);cases=[];winner=None
    # Raise/support at the index root and compensate with distal thumb posture.
    for root_delta,thumb_tip,thumb_cmc in itertools.product((-64,0,64),(-24,0,24),(-24,0,24)):
        goal=start.copy();goal[7]+=root_delta;goal[0]+=thumb_tip;goal[3]+=thumb_cmc
        name=f'root{root_delta:+}_tip{thumb_tip:+}_cmc{thumb_cmc:+}';folder=OUT/name;folder.mkdir(exist_ok=True)
        row={'name':name,'offsets_ticks':{'ID8':root_delta,'ID1':thumb_tip,'ID4':thumb_cmc},'goal':goal.tolist(),'native_goal_rejected':bool(np.any(goal<lo) or np.any(goal>hi)),'passed':False}
        if not row['native_goal_rejected']:
            rows=np.vstack([ramp(start,goal),np.tile(goal,(134,1))]);np.savez_compressed(folder/'research_path.npz',start=start,goals=rows)
            dynamic=run(m,z,rows,1,folder/'end.npz',None,False,required_lower_contacts=('if_tip','th_tip'))
            end=restore(m,np.load(folder/'end.npz',allow_pickle=False));yaw=angle(end.xmat[m.body('cube').id].reshape(3,3),B,z['up'])
            row.update(dynamic=dynamic,cumulative_yaw_from_probe_deg=yaw,coarse_passed=bool(dynamic['whole_grip_task_passed'] and abs(yaw)<5))
            if row['coarse_passed']:
                certified=run(m,z,rows,1,folder/'certified_end.npz',folder/'trajectory.npz',True,required_lower_contacts=('if_tip','th_tip'))
                row['full_1ms_certificate_replay']=certified;row['passed']=bool(certified['whole_grip_task_passed'])
            print(json.dumps({'name':name,'safety':dynamic['safety_passed'],'yaw':yaw,'tilt':dynamic['final_up_tilt_deg'],
                              'gap':dynamic['settled_tail']['minimum_upper_gap_sampled_75ms_mm'],
                              'contact':dynamic['settled_tail']['opposed_lower_body_contact_fraction'],'passed':row['passed']}),flush=True)
        cases.append(row);(folder/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
        if row['passed']:winner=name
        report={'status':'free_lower_grip_leveling_search','source_model_sha256':sha(SOURCE),'source_state_sha256':sha(BASE),
                'planned_cases':27,'evaluated_cases':len(cases),'cases':cases,'winner':winner,'hardware_execution_allowed':False,
                'devices_opened':False,'elapsed_seconds':time.perf_counter()-began}
        (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        if winner:break


if __name__=='__main__':main()
