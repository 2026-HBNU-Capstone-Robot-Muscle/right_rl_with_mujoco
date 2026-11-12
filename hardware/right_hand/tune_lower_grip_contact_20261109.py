"""Search small legal DIP/MCP/CMC offsets of a free lower-body grip."""
from pathlib import Path
import json,time,hashlib,itertools
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,motor,angle,native
from research_probe_index_handoff_20261109 import ramp
from canonical_grip_dynamics_20261105 import run

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
BASE=ROOT/'right_hand_rotation_20261109/lower_index_refinement/height-6/end.npz'
PARENT=ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction_linux/checkpoint_1042.npz'
OUT=ROOT/'right_hand_rotation_20261109/lower_grip_contact_tuning'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path('source.mjb',assets={'source.mjb':SOURCE.read_bytes()})
    z=np.load(BASE,allow_pickle=False);d=restore(m,z);start=motor(d.ctrl);parent=restore(m,np.load(PARENT,allow_pickle=False));B=parent.xmat[m.body('cube').id].reshape(3,3).copy();lo,hi,_,_=native(m);cases=[]
    for index_dip,thumb_mcp,thumb_cmc in itertools.product((-12,0,12),(-16,0,16),(-12,0,12)):
        goal=start.copy();goal[4]+=index_dip;goal[1]+=thumb_mcp;goal[3]+=thumb_cmc
        name=f'i{index_dip:+}_tm{thumb_mcp:+}_tc{thumb_cmc:+}';folder=OUT/name;folder.mkdir(exist_ok=True)
        if np.any(goal<lo) or np.any(goal>hi):
            cases.append({'name':name,'native_goal_rejected':True,'goal':goal.tolist(),'coarse_passed':False});continue
        rows=np.vstack([ramp(start,goal),np.tile(goal,(134,1))]);np.savez_compressed(folder/'research_path.npz',start=start,goals=rows)
        dynamic=run(m,z,rows,1,folder/'end.npz',None,False,required_lower_contacts=('if_tip','th_tip'))
        end=restore(m,np.load(folder/'end.npz',allow_pickle=False));cumulative=angle(end.xmat[m.body('cube').id].reshape(3,3),B,z['up'])
        passed=bool(dynamic['whole_grip_task_passed'] and abs(cumulative)<5)
        row={'name':name,'offsets_ticks':{'ID5':index_dip,'ID2':thumb_mcp,'ID4':thumb_cmc},'native_goal_rejected':False,
             'goal':goal.tolist(),'dynamic':dynamic,'cumulative_yaw_from_probe_deg':cumulative,
             'coarse_passed':passed,'full_1ms_sweep_verified':False,'hardware_execution_allowed':False}
        cases.append(row);(folder/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
        report={'status':'small_legal_contact_offsets_after_lower_index_grip','source_model_sha256':sha(SOURCE),
                'source_state_sha256':sha(BASE),'cases':cases,'coarse_passed':sum(case['coarse_passed'] for case in cases),
                'devices_opened':False,'hardware_execution_allowed':False,'elapsed_seconds':time.perf_counter()-began}
        (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({'name':name,'safety':dynamic['safety_passed'],'yaw':cumulative,
                          'gap':dynamic['settled_tail']['minimum_upper_gap_sampled_75ms_mm'],
                          'contact':dynamic['settled_tail']['opposed_lower_body_contact_fraction'],'passed':passed}),flush=True)


if __name__=='__main__':main()
