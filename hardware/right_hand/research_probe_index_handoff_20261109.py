"""Test the prior IF/TH body-grip endpoint from the new natural-grasp state."""
from pathlib import Path
import sys,json,time,hashlib
ROOT=Path(__file__).resolve().parent
if sys.platform=='win32':sys.path.insert(0,str(ROOT/'.runtime/sim-win'))
import numpy as np
import mujoco
from research_rotation_body_hold_20261104 import motor
from canonical_grip_dynamics_20261105 import run

SOURCE=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
CP=ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction_linux'
TARGET=ROOT/'right_hand_rotation_20261105/canonical_body_grip/continuous_handoff/end_state.npz'
OUT=ROOT/'right_hand_rotation_20261109/index_thumb_transfer'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def ramp(a,b):
    n=max(1,int(np.ceil(max(abs(b-a))/2)))
    return np.rint(a+(b-a)*np.arange(1,n+1)[:,None]/n).astype(int)


def main():
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True)
    reconstructed=json.loads((CP/'review.json').read_bytes())
    if not reconstructed['same_original_path_reconstructed']:raise RuntimeError('Exact original Linux checkpoint was not reconstructed')
    model=mujoco.MjModel.from_binary_path('source.mjb',assets={'source.mjb':SOURCE.read_bytes()})
    target=np.load(TARGET,allow_pickle=False);goal=motor(target['ctrl']);cases=[]
    for checkpoint in (927,1042):
        z=np.load(CP/f'checkpoint_{checkpoint}.npz',allow_pickle=False);start=motor(z['ctrl'])
        center_difference=float(np.linalg.norm(target['qpos'][-7:-4]-z['qpos'][-7:-4])*1000)
        for order in ('together','index_first','index_thumb_first'):
            joints={'index':np.arange(4,8),'thumb':np.arange(4)};previous=start.copy();parts=[]
            phases=[]
            sequence=[] if order=='together' else ['index'] if order=='index_first' else ['index','thumb']
            for role in sequence:
                via=previous.copy();via[joints[role]]=goal[joints[role]];values=ramp(previous,via);parts.append(values);phases.extend([role+'_first']*len(values));previous=via
            values=ramp(previous,goal);parts.extend([values,np.tile(goal,(134,1))]);phases.extend(['body_grip']*len(values)+['fixed_hold']*134)
            rows=np.vstack(parts);folder=OUT/f'cp{checkpoint}_{order}';folder.mkdir(exist_ok=True)
            np.savez_compressed(folder/'research_path.npz',start=start,goals=rows,phases=phases)
            dynamic=run(model,z,rows,1,folder/'end.npz',folder/'trajectory.npz',False,required_lower_contacts=('if_tip','th_tip'))
            row={'checkpoint':checkpoint,'order':order,'proposal_source':'prior nominal IF/TH common-grip endpoint; not prior safety admission',
                 'source_target_center_difference_mm':center_difference,'goal':goal.tolist(),'goals':len(rows),
                 'dynamic':dynamic,'coarse_task_passed':dynamic['whole_grip_task_passed'],
                 'full_1ms_layer_sweep_certified':False,'physical_grip_verified':False}
            cases.append(row);(folder/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
            report={'status':'index_then_thumb_support_transfer_research','source_model_sha256':sha(SOURCE),
                    'source_endpoint_sha256':sha(TARGET),'cases':cases,
                    'safety_completed':sum(case['dynamic']['safety_passed'] for case in cases),
                    'coarse_task_passed':sum(case['coarse_task_passed'] for case in cases),
                    'hardware_execution_allowed':False,'devices_opened':False,'elapsed_seconds':time.perf_counter()-began}
            (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
            print(json.dumps({'checkpoint':checkpoint,'order':order,'center_difference_mm':center_difference,'goals':len(rows),
                              'completed':dynamic['completed_goals'],'safety':dynamic['safety_passed'],
                              'task':dynamic['whole_grip_task_passed'],'stop':dynamic['stop'],
                              'hold':dynamic['settled_tail']}),flush=True)


if __name__=='__main__':main()
