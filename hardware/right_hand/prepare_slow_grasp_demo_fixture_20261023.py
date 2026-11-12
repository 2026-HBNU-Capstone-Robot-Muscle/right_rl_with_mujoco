"""Build an offline fixture from actual READ ONLY angles, never motion approval."""
import hashlib
import json
from pathlib import Path
import numpy as np

from slow_grasp_demo_plan import SlowGraspDemoPlan,OBSERVATION_PHASE
from thumb_live_start_gate import write_request
from thumb_live_start_plan import ROOT


def main():
    source=ROOT/'cube_pose_readonly_20261023_171233_601943/result.json'
    data=json.loads(source.read_bytes());sample=data['pairs'][5]['before']['snapshot']
    assert data['status']=='readonly_cube_pose_pairs_saved'
    state=dict(position=np.array(sample['position_ticks']),torque=np.array(sample['torque']),
        error=np.array(sample['hardware_error']),watchdog=np.array(sample['watchdog']),
        voltage=np.rint(np.array(sample['voltage_v'])*10).astype(int),temperature=np.array(sample['temperature_c']))
    folder=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_fixture';folder.mkdir(exist_ok=True)
    request=write_request(folder,state,SlowGraspDemoPlan.PLAN_VERSION,SlowGraspDemoPlan.validate_off_state)
    record=json.loads(request.read_bytes());record.update(
        origin='offline_fixture_from_actual_readonly_encoder_pair_not_live_motion_authorization',
        source_capture_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_image='pair_05.png',
        required_operator_precondition='Cube removed before actual execution; online fresh reads + new native proof required.')
    request.write_text(json.dumps(record,indent=2),encoding='utf-8')
    plan=SlowGraspDemoPlan(request);rows=plan.plan(plan.source_position())
    print('REQUEST',request)
    print('PATH',len(rows),'samples; fixed hold',sum(p==OBSERVATION_PHASE for p,q in rows),
        '; nominal motion seconds',len(rows)*.05)
    print('DEFAULT CAMERA',plan.CAMERA_TYPE.__name__)


if __name__=='__main__':main()
