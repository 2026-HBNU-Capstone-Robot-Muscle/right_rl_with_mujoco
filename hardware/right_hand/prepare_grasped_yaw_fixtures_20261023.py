"""Offline actual-start fixtures. Does not grant live execution approval."""
import hashlib
import json
from pathlib import Path
from grasped_yaw_plan import GraspedYawPlan
from thumb_live_start_gate import write_request

ROOT=Path(__file__).resolve().parent


def main():
    sources=(
        ('grasped_yaw_fixture','slow_grasp_demo_20261023_182105_573825/result.json','final_off_snapshots'),
        ('grasped_yaw_prior_fixture','right_hand_rotation_20261023/slow_grasp_demo_fixture/fresh_off_request.json','off_state'),
        ('grasped_yaw_open_fixture','slow_grasp_demo_20261023_180831_999579/fresh_off_request.json','off_state'))
    for label,relative,key in sources:
        source=ROOT/relative;blob=source.read_bytes();r=json.loads(blob)
        state=r[key][-1] if key=='final_off_snapshots' else r[key]
        folder=ROOT/'right_hand_rotation_20261023'/label;folder.mkdir(exist_ok=True)
        old_proof=folder/'native_screen.json'
        if old_proof.exists():
            old=old_proof.read_bytes();archive=folder/('native_screen_before_'+hashlib.sha256(old).hexdigest()[:16]+'.json')
            if not archive.exists():archive.write_bytes(old)
        request=write_request(folder,state,GraspedYawPlan.PLAN_VERSION,GraspedYawPlan.validate_off_state)
        record=json.loads(request.read_bytes());record.update(origin='offline_fixture_not_live_motion_authorization',
            input_source_sha256=hashlib.sha256(blob).hexdigest(),required_operator_precondition='Hand empty before preparation; cube placed only at PLACE CUBE.')
        request.write_text(json.dumps(record,indent=2),encoding='utf-8')
        plan=GraspedYawPlan(request);rows=plan.plan(plan.source_position())
        print(label,len(rows),'targets; nominal',len(rows)*.05,'seconds',flush=True)


if __name__=='__main__':main()
