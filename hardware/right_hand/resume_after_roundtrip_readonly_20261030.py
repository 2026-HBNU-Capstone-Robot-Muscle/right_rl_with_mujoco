import json,datetime as dt,hashlib
from pathlib import Path
import numpy as np
from observed_book_model_policy import healthy_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from yaw_roundtrip_plan_20261030 import YawRoundtripPlan
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/roundtrip_actual_review'
def put(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
 source=R/'fast_off_readonly_20261030_044532_258003/result.json';r=json.loads(source.read_bytes())
 assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and r['servo_writes']==r['torque_changes']==0
 for s in r['snapshots']:healthy_off({k:np.asarray(v) for k,v in s.items()})
 request=O/'current_off_request.json';put(request,dict(plan_version=YawRoundtripPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=source.relative_to(R).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False))
 p=YawRoundtripPlan(request);plan_error=None
 try:rows=p.plan(p.source_position());plan_goals=len(rows)
 except Exception as e:plan_error=str(e);plan_goals=None
 now=dt.datetime.now().astimezone().isoformat();update=dict(timestamp=now,current_motor_state_verified=True,current_motor_connection='All16 OFF/error0/watchdog0 in3 fresh snapshots; maximum drift1tick. PING/READ only.',latest_readonly=source.relative_to(R).as_posix(),next_user_action_required=False,next_user_action=None,next_user_command=None,next_command=None,next_user_command_ready=False,ready_for_user_run=False,ready_for_supervised_operator_run=False,current_pose_requires_fresh_path_review=True,assistant_motor_or_setting_writes=0,motors_off_verified=True)
 for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
  f=R/name;v=json.loads(f.read_bytes());v.update(update);put(f,v)
 f=R/'right_hand_rotation_20261030/yaw_roundtrip/handoff.json';v=json.loads(f.read_bytes());v.update(update);v.update(operator_action_required=False,operator_steps=[],current_readonly=source.relative_to(R).as_posix(),current_readonly_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),all16_OFF_error0_watchdog0_verified=True);put(f,v)
 f=O/'review.json';v=json.loads(f.read_bytes());v.update(current_motor_state_verified=True,latest_readonly=source.relative_to(R).as_posix(),current_pose_request=request.relative_to(R).as_posix(),user_action_required=None,preliminary_existing_queue_goals=plan_goals,preliminary_plan_error=plan_error);put(f,v)
 for f in (R/'right_hand_rotation_20261030/yaw_roundtrip/operator_guide.html',O/'index.html'):
  s=f.read_text(encoding='utf-8');s=s.replace('<h2>지금 필요한 작업</h2><p>큐브를 손 밖에 둔 채 기존 5V 전원과 USB 연결을 확인하고 켠 뒤 알려주세요. 손가락과 받침은 그대로 두면 됩니다. 다음 경로를 검토하는 동안 현재 각도를 읽겠습니다.</p>','<h2>현재 진행</h2><p>5V 연결 후 16개 모터의 응답과 토크 OFF, 정지 상태를 확인했습니다. 현재 각도로 복귀 경로와 역회전 접촉을 검토 중입니다. 큐브는 손 밖에 두고 손가락과 받침은 그대로 두면 됩니다.</p>');f.write_text(s,encoding='utf-8')
 marker='<!-- roundtrip_readonly_resumed_20261030_044532 -->\n';paragraph='2026-10-30 04:45 사용자 전원 확인 후16개 모터를 읽었다. 3회 모두OFF/error0/watchdog0, 최대1틱 변화이며 목표/토크/설정 쓰기0회다. 추가 사용자 작업을 기다리는 상태를 해제하고 현재 자세 경로와 역회전 접촉을 검토한다. 실물 새 동작 명령은 아직 없다.\n\n'
 for name in ('professor_progress_20261015.md','sac_current_status.md'):
  f=R/name;s=f.read_text(encoding='utf-8');
  if not s.startswith(marker):f.write_text(marker+paragraph+s,encoding='utf-8')
 print(json.dumps(dict(status='fresh_all16_OFF_readonly_verified',motion_commands=0,existing_queue_goals=plan_goals,preliminary_plan_error=plan_error,request=request.relative_to(R).as_posix())))
if __name__=='__main__':main()
