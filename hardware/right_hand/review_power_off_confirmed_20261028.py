"""Record user-confirmed supply OFF and inspect saved image references only."""
import json,datetime as dt
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'right_hand_rotation_20261028/loaded_comm_stop_000157'
path=folder/'review.json';r=json.loads(path.read_bytes())
r.update(user_confirmed_motor_supply_off=True,user_confirmation_time=dt.datetime.now().astimezone().isoformat(),
    cube_removal_confirmed=False,pending_user_input=None,
    next_user_task='Keep motor supply OFF. Offline saved-video and connection diagnosis; no motion retry.')
path.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
ready=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v8_receive_resync/readiness.json'
item=json.loads(ready.read_bytes());item.update(status='user_power_off_confirmed_connection_diagnosis',
    pending_user_input=None,user_confirmed_motor_supply_off=True,next_user_command_ready=False)
ready.write_text(json.dumps(item,indent=2)+'\n',encoding='utf-8')
marker='<!-- END LOADED COMM LOSS POWER OFF 20261028 -->'
header='''2026-10-28 현재 상태: 00:01 실물 시험은 준비와 큐브 배치까지 통과한 뒤 파지22샘플에서ID9 응답이 연속 누락돼 중단됐다. 짧은 수신 오류3회는 복구했으나 마지막 오류는 재시도3회에도 회복되지 않았다. 토크OFF 명령은 두 번 송신됐지만 ID9 응답이 없어 OFF를 검증하지 못했다. 이후 사용자가 모터 전원을 껐다고 확인했다. 현재 전원을 끈 채로 유지하고, 동작 재시도는 보류한다. 통신 장애 원인과 큐브 제거 여부는 아직 확인 중이다. 현재 근거: `right_hand_rotation_20261028/loaded_comm_stop_000157/review.json`.

'''+marker+'\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if marker not in old:p.write_text(header+old,encoding='utf-8')
source=json.loads((ROOT/'horizontal_quarter_trial_20261028_000157_905217/result.json').read_bytes())
photos={k:v for k,v in source.items() if 'photo' in k}
camera=source.get('camera_recording',{});photos['camera']=camera
print(json.dumps(photos,ensure_ascii=False))
