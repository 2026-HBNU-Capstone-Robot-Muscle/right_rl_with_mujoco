"""Record the new paired observation; no device access."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_preparation_20261021'
SOURCE=ROOT/'pose_pair_readonly_20261021_183730_320810/result.json'
r=json.loads(SOURCE.read_bytes())
p=json.loads((OUT/'thumb_clearance_sensitivity.json').read_text())
assert p['source_sha256']==hashlib.sha256(SOURCE.read_bytes()).hexdigest()
assert r['status']=='photo_and_readings_saved' and r['pair_check']['maximum_drift_ticks']==0
old=json.loads((ROOT/'right_hand_root_success_20261020/source_result.json').read_bytes())
raw=p['start_ticks']
diff=[a-b for a,b in zip(raw,old['final_on_state']['position'])]
review=dict(source_timestamp=r['timestamp'],source_sha256=p['source_sha256'],
    photo_sha256=hashlib.sha256((SOURCE.parent/'paired_photo.png').read_bytes()).hexdigest(),
    photo_review='Thumb, middle finger and palm visible for qualitative comparison; some distal fingers are cropped. No repeat capture needed for this purpose. No metric calibration or hidden-contact claim.',
    photo_and_encoder_pair_accepted=True,endpoint_drift_ticks=0,
    change_since_previous_final_on_ticks=diff,
    physical_root_trial_contact='Unknown: operator did not see clearly.',
    hardware_execution_allowed=False,next_user_task=None,
    next_engineering_step='Review preparation including root clearance from the latest state, tracking error and native thumb range; do not treat static scans as dynamics validation.')
(OUT/'paired_pose_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-21 18:37 촬영: 새 사진·각도 확인 완료

- `pose_pair_readonly_20261021_183730_320810`: 16개 모터 촬영 전후 변화 0틱, 토크 OFF. 사진을 직접 열어 엄지·중지·손바닥 배치 확인. 일부 손끝이 프레임 밖이지만 이번 비교용 재촬영은 불필요. 3D 보정이나 숨은 접촉까지 검증한 것은 아니다.
- 새 각도로 엄지 후보 3개 × 주변 시작각 35개 재검사. 명목 자세 경로에서는 자기/받침 겹침 없음. -64틱 후보의 주요 검사 부위 간격 추정 최소 4.938mm.
- ±32틱 표본 일부에서는 받침과 최대 0.054mm 모델 겹침이 생겨, 시작각 전체 범위를 통과했다고 처리하지 않는다. 실물 충돌 깊이가 아니라 근사 모델의 정적 결과다.
- 현재 자세는 이전 ON 최종 각도와 차이가 있으므로 이전 경로 승인에 그대로 대입하지 않는다. 실제 현재 OFF 자세 자료 확보 단계는 완료.
- 다음 작업: 뿌리 여유 확보를 포함한 준비 경로, 추종 오차와 엄지 모델 범위 검토. 실물 실행 파일·모델 한계·보호 조건 변경 없음. 사용자에게 지금 추가 촬영/동작은 요청하지 않음.
- 증거: `right_hand_preparation_20261021/paired_pose_review.json`, `thumb_clearance_sensitivity.json`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    f=ROOT/name;text=f.read_text(encoding='utf-8')
    if note.splitlines()[0] not in text:f.write_text(note+text,encoding='utf-8')
f=ROOT/'right_hand_start_20261019/objective_and_schedule.json'
x=json.loads(f.read_text(encoding='utf-8'))
x['current_user_task']='None. Fresh paired photo/encoders accepted on 2026-10-21; no repeated capture requested.'
x['current_blocker']='Current OFF pose now recorded. Preparation dynamics, native thumb range and support-clearance sensitivity remain unresolved; physical root contact observation is unknown.'
x['latest_offline_review']='right_hand_preparation_20261021/thumb_clearance_sensitivity.json'
f.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('Review saved. Raw delta vs prior final ON:',diff)
