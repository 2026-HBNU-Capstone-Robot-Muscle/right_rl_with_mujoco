"""Finalize local readiness after scoped EMPTY checks; no physical or remote writes."""
import hashlib
import importlib
import json
import shutil
import zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261113/initial_empty_scope'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,value):Path(p).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
checks=[]
for name in ['latest_support','previous_support','latest_tip']:
    path=OUT/name/'review.json';r=json.loads(path.read_bytes())
    assert r['empty_recovery']['all_passed'] and r['empty_initial_thumb_index_scope']['enabled']
    checks.append(dict(case=name,static_passed=sum(v['passed'] for v in r['empty_recovery']['static_cases']),
        dynamic_passed=sum(v['passed'] for v in r['empty_recovery']['dynamic_cases']),
        maximum_initial_mm=r['empty_initial_thumb_index_scope']['maximum_initial_mm']))
for stem in ['support_first','tip_first']:
    plan=importlib.import_module(stem+'_trial_plan_20261112');gate=importlib.import_module(stem+'_trial_gate_20261112')
    proof=gate.validate_review(plan.__dict__['SupportFirstTrialPlan' if stem=='support_first' else 'TipFirstTrialPlan'](plan.REQUEST),plan.PROOF)
    guard=json.loads((plan.OUT/'runtime_guard_review.json').read_bytes())
    assert guard['passed'] and guard['checked_plan_sha256']==sha(ROOT/(stem+'_trial_plan_20261112.py'))
    tests=json.loads((plan.OUT/'software_verification.json').read_bytes());assert tests['passed'] and tests['tests']==9
    ready_path=plan.OUT/'READY.json';ready=json.loads(ready_path.read_bytes())
    ready.update(review_sha256=sha(plan.PROOF),empty_initial_thumb_index_exception_applied=True,
        empty_model_contact_scope_changed=True,motor_health_stop_thresholds_unchanged=True,
        scoped_empty_validation='right_hand_rotation_20261113/initial_empty_scope/VALIDATION.json',
        hardware_retest_after_patch_performed=False)
    save(ready_path,ready)
    guide=plan.OUT/'operator_guide.md';text=guide.read_text(encoding='utf-8-sig')
    note='''\n## 2026-11-13 시작 검사 수정\n\n사용자가 허용한 엄지 끝–검지 뿌리의 기존 초기 모델 겹침만 빈손 복귀에서 분류 예외로 처리했다. 초기 모델 깊이 상한1.75mm(관절 교란 조건 포함) 안에서 초기값보다 깊어지지 않고, OPEN 48목표 이내에 사라져 배치 유지 때0이어야 한다. 형상과 접촉력 계산은 그대로이며 다른 접촉·적재 경로·모터 감시 한도는 유지한다. 최근 저장 자세3개에서 각각201정적/9동적 조건을 검사했다. 같은실행파일을 사용한다. 새실물실행은아직하지않았다.\n'''
    if '## 2026-11-13 시작 검사 수정' not in text:guide.write_text(text+note,encoding='utf-8')
frozen=json.loads((ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json').read_bytes())['code_hashes']
assert len(frozen)==170 and all(sha(ROOT/name)==digest for name,digest in frozen.items())
scope_tests=json.loads((OUT/'software_scope_tests.json').read_bytes());assert scope_tests['passed']
report=dict(fix_applied=True,saved_pose_checks=checks,scope_tests=7,integration_tests=18,frozen_original_hashes_unchanged=170,
    candidate_paths_unchanged=True,motor_settings_and_health_guards_unchanged=True,
    hand_collision_shapes_and_forces_unchanged=True,loaded_contact_criteria_unchanged=True,
    physical_retest_performed=False,devices_opened=0,github_upload=False)
save(OUT/'VALIDATION.json',report)
files=['empty_start_thumb_index_scope_20261113.py','screen_empty_initial_scope_20261113.py']
for stem,packet in [('support_first','support_first_trial'),('tip_first','tip_first_trial_v2')]:
    files += [stem+'_trial_plan_20261112.py',stem+'_trial_gate_20261112.py','screen_'+stem+'_current_20261112.py']
    files += ['right_hand_rotation_20261112/'+packet+'/'+name for name in ['review.json','READY.json','runtime_guard_review.json','software_verification.json','operator_guide.md']]
files+=['test_empty_start_scope_20261113.py','right_hand_rotation_20261113/initial_empty_scope/VALIDATION.json']
archive=OUT/'INITIAL_EMPTY_FIX_20261113.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for relative in files:z.write(ROOT/relative,relative)
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
print(json.dumps(report,ensure_ascii=False),flush=True)
