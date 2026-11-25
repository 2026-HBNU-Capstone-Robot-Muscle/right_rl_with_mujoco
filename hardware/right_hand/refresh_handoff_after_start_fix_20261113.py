"""Update the local handoff with the completed fix, preserving remote boundaries."""
import json
import shutil
import zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'team_handoff_20261113'
validation=ROOT/'right_hand_rotation_20261113/initial_empty_scope/VALIDATION.json'
assert json.loads(validation.read_bytes())['fix_applied']
p=OUT/'START_HERE.md';text=p.read_text(encoding='utf-8')
text=text.replace('자료 기준: 2026-11-12.','자료 기준: 2026-11-13 시작 검사 수정 반영.')
text=text.replace('**초기 간섭 예외는 아직 코드 미반영.**','**초기 간섭 예외 적용 후 실물 확인이 남아 있다.**')
text=text.replace('현재까지는 분석만 완료했다. 해당 쌍·빈손 구간·초기값 이하·준비 뒤 겹침0 조건으로 구현하고 정적201/동적9조건과 출처 검사를 다시 통과시켜야 한다.',
    '2026-11-13 해당 쌍·빈손 구간·초기값 이하 조건으로 반영했고, OPEN 48목표 이내 분리·배치 유지 때 겹침0을 요구한다. 입력 교란을 포함한 초기 모델 깊이 상한은1.75mm다. 저장 자세3개에서 각각201정적/9동적 조건, 실행기18시험과 예외 처리7시험을 통과했다. 기존 모터·적재 경로·형상·힘 계산은 유지했다. 새 실물 확인은 다음 작업이다.')
text=text.replace('재개 우선순위: **초기 겹침의 제한적 처리 구현·검증 → 끝마디 우선 이동 관측',
    '재개 우선순위: **수정된 시작 검사로 실물 재개 → 끝마디 우선 이동 관측')
text=text.replace('## 2. 목표와 현재 실행 방식','이번 시작 검사 수정 파일은 `runtime_patch/`에 담았다. GitHub 기준본을 받은 PC에서는 이 폴더 **안의 내용**을 `hardware/right_hand/`에 경로를 유지해 복사한다. 원래 작업 PC의 파일은 이미 수정됐다.\n\n## 2. 목표와 현재 실행 방식')
p.write_text(text,encoding='utf-8')
p=OUT/'PROFESSOR_MEETING.md';text=p.read_text(encoding='utf-8')
text=text.replace('자료 기준 2026-11-12','자료 기준 2026-11-13 시작 검사 수정 반영')
text=text.replace('해당 쌍과 빈손 구간으로 한정하는 코드 반영·재검증이 다음 작업입니다.',
    '2026-11-13 해당 쌍과 빈손 구간으로 한정해 반영했고, 저장 자세3개의 정적201/동적9조건씩을 통과했습니다. 초기값보다 깊어지지 않고 OPEN 48목표 이내에 사라지는 조건이며, 새 실물 관측이 다음 작업입니다.')
text=text.replace('**다음 순서:** 초기 겹침 처리 →','**다음 순서:** 수정된 시작 검사로 실물 재개 →')
text=text.replace('기존 시작 겹침을 제한적으로 처리한 뒤 수정한 밀기 동작을 확인하고,','초기 겹침을 제한적으로 처리하는 수정과 모델 재검증을 마쳤으며, 이제 밀기 동작을 실물에서 확인하고,')
p.write_text(text,encoding='utf-8')
archive=ROOT/'right_hand_rotation_20261113/initial_empty_scope/INITIAL_EMPTY_FIX_20261113.zip'
with zipfile.ZipFile(archive) as z:
    for info in z.infolist():
        relative=Path(info.filename)
        target=(OUT/'runtime_patch'/relative).resolve();target.relative_to(OUT.resolve())
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(z.read(info))
shutil.copyfile(validation,OUT/'evidence/START_FIX_VALIDATION.json')
print('Local handoff updated; no GitHub upload.')
