"""Preserve unique cloud scaffolding and finalize the complete local delivery."""
import hashlib
import json
import zipfile
from pathlib import Path
from create_full_github_upload_20261112 import PATTERNS
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'full_project_delivery_20261112';ARCHIVE=OUT/'RIGHT_HAND_FULL_GITHUB_UPLOAD_20261112.zip'
snapshot=ROOT/'cloud_handoff_20261106/upload_repo'
rows=json.loads((OUT/'snapshot_only_sources.json').read_bytes());extra=[]
with zipfile.ZipFile(ARCHIVE,'a',compression=zipfile.ZIP_DEFLATED,compresslevel=1,allowZip64=True) as z:
    existing=set(z.namelist())
    for row in rows:
        source=snapshot/row['path'];data=source.read_bytes()
        if any(p.search(data) for p in PATTERNS):raise RuntimeError('Credential-like text in snapshot support; no values exported')
        name='hardware/right_hand/cloud_handoff_20261106/snapshot_support/'+row['path']
        if name in existing:raise RuntimeError('Do not create duplicate ZIP entries')
        z.writestr(name,data);extra.append(dict(path=name,source_snapshot_path=row['path'],bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))
    note='''# 이전 선별 스냅샷의 고유 환경 안내\n\n이 폴더는 중복된 전체 업로드 체크아웃을 빼면서, 그 안에만 있던 환경 설정·의존성·안내·선별 범위 기록15개를 원본 바이트로 보존한 것이다. 과거 선별 스냅샷 기준의 기록이므로 현재 전체 작업 폴더를 인증하는 검증 결과로 사용하지 않는다. 현재 범위는 저장소 루트의 `RIGHT_HAND_FULL_MANIFEST.json`과 `RIGHT_HAND_SUPPLEMENT_MANIFEST.json`, 최신 진행은 본 작업 폴더의 `cloud_handoff_20261106/CLOUD_START_PROMPT.md`를 따른다. GitHub 원격 브랜치는 삭제된 상태이며 이 전달 중 자동 업로드는 하지 않았다.\n'''
    z.writestr('hardware/right_hand/cloud_handoff_20261106/snapshot_support/ARCHIVE_NOTE.md',note)
    readme='''# 오른손 실물 후속 개발 작업 전체\n\n이 폴더는 원본 `right_rl_with_mujoco`에서 시작해 실물 문제를 수정·확장해 온 개발 작업 전체다. 코드·원본 정책과 SAC 추가 학습 자료·모델·경로 후보·실물/시뮬레이션 검증·분석 사진·진행 및 인계 문서를 포함한다. 설치 환경·인증정보·대량 원시 촬영 폴더와 영상, 중복 체크아웃은 제외했다.\n\n- 최신 상태: `cloud_handoff_20261106/CLOUD_START_PROMPT.md`와 `professor_progress_20261015.md`의 맨 위 항목\n- 현재 관찰 시험: `leap_cycle_tip_first_transfer.cmd`\n- 시험 안내: `right_hand_rotation_20261112/tip_first_trial_v2/operator_guide.md`\n- 전체 원본 경로/해시: 저장소 루트의 `RIGHT_HAND_FULL_MANIFEST.json`\n- 과거 스냅샷의 고유 환경 안내: `cloud_handoff_20261106/snapshot_support/`\n\n현재 시험 범위는 주황 옆면 뒤쪽 접촉을 목표로 끝마디 → 다음 마디 순서로 미는 관찰이다. 최종 세 손가락 파지와 다음 회전은 아직 미완료다. 실제 실물은 저장 경로 재생이며, 새 SAC 학습·증류 또는 실시간 정책 구동이 완료됐다고 표현하지 않는다. 모델 검사 통과와 실제 성공을 구분한다.\n\n현재 원래 PC의 Python 설치 환경 `.runtime`은 배포하지 않았다. 다른 PC의 포트/USB/모터 설정을 그대로 적용하지 않는다. 물리 실행은 사용자가 시작하며 기존450mA·PID·2틱/75ms와 모든 중단 기준을 유지한다.\n\nGitHub에 올릴 때는 원본 저장소의 `feature/hardware` 브랜치에 이 `hardware` 폴더와 전체 목록 문서를 넣는다. `apps/monitoring_ui`는 수정하지 않는다. 이 파일 전달 과정에서는 GitHub에 업로드하지 않았다.\n'''
    readme_path='hardware/right_hand/README.md'
    if readme_path not in existing:z.writestr(readme_path,readme)
    z.writestr('RIGHT_HAND_SUPPLEMENT_MANIFEST.json',json.dumps(dict(snapshot_support_files=extra,reference_only=True,credentials_included=False),indent=2,ensure_ascii=False))
with zipfile.ZipFile(ARCHIVE) as z:
    if len(z.namelist())!=len(set(z.namelist())):raise RuntimeError('Duplicate ZIP member')
    if z.testzip() is not None:raise RuntimeError('ZIP integrity check failed')
    base=json.loads(z.read('RIGHT_HAND_FULL_MANIFEST.json'))
h=hashlib.sha256()
with ARCHIVE.open('rb') as stream:
    for chunk in iter(lambda:stream.read(4*1024*1024),b''):h.update(chunk)
result=json.loads((OUT/'DELIVERY.json').read_bytes())
result.update(zip_bytes=ARCHIVE.stat().st_size,sha256=h.hexdigest(),supplemental_unique_snapshot_sources=len(extra),
              total_project_source_files=base['total_source_files']+len(extra),unique_zip_member_names_verified=True,zip_crc_verified=True)
(OUT/'DELIVERY.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
(OUT/'RIGHT_HAND_FULL_GITHUB_UPLOAD_20261112.sha256').write_text(h.hexdigest()+'  '+ARCHIVE.name+'\n',encoding='ascii')
print(json.dumps(result,ensure_ascii=False))
