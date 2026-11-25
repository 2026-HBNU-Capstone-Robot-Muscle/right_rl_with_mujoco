"""Package all accessible project development material for manual Git upload.

No remote calls, account login, motor commands or camera commands are performed.
Source bytes and original relative paths are retained below hardware/right_hand.
"""
import hashlib
import json
import re
import time
import zipfile
from datetime import datetime,timezone,timedelta
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'full_project_delivery_20261112'
ARCHIVE=OUT/'RIGHT_HAND_FULL_GITHUB_UPLOAD_20261112.zip'
PREFIX='hardware/right_hand/'
STORED={'.png','.jpg','.jpeg','.gif','.npz','.pdf','.gz','.bz2','.xz','.7z','.zst'}
TEXT={'.py','.json','.jsonl','.md','.txt','.yaml','.yml','.ini','.cfg','.toml','.sh','.cmd','.ps1','.xml','.html','.csv'}
PATTERNS=[re.compile(rb'gh[pousr]_[A-Za-z0-9]{30,255}'),re.compile(rb'github_pat_[A-Za-z0-9_]{30,255}'),
          re.compile(rb'AKIA[0-9A-Z]{16}'),re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----')]

def main():
    inventory=json.loads((OUT/'inventory.json').read_bytes());rows=inventory['files']
    if inventory['workspace_free_bytes']<inventory['total_bytes']:raise RuntimeError('Insufficient space for a conservative archive estimate')
    frozen=json.loads((ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json').read_bytes())['code_hashes']
    selected={r['path'] for r in rows}
    missing=[name for name in frozen if name not in selected]
    if missing:raise RuntimeError('Required frozen source was excluded: '+repr(missing))
    critical=('sac_real_start_100k','sac_real_start_residual_100k','sac_forward_support_reference_v1_20k',
              'right_hand_rotation_20261109/policy_audit','right_hand_rotation_20261112/tip_first_trial_v2')
    coverage={name:sum(r['path']==name or r['path'].startswith(name+'/') for r in rows) for name in critical if (ROOT/name).exists()}
    if any(count==0 for count in coverage.values()):raise RuntimeError('A requested policy/result directory is missing')
    began=time.monotonic();manifest=[];omitted=[];total=0;last_progress=began
    with zipfile.ZipFile(ARCHIVE,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=1,allowZip64=True) as z:
        for number,row in enumerate(rows,1):
            source=ROOT/row['path'];source.resolve().relative_to(ROOT);data=source.read_bytes()
            if source.suffix.lower() in TEXT and any(p.search(data) for p in PATTERNS):
                if row['path'] in frozen:raise RuntimeError('Credential-looking content in a protected source; delivery stopped')
                omitted.append(dict(path=row['path'],reason='credential_pattern_detected_no_value_exported'));continue
            digest=hashlib.sha256(data).hexdigest()
            if row['path'] in frozen and digest!=frozen[row['path']]:raise RuntimeError('Frozen source changed: '+row['path'])
            compression=zipfile.ZIP_STORED if source.suffix.lower() in STORED else zipfile.ZIP_DEFLATED
            z.writestr(PREFIX+row['path'],data,compress_type=compression,compresslevel=1)
            manifest.append(dict(path=row['path'],bytes=len(data),sha256=digest));total+=len(data)
            now=time.monotonic()
            if number%1000==0 or now-last_progress>25:
                print(json.dumps(dict(files_done=number,total_files=len(rows),source_MB=round(total/1e6),zip_MB=round(z.fp.tell()/1e6))),flush=True);last_progress=now
        attributes='* -text\n'
        ignore='''# Preserve installed environments and account files outside version control.\n.runtime/\n.venv/\nvenv/\n.analysis_packages/\n__pycache__/\n*.py[cod]\n.env\n.env.*\n.aws/\n.codex/\n.agents/\nhosts.yml\nauth.json\n**/camera/\n*.mp4\n*.avi\n*.mkv\n*.mov\ndeliverables_*/\nfull_project_delivery_*/\n'''
        ignore=ignore.replace('**/camera/\n','**/camera/*\n')
        for name in frozen:
            if '/camera/' in name:ignore+='!'+name+'\n'
        if '.gitattributes' not in selected:z.writestr(PREFIX+'.gitattributes',attributes)
        if '.gitignore' not in selected:z.writestr(PREFIX+'.gitignore',ignore)
        summary=dict(kind='complete_accessible_project_development_material_not_machine_backup',
            snapshot_time=datetime.now(timezone(timedelta(hours=9))).isoformat(),files=manifest,
            total_source_files=len(manifest),total_source_bytes=total,excluded_categories=inventory['excluded'],
            credentials_pattern_omissions=omitted,frozen_motion_bindings_verified=len(frozen),important_policy_result_coverage=coverage,
            directory_prefix=PREFIX,source_relative_paths_and_bytes_preserved=True,
            installed_environments_included=False,raw_camera_recording_directories_included=False,raw_videos_included=False,
            required_frozen_reference_exceptions=inventory.get('required_frozen_reference_exceptions',[]),
            duplicate_git_checkouts_and_delivery_archives_included=False,
            remote_upload_performed=False,full_robot_cycle_complete=False)
        z.writestr('RIGHT_HAND_FULL_MANIFEST.json',json.dumps(summary,indent=2,ensure_ascii=False))
        guide=f'''# 오른손 전체 개발 자료 — GitHub 수동 업로드용\n\n이 ZIP은 기존의 작은 경로 모듈 ZIP이나23MB 시험용 선별 ZIP을 대신하는 전체 개발 자료 묶음이다. 현재 작업 폴더에서 접근 가능한 코드·모델·SAC 학습/체크포인트·실물 및 오프라인 검증 결과·분석 사진·인계 자료를 포함한다. {len(manifest):,}파일, 원본 약{total/1e9:.2f}GB다.\n\n## GitHub에 넣는 위치\n\n압축을 풀면 `hardware/right_hand/` 안에 기존 `outputs`의 상대 경로가 보존돼 있다. 원본 팀 저장소 `2026-HBNU-Capstone-Robot-Muscle/right_rl_with_mujoco`의 로컬 체크아웃 루트에 `hardware` 폴더를 넣는다. 기존 `deploy`, `train`, `policy`, `vision`과 함께 있는 구조다. `feature/hardware` 브랜치에서 커밋·푸시한다. `apps/monitoring_ui`는 수정하지 않는다.\n\n모델 파일 일부는 약65MB라 GitHub 웹 업로드의25MiB 제한을 초과한다. 파일 수도 웹 업로드의 한 번100개 제한보다 많다. GitHub Desktop 또는 Git을 사용한다. ZIP 파일 자체를 저장소에 올리는 것이 아니라 압축을 푼 소스 폴더를 올린다. 큰 초기 업로드는 코드/모델과 과거 결과를 나눠 각각 커밋 후 푸시하면 관리하기 쉽다.\n\n공식 안내: https://docs.github.com/en/repositories/working-with-files/managing-files/adding-a-file-to-a-repository\n파일 크기 안내: https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github\n\n## 포함 범위와 보존\n\n- 코드와 현재 실물 실행기 및 안전 감시 코드\n- 원본 배포 정책과 접근 가능한 SAC 추가 학습·체크포인트·NumPy 내보내기\n- 날짜별 경로 후보, 모델, 초기/종료 상태, 실패를 포함한 검증 결과\n- 실물 로그와 분석 사진, 최신 인계·진행 기록\n- 원본170개 결합 검증 소스의 해시 일치\n\n설치된 Python/가상환경, 인증·계정 정보, 원시 영상, `camera` 원시 프레임 폴더, 중복 Git 체크아웃과 전달용 압축 파일은 제외했다. 정확한 목록과 SHA256은 `RIGHT_HAND_FULL_MANIFEST.json`에 있다. 전체 PC 백업이나 원시 기록 전체 백업은 아니다. 원래 로컬 파일은 변경하거나 삭제하지 않았다.\n\n## 현재 동작 상태\n\n현재 새 시험은 주황 옆면 뒤쪽 접촉을 목표로 끝마디 → 다음 마디 순서로 미는 관찰 시험이다. 최종 세 손가락 파지와 다음 회전은 미완료다. 모델 검사 통과를 실제 성공으로 표시하지 않았다. 새 SAC 학습이나 증류가 완료됐다는 주장도 하지 않는다.\n\nWindows 원래 실행 환경의 `.runtime`은 포함하지 않았다. 다른 PC의 포트·USB·장치 설정을 그대로 적용하지 않는다. 실물 실행은 사용자가 명령과 SPACE로 시작하며 보조자가 직접 구동한 것은 없다.\n\n이 ZIP을 만드는 동안 GitHub에는 업로드하지 않았다. 사용자가 다시 명시적으로 지시하기 전에는 자동 업로드하지 않는다.\n'''
        z.writestr('UPLOAD_README_KO.md',guide)
    print(json.dumps(dict(stage='crc_verification',zip_MB=round(ARCHIVE.stat().st_size/1e6),files=len(manifest))),flush=True)
    with zipfile.ZipFile(ARCHIVE) as z:
        bad=z.testzip()
        if bad is not None:raise RuntimeError('ZIP CRC failure: '+bad)
        archived=json.loads(z.read('RIGHT_HAND_FULL_MANIFEST.json'))
        if archived['total_source_files']!=len(manifest):raise RuntimeError('Archive inventory mismatch')
    h=hashlib.sha256()
    with ARCHIVE.open('rb') as source:
        for chunk in iter(lambda:source.read(4*1024*1024),b''):h.update(chunk)
    result=dict(path=str(ARCHIVE),source_files=len(manifest),source_bytes=total,zip_bytes=ARCHIVE.stat().st_size,
                sha256=h.hexdigest(),zip_crc_verified=True,frozen_motion_bindings_verified=len(frozen),
                credentials_pattern_omissions=len(omitted),important_policy_result_coverage=coverage,
                elapsed_seconds=round(time.monotonic()-began,1),remote_upload_performed=False)
    (OUT/'DELIVERY.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
