"""Create local ZIP deliveries after cancellation; never upload or open devices."""
import hashlib
import json
import re
import shutil
import subprocess
import zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parent
PRIVATE=ROOT/'cloud_handoff_20261106/upload_repo'
DEST=ROOT/'deliverables_20261112'

def digest(data):return hashlib.sha256(data).hexdigest()
def safe_source(name):
    p=(ROOT/name).resolve();p.relative_to(ROOT)
    if any(s in ('.git','.runtime','.aws','.codex','.agents','__pycache__','camera') for s in p.relative_to(ROOT).parts):
        raise RuntimeError('Excluded delivery path: '+name)
    return p

def zip_files(path,files,extra):
    hashes={name:digest(data) for name,data in files.items()}
    metadata=json.dumps(dict(files=hashes,runtimes_included=False,credentials_included=False,raw_camera_directory_included=False),indent=2,ensure_ascii=False).encode('utf-8')
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for name,data in sorted(files.items()):z.writestr(name,data)
        for name,data in extra.items():z.writestr(name,data)
        z.writestr('DELIVERY_MANIFEST.json',metadata)
    with zipfile.ZipFile(path) as z:
        if z.testzip() is not None:raise RuntimeError('ZIP CRC check failed')
    return dict(path=str(path),files=len(files),bytes=path.stat().st_size,sha256=digest(path.read_bytes()))

def main():
    DEST.mkdir(exist_ok=True)
    receipt_path=ROOT/'team_integration_20261112/upload_cancel_receipt_20261112.json'
    receipt=json.loads(receipt_path.read_bytes());assert all(r['remote_verified'] for r in receipt['repositories'])
    receipt.update(remote_refs_removed_not_commit_object_erasure=True,old_team_commit_url_still_resolves=True)
    receipt_path.write_text(json.dumps(receipt,indent=2,ensure_ascii=False),encoding='utf-8')
    marker='user_revoked_github_upload_20261112'
    note='''\n<!-- user_revoked_github_upload_20261112 -->\n## 최신 사용자 지침: GitHub 업로드 취소, 파일로만 전달\n\n사용자가 직전 GitHub 업로드를 취소하고 파일로 달라고 요청했다. 이번 세션에서 생성한 팀/개인 `feature/hardware` 원격 업로드를 취소하고 원격 상태를 확인했다. main과 기존 브랜치, 로컬 파일과 커밋은 보존했다. 기록은 `team_integration_20261112/upload_cancel_receipt_20261112.json`이다.\n\n이 지침은 앞선 수시 GitHub 업데이트 요청보다 우선한다. 사용자가 다시 명시적으로 요청하기 전에는 GitHub에 푸시하거나 업로드하지 않는다. 수정 코드와 실물 시험 자료는 로컬 ZIP으로 전달한다. 현재 시험 범위는 뒤쪽 접촉·끝마디 우선 밀기이며 최종 세 손가락 파지와 다음 회전은 미완료다.\n<!-- /user_revoked_github_upload_20261112 -->\n'''
    note=note.replace('main과 기존 브랜치, 로컬 파일과 커밋은 보존했다.',
        'main과 기존 브랜치, 로컬 파일과 커밋은 보존했다. 원격 브랜치 삭제는 커밋 객체 영구 삭제가 아니며, 이전 팀 커밋 URL은 아직 접근 가능함을 확인했다.')
    for path in (ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',ROOT/'professor_progress_20261015.md'):
        doc=path.read_text(encoding='utf-8-sig')
        if marker not in doc:
            first,sep,rest=doc.partition('\n');path.write_text(first+'\n'+note+'\n'+rest,encoding='utf-8')
        else:
            begin=doc.index('<!-- '+marker+' -->');end=doc.index('<!-- /'+marker+' -->',begin)+len('<!-- /'+marker+' -->')
            path.write_text(doc[:begin]+note.strip()+doc[end:],encoding='utf-8')
    inventory=json.loads((PRIVATE/'cloud/hardware_update_manifest_20261112.json').read_bytes())
    files={}
    for name in inventory['files']:
        source=safe_source(name)
        if source.is_file():files[name]=source.read_bytes()
    # Verify the source bundle in a fresh folder with the existing Python
    # environment. This is CHECK ONLY, with no --execute or device commands.
    verify=DEST/'verify_tip_first_bundle';verify.mkdir(exist_ok=True)
    for name,data in files.items():
        target=verify/name;target.resolve().relative_to(verify.resolve());target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    check=None
    for attempt in range(20):
        check=subprocess.run([str(ROOT/'.runtime/python/python.exe'),'-X','utf8','-B',str(verify/'leap_cycle_tip_first_transfer_20261112.py')],
                             cwd=verify,capture_output=True,text=True,encoding='utf-8',errors='replace')
        if check.returncode==0:break
        output=check.stdout+'\n'+check.stderr;added=False
        # Missing static source files are copied explicitly, inside the workspace.
        for value in re.findall(r"No such file or directory: ['\"]([^'\"]+)['\"]",output):
            missing=Path(value).resolve()
            try:relative=missing.relative_to(verify.resolve())
            except ValueError:continue
            name=relative.as_posix();source=safe_source(name)
            if source.is_file() and name not in files:
                data=source.read_bytes();files[name]=data;missing.parent.mkdir(parents=True,exist_ok=True);missing.write_bytes(data);added=True
        for module in re.findall(r"No module named ['\"]([^'\"]+)['\"]",output):
            name=module.replace('.','/')+'.py';source=safe_source(name)
            if source.is_file() and name not in files:
                data=source.read_bytes();files[name]=data;target=verify/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data);added=True
        if not added:
            (DEST/'bundle_check_failure.txt').write_text(output,encoding='utf-8')
            raise RuntimeError('Standalone source CHECK ONLY failed; inspect bundle_check_failure.txt')
    if check is None or check.returncode:raise RuntimeError('Bundle dependency resolution did not finish')
    if 'CHECK ONLY' not in check.stdout:raise RuntimeError('Unexpected bundle verification mode')
    verification=dict(check_only_passed=True,exit_code=0,devices_opened=0,assistant_motor_writes=0,source_files=len(files))
    readme='''# 로컬 파일 전달\n\n직전 GitHub 업로드는 취소했다. 향후 업로드는 사용자가 다시 명시적으로 요청할 때만 진행한다. 로컬 작업 파일은 그대로 보존했다.\n\n이 묶음은 기존 Windows `outputs` 작업 폴더의 소스·검증 자료다. Python 설치 환경과 계정 정보는 포함하지 않았다. 원래 PC의 기존 `outputs` 폴더에 풀어 기존 `.runtime` 환경으로 사용한다. 다른 PC의 USB 포트나 모터 설정이 같다고 가정하지 않는다.\n\n실물 시험 시작 파일: `leap_cycle_tip_first_transfer.cmd`\n실행 안내: `right_hand_rotation_20261112/tip_first_trial_v2/operator_guide.md`\n\n빈손으로 실행 → 첫 SPACE → PLACE 안내 때 초록 위/주황 손목 방향으로 큐브 배치 → 두 번째 SPACE. 미끄러지거나 떨어지면 Q. 종료 안내 때 큐브를 받치고 마지막 SPACE로 OFF한다.\n\n이번 파일은 주황 옆면 뒤쪽 접촉을 목표로 끝마디 → 다음 마디 순서로 미는 관찰 시험이다. 약지 닫힘·최종 세 손가락 파지·다음 회전은 포함하지 않는다. 실제 최대 굽힘이나 전체 작업 성공을 인증한 파일이 아니다.\n\n압축 묶음의 소스를 별도 폴더에 풀어 기존 Python 환경에서 CHECK ONLY 검사를 통과했다. 이 검사는 실제 모터·카메라를 열지 않는다. 원시 영상과 camera 폴더 대신 이미 검토한 선별 사진4장만 포함했다.\n'''
    readme+='\n업로드 취소는 이번에 만든 두 원격 브랜치를 삭제한 것이다. GitHub의 커밋 객체 영구 삭제는 아니며, 기존 팀 커밋 URL은 아직 접근 가능하다.\n'
    extra={'README_먼저읽기.md':readme.encode('utf-8'),'UPLOAD_CANCELLED.json':receipt_path.read_bytes(),
           'BUNDLE_CHECK.json':json.dumps(verification,indent=2).encode('utf-8')}
    runtime=zip_files(DEST/'right_hand_tip_first_test_20261112.zip',files,extra)
    code_files={name:(ROOT/name).read_bytes() for name in ('tip_first_transfer_20261112.py','test_tip_first_transfer_20261112.py')}
    code_readme='''# 경로 수정 코드\n\n엄지1·중지9 끝마디를 먼저, 엄지2·중지11을 다음에 굽히는 순서와 뒤쪽 접촉 벡터 계산 모듈이다. NumPy가 있는 Python에서 `python -B test_tip_first_transfer_20261112.py`로4개 시험을 실행한다.\n\n모터 실행기가 아닌 경로 제안 모듈이다. 실물 시험에는 별도로 전달한 실물 시험 ZIP과 기존 작업 환경을 사용한다. 직전 GitHub 업로드는 취소했고 로컬 파일로 전달한다.\n'''
    code=zip_files(DEST/'right_hand_transfer_code_20261112.zip',code_files,{'README.md':code_readme.encode('utf-8'),'UPLOAD_CANCELLED.json':receipt_path.read_bytes()})
    result=dict(github_upload_cancelled=True,code_zip=code,runtime_zip=runtime,bundle_check=verification)
    (DEST/'DELIVERY.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))

if __name__=='__main__':main()
