"""Record the verified first push and refresh handoff docs without devices."""
from pathlib import Path
import hashlib,json,shutil,subprocess

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
DEST=HERE/'upload_repo'
GH=ROOT/'.runtime/github-cli/gh.exe'
REPOSITORY='star1209042/xlerobot'
INITIAL='2573a39e21f163d3263cd8801f6b59c207727301'


def api(path):
    result=subprocess.run([str(GH),'api',path],check=True,text=True,encoding='utf-8',stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    return json.loads(result.stdout)


def main():
    meta=api('repos/'+REPOSITORY)
    if not meta['private']:raise RuntimeError('Repository no longer private; stop before continuing')
    refs={branch:api('repos/'+REPOSITORY+'/git/ref/heads/'+branch)['object']['sha']
          for branch in ('main','codex/cloud-handoff-20261106')}
    if any(commit!=INITIAL for commit in refs.values()):raise RuntimeError('Unexpected remote head; review before changing it')
    tree=api('repos/'+REPOSITORY+'/git/trees/'+INITIAL+'?recursive=1')
    if tree.get('truncated'):raise RuntimeError('Remote tree response truncated')
    files=[entry['path'] for entry in tree['tree'] if entry['type']=='blob']
    if len(files)!=3529 or any('.runtime' in Path(name).parts or '.codex' in Path(name).parts for name in files):
        raise RuntimeError('Unexpected uploaded file set')
    confirmation={
        'status':'initial_github_upload_verified', 'verified_on_client_date':'2026-11-07',
        'repository':REPOSITORY,'private':True,'remote_refs':refs,
        'initial_remote_blob_count':len(files),'original_source_files':3519,
        'frozen_source_bindings_preserved':170,'cloud_environment_created':False,
        'hardware_devices_opened':False,'full_local_chat_history_synced':False,
        'complete_original_workspace_backup':False,
    }
    (HERE/'initial_remote_upload_verified.json').write_text(json.dumps(confirmation,indent=2)+'\n',encoding='utf-8')

    handoff=HERE/'CLOUD_START_PROMPT.md';text=handoff.read_text(encoding='utf-8')
    first=text.index('## 지금 먼저 할 일');last=text.index('## 로봇 목표와 사용자 선호')
    new='''## 지금 먼저 할 일

사용자는 현재 VS Code Codex의 대화와 프로젝트를 Codex 웹과 다른 PC에서도 이어가려 한다. GitHub 계정은 이미 있으며 개인 계정에 먼저 올리고 팀 또는 프로젝트 쪽으로의 이전은 나중에 한다. **2026-11-07에 비공개 저장소 `https://github.com/star1209042/xlerobot`로 코드와 모델 및 인계 자료를 실제 업로드했고 원격 커밋을 확인했다.** 브랜치는 `main`과 `codex/cloud-handoff-20261106`이다.

현재 다음 단계는 이 저장소를 선택해 Codex 클라우드 환경을 만들고, `bash cloud/setup.sh`로 의존성과 저장 모델 검사를 확인한 뒤 환경을 게시하는 것이다. **클라우드 환경 생성이나 실제 클라우드 계산은 아직 확인하지 않았다.** 계정이나 GitHub CLI 로그인부터 다시 요구하지 말고 웹의 저장소 연결 상태를 구분한다. 환경 준비 순서는 `cloud/SETUP.md`, 첫 로봇 분석 요청은 `cloud/START_PROMPT.txt`에 있다.

원래 Windows의 `files-mentioned-by-the-user-leap/outputs` 폴더는 설치 환경 등을 제외해도 약 38.7GB였다. 업로드는 그중 원본 3,519개 파일 약 496MB와 안내 파일을 선별한 것으로 전체 백업은 아니다. 원본 작업 자료를 보존했고 계정 인증 정보와 실행 환경 및 대부분의 원시 영상을 올리지 않았다. 원본 코드와 결과의 SHA256 및 포함 범위는 `cloud/upload_manifest.json`에 있다. 현재 동작에 결합된 170개 원본 해시가 유지된다.

이 인계문은 대화 원문 전체의 자동 동기화를 뜻하지 않는다. IDE에서 새 클라우드 작업으로 맥락을 넘기는 기능과 웹 새 대화에 인계문을 붙여 넣는 방식은 구분한다. 클라우드에서 원래 PC의 USB나 웹캠에 접근할 수 있다고 가정하지 않는다. 나중에 팀/조직으로 저장소 소유권을 옮기거나 기존 팀 저장소에 코드를 합치는 것은 별도 작업이다.

'''
    handoff.write_text(text[:first]+new+text[last:],encoding='utf-8')

    guide=HERE/'웹에서_이어가기.md';g=guide.read_text(encoding='utf-8')
    g=g.replace('이 파일들을 만든 것만으로 클라우드 작업이 생성되거나 기존 대화 원문이 웹에 동기화되지는 않는다. 현재 코드는 이 PC에 있고 외부 저장소에는 올리지 않았다.',
                '2026-11-07에 코드와 모델을 선별하여 `star1209042/xlerobot` 비공개 저장소에 실제 업로드했고 원격 커밋을 확인했다. 클라우드 환경은 아직 만들었다고 확인되지 않았다. 기존 대화 원문 전체가 자동으로 웹에 동기화된 것은 아니다.')
    g=g.replace('2. 사용자가 지정한 `https://github.com/star1209042/xlerobot.git`에 정리한 프로젝트를 올린다. 아직 원격 조회 인증이 없어 실제 내용과 공개 여부를 확인하지 못했고, 공식 GitHub CLI 브라우저 로그인을 요청했다. 기존 파일이 있으면 보존한다. `codex_web_context_20261106.zip`은 문맥 인계용이고 별도 `upload_repo`가 코드와 모델을 포함한 업로드 준비 폴더다.',
                '2. 완료: `https://github.com/star1209042/xlerobot.git`가 비공개 빈 저장소임을 확인하고 `main` 및 인계 브랜치로 업로드했다. `codex_web_context_20261106.zip`은 문맥 인계용이고 `upload_repo`가 실제 Git 저장소다. 이제 `cloud/SETUP.md`에 따라 Codex 웹에 저장소를 연결한다.')
    guide.write_text(g,encoding='utf-8')

    progress=ROOT/'professor_progress_20261015.md';p=progress.read_text(encoding='utf-8')
    begin='<!-- cloud_handoff_20261106_current -->';end='<!-- cloud_handoff_20261106_end -->'
    a=p.index(begin);b=p.index(end,a)+len(end)
    current='''<!-- cloud_handoff_20261106_current -->
# 현재 우선 작업: GitHub 업로드 완료, Codex 클라우드 환경 연결

- 2026-11-07에 `star1209042` GitHub CLI 로그인을 완료했다. 지정한 `star1209042/xlerobot`가 비공개 빈 저장소이고 쓰기 권한이 있음을 확인했다.
- 원본 3,519개 파일과 안내 자료의 첫 커밋 `2573a39e21f163d3263cd8801f6b59c207727301`을 `main` 및 `codex/cloud-handoff-20261106`에 올렸다. GitHub API로 두 커밋과 원격 파일 3,529개를 확인한 뒤 현재 인계 문서를 갱신한다. 최종 원격 확인은 `cloud_handoff_20261106/remote_upload_result.json`에 기록한다.
- 원본은 약 38.7GB이고 업로드한 선별 원본은 약 496MB다. 완전한 원본 백업이 아니다. 현재 동작의 결합 원본 170개 SHA256을 보존했고 Windows 파일 검사 및 Linux MuJoCo 3.9.0 저장 모델 200단계 기본 검사를 통과했다. 전체 경로 성공이나 클라우드 환경 게시를 뜻하지 않는다.
- 다음 사용자 작업은 Codex 웹에서 이 저장소를 연결하고 환경을 만드는 것이다. `cloud/SETUP.md`와 `cloud/START_PROMPT.txt`를 준비한다. 실제 클라우드 환경 생성과 실행은 아직 미확인이다. 기존 로컬 대화 원문 전체의 자동 동기화로 설명하지 않는다.
- 최신 실물 중지 관찰은 완료됐으며 토크 OFF와 설정 복구 및 큐브 제거가 확인됐다. 실제 공간/파지 효과는 사진 검토가 남아 있다. 이번 전환 작업에서는 모터나 웹캠을 열지 않았다.
<!-- cloud_handoff_20261106_end -->'''
    progress.write_text(p[:a]+current+p[b:],encoding='utf-8')

    state='''# 현재 상태: 2026-11-07

`star1209042/xlerobot` 비공개 저장소로 코드와 모델 및 인계 자료를 올리고 두 원격 브랜치의 첫 커밋과 파일 목록을 확인했다. GitHub 인증은 완료됐다. 기본 작업 브랜치는 `main`, 인계 체크포인트는 `codex/cloud-handoff-20261106`이다.

이제 Codex 웹에서 저장소를 연결하고 환경을 만들면 된다. `cloud/SETUP.md`를 따르고 첫 요청은 `cloud/START_PROMPT.txt`를 사용한다. 실제 클라우드 환경 생성/게시와 계산은 아직 확인하지 않았다.

로봇 목표는 원하는 방향 약 90도 회전, 하부 파지와 윗단 공간 확보, 다음 회전 연결이다. 양손 통합과 전체 풀이가 끝난 상태는 아니다. 가장 최근 `middle_clearance_probe_20261106_230902_998326/result.json`은 중지 관찰 완료, 토크 OFF, 설정 복구와 큐브 제거를 확인했다. 해당 폴더의 `cube_grasp_hold.jpg`와 `middle_probe_hold.jpg` 효과는 아직 검토하지 않았다.

소스와 결과 원본 3,519개 약 496MB를 선별했다. 원래 38.7GB 전체의 백업이나 로컬 대화 원문 전체의 동기화는 아니다. 원본 Windows 자료는 보존했다. 현재 동작 결합 원본 170개는 원래 SHA256을 유지한다. 장치 명령 대신 오프라인 작업을 이어간다.

상세 문맥은 `cloud_handoff_20261106/CLOUD_START_PROMPT.md`와 `professor_progress_20261015.md`에 있다. `cloud/transfer_status.json`은 첫 업로드의 실제 확인 기록이다. 이후 변경의 원격 HEAD와 구분해서 읽는다.
'''
    (DEST/'CURRENT_STATE.md').write_text(state,encoding='utf-8')
    (DEST/'cloud/transfer_status.json').write_text(json.dumps(confirmation,indent=2)+'\n',encoding='utf-8')
    manifest_path=DEST/'cloud/upload_manifest.json';manifest=json.loads(manifest_path.read_bytes())
    for name in ('cloud_handoff_20261106/CLOUD_START_PROMPT.md','cloud_handoff_20261106/웹에서_이어가기.md','professor_progress_20261015.md'):
        target=DEST/name;shutil.copy2(ROOT/name,target)
        entry=next(row for row in manifest['source_manifest'] if row['path']==name)
        entry.update(bytes=target.stat().st_size,sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    manifest['source_bytes']=sum(row['bytes'] for row in manifest['source_manifest'])
    manifest_path.write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(confirmation))


if __name__=='__main__':main()
