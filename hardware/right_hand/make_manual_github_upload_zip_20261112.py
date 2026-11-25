"""Prepare the former team upload as a local ZIP; make no remote changes."""
from pathlib import Path
import json
import zipfile
ROOT=Path(__file__).resolve().parent
folder=ROOT/'team_integration_20261112/right_rl_with_mujoco/hardware/right_hand'
target=ROOT/'deliverables_20261112/GitHub_upload_right_hand_20261112.zip'
target.parent.mkdir(exist_ok=True)
names=('tip_first_transfer_20261112.py','test_tip_first_transfer_20261112.py','README.md')
with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for name in names:
        data=(folder/name).read_bytes()
        if name=='README.md':
            text=data.decode('utf-8')
            prefix=text.split('\n전체 원본 작업·모델·실행기 검증은',1)[0]
            text=prefix+'\n\n이 묶음은 방금 팀 저장소에 공유했던 경로 제안 모듈·시험·검토 문서3개다. 전체 실물 실행기와 검증 자료는 별도 전달 파일에 보존한다. 자동 GitHub 업로드는 취소됐으며, 이 ZIP은 사용자가 직접 올리기 위한 파일이다. `feature/hardware` 브랜치의 `hardware/right_hand/` 경로에 배치한다. `apps/monitoring_ui`는 수정하지 않는다.\n'
            data=text.encode('utf-8')
        z.writestr('hardware/right_hand/'+name,data)
with zipfile.ZipFile(target) as z:
    assert z.testzip() is None
    entries=z.namelist()
print(json.dumps(dict(path=str(target),bytes=target.stat().st_size,files=entries,remote_changes=0),ensure_ascii=False))
