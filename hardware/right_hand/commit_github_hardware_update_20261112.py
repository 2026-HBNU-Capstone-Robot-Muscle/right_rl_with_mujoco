"""Commit explicit selected changes and verify branch heads; never force-push."""
import argparse
import json
import subprocess
from pathlib import Path
from prepare_github_hardware_update_20261112 import ROOT,PRIVATE,TEAM,OUT,git

def commit_and_push(repo,paths,message,repository,resume_prepared=False):
    branch='feature/hardware';current=git(repo,'branch','--show-current')
    if current!=branch:
        if git(repo,'branch','--list',branch):raise RuntimeError('Preserve an existing local hardware branch')
        git(repo,'checkout','-b',branch)
    existing=git(repo,'diff','--cached','--name-only')
    if existing and not (resume_prepared and set(existing.splitlines()).issubset(set(paths))):
        raise RuntimeError('Preserve pre-existing staged changes')
    git(repo,'add','--',*paths)
    staged=git(repo,'diff','--cached','--name-only').splitlines()
    if any(p.startswith('apps/monitoring_ui/') for p in staged):raise RuntimeError('UI changes are out of scope')
    # Keep the approved snapshot's CRLF bytes; do not normalize frozen sources.
    git(repo,'-c','core.whitespace=cr-at-eol','diff','--cached','--check')
    if staged:git(repo,'commit','-m',message)
    head=git(repo,'rev-parse','HEAD')
    git(repo,'push','--set-upstream','origin',branch)
    remote=git(repo,'ls-remote','--heads','origin','refs/heads/'+branch).split()[0]
    if remote!=head:raise RuntimeError('Remote head verification failed')
    return dict(repository=repository,branch=branch,commit=head,url='https://github.com/'+repository+'/commit/'+head,
                changed_files=staged,remote_verified=True,force_push=False)

def main():
    p=argparse.ArgumentParser();p.add_argument('--team',action='store_true');p.add_argument('--private',action='store_true');p.add_argument('--message');p.add_argument('--resume-prepared',action='store_true');a=p.parse_args()
    assert a.team!=a.private
    if a.team:
        paths=['hardware/right_hand/'+n for n in ('tip_first_transfer_20261112.py','test_tip_first_transfer_20261112.py','README.md')]
        test=TEAM/'hardware/right_hand/test_tip_first_transfer_20261112.py'
        r=subprocess.run([str(ROOT/'.runtime/python/python.exe'),'-X','utf8','-B',str(test)],capture_output=True,text=True,encoding='utf-8')
        if r.returncode:raise RuntimeError('Team software checks failed: '+r.stderr[-1500:])
        receipt=commit_and_push(TEAM,paths,a.message or 'feat(hardware): add bounded distal-first transfer proposals','2026-HBNU-Capstone-Robot-Muscle/right_rl_with_mujoco')
        receipt['software_tests_passed']=4;receipt['full_robot_motion_ready']=False
        target=OUT/'team_push_receipt.json'
    else:
        manifest=json.loads((PRIVATE/'cloud/hardware_update_manifest_20261112.json').read_bytes())
        paths=list(manifest['files'])+['CURRENT_STATE.md','README.md','cloud/upload_manifest.json','cloud/hardware_update_manifest_20261112.json',
            'cloud/SETUP.md','cloud/START_PROMPT.txt','cloud/setup.sh','cloud/transfer_status.json',
            'cloud_handoff_20261106/웹에서_이어가기.md','cloud/smoke.py']
        paths=sorted(set(p for p in paths if (PRIVATE/p).is_file()))
        receipt=commit_and_push(PRIVATE,paths,'chore(hardware): sync current right-hand paths and verified model evidence','star1209042/xlerobot',resume_prepared=a.resume_prepared)
        target=OUT/'private_push_receipt.json'
    if target.exists():
        previous=json.loads(target.read_bytes());receipt['previous_verified_commits']=previous.get('previous_verified_commits',[])+[previous['commit']]
    target.write_text(json.dumps(receipt,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in receipt.items() if k!='changed_files'},ensure_ascii=False))

if __name__=='__main__':main()
