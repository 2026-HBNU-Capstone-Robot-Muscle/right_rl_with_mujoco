"""Make an isolated local Git snapshot and portable bundle, without pushing."""
from pathlib import Path
import json
import shlex
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
REPO=HERE/'upload_repo'
BRANCH='codex/cloud-handoff-20261106'


def git(*args):
    return subprocess.run(['/usr/bin/git','-C',str(REPO),*args],check=True,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE).stdout.strip()


def main():
    if (REPO/'.git').exists():raise RuntimeError('Existing prepared Git repository must be reviewed before changing')
    validation={
        'status':'selected_snapshot_prepared_and_saved_model_smoke_passed',
        'source_files_verified':3519,'frozen_source_bindings_verified':170,
        'windows_source_hash_verification_passed':True,
        'linux_saved_model_smoke_passed':True,
        'linux_mujoco_version':'3.9.0','model_nq':23,'model_nu':16,
        'saved_model_steps':200,'candidate_goals':1991,
        'full_historical_environment_verified':False,
        'full_path_or_physical_success_claimed':False,'devices_opened':0,
        'remote_upload_status_at_snapshot_creation':'not_uploaded'
    }
    (REPO/'cloud/preparation_validation.json').write_text(json.dumps(validation,indent=2)+'\n',encoding='utf-8')
    git('init','-q','--initial-branch',BRANCH)
    for key,value in (('core.autocrlf','false'),('core.filemode','false'),('user.name','Codex'),('user.email','codex@local.invalid')):
        git('config','--local',key,value)
    gh=ROOT/'.runtime/github-cli/gh.exe'
    git('config','--local','credential.helper','')
    git('config','--local','--add','credential.helper','!'+shlex.quote(str(gh))+' auth git-credential')
    git('remote','add','origin','https://github.com/star1209042/xlerobot.git')
    git('add','--all')
    git('commit','-q','-m','Preserve LEAP right-hand sources, current evidence and offline cloud handoff')
    bundle=HERE/'xlerobot_handoff_20261106.bundle'
    git('bundle','create',str(bundle),BRANCH)
    report={'status':'local_git_snapshot_ready_not_pushed','branch':BRANCH,'commit':git('rev-parse','HEAD'),
            'tracked_files':len(git('ls-files').splitlines()),'bundle':bundle.name,'bundle_bytes':bundle.stat().st_size,
            'remote':'https://github.com/star1209042/xlerobot.git','public_visibility_verified':False,
            'repository_visibility_verified':False,'remote_upload_complete':False,'working_tree_clean':not bool(git('status','--porcelain'))}
    (HERE/'local_git_snapshot.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report))


if __name__=='__main__':main()
