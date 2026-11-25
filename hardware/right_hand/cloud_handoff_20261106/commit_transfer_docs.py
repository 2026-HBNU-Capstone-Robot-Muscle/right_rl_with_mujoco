"""Commit only the verified transfer notes and cloud setup documentation."""
from pathlib import Path
import json,subprocess

HERE=Path(__file__).resolve().parent
REPO=HERE/'upload_repo'


def git(*args):
    result=subprocess.run(['/usr/bin/git','-C',str(REPO),*args],check=True,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    return result.stdout.strip()


def main():
    expected='2573a39e21f163d3263cd8801f6b59c207727301'
    if git('rev-parse','HEAD')!=expected:raise RuntimeError('Review unexpected local commit before continuing')
    setup=REPO/'cloud/setup.sh';setup.write_bytes(setup.read_bytes().replace(b'\r\n',b'\n'))
    subprocess.run(['/bin/bash','-n',str(setup)],check=True)
    git('diff','--check')
    names=['README.md','CURRENT_STATE.md','professor_progress_20261015.md',
           'cloud_handoff_20261106/CLOUD_START_PROMPT.md','cloud_handoff_20261106/웹에서_이어가기.md',
           'cloud/upload_manifest.json','cloud/transfer_status.json','cloud/SETUP.md','cloud/START_PROMPT.txt','cloud/setup.sh']
    git('add','--',*names)
    git('commit','-q','-m','Record verified private GitHub transfer and cloud setup workflow')
    result={'status':'transfer_docs_committed_pending_push','commit':git('rev-parse','HEAD'),
            'parent':expected,'working_tree_clean':not bool(git('status','--porcelain'))}
    if not result['working_tree_clean']:raise RuntimeError('Unexpected unstaged changes remain')
    (HERE/'transfer_docs_commit.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
