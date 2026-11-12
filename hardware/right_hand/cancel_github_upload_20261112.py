"""Undo this session's remote uploads with exact-head leases; retain local work."""
import json
from pathlib import Path
from prepare_github_hardware_update_20261112 import ROOT,PRIVATE,TEAM,OUT,git

def heads(repo):
    text=git(repo,'ls-remote','--heads','origin','refs/heads/feature/hardware','refs/heads/main','refs/heads/codex/cloud-handoff-20261106')
    return {line.split()[1]:line.split()[0] for line in text.splitlines() if line.strip()}

def main():
    specs=[(TEAM,OUT/'team_push_receipt.json'),(PRIVATE,OUT/'private_push_receipt.json')]
    plans=[]
    # Read and validate both repositories before changing either remote.
    for repo,path in specs:
        receipt=json.loads(path.read_bytes());ref='refs/heads/'+receipt['branch'];before=heads(repo)
        current=before.get(ref)
        if current is not None and current!=receipt['commit']:
            raise RuntimeError('Remote advanced after our upload; preserve others work: '+receipt['repository'])
        log=repo/'.git/logs/refs/remotes/origin'/receipt['branch']
        if not log.exists():raise RuntimeError('Missing branch creation evidence; do not delete blindly')
        first=log.read_text(encoding='utf-8').splitlines()[0];fields=first.split();message=first.split('\t',1)[-1]
        created_by_our_push=(fields[0]=='0'*40 and fields[1] in [receipt['commit'],*receipt.get('previous_verified_commits',[])] and message=='update by push')
        prior=None if created_by_our_push else fields[1]
        plans.append(dict(repo=repo,repository=receipt['repository'],ref=ref,expected=receipt['commit'],
                          before=before,current=current,restore_prior_sha=prior,created_by_our_push=created_by_our_push))
    results=[]
    for plan in plans:
        repo=plan['repo'];ref=plan['ref'];prior=plan['restore_prior_sha']
        if plan['current'] is not None:
            target=(prior or '')+':'+ref
            git(repo,'push','--force-with-lease='+ref+':'+plan['expected'],'origin',target)
        after=heads(repo)
        if after.get(ref)!=prior:raise RuntimeError('Upload rollback remote verification failed')
        for preserved in ('refs/heads/main','refs/heads/codex/cloud-handoff-20261106'):
            if after.get(preserved)!=plan['before'].get(preserved):
                raise RuntimeError('An unrelated branch changed concurrently; inspect before reporting')
        results.append(dict(repository=plan['repository'],branch=ref.removeprefix('refs/heads/'),
            action='deleted_session_created_branch' if prior is None else 'restored_previous_branch_head',
            uploaded_head=plan['expected'],restored_sha=prior,remote_verified=True,
            local_branch_preserved=True,local_files_preserved=True,main_preserved=True))
    report=dict(user_requested_upload_cancellation=True,repositories=results,
                future_push_requires_new_explicit_user_instruction=True,local_work_deleted=False)
    (OUT/'upload_cancel_receipt_20261112.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False))

if __name__=='__main__':main()
