"""Read selected GitHub metadata through gh without printing credentials."""
from pathlib import Path
import json,subprocess

HERE=Path(__file__).resolve().parent
GH=HERE.parent/'.runtime/github-cli/gh.exe'


def api(endpoint):
    result=subprocess.run([str(GH),'api',endpoint],check=True,text=True,encoding='utf-8',stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    return json.loads(result.stdout)


def main():
    actor=api('user')['login']
    repo=api('repos/star1209042/xlerobot')
    metadata={key:repo.get(key) for key in ('full_name','private','default_branch','permissions','size','fork','archived','disabled','pushed_at','html_url')}
    metadata['authenticated_account']=actor
    (HERE/'remote_repository_metadata.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(metadata,ensure_ascii=False))


if __name__=='__main__':main()
