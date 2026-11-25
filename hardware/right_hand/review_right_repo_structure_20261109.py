"""Read and cache existing team repository sources; no remote writes or imports."""
from pathlib import Path
import base64
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parent
GH = ROOT / '.runtime/github-cli/gh.exe'
REPO = '2026-HBNU-Capstone-Robot-Muscle/right_rl_with_mujoco'
OUT = ROOT / 'right_hand_rotation_20261109/team_sources/right_rl_with_mujoco'


def api(endpoint):
    result = subprocess.run(
        [str(GH), 'api', endpoint], capture_output=True, text=True,
        encoding='utf-8', check=True,
    )
    return json.loads(result.stdout)


def main():
    details = api('repos/' + REPO)
    commit = api('repos/' + REPO + '/commits/' + details['default_branch'])['sha']
    reports = []
    for relative in ('train/README.md', 'docs/guide.md', 'deploy/leap_hw.py', 'deploy/limits.py'):
        source = api('repos/' + REPO + '/contents/' + relative + '?ref=' + commit)
        data = base64.b64decode(source['content'])
        target = OUT / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        reports.append({'path': relative, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()})
        lines = data.decode('utf-8', errors='replace').splitlines()
        if relative == 'docs/guide.md':
            display = [f'{i}: {line}' for i, line in enumerate(lines, 1) if line.startswith('#')][:70]
        else:
            display = [f'{i}: {line}' for i, line in enumerate(lines, 1)][:125]
        print(json.dumps({'path': relative, 'excerpt': display}, ensure_ascii=False))
    result = {'repository': REPO, 'commit': commit, 'files': reports, 'remote_writes': 0}
    (OUT / 'structure_review_sources.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
