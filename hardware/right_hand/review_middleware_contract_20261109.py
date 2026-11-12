"""Cache a pinned, read-only middleware source review without loading its code."""
from pathlib import Path
import base64
import concurrent.futures
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parent
GH = ROOT / '.runtime/github-cli/gh.exe'
REPO = '2026-HBNU-Capstone-Robot-Muscle/middleware-bridge'
OUT = ROOT / 'team_integration_20261109/middleware_review'


def api(endpoint):
    result = subprocess.run([str(GH), 'api', endpoint], capture_output=True,
                            text=True, encoding='utf-8', check=True)
    return json.loads(result.stdout)


def main():
    repo = api('repos/' + REPO)
    branch = repo['default_branch']
    commit = api('repos/' + REPO + '/commits/' + branch)['sha']
    branches = api('repos/' + REPO + '/branches?per_page=100')
    tree = api('repos/' + REPO + '/git/trees/' + commit + '?recursive=1')
    paths = [item['path'] for item in tree['tree'] if item['type'] == 'blob']
    selected = [p for p in paths if (
        p == 'README.md' or
        (p.startswith('docs/') and p.endswith('.md')) or
        p.endswith('AGENTS.md') or
        (p.startswith(('src/control/', 'src/system_interfaces/', 'src/bringup/', 'src/system_bringup/'))
         and p.endswith(('.py', '.md', '.json', '.srv', '.action', '.msg', '.yaml', '.yml', '.xml')))
    ) and not p.startswith('apps/monitoring_ui/')]
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'tree.json').write_text(json.dumps(tree, indent=2) + '\n', encoding='utf-8')

    def fetch(path):
        item = api('repos/' + REPO + '/contents/' + path + '?ref=' + commit)
        data = base64.b64decode(item['content'])
        target = OUT / 'source' / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        return {'path': path, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        files = list(pool.map(fetch, selected))
    report = {'repository': REPO, 'branch': branch, 'commit': commit,
              'branches': [{'name': b['name'], 'sha': b['commit']['sha']} for b in branches],
              'files': files, 'remote_writes': 0, 'devices_opened': 0,
              'monitoring_ui_files_read_or_modified': 0}
    (OUT / 'manifest.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'repository': REPO, 'commit': commit, 'branches': report['branches'],
                      'files': selected}, ensure_ascii=False))


if __name__ == '__main__':
    main()
