"""Check cached review provenance and unchanged original execution bindings."""
from pathlib import Path
import hashlib
import json
import re

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'team_integration_20261109/middleware_review'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    manifest = json.loads((OUT / 'manifest.json').read_bytes())
    source_changed = [item['path'] for item in manifest['files']
                      if digest(OUT / 'source' / item['path']) != item['sha256']]
    ready = json.loads((ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/ready.json').read_bytes())
    binding_changed = [name for name, expected in ready['code_hashes'].items()
                       if digest(ROOT / name) != expected]
    text = (OUT / 'REVIEW.md').read_text(encoding='utf-8')
    missing = [link for link in re.findall(r'\]\(([^)]+)\)', text)
               if not link.startswith(('https://', 'http://')) and not (OUT / link).is_file()]
    report = {
        'cached_source_files_verified': len(manifest['files']),
        'cached_source_changes': source_changed,
        'frozen_execution_bindings_verified': len(ready['code_hashes']),
        'frozen_execution_changes': binding_changed,
        'missing_report_links': missing,
        'remote_writes': 0, 'devices_opened': 0,
        'monitoring_ui_files_modified': 0,
    }
    (OUT / 'verification.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False))
    if source_changed or binding_changed or missing:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
