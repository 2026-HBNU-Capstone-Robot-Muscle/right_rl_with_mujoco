"""Package a small context handoff; never open devices or account data."""
from pathlib import Path
import hashlib
import json
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    raw_path = ROOT / 'middle_clearance_probe_20261106_230902_998326/result.json'
    raw = json.loads(raw_path.read_bytes())
    summary = json.loads((HERE / 'last_robot_run_summary.json').read_bytes())
    fields = ('status', 'error', 'failed_phase', 'all_torque_off_verified',
              'settings_restored', 'cube_removal_confirmed', 'middle_probe_position_ticks',
              'middle_probe_photo', 'grasp_observation_photo')
    for key in fields:
        if raw.get(key) != summary.get(key):
            raise ValueError(f'Run summary differs from original: {key}')
    if summary.get('review_pending') is not True:
        raise ValueError('Visual review must remain pending')

    ready = ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/ready.json'
    bindings = json.loads(ready.read_bytes())['code_hashes']
    failures = []
    for name, expected in bindings.items():
        path = Path(name)
        if not path.is_absolute():
            path = ROOT / path
        if not path.is_file() or sha(path) != expected:
            failures.append(name)
    if failures:
        raise ValueError(f'Frozen source bindings changed: {failures}')

    names = ('웹에서_이어가기.md', 'CLOUD_START_PROMPT.md', 'last_robot_run_summary.json')
    manifest = {
        'kind': 'context_only_not_project_or_full_chat_export',
        'repository_upload_deferred': False,
        'initial_destination': 'personal_private_github_repository',
        'team_or_organization_transfer_later': True,
        'repository_address_and_authentication_confirmed': False,
        'cloud_environment_created': False,
        'contains_code_and_models': False,
        'robot_visual_review_pending': True,
        'source_run': raw_path.relative_to(ROOT).as_posix(),
        'source_run_sha256': sha(raw_path),
        'frozen_source_bindings_checked': len(bindings),
        'files': {name: {'bytes': (HERE / name).stat().st_size, 'sha256': sha(HERE / name)}
                  for name in names},
    }
    (HERE / 'context_manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    destination = HERE / 'codex_web_context_20261106.zip'
    with zipfile.ZipFile(destination, 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in (*names, 'context_manifest.json'):
            archive.write(HERE / name, name)
    with zipfile.ZipFile(destination) as archive:
        if archive.testzip() is not None:
            raise ValueError('ZIP integrity check failed')
        if set(archive.namelist()) != set((*names, 'context_manifest.json')):
            raise ValueError('Unexpected archive entries')
        for name in names:
            digest = hashlib.sha256(archive.read(name)).hexdigest()
            if digest != manifest['files'][name]['sha256']:
                raise ValueError(f'Archive digest mismatch: {name}')
    print(json.dumps({'archive': str(destination), 'bytes': destination.stat().st_size,
                      'frozen_source_bindings_unchanged': len(bindings),
                      'latest_run_summary_matches': True, 'robot_devices_opened': 0,
                      'external_uploads': 0}, ensure_ascii=False))


if __name__ == '__main__':
    main()
