"""Extract the latest user image from this day's local conversation logs, if present.

Only image metadata is printed; no conversation text or credentials are output.
"""
import base64
import json
from pathlib import Path

root = Path(__file__).resolve().parent
sessions = Path.home() / '.codex' / 'sessions' / '2026' / '10' / '11'
found = []
image_metadata = []
for path in sorted(sessions.glob('*.jsonl')):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            try:
                item = json.loads(line)
            except ValueError:
                continue
            payload = item.get('payload', {})
            if payload.get('type') == 'user_message':
                for key in ('images', 'local_images'):
                    for value in payload.get(key, []) or []:
                        if isinstance(value, str) and value.startswith('data:image/'):
                            found.append((item.get('timestamp', ''), value))
                        elif value:
                            image_metadata.append({'timestamp': item.get('timestamp'),
                                                   'field': key, 'reference': value})
            if payload.get('role') != 'user':
                continue
            for part in payload.get('content', []):
                if not isinstance(part, dict) or part.get('type') not in ('input_image', 'image'):
                    continue
                value = part.get('image_url', part.get('image', ''))
                if isinstance(value, dict):
                    value = value.get('url', '')
                if isinstance(value, str) and value.startswith('data:image/'):
                    found.append((item.get('timestamp', ''), value))

if not found:
    print('No inline image data found in the local user-message logs.')
    print(json.dumps(image_metadata[-8:], ensure_ascii=False))
else:
    stamp, uri = sorted(found, key=lambda x: x[0])[-1]
    header, encoded = uri.split(',', 1)
    suffix = '.png' if 'image/png' in header else '.jpg' if 'image/jpeg' in header else '.webp'
    output = root / ('support_ruler_source_20261011' + suffix)
    if output.exists():
        raise RuntimeError('Source already exists; inspect before replacing it.')
    output.write_bytes(base64.b64decode(encoded))
    print(json.dumps({'saved': str(output), 'timestamp': stamp,
                      'inline_user_images_found': len(found)}, ensure_ascii=False))
