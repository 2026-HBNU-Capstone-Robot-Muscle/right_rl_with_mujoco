import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'sync_burst_readonly_20261028_042302_708111'
r=json.loads((folder/'result.json').read_bytes())
for e in r['bad_transactions']:
    raw=(folder/e['wire_file']).read_bytes();heads=[]
    for i in range(len(raw)-7):
        if raw[i:i+4]==b'\xff\xff\xfd\x00':heads.append((i,raw[i+4]))
    print(json.dumps(dict(camera_on=bool(e['block']),wire_bytes=len(raw),expected_unstuffed_bytes=16*94,
        missing_bytes=16*94-len(raw),headers=heads,crc_invalid=[f['id'] for f in e['decode']['frames'] if f.get('crc_ok') is False])))
