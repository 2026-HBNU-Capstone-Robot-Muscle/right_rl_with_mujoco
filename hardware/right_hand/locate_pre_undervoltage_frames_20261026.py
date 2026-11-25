"""Find saved frames before the first low-voltage observation, no device access."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'recovered_forward_placement_20261026_041024_485668'
def main():
    r=json.loads((SOURCE/'result.json').read_bytes());frames=json.loads((SOURCE/'camera/frames.json').read_bytes())['frames']
    stop=r['enabled_at']+r['samples'][-1]['t']
    rows=[]
    for delta in (8.,3.,1.,.2):
        selected=max((f for f in frames if f['camera_read_monotonic']<=stop-delta),key=lambda f:f['camera_read_monotonic'])
        rows.append(dict(seconds_before_fault=stop-selected['camera_read_monotonic'],image=str(SOURCE/'camera'/selected['image']),phase=selected['phase']))
    print(json.dumps(rows,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
