"""Inspect saved voltage confirmation timing only."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    folder=ROOT/'thumb_clear_root_empty_20261026_115050_905503';r=json.loads((folder/'result.json').read_bytes())
    print('VOLTAGE_EVENTS',json.dumps(r.get('voltage_events'),ensure_ascii=False))
    for k in ('final_off_reference_valid','final_off_endpoint_drift_ticks','final_off_snapshots'):
        v=r.get(k)
        if v is not None:print(k,json.dumps(v,ensure_ascii=False)[:7000])
    print('TAIL',json.dumps([dict(t=s['t'],phase=s['phase'],voltage=s['voltage_v'],read_ms=s.get('read_ms'),current=s['current_ma']) for s in r['samples'][-12:]],ensure_ascii=False))
    print('KEYS',list(r))
    print('READONLY_SCRIPTS',[p.name for p in ROOT.glob('*.py') if ('read' in p.name or 'off_state' in p.name) and any(v in p.name for v in ('finish','connected','slow'))])
if __name__=='__main__':main()
