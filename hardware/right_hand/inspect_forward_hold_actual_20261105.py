"""Read saved forward-hold runs only; never open robot devices."""
import json
from pathlib import Path
from collections import Counter

ROOT = Path(__file__).resolve().parent

def short(value):
    if isinstance(value, dict):
        return {k: short(v) for k, v in value.items() if k not in ('data', 'image_data')}
    if isinstance(value, list) and len(value) > 20:
        return {'length': len(value), 'first': short(value[0]), 'last': short(value[-1])}
    return value

def main():
    for path in sorted(ROOT.glob('forward_hold_observe_20261105_*/result.json')):
        d = json.loads(path.read_bytes())
        scalars = {k:v for k,v in d.items() if not isinstance(v, (dict,list))}
        print(json.dumps({'folder': path.parent.name, 'scalars':scalars}, ensure_ascii=False))
        samples = d.get('samples', [])
        if samples:
            print('SAMPLES', json.dumps({'count':len(samples),'phases':dict(Counter(s.get('phase') for s in samples)), 'first': short(samples[0]), 'last': short(samples[-1])}, ensure_ascii=False))
        for k in ('camera_recording', 'cube_placement_photo', 'rotation_observation_photo', 'fixed_hold_observation_photo'):
            if k in d: print(k, json.dumps(short(d[k]), ensure_ascii=False))
        if samples:
            ranges = {}
            for phase in dict.fromkeys(s['phase'] for s in samples):
                phase_samples=[s for s in samples if s['phase']==phase]
                ranges[phase]={'samples':len(phase_samples),'t_first':phase_samples[0]['t'],'t_last':phase_samples[-1]['t']}
            print('PHASE_RANGES', json.dumps(ranges))
            print('HEALTH', json.dumps({'minimum_voltage_v':min(min(s['voltage_v']) for s in samples), 'peak_absolute_current_ma':max(max(abs(v) for v in s['current_ma']) for s in samples), 'maximum_temperature_c':max(max(s['temperature_c']) for s in samples), 'maximum_tracking_rad':max(max(s['tracking_rad']) for s in samples)}))
        camera = path.parent/'camera'
        if camera.exists():
            meta=camera/'frames.json'
            if meta.exists():
                m=json.loads(meta.read_bytes())
                print('FRAME_METADATA', json.dumps({'keys':list(m) if isinstance(m,dict) else None,'value':short(m) if not isinstance(m,dict) else {k:short(v) for k,v in m.items()}},ensure_ascii=False))

if __name__ == '__main__': main()
