"""Small durable progress summary; no JAX or hardware imports."""
import json,time
from pathlib import Path
root=Path(__file__).resolve().parent/'sac_forward_support_reference_v1_20k'
result={'elapsed_seconds_since_settings':round(time.time()-(root/'settings.json').stat().st_mtime,1)}
for name in ('result.json','metrics.jsonl'):
    path=root/name
    if path.exists():
        doc=json.loads(path.read_text().splitlines()[-1]) if name.endswith('jsonl') else json.loads(path.read_text())
        if name.endswith('jsonl'):doc={'steps':doc['steps'],'success':doc['metrics'].get('eval/episode_forward/success'),'walltime':doc['metrics'].get('eval/walltime')}
        result[name]=doc
result['checkpoints']=[p.name for p in (root/'checkpoints').glob('[0-9]*')]
print(json.dumps(result,indent=2))
