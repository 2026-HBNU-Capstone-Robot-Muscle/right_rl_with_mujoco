"""Compact saved setup result, without devices."""
import json,sys
from pathlib import Path
r=json.loads(Path(sys.argv[1]).read_bytes())
out={k:v for k,v in r.items() if k not in ('before','after')}
out['before_count']=len(r.get('before',{}));out['after_count']=len(r.get('after',{}))
print(json.dumps(out,ensure_ascii=False))
