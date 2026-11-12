import json
from pathlib import Path
from collections import Counter
r=Path(__file__).resolve().parent
d=json.loads((r/'middle_mapping_review_20261018/path_audit.json').read_text())
for key in ('historical_commands_interpreted_with_confirmed_mapping','original_intent_with_corrected_encoding','measured_backoff_interpreted_with_confirmed_mapping'):
    rows=d[key]['rows'];pairs=Counter(tuple(sorted((c['a'],c['b']))) for row in rows for c in row['contacts'])
    limits=Counter(j for row in rows for j in row['outside_limits'])
    print(key,'limits',dict(limits),'contacts',pairs.most_common(10))
    middle=[(row['index'],c) for row in rows for c in row['contacts'] if c['a'].startswith('mf_') or c['b'].startswith('mf_')]
    print('middle_contact_samples',len(set(x[0] for x in middle)),'first',middle[:2])
