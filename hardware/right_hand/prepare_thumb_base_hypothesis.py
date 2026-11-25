"""Offline ONLY: test a small manual thumb-base adjustment, not live input."""
import hashlib
import json
import sys
from pathlib import Path
root=Path(__file__).resolve().parent
folder=root/'book_settled_off_readonly_20261024_174424_563949'
source=folder/'request.json';data=source.read_bytes();request=json.loads(data)
request.update(origin='HYPOTHETICAL_manual_thumb_base_adjustment_NOT_observed',
    hardware_execution_allowed=False,original_request_sha256=hashlib.sha256(data).hexdigest(),
    hypothetical_change=dict(motor_id=4,from_ticks=1782,to_ticks=1882,approximately_degrees=8.7890625))
request['off_state']['position'][3]=1882
if '--ring-clearance' in sys.argv:
    request['off_state']['position'][14]-=80
    request['hypothetical_ring_change']=dict(motor_id=15,from_ticks=2797,to_ticks=2717,approximately_degrees=-7.03125)
out=folder/('hypothetical_thumb_ring_adjustment_request.json' if '--ring-clearance' in sys.argv else 'hypothetical_thumb_base_1882_request.json')
out.write_text(json.dumps(request,indent=2),encoding='utf-8')
print(str(out))
