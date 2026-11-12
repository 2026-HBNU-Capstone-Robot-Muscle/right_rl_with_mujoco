"""Locate the native weak-response contact; never opens devices."""
import json
import numpy as np
import screen_thumb_clear_root_empty_20261026 as c
from thumb_clear_root_empty_plan_20261026 import FIXTURE
original=c.cached_contacts
events=[];peaks={}
def logged(m,hand,support):
    inspect=original(m,hand,support)
    def examine(d):
        a,b=inspect(d);hits={p:v for p,v in b.items() if v>.010001}
        if d.time>5 and hits:
            for p,v in hits.items():
                if v>peaks.get(p,{}).get('depth_mm',0):peaks[p]=dict(depth_mm=v,time_s=float(d.time))
            if len(events)<8:events.append(dict(time_s=float(d.time),contacts=hits))
        return a,b
    return examine
def main():
    c.cached_contacts=logged;c.FRONTS=(-10.,);c.SCALES=(.5,);c.offsets=lambda:[np.zeros(16,dtype=int)]
    try:c.main(FIXTURE/'request.json',FIXTURE/'weak_diagnostic.json')
    finally:
        out=dict(first_contacts=events,peaks=peaks)
        (FIXTURE/'weak_contact_events.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print('EVENTS',json.dumps(out),flush=True)
if __name__=='__main__':main()
