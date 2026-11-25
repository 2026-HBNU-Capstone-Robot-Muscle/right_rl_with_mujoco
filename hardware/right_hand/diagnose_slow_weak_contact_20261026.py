"""Identify the exact tiny contact in the slow-response diagnostic, no devices."""
import json
import numpy as np
import screen_slow_forward_20261026 as c
from slow_forward_plan_20261026 import FIXTURE
original=c.cached_contacts
events=[]
def logged(m,hand,support):
    inspect=original(m,hand,support)
    def examine(d):
        a,b=inspect(d)
        hits={p:v for p,v in b.items() if abs(v-.004853874956628179)<1e-6}
        if d.time>.5 and hits and len(events)<20:
            events.append(dict(time=d.time,contacts=hits,sim_qpos=d.qpos[:16].tolist()))
        return a,b
    return examine
def main():
    c.cached_contacts=logged;c.FRONTS=(-10.,);c.SCALES=(.5,);c.offsets=lambda:[np.zeros(16,dtype=int)]
    try:c.main(FIXTURE/'request.json',FIXTURE/'weak_contact_diagnostic.json')
    finally:
        (FIXTURE/'weak_contact_events.json').write_text(json.dumps(events,indent=2),encoding='utf-8')
        print('CONTACT EVENTS',json.dumps(events))
if __name__=='__main__':main()
