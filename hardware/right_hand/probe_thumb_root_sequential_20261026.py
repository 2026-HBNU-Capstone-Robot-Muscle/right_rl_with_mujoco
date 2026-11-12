"""Offline diagnostic only: worst response in one nominal geometry case."""
import numpy as np
import screen_thumb_clear_root_empty_20261026 as c
from thumb_clear_root_empty_plan_20261026 import FIXTURE
if __name__=='__main__':
    c.FRONTS=(-10.,);c.SCALES=(.5,);c.offsets=lambda:[np.zeros(16,dtype=int)]
    raise SystemExit(c.main(FIXTURE/'request.json',FIXTURE/'weak_diagnostic.json'))
