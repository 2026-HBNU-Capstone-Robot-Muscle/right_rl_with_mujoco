"""Single weak-response replay at5ms integrator steps; offline only."""
import numpy as np
import screen_slow_forward_20261026 as c
from slow_forward_plan_20261026 import FIXTURE
original=c.build_cad
def refined(*args,**kwargs):
    model,manifest=original(*args,**kwargs)
    manifest['native_timestep_before_refinement_s']=float(model.opt.timestep)
    model.opt.timestep=.005
    manifest['refined_simulation_timestep_s']=.005
    return model,manifest
def main():
    c.build_cad=refined;c.FRONTS=(-10.,);c.SCALES=(.5,);c.offsets=lambda:[np.zeros(16,dtype=int)]
    c.main(FIXTURE/'request.json',FIXTURE/'weak_refined_timestep_diagnostic.json')
if __name__=='__main__':main()
