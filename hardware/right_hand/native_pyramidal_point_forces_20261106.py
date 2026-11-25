"""Exact native MuJoCo force rays for condim3 pyramidal point contacts.

MuJoCo itself decodes each nonnegative edge basis. This avoids substituting
an elliptic cone in an optimizer while the dynamics uses a pyramid.
This module changes neither the contact engine nor any model parameters.
"""
from functools import lru_cache
import numpy as np,mujoco

@lru_cache(maxsize=32)
def point_rays(mu_x,mu_y):
    if not np.isfinite([mu_x,mu_y]).all() or min(mu_x,mu_y)<=0:raise ValueError('positive finite sliding friction coefficients required')
    # The Python interface validates force length=mu length+1 and edge
    # length=2*mu length. Use a padded4/3/6 buffer and only the first four
    # point-force rays. The unused coefficient, edges and output stay zero.
    # Reference: google-deepmind/mujoco python/mujoco/functions.cc.
    friction=np.array([mu_x,mu_y,0.]);columns=[]
    for index in range(4):
        edge=np.zeros(6);edge[index]=1.;force=np.zeros(4);mujoco.mju_decodePyramid(force,edge,friction);columns.append(force[:3].copy())
    result=np.column_stack(columns);result.flags.writeable=False;return result

def pyramid_margin(force,mu_x,mu_y):
    f=np.asarray(force,float)
    if f.shape!=(3,) or not np.isfinite(f).all():raise ValueError('finite normal and two tangent force components required')
    point_rays(float(mu_x),float(mu_y))
    return float(f[0]-abs(f[1])/mu_x-abs(f[2])/mu_y)
