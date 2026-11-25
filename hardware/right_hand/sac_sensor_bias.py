"""Raw 110-observation offsets for explicit camera-position bias experiments."""
import numpy as np

def position_bias_offset_mm(bias):
    bias=np.asarray(bias,dtype=np.float32)
    if bias.shape!=(3,) or not np.isfinite(bias).all():raise ValueError('Need finite camera position bias XYZ in mm')
    offset=np.zeros(110,np.float32)
    # The observation stores grasp-site position minus measured cube position.
    offset[64:73]=-np.tile(bias/1000,3)
    return offset

def position_bias_cases(mm):
    if not np.isfinite(mm) or mm<=0 or mm>100:raise ValueError('Bias sweep must be within (0,100] mm')
    return [[0.,0.,0.]]+[(np.eye(3)[axis]*sign*mm).tolist() for axis in range(3) for sign in (-1,1)]
