"""Select a cube's working top face without assuming GREEN stays up.

Pure geometry. This does not estimate a physical pose or prove any motion.
Face IDs retain the existing centre-marker configuration0..5.
"""
import numpy as np
from cube_orientation_routes_20261028 import NAMES,NORMALS

def rotation(value):
    B=np.asarray(value,float)
    if B.shape!=(3,3) or not np.isfinite(B).all() or not np.allclose(B.T@B,np.eye(3),atol=1e-6,rtol=0) or not np.isclose(np.linalg.det(B),1.,atol=1e-6,rtol=0):raise ValueError('proper finite3x3 rotation required')
    return B

def normal(face_id):
    if isinstance(face_id,(bool,np.bool_)) or not isinstance(face_id,(int,np.integer)) or not 0<=int(face_id)<6:raise ValueError('explicit configured face ID0..5 required')
    return NORMALS[int(face_id)].astype(float)

def local_working_basis(face_id):
    n=normal(face_id)
    if int(face_id)==5:return np.eye(3)
    y=-n;x=next(v for v in np.eye(3) if abs(v@n)<.5);z=np.cross(x,y)
    return np.column_stack([x,y,z])

def working_rotation(cube_rotation,face_id=5):
    B=rotation(cube_rotation)
    return B if int(face_id)==5 and np.array_equal(normal(face_id),[0,-1,0]) else B@local_working_basis(face_id)

def face_up_error_deg(cube_rotation,face_id,world_up):
    B=rotation(cube_rotation);u=np.asarray(world_up,float)
    if u.shape!=(3,) or not np.isfinite(u).all() or np.linalg.norm(u)<1e-12:raise ValueError('finite nonzero up direction required')
    return float(np.degrees(np.arccos(np.clip((B@normal(face_id))@(u/np.linalg.norm(u)),-1,1))))

def aligned_top_face(cube_rotation,world_up,maximum_tilt_deg=5.):
    if not 0<=maximum_tilt_deg<45:raise ValueError('unambiguous top-face tolerance required')
    errors=[face_up_error_deg(cube_rotation,i,world_up) for i in range(6)];best=int(np.argmin(errors));accepted=errors[best]<=maximum_tilt_deg
    return dict(face_id=best if accepted else None,face_name=NAMES[best] if accepted else None,best_candidate_id=best,tilt_deg=errors[best],orientation_motion_not_certified=True)

def orientation_error_deg(cube_rotation,target_rotation):
    B,T=rotation(cube_rotation),rotation(target_rotation)
    return float(np.degrees(np.arccos(np.clip((np.trace(T.T@B)-1)/2,-1,1))))

def upright_yaw_target(initial_rotation,world_up,yaw_deg,top_face_id=5):
    """Remove initial placement tilt before applying a yaw about gravity up.

    Rotating a tipped initial matrix around up preserves its tip. Aligning its
    selected normal by a minimal rotation first adds no twist about world up.
    This defines a target only; it never modifies an engine state.
    """
    B=rotation(initial_rotation);up=np.asarray(world_up,float)
    if up.shape!=(3,) or not np.isfinite(up).all() or np.linalg.norm(up)<1e-12 or not np.isfinite(yaw_deg):raise ValueError('finite up and yaw required')
    up=up/np.linalg.norm(up);n=B@normal(top_face_id);c=float(n@up)
    if c<-.999999:raise ValueError('antipodal top alignment is ambiguous')
    def skew(v):return np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])
    A=skew(np.cross(n,up));align=np.eye(3)+A+A@A/(1+c);U=skew(up);a=np.radians(yaw_deg);yaw=np.eye(3)+np.sin(a)*U+(1-np.cos(a))*(U@U)
    return rotation(yaw@align@B)
