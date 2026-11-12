"""Conservative mesh-to-box clearance, independent of the contact solver.

The mesh's AABB is computed in the obstacle's frame. Distance between that
containing AABB and the obstacle is a LOWER bound on actual surface clearance.
Zero means inconclusive/overlapping bounds, never certified free space.
"""
import numpy as np
from scipy.spatial import ConvexHull


def aabb_clearance_lower_bound(points, half_size):
    points=np.asarray(points,dtype=float); half_size=np.asarray(half_size,dtype=float)
    if not np.isfinite(points).all() or not np.isfinite(half_size).all():
        raise ValueError('Non-finite geometry')
    gaps=np.maximum(np.maximum(points.min(0)-half_size,-half_size-points.max(0)),0.)
    return float(np.linalg.norm(gaps))


class SupportSeparation:
    def __init__(self,model):
        self.model=model; self.vertices={}
        for gid in range(model.ngeom):
            if not model.geom(gid).name.endswith('_support_proxy'):continue
            mid=model.geom_dataid[gid]; start=model.mesh_vertadr[mid]; count=model.mesh_vertnum[mid]
            points=np.asarray(model.mesh_vert[start:start+count],dtype=float)
            self.vertices[gid]=points[ConvexHull(points).vertices]

    def lower_bound(self,data,gid,obstacle):
        world=self.vertices[gid]@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]
        local=(world-data.geom_xpos[obstacle])@data.geom_xmat[obstacle].reshape(3,3)
        return aabb_clearance_lower_bound(local,self.model.geom_size[obstacle])


def test_known_boxes():
    from itertools import product
    cube=np.array(list(product((-1.,1.),repeat=3)))
    assert aabb_clearance_lower_bound(cube,[1,1,1])==0
    assert np.isclose(aabb_clearance_lower_bound(cube+[3,0,0],[1,1,1]),1)
    assert np.isclose(aabb_clearance_lower_bound(cube+[3,4,0],[1,1,1]),np.sqrt(5))
    assert aabb_clearance_lower_bound(cube+[2,0,0],[1,1,1])==0
    print('4 analytic clearance cases passed')


if __name__=='__main__':test_known_boxes()
