"""Independent Euclidean clearance for two oriented boxes (metres).

Positive separation: vertex/box and edge/edge feature distances. Overlap:
returns zero and a separate SAT overlap indicator, never an invented depth.
"""
import itertools
import numpy as np

SIGNS = np.array(list(itertools.product((-1., 1.), repeat=3)))
EDGES = np.array([(i, j) for i in range(8) for j in range(i + 1, 8)
                  if np.count_nonzero(SIGNS[i] != SIGNS[j]) == 1])

def clearance(ca, ra, ha, cb, rb, hb):
    ca, ra, ha, cb, rb, hb = map(np.asarray, (ca, ra, ha, cb, rb, hb))
    va = ca + (SIGNS * ha) @ ra.T
    vb = cb + (SIGNS * hb) @ rb.T
    axes = [*ra.T, *rb.T, *(np.cross(a, b) for a in ra.T for b in rb.T)]
    gaps = []
    for axis in axes:
        norm = np.linalg.norm(axis)
        if norm < 1e-10:
            continue
        axis = axis / norm
        gaps.append(abs(np.dot(cb-ca, axis)) - np.sum(ha * abs(ra.T @ axis)) - np.sum(hb * abs(rb.T @ axis)))
    if max(gaps) <= 1e-12:
        return 0., True

    def vertex_box(vertices, c, r, h):
        local = (vertices - c) @ r
        return np.min(np.sum((local - np.clip(local, -h, h)) ** 2, axis=1))

    best = min(vertex_box(va, cb, rb, hb), vertex_box(vb, ca, ra, ha))
    # Endpoint-to-segment candidates and unconstrained interior/interior minima.
    p = va[EDGES[:, 0]][:, None, :]
    q = vb[EDGES[:, 0]][None, :, :]
    u = (va[EDGES[:, 1]] - va[EDGES[:, 0]])[:, None, :]
    v = (vb[EDGES[:, 1]] - vb[EDGES[:, 0]])[None, :, :]
    w = p - q
    a = np.sum(u*u, axis=2); b = np.sum(u*v, axis=2); c = np.sum(v*v, axis=2)
    d = np.sum(u*w, axis=2); e = np.sum(v*w, axis=2)
    candidates = []
    for s in (0., 1.):
        t = np.clip((e+s*b)/c, 0., 1.)
        candidates.append(np.sum((w+s*u-t[..., None]*v)**2, axis=2))
    for t in (0., 1.):
        s = np.clip((t*b-d)/a, 0., 1.)
        candidates.append(np.sum((w+s[..., None]*u-t*v)**2, axis=2))
    det = a*c-b*b
    valid = det > 1e-20
    s = np.divide(b*e-c*d, det, out=np.zeros_like(det), where=valid)
    t = np.divide(a*e-b*d, det, out=np.zeros_like(det), where=valid)
    valid &= (s >= 0) & (s <= 1) & (t >= 0) & (t <= 1)
    interior = np.sum((w+s[..., None]*u-t[..., None]*v)**2, axis=2)
    candidates.append(np.where(valid, interior, np.inf))
    return float(np.sqrt(min(best, *(np.min(x) for x in candidates)))), False
