"""Offline ID41-size sensitivity; never fits scale to a desired simulation pose."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from sac_registration_from_anchor import detect, marker_pose, candidate

ROOT = Path(__file__).resolve().parent

def main():
    audit = json.loads((ROOT/'sac_revised_placement_audit_result.json').read_text())
    folder = Path(audit['source'])
    with np.load(ROOT/'camera1_720.npz') as z: K, dist = z['K'], z['dist']
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    _, ec, ei, _ = detect(folder/'anchor_empty_preflight.png', dictionary)
    _, pc, pi, _ = detect(folder/'placement_at_space.png', dictionary)
    Rh, th, _ = marker_pose(ec, ei, 40, .0282, K, dist)
    Ra0, ta0, _ = marker_pose(ec, ei, 41, .030, K, dist)
    Ra1, ta1, _ = marker_pose(pc, pi, 41, .030, K, dist)
    # Infer the fixed camera cube position from the existing candidate at 30 mm,
    # then vary only anchor translation scale in both same-run frames.
    ph = np.asarray(audit['registration']['cube_center_in_hand_marker_m'])
    tc = ta1 + Ra1 @ Ra0.T @ (th + Rh@ph - ta0)
    geometry = candidate()
    Rwh = np.asarray(geometry['hand_to_world_candidate'])
    grasp = np.asarray(geometry['grasp_site_in_marker_m_candidate'])
    rows = []
    for side in (27., 28., 29., 30., 31., 32.):
        scale = side/30
        cube_hand = Rh.T @ (scale*ta0 + Ra0@Ra1.T@(tc-scale*ta1) - th)
        world = np.array([.11, 0, .03]) + Rwh@(cube_hand-grasp)
        rows.append(dict(anchor_side_mm=side, center_world_m=world.tolist(),
                         change_from_30mm_mm=(Rwh@(cube_hand-ph)*1000).tolist()))
    # Cube marker scaling also scales its face-plane offset in a planar solve.
    # Re-solve the actual board instead of scaling cube-center t directly.
    from sac_registration_from_anchor import cube_board
    rotations = tuple(json.loads((ROOT/'cube_face_rot_result.json').read_text())['face_rot'])
    corners = np.asarray(pc[pi.index(5)], np.float32).reshape(4, 2)
    cube_rows = []
    for side in (9.8, 10., 10.2, 10.4, 10.6, 10.8):
        objects, ids = cube_board.object_points(cube_mm=54.5, marker_mm=side, layout='centers', face_rot=rotations)
        obj = objects[list(np.asarray(ids).ravel()).index(5)]
        ok, rv, tv = cv.solvePnP(obj, corners, K, dist)
        if not ok: continue
        cube_hand = Rh.T @ (ta0+Ra0@Ra1.T@(tv.ravel()-ta1)-th)
        world = np.array([.11, 0, .03])+Rwh@(cube_hand-grasp)
        cube_rows.append(dict(cube_marker_side_mm=side, center_world_m=world.tolist(),
                              change_from_10_6mm_mm=((world-np.asarray(audit['registration']['cube_center_in_sim_world_candidate_m']))*1000).tolist()))
    result = dict(status='diagnostic_sensitivity_not_calibration', source=audit['source'],
                  motor_commands_sent=0, rows=rows, cube_marker_rows=cube_rows,
                  caveat='Uniform scale only. Does not model non-square print, planar ambiguity, intrinsic error, or mounting tilt. Do not pick a size to force agreement with simulation.')
    (ROOT/'sac_anchor_scale_audit_result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))

if __name__ == '__main__': main()
