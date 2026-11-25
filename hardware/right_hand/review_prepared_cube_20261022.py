"""Review saved placement images and encoder records. Opens no devices."""
import hashlib
import itertools
import json
from pathlib import Path
import sys

import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector

ROOT = Path(__file__).resolve().parent
RUN = ROOT / 'prepared_placement_capture_20261022_010440_125130'
OUT = ROOT / 'prepared_cube_review_20261022'
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/vision/rubik_center'))
import cube_board
from hand_registration_geometry import candidate


def marker_solutions(corners, ids, ident, side_m, K, dist):
    if ident not in ids: return []
    s=side_m/2
    op=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],np.float64)
    ip=np.asarray(corners[ids.index(ident)],np.float64).reshape(4,2)
    ok,rvs,tvs,_=cv.solvePnPGeneric(op,ip,K,dist,flags=cv.SOLVEPNP_IPPE_SQUARE)
    result=[]
    for rv,tv in zip(rvs,tvs):
        p=cv.projectPoints(op,rv,tv,K,dist)[0].reshape(4,2)
        result.append((cv.Rodrigues(rv)[0],tv.ravel(),float(np.linalg.norm(p-ip,axis=1).mean())))
    return result


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def detect(path):
    im = cv.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv.IMREAD_COLOR)
    det = MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50), range(41),
                         hand_id=41, required_cube_count=2)
    corners, ids, _ = det.detectMarkers(cv.cvtColor(im, cv.COLOR_BGR2GRAY))
    return im, corners, [] if ids is None else ids.ravel().tolist()


def main():
    OUT.mkdir(exist_ok=True)
    r = json.loads((RUN / 'result.json').read_bytes())
    assert r['status'] == 'prepared_placement_capture_complete'
    assert r['all_torque_off_verified'] and r['settings_restored'] and r['cube_removal_confirmed']
    images = {}
    for name in ('prepared_empty.jpg', 'cube_placed.jpg', 'camera/frame_00000.jpg'):
        im, cs, ids = detect(RUN / name)
        images[name] = dict(ids=ids, corners_px={str(i): np.asarray(c).reshape(4,2).tolist()
                                                for c, i in zip(cs, ids)}, sha256=sha(RUN/name))
    im, cs, ids = detect(RUN / 'cube_placed.jpg')
    with np.load(ROOT / 'camera1_720.npz') as z:
        K, dist = z['K'], z['dist']
    cube_ids = sorted(i for i in ids if 0 <= i < 6)
    solutions = []
    # Face-center markers rotate when a Rubik face is turned. Enumerate their
    # in-plane spins rather than silently assuming an old sticker orientation.
    if len(cube_ids) >= 2:
        for spins in itertools.product(range(4), repeat=len(cube_ids)):
            rot = [0]*6
            for i, s in zip(cube_ids, spins): rot[i] = s
            objs, boardids = cube_board.object_points(cube_mm=55, marker_mm=10.7,
                                                       layout='centers', face_rot=rot)
            op = np.concatenate([objs[i] for i in cube_ids]).astype(np.float64)
            ip = np.concatenate([np.asarray(cs[ids.index(i)]).reshape(4,2) for i in cube_ids]).astype(np.float64)
            ok, rv, tv = cv.solvePnP(op, ip, K, dist, flags=cv.SOLVEPNP_SQPNP)
            if not ok: continue
            rv, tv = cv.solvePnPRefineLM(op, ip, K, dist, rv, tv)
            R = cv.Rodrigues(rv)[0]
            if np.min((op @ R.T + tv.reshape(3))[:,2]) <= 0: continue
            proj = cv.projectPoints(op, rv, tv, K, dist)[0].reshape(-1,2)
            projections = {}
            for face in cube_ids:
                n, u, v = cube_board.FACES[face]
                face_obj = np.array([.0275*n + .0275*(a*u+b*v)
                                     for a,b in ((-1,1),(1,1),(1,-1),(-1,-1))])
                projections[str(face)] = cv.projectPoints(face_obj, rv, tv, K, dist)[0].reshape(4,2).tolist()
            solutions.append(dict(face_spins=rot, mean_reprojection_px=float(np.linalg.norm(proj-ip,axis=1).mean()),
                rvec=rv.ravel().tolist(), cube_center_camera_m=tv.ravel().tolist(),
                cube_rotation_camera=R.tolist(), predicted_face_corners_px=projections))
        solutions.sort(key=lambda s:s['mean_reprojection_px'])
    show = im.copy()
    if ids: cv.aruco.drawDetectedMarkers(show, cs, np.array(ids,np.int32).reshape(-1,1))
    if solutions:
        for f, pp in solutions[0]['predicted_face_corners_px'].items():
            cv.polylines(show,[np.rint(pp).astype(np.int32)],True,(0,255,255),2)
        cv.putText(show,f"Marker fit {solutions[0]['mean_reprojection_px']:.2f}px; yellow = predicted full face",(8,28),cv.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),2)
    cv.putText(show,'OFFLINE IMAGE DIAGNOSTIC; camera-to-hand calibration remains unverified',(8,52),cv.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),2)
    cv.imencode('.jpg',show)[1].tofile(str(OUT/'cube_marker_diagnostic.jpg'))
    a=np.array(r['prepared_empty_ticks']);b=np.array(r['placement_position_ticks'])
    samples=r['samples']
    report=dict(status='saved_placement_review',source=str(RUN.relative_to(ROOT)),source_sha256=sha(RUN/'result.json'),
        mapping=r['joint_mapping'],status_in_source=r['status'],all_torque_off_verified=True,settings_restored=True,
        cube_removal_confirmed=True,prepared_empty_ticks=a.tolist(),placement_position_ticks=b.tolist(),
        placement_goal_ticks=r['placement_goal_ticks'],empty_to_loaded_tick_delta=(b-a).tolist(),
        max_empty_to_loaded_tick_delta=int(np.max(abs(b-a))),
        minimum_voltage_v=min(min(x['voltage_v']) for x in samples),
        peak_current_ma=max(max(abs(i) for i in x['current_ma']) for x in samples),
        max_tracking_rad=max(max(x['tracking_rad']) for x in samples),images=images,
        cube_marker_pose_candidates=solutions[:3],cube_side_mm=55,marker_side_mm=10.7,
        wrist_marker_side_mm=27,registration_verified=False,physical_cube_rotation_verified=False,
        hardware_commands_sent=0,live_sac=False,
        limitations=['Photograph confirms placement appearance, not full 3D registration.',
                     'Raw image marker fit does not establish its transform to the hand model.',
                     'OFF endpoint still relaxes; a later run must measure the current pose.'])
    # Diagnostic only: old rigid wrist/palm marker bridge. Never install this
    # transform in a controller, or assert it remained rigid over nine days.
    old=ROOT/'sequence_vision_20261013_213113_258536/anchor_empty_preflight.png'
    if old.exists() and solutions:
        _,hc,hi=detect(old)
        _,ec,ei=detect(RUN/'prepared_empty.jpg')
        hs=marker_solutions(hc,hi,40,.0282,K,dist)
        ass=marker_solutions(hc,hi,41,.027,K,dist)
        current=marker_solutions(ec,ei,41,.027,K,dist)
        reg=candidate();Rw=np.asarray(reg['hand_to_world_candidate'])
        gravity=np.asarray(json.loads((ROOT/'sac_stage0_reset_audit_result.json').read_bytes())['gravity_m_s2'])
        gravity_up=-gravity/np.linalg.norm(gravity)
        grasp=np.asarray(reg['grasp_site_in_marker_m_candidate'])
        Rc=np.array(solutions[0]['cube_rotation_camera']);tc=np.array(solutions[0]['cube_center_camera_m'])
        hypotheses=[]
        for (ih,(Rh,th,he)),(ia,(Ra,ta,ae)),(ic,(Rnow,tnow,ne)) in itertools.product(enumerate(hs),enumerate(ass),enumerate(current)):
            Rah=Ra.T@Rh;tah=Ra.T@(th-ta)
            ph=Rah.T@(Rnow.T@(tc-tnow)-tah)
            pw=np.array([.11,0,.03])+Rw@(ph-grasp)
            Rworld=Rw@Rah.T@Rnow.T@Rc
            Rcw=Rnow@Rah@Rw.T
            tcw=tnow+Rnow@(tah+Rah@grasp)-Rcw@np.array([.11,0,.03])
            up=Rworld@cube_board.FACES[5][0]
            hypotheses.append(dict(ippe_indices=[ih,ia,ic],
                old_marker_errors_px=[he,ae],current_wrist_error_px=ne,
                cube_center_world_m=pw.tolist(),cube_rotation_world=Rworld.tolist(),
                world_to_camera_rotation=Rcw.tolist(),world_to_camera_translation=tcw.tolist(),
                green_up_error_relative_measured_gravity_deg=float(np.degrees(np.arccos(np.clip(up@gravity_up,-1,1)))),
                green_up_error_deg=float(np.degrees(np.arccos(np.clip(up[2],-1,1))))))
        report['historical_bridge_diagnostic']=dict(source=str(old.relative_to(ROOT)),source_sha256=sha(old),
            old_ids=hi,current_empty_ids=ei,hypotheses=hypotheses,
            registration_verified=False,executable=False,
            orientation_note='World Z is not the measured gravity direction. Neither green-up value certifies pose, especially with user-observed tipping.',
            assumptions=['Old wrist-to-palm mounting relation unchanged.',
                'Camera/base stayed fixed between empty and placed photo.',
                'Historical marker-to-model axis offsets are approximate.'])
    (OUT/'placement_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('status','source_sha256','minimum_voltage_v','peak_current_ma',
        'max_tracking_rad','max_empty_to_loaded_tick_delta')},indent=2))
    print('Detected IDs:',{k:v['ids'] for k,v in images.items()})
    print('Cube fits:',[(s['face_spins'],s['mean_reprojection_px']) for s in solutions[:3]])
    if 'historical_bridge_diagnostic' in report:
        print('Historical bridge (NOT calibrated):',[(h['ippe_indices'],np.round(1000*np.array(h['cube_center_world_m']),1).tolist(),
             round(h['green_up_error_relative_measured_gravity_deg'],1)) for h in report['historical_bridge_diagnostic']['hypotheses']])


if __name__=='__main__': main()
