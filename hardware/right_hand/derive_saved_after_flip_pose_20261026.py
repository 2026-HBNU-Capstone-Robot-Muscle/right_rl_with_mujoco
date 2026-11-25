"""Approximate metric end position from validated saved red-face detections."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from sac_registration_from_anchor import marker_pose
from center_marker_cube_pose_20261026 import FACE_BASES
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/saved_marker_registration'
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--outline-red-calibration',action='store_true');parser.add_argument('--red-marker-mm',type=float);args=parser.parse_args();outline=args.outline_red_calibration;mount=json.loads((ROOT/'cube_red_green_20261026_215059_362527/outer_face_marker_diagnostic.json').read_bytes())['faces']['2'] if outline else None
    red_size_mm=args.red_marker_mm if args.red_marker_mm is not None else (mount['side_mean_mm'] if outline else 10.7)
    if not 5<=red_size_mm<=20:raise ValueError('Diagnostic marker size out of bounds')
    offset_marker=np.r_[mount['marker_frame_center_offset_mm'],0.]*.001 if outline else np.zeros(3)
    source=json.loads((OUT/'saved_red_marker_recovery.json').read_bytes());reg=json.loads((OUT/'conditional_raw_frame_start_pose.json').read_bytes());model=json.loads((OUT/'model_coordinate_frames.json').read_bytes());Rh=np.asarray(reg['R_anchor_marker40']);th=np.asarray(reg['t_anchor_marker40_m']);Rpm=np.asarray(reg['R_palm_marker40']);mp=np.asarray(reg['marker40_center_palm_m']);Rwp=np.asarray(model['palm_world_rotation']);wp=np.asarray(model['palm_world_position']);up=np.asarray(reg['table_green_normal_in_anchor']);cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist'];rows=[]
    for row in source['frames_with_red_and_anchor']:
        choices=[]
        for method in row['methods']:
            # Keep exact 16-bit identities. Border glare is recorded separately;
            # allow at most 3/20 affected border cells in this offline hypothesis.
            if method['sampled_code_bit_errors'] or method['sampled_border_bit_errors']>3 or method['red_context_fraction']<.1:continue
            Ra,ta,ae=marker_pose([method['corners']['41']],[41],41,.027,K,dist)
            if ae>1:continue
            for p in method['red_plane_candidates']:
                if p['reprojection_rms_px']>1:continue
                n=np.asarray(p['normal']);na=Ra.T@n;error=float(np.degrees(np.arccos(np.clip(na@up,-1,1))))
                if error>25:continue
                Rs=np.asarray(p['R']);cam=np.asarray(p['t'])*(red_size_mm/10.7)-n*.0275-Rs@offset_marker;anchor=Ra.T@(cam-ta);marker=Rh.T@(anchor-th);palm=mp+Rpm@marker;world=wp+Rwp@palm
                extra={}
                if outline:
                    normal,U,V=(np.asarray(v,dtype=float) for v in FACE_BASES[2]);angle=np.radians(mount['marker_angle_in_face_deg']);u=np.cos(angle)*U+np.sin(angle)*V;v=-np.sin(angle)*U+np.cos(angle)*V;Rc=Rs@np.column_stack([u,v,normal]).T;Rp=Rpm@Rh.T@Ra.T@Rc
                    extra=dict(R_cube_palm=Rp.tolist(),R_cube_world=(Rwp@Rp).tolist(),green_wrist_error_deg=float(np.degrees(np.arccos(np.clip((-Rp[:,1])@np.array([-1.,0.,0.]),-1,1)))))
                choices.append(dict(image=row['image'],time_from_end_s=row['time_from_end_s'],threshold_constant=method['threshold_constant'],sampled_code_bit_errors=0,sampled_border_bit_errors=method['sampled_border_bit_errors'],red_context_fraction=method['red_context_fraction'],anchor_error_px=float(ae),red_error_px=p['reprojection_rms_px'],red_up_error_deg=error,center_marker40_mm=(marker*1000).tolist(),center_palm_mm=(palm*1000).tolist(),center_world_m=world.tolist(),red_normal_palm=(Rpm@Rh.T@na).tolist(),**extra))
        if choices:rows.append(min(choices,key=lambda v:v['anchor_error_px']+v['red_error_px']))
    centers=np.asarray([v['center_palm_mm'] for v in rows]);result=dict(status='saved_after_flip_single_face_metric_position_hypothesis',source_frames=rows,qualified_frames=len(rows),
        median_center_palm_mm=np.median(centers,axis=0).tolist() if len(rows) else None,center_peak_to_peak_mm=np.ptp(centers,axis=0).tolist() if len(rows) else None,
        minimum_and_maximum_red_up_deg=[min(v['red_up_error_deg'] for v in rows),max(v['red_up_error_deg'] for v in rows)] if rows else None,
        red_face_size_mm_assumed=red_size_mm,red_marker_frame_center_offset_m=offset_marker.tolist(),red_marker_geometry_from_manual_outer_face_annotation=outline,red_marker_reference_quarter_assumed_unchanged=outline,maximum_glare_affected_border_cells=3,total_border_cells=20,exact_16_code_bits_required=True,known_red_up_used_to_select_planar_pose_branch=True,whole_orientation_not_independently_certified=True,hand_registration_approximate=True,real_devices_opened=False,hardware_execution_allowed=False)
    name='saved_after_flip_outline_calibrated_hypothesis.json' if outline else 'saved_after_flip_metric_position.json'
    if args.red_marker_mm is not None:name=f'saved_after_flip_red_{red_size_mm:.1f}mm'+('_offset_hypothesis' if outline else '_centered')+'.json'
    (OUT/name).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in result.items() if k!='source_frames'},indent=2),flush=True)
if __name__=='__main__':main()
