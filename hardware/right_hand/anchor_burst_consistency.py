"""Offline rejection of inconsistent marker corners within stationary bursts.

No corners are synthesized and no fitting toward a desired robot pose occurs.
Camera motion is assessed separately by gray-image affine ECC alignment.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent

def screen(report,folder):
    accepted=[];diagnostics=[]
    for view in sorted(set(f['view'] for f in report['frames'])):
        frames=[f for f in report['frames'] if f['view']==view]
        if len(frames)<3:raise ValueError('Need at least three frames per burst')
        points=np.array([[f['markers'][k]['corners_px'] for k in ('40','41')] for f in frames])
        # Medoid avoids introducing synthetic marker corners.
        distances=np.max(np.linalg.norm(points[:,None]-points[None,:],axis=-1),axis=(2,3))
        ref_index=int(np.argmin(np.median(distances,axis=1)))
        reference=points[ref_index]
        def read(frame):
            image=cv.imdecode(np.fromfile(folder/frame['image'],np.uint8),cv.IMREAD_GRAYSCALE)
            if image is None:raise ValueError('Missing raw image')
            return image
        ref=read(frames[ref_index])
        mask=np.zeros(ref.shape,np.uint8)
        for marker in reference:
            x,y,w,h=cv.boundingRect(marker.astype(np.float32))
            mask[max(0,y-10):y+h+10,max(0,x-10):x+w+10]=255
        rows=[]
        for frame,p in zip(frames,points):
            cc,W=cv.findTransformECC(ref,read(frame),np.eye(2,3,dtype=np.float32),cv.MOTION_AFFINE,
                                    (cv.TERM_CRITERIA_COUNT|cv.TERM_CRITERIA_EPS,80,1e-6),mask,5)
            predicted=cv.transform(reference.reshape(1,8,2).astype(np.float32),W).reshape(2,4,2)
            error=float(np.max(np.linalg.norm(predicted-p,axis=-1)))
            displacement=float(np.max(np.linalg.norm(predicted-reference,axis=-1)))
            coherent=cc>=.98 and displacement<=2. and error<=3.
            row=dict(image=frame['image'],view=view,reference_image=frames[ref_index]['image'],
                     image_correlation=float(cc),image_displacement_px=displacement,
                     corner_disagreement_px=error,accepted=coherent)
            rows.append(row)
        if sum(r['accepted'] for r in rows)<3:
            raise ValueError(f'View {view}: insufficient stationary agreement; do not auto-filter')
        accepted.extend(copy.deepcopy(f) for f,r in zip(frames,rows) if r['accepted'])
        diagnostics.extend(rows)
    output=copy.deepcopy(report);output['frames']=accepted
    output['status']='offline_burst_screened_unvalidated'
    output['screening']=dict(method='affine_image_alignment_and_corner_consistency_v1',
                             maximum_image_motion_px=2.,maximum_corner_disagreement_px=3.,
                             minimum_image_correlation=.98,minimum_consistent_frames_per_view=3,
                             source_report=str(folder/'result.json'),frames=diagnostics,
                             source_report_sha256=hashlib.sha256((folder/'result.json').read_bytes()).hexdigest() if (folder/'result.json').exists() else None,
                             source_image_sha256={f['image']:hashlib.sha256((folder/f['image']).read_bytes()).hexdigest() for f in report['frames']},
                             note='Rejects isolated contour failures, not persistent systematic corner or size bias.')
    return output

def main():
    p=argparse.ArgumentParser();p.add_argument('folder');p.add_argument('--output',default='anchor_burst_screened_input.json')
    args=p.parse_args();folder=ROOT/args.folder
    result=screen(json.loads((folder/'result.json').read_text(encoding='utf-8')),folder)
    (ROOT/args.output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result['screening'],indent=2))

if __name__=='__main__':main()
