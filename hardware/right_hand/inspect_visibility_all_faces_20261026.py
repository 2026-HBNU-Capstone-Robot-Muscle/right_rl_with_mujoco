"""Independent same-frame face detections in saved images only."""
import json
from pathlib import Path
import cv2,numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'cube_visibility_20261026_192140_408522'
def main():
    detector=MarkerDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),range(6),hand_id=41,required_cube_count=3);r=json.loads((FOLDER/'result.json').read_bytes());out=[]
    for profile in r['profiles']:
        for f in profile.get('frames',[]):
            image=cv2.imdecode(np.frombuffer((FOLDER/f['image']).read_bytes(),np.uint8),cv2.IMREAD_GRAYSCALE);co,ids,_=detector.detectMarkers(image);decoded=[] if ids is None else ids.ravel().tolist()
            row=dict(profile=profile['name'],image=f['image'],ids=decoded,corners={str(mid):np.asarray(p).reshape(4,2).tolist() for mid,p in zip(decoded,co)},stats=dict(detector.stats));out.append(row)
            print(profile['name'],f['image'],decoded,detector.stats['min_edge_px_by_id'],flush=True)
    (FOLDER/'all_face_detections.json').write_text(json.dumps(dict(frames=out,real_devices_opened=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
