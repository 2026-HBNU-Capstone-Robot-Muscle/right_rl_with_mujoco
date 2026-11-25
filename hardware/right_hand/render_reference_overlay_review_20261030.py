import json,time,hashlib
from pathlib import Path
import cv2 as cv,numpy as np
from placement_reference_overlay_20261030 import load_model,estimate,draw
R=Path(__file__).resolve().parent;out=R/'right_hand_rotation_20261030/placement_alignment_capture';model=load_model();rows=[]
for name,path in [('reference',R/model['reference_photo']),('current',R/'placement_alignment_capture_20261030_092639_869097/cube_placed.jpg')]:
 frame=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR);saved=frame.copy();began=time.perf_counter();measurement=estimate(frame,model);duration=time.perf_counter()-began;assert np.array_equal(saved,frame);preview=draw(frame,measurement);assert np.array_equal(saved,frame)
 target=out/(name+'_target_overlay.jpg');cv.imencode('.jpg',preview)[1].tofile(str(target));rows.append(dict(name=name,source=path.relative_to(R).as_posix(),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),measurement=measurement,processing_s=duration,preview=target.relative_to(R).as_posix(),raw_frame_unchanged=True))
(out/'overlay_saved_image_review.json').write_text(json.dumps(dict(rows=rows,devices_opened=False,servo_writes=0),indent=2)+'\n',encoding='utf-8');print(json.dumps([{k:r[k] for k in ('name','processing_s','measurement')} for r in rows]))
