from pathlib import Path
import argparse,json
import cv2,numpy as np
ROOT=Path(__file__).resolve().parent;parser=argparse.ArgumentParser();parser.add_argument('--folder');args=parser.parse_args();folder=ROOT/args.folder if args.folder else ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2';folder.resolve().relative_to(ROOT);frames=sorted((folder/'movie_frames').glob('frame_*.jpg'));assert frames
out=folder/'model_yaw_diagnostic_3x.mp4';writer=cv2.VideoWriter(str(out),cv2.VideoWriter_fourcc(*'mp4v'),10,(900,660));assert writer.isOpened()
for frame in frames:writer.write(cv2.imdecode(np.fromfile(frame,np.uint8),cv2.IMREAD_COLOR))
writer.release();capture=cv2.VideoCapture(str(out));assert capture.isOpened();ok,first=capture.read();assert ok and first is not None;metadata=dict(video=out.relative_to(ROOT).as_posix(),frames=int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),fps=capture.get(cv2.CAP_PROP_FPS),width=int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),height=int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),model_only=True,physical_yaw_verified=False);capture.release();assert metadata['frames']==len(frames);(folder/'video_review.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8');print(json.dumps(metadata))
