"""Saved-frame CPU benchmark only; never opens camera or motor interfaces."""
import json
from pathlib import Path
import statistics
import time
import cv2 as cv
import numpy as np
from empty_start_marker_check import make_empty_detector,inspect_empty_frame

root=Path(__file__).resolve().parent
frame=cv.imdecode(np.frombuffer((root/'book_prepare_empty_20261024_162948_678188/camera/frame_00000.jpg').read_bytes(),np.uint8),cv.IMREAD_COLOR)
original=cv.getNumThreads();rows=[]
try:
    for threads in dict.fromkeys((original,1)):
        cv.setNumThreads(threads);detector=make_empty_detector();times=[]
        for _ in range(25):
            began=time.perf_counter();inspect_empty_frame(frame,detector);times.append(time.perf_counter()-began)
        rows.append(dict(requested_threads=threads,actual_threads=cv.getNumThreads(),
            median_ms=statistics.median(times)*1000,maximum_ms=max(times)*1000))
finally:cv.setNumThreads(original)
out=root/'right_hand_rotation_20261024/book20_current_review/marker_cpu_benchmark.json'
out.write_text(json.dumps(rows,indent=2),encoding='utf-8');print(json.dumps(rows,indent=2))
