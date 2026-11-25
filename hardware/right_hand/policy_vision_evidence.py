"""Bounded raw examples in RAM during motion; encode/write only after cleanup."""
import json
from pathlib import Path
import numpy as np


class Evidence:
    def __init__(self):
        self.examples = {}

    def remember(self, frame, pose, metrics, detector):
        category = metrics['reason'] or 'eligible'
        old = self.examples.get(category)
        score = metrics.get('reprojection_px') or 0.
        if old is not None and (category != 'reprojection_over_3px_or_invalid' or
                                score <= (old['metrics'].get('reprojection_px') or 0.)):
            return
        if old is None and len(self.examples) >= 8:
            return
        ids = getattr(detector, 'last_ids', None)
        corners = getattr(detector, 'last_corners', ())
        geometry = dict(markers=[])
        if ids is not None:
            geometry['markers'] = [dict(id=int(i), corners_px=np.asarray(c).reshape(4, 2).tolist())
                                   for i, c in zip(np.asarray(ids).ravel(), corners)]
        for key in ('R_cube', 't_cube'):
            if pose and key in pose:
                geometry[key] = np.asarray(pose[key]).tolist()
        self.examples[category] = dict(frame=frame.copy(), metrics=dict(metrics), geometry=geometry)

    def save(self, folder, cv, camera, report):
        if not self.examples:
            return
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        report['policy_evidence_folder'] = str(folder)
        saved = report['policy_evidence_images'] = []
        for category, example in self.examples.items():
            try:
                ok, encoded = cv.imencode('.png', example['frame'])
                if not ok:
                    raise RuntimeError('PNG encoding failed')
                path = folder / (category + '.png')
                path.write_bytes(encoded.tobytes())
                geometry = dict(example['geometry'], camera_matrix=camera.K.tolist(),
                                distortion=camera.dist.tolist())
                (folder / (category + '.json')).write_text(
                    json.dumps(dict(metrics=example['metrics'], geometry=geometry), indent=2),
                    encoding='utf-8')
                saved.append(str(path))
            except Exception as exc:
                report.setdefault('policy_evidence_save_errors', []).append(str(exc))
