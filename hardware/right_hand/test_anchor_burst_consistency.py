import copy
import json
from pathlib import Path
import tempfile
import unittest
import cv2 as cv
import numpy as np
from anchor_burst_consistency import screen

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'anchor_multiview_20261014_030307_761969'

class BurstTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report=json.loads((FOLDER/'result.json').read_text(encoding='utf-8'))

    def test_real_contour_failures_rejected_without_mutation(self):
        source=copy.deepcopy(self.report)
        result=screen(source,FOLDER)
        rejected=[f['image'] for f in result['screening']['frames'] if not f['accepted']]
        self.assertEqual(rejected,['view_3_2.png','view_3_5.png'])
        self.assertEqual(len(result['frames']),13)
        self.assertEqual(source,self.report)
        original={f['image']:f for f in source['frames']}
        for frame in result['frames']:self.assertEqual(frame,original[frame['image']])

    def test_small_camera_motion_is_not_a_corner_failure(self):
        frame=copy.deepcopy(self.report['frames'][0]);raw=cv.imdecode(np.fromfile(FOLDER/frame['image'],np.uint8),cv.IMREAD_GRAYSCALE)
        report=copy.deepcopy(self.report);report['frames']=[]
        with tempfile.TemporaryDirectory(dir=ROOT) as temp:
            folder=Path(temp)
            for i,shift in enumerate((0,.4,.8,1.2,1.6)):
                row=copy.deepcopy(frame);row['image']=f'{i}.png'
                for marker in row['markers'].values():
                    marker['corners_px']=(np.array(marker['corners_px'])+[shift,0]).tolist()
                image=cv.warpAffine(raw,np.array([[1,0,shift],[0,1,0]],np.float32),(raw.shape[1],raw.shape[0]))
                (folder/row['image']).write_bytes(cv.imencode('.png',image)[1].tobytes())
                report['frames'].append(row)
            result=screen(report,folder)
            self.assertEqual(len(result['frames']),5)

    def test_insufficient_burst_cannot_be_screened(self):
        report=copy.deepcopy(self.report);report['frames']=report['frames'][:2]
        with self.assertRaisesRegex(ValueError,'at least three'):screen(report,FOLDER)

if __name__=='__main__':unittest.main()
