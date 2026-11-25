import json
import unittest
from pathlib import Path
from path_progress_guard import check_progress

class ProgressTest(unittest.TestCase):
    def rows(self):
        return [dict(t=i*.05,actual=[2000]*16,goal=[2000+i*4]*16,current_ma=[100]*16) for i in range(20)]

    def test_recorded_stall_is_detected_before_old_trip(self):
        data=json.loads((Path(__file__).resolve().parent/'leap_sac_path_empty_20261014_194239_306843.json').read_text(encoding='utf-8'))
        rows=[r for r in data['samples'] if r['phase']=='reachable_empty_path']
        caught=None
        for n in range(1,len(rows)+1):
            try:check_progress(rows[:n])
            except RuntimeError as exc:
                caught=n;self.assertIn('ID16',str(exc));break
        self.assertIsNotNone(caught);self.assertLess(caught,len(rows))
        self.assertLess(rows[caught-1]['tracking_rad'][15],.18)

    def test_following_joint_does_not_trip(self):
        rows=self.rows()
        for i,row in enumerate(rows):row['actual']=[1960+i*4]*16
        check_progress(rows)

    def test_fixed_hold_does_not_trip(self):
        rows=self.rows()
        for row in rows:row['goal']=[2080]*16
        check_progress(rows)

    def test_goal_retracting_toward_stationary_joint_does_not_trip(self):
        rows=self.rows()[:6]
        for i,row in enumerate(rows):row['goal']=[2100-i*4]*16
        check_progress(rows)

    def test_negative_direction_advancing_away_still_trips(self):
        rows=self.rows()[:6]
        for i,row in enumerate(rows):row['goal']=[1950-i*4]*16
        with self.assertRaises(RuntimeError):check_progress(rows)

    def test_reversal_in_window_is_not_sustained_advancement(self):
        rows=self.rows()[:6]
        for row,value in zip(rows,[2050,2060,2070,2080,2078,2075]):row['goal']=[value]*16
        check_progress(rows)

    def test_revision2_archived_retracting_id15_not_false_alarm(self):
        root=Path(__file__).resolve().parent
        files=sorted(root.glob('leap_sac_path_empty_2*.json'))
        evidence=None
        for path in files:
            d=json.loads(path.read_text(encoding='utf-8'))
            if d.get('timestamp')=='2026-10-14T22:06:55.799110+09:00':evidence=d;break
        self.assertIsNotNone(evidence)
        rows=[r for r in evidence['samples'] if r['phase']=='reachable_empty_path']
        for n in range(1,len(rows)+1):check_progress(rows[:n])

    def test_short_or_stale_window_does_not_trip(self):
        rows=self.rows();check_progress(rows[:3])
        for i,row in enumerate(rows):row['t']=i
        check_progress(rows)

if __name__=='__main__':unittest.main()
