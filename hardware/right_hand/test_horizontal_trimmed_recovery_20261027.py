"""Regression checks for the two actual undervoltage approach starts."""
import json,tempfile,unittest
from pathlib import Path
import numpy as np
import horizontal_quarter_plan_20261027 as p

class TrimmedRecoveryTests(unittest.TestCase):
    def test_both_recorded_starts_avoid_the_old_2150_index_excursion(self):
        for name in ('horizontal_quarter_trial_20261027_152511_177363','horizontal_quarter_trial_20261027_152632_508556'):
            with self.subTest(source=name):
                record=json.loads((p.ROOT/name/'result.json').read_bytes());request=json.loads(p.REQUEST.read_bytes());request['off_state']=record['initial']
                with tempfile.TemporaryDirectory(prefix='trimmed_recovery_replay_',dir=p.ROOT) as temp:
                    path=Path(temp)/'request.json';path.write_text(json.dumps(request),encoding='utf-8');plan=p.HorizontalQuarterPlan(path);rows=plan.plan(plan.source_position())
                root_goals=np.array([q for phase,q in rows if phase==p.ROOTS]);self.assertEqual(int(max(root_goals[:,7])),2064)
                self.assertEqual(int(max(root_goals[:,11])),2064)
                self.assertLessEqual(int(max(abs(np.diff(np.vstack([plan.center]+[q for _,q in rows]),axis=0)).ravel())),2)
                opened=[q for phase,q in rows if phase==p.OPEN][-1]
                self.assertTrue(np.array_equal(opened,np.load(p.OPEN_PATH)['open_goal']))
                self.assertTrue(np.array_equal([q for phase,q in rows if phase not in p.PREFIX_PHASES],np.load(p.LOADED_PATH)['goals']))

if __name__=='__main__':unittest.main(verbosity=2)
