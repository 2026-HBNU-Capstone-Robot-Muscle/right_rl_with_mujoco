"""Separate proof and runner for an interrupted, source-verified continuation."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def once(s,a,b):
    if s.count(a)!=1:raise ValueError('Ambiguous resume template: '+a[:80])
    return s.replace(a,b)

def main():
    screen=(ROOT/'screen_reverse_quarter_path_empty_20261025.py').read_text(encoding='utf-8')
    screen=once(screen,'SCALES=(.5,1.,1.5)',
        'from reverse_quarter_resume_empty_plan_20261025 import ReverseQuarterResumeEmptyPlan as BookGraspPlan,extra_hashes,resume_goals as motion_goals,EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE\n'
        'RING_SELF_PAIRS=RING_SELF_PAIRS+(EXTRA_RING_PAIR,)\nSELF_PAIRS=SELF_PAIRS+(EXTRA_RING_PAIR,)\nSCALES=(.5,1.,1.5)')
    screen=screen.replace("if self_pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM:",
        "if self_pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM or self_pairs.get(EXTRA_RING_PAIR,0.)>EXTRA_RING_INITIAL_LIMIT_MM:")
    screen=screen.replace('OFF casing contact exceeds the dedicated 1mm model bound','OFF initial casing/ring contact exceeds its separately recorded model bound')
    screen=once(screen,'if pair not in initial_bound:', 'if pair not in nominal_initial_support:')
    screen=once(screen,'initial_bound[pair]=depth','initial_bound[pair]=max(initial_bound.get(pair,0.),depth)')
    screen=once(screen,'full_quarter_reference_path=True', 'full_quarter_reference_path=False,complete_recovery_and_suffix_checked=True')
    screen=once(screen,'no_contact_outside_reviewed_initial_envelope=True,',
        'no_contact_outside_reviewed_initial_envelope=True,interrupted_suffix_verified=True,initial_ring_neighbor_limit_mm=EXTRA_RING_INITIAL_LIMIT_MM,')
    (ROOT/'screen_reverse_quarter_resume_empty_20261025.py').write_text(screen,encoding='utf-8')
    gate=(ROOT/'reverse_quarter_path_empty_gate_20261025.py').read_text(encoding='utf-8')
    gate=once(gate,'from reverse_quarter_path_empty_plan_20261025 import extra_hashes,START_HOLD,TURN_PHASE,motion_goals',
        'from reverse_quarter_resume_empty_plan_20261025 import extra_hashes,START_HOLD,TURN_PHASE,resume_goals as motion_goals,EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE')
    # The later parent import must not replace this versioned scope.
    gate=once(gate,'from reverse_place_capture_plan_20261025 import BASE_RELEASE,CASING_PAIR,CASING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE',
        'from reverse_place_capture_plan_20261025 import BASE_RELEASE,CASING_PAIR,CASING_INITIAL_LIMIT_MM')
    gate=once(gate,'SELF_PAIRS=SELF_PAIRS+(CASING_PAIR,)',
        'SELF_PAIRS=SELF_PAIRS+(CASING_PAIR,EXTRA_RING_PAIR)')
    gate=once(gate,"'full_quarter_reference_path':True,","'full_quarter_reference_path':True,'interrupted_suffix_verified':True,")
    gate=once(gate,"or r.get('initial_support_scope')!=INITIAL_SUPPORT_SCOPE",
        "or r.get('initial_support_scope')!=INITIAL_SUPPORT_SCOPE or r.get('initial_ring_neighbor_limit_mm')!=EXTRA_RING_INITIAL_LIMIT_MM")
    gate=once(gate,"if kind=='self' and pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM:return False",
        "if kind=='self' and (pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM or pairs.get(EXTRA_RING_PAIR,0.)>EXTRA_RING_INITIAL_LIMIT_MM):return False")
    gate=once(gate,"checker=ROOT/'screen_reverse_quarter_path_empty_20261025.py'","checker=ROOT/'screen_reverse_quarter_resume_empty_20261025.py'")
    gate=once(gate,"'full_quarter_reference_path':True", "'full_quarter_reference_path':False,'complete_recovery_and_suffix_checked':True")
    gate=once(gate,"bound=envelope_bound(front);e=s.get('passive_enable_hold_support',{})",
        "bound=envelope_bound(front);e=s.get('passive_enable_hold_support',{});nominal=s.get('nominal_initial_support_contacts_mm',{})")
    gate=once(gate,"depth>(bound[pair] if pair in bound else .15)+1e-6", "depth>(bound[pair] if pair in nominal else .15)+1e-6")
    gate=once(gate,"if pair not in bound:bound[pair]=depth", "if pair not in nominal:bound[pair]=max(bound.get(pair,0.),depth)")
    gate=once(gate,"except (OSError,KeyError,ValueError,TypeError):return 'no readable complete book grasp screen'",
        "except (OSError,KeyError,ValueError,TypeError):\n"
        "        try:\n            lines=Path(path).with_suffix('.log').read_text(encoding='utf-8',errors='replace').splitlines()\n"
        "            error=next(line for line in reversed(lines) if line.startswith(('ValueError:','RuntimeError:')))\n"
        "            return 'initial-state/path check: '+error[:240]\n"
        "        except (OSError,StopIteration):return 'no complete recovery screen; no writes'")
    (ROOT/'reverse_quarter_resume_empty_gate_20261025.py').write_text(gate,encoding='utf-8')
    cmd='@echo off\nchcp 65001 >nul\ncd /d "%~dp0"\n"%~dp0.runtime\\python\\python.exe" -X utf8 -B "%~dp0leap_reverse_quarter_resume_empty_20261025.py" --execute\npause\n'
    (ROOT/'leap_reverse_quarter_resume_empty.cmd').write_bytes(cmd.replace('\n','\r\n').encode('utf-8'))
    from reverse_quarter_resume_empty_plan_20261025 import ReverseQuarterResumeEmptyPlan
    from hand_motor_mapping import SIM_TO_MOTOR_IDS
    source=json.loads((ROOT/'right_hand_rotation_20261024/reverse_quarter_resume_empty/actual_off_source.json').read_bytes())
    folder=ROOT/'right_hand_rotation_20261024/reverse_quarter_resume_empty/fixture_v4';folder.mkdir(exist_ok=True)
    request=dict(plan_version=ReverseQuarterResumeEmptyPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
        off_state=source['off_state'],development_fixture_only=True,actual_off_source=source['source'],actual_off_source_sha256=source['source_sha256'])
    (folder/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
    p=ReverseQuarterResumeEmptyPlan(folder/'request.json');rows=p.plan(p.source_position());print('RESUME_GOALS',len(rows),'nominal_seconds',(len(rows)+20)*.05)

if __name__=='__main__':main()
