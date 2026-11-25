"""Fork existing reviewed gates for a separate EMPTY-only full-CAD trial."""
import json,hashlib,ast
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def replace_once(text,a,b):
    if text.count(a)!=1:raise ValueError('Expected unique source template: '+a[:80])
    return text.replace(a,b)

def main():
    old=(ROOT/'screen_book_grasp_trial.py').read_text(encoding='utf-8')
    screen=replace_once(old,'def cached_contacts(m,hand,support):',
        'from fingertip_cad_model_20261025 import build as build_cad\n'
        'from reverse_start_empty_plan_20261025 import ReverseStartPlan as BookGraspPlan,extra_hashes,END_HOLD\n'
        'LOADED_PHASE=END_HOLD\nSCALES=(.5,1.,1.5)\n\n\ndef cached_contacts(m,hand,support):')
    screen=replace_once(screen,"m,manifest=build_book(env,out.with_name('book_grasp_scene.xml'),book_projection_mm=front,\n            book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)",
        "m,manifest=build_cad(env,out.with_name('reverse_empty_CAD_scene.xml'),book_projection_mm=front)")
    screen=replace_once(screen,'for scale in (.5,1.,1.5):','for scale in SCALES:')
    screen=replace_once(screen,'accepted=unchanged and extra_unchanged and all(passed(s) for s in static) and all(passed(s,True) for s in dynamic)',
        'accepted=unchanged and extra_unchanged and len(static)==201 and len(dynamic)==9 and all(passed(s) for s in static) and all(passed(s,True) for s in dynamic)')
    screen=replace_once(screen,'accepted_for_supervised_empty_preparation=accepted,accepted_for_supervised_grasp_approach=accepted,',
        'accepted_for_supervised_empty_preparation=accepted,accepted_for_supervised_grasp_approach=False,\n        empty_hand_only=True,full_CAD_tip_contacts=True,cube_disabled_entire_screen=True,')
    screen=replace_once(screen,'cube_contact_verified=False,placement_checkpoint_required=True,','cube_contact_verified=False,placement_checkpoint_required=False,')
    screen=replace_once(screen,'initial_thumb_contact_operator_observed=True,','initial_thumb_contact_operator_observed=False,')
    screen=screen.replace('Explicit PLACE/REMOVE checkpoints. Cube contact and physical grasp remain unverified.',
        'EMPTY-only entry, no cube placement checkpoint, no loaded motion or live policy. Full CAD fingertip and middle/ring housing checks.')
    screen=replace_once(screen,"args=parser.parse_args();raise SystemExit(main(args.request,args.output))",
        "parser.add_argument('--diagnostic-single',action='store_true')\n    args=parser.parse_args()\n"
        "    if args.diagnostic_single:\n        FRONTS=(-10.,);SCALES=(1.,);offsets=lambda:[np.zeros(16,dtype=int)]\n"
        "    raise SystemExit(main(args.request,args.output))")
    (ROOT/'screen_reverse_start_empty_20261025.py').write_text(screen,encoding='utf-8')
    gate=(ROOT/'book_grasp_trial_gate.py').read_text(encoding='utf-8')
    tree=ast.parse((ROOT/'fingertip_cad_model_20261025.py').read_text(encoding='utf-8'))
    revision=next(ast.literal_eval(node.value) for node in tree.body if isinstance(node,ast.Assign)
        and any(isinstance(target,ast.Name) and target.id=='REVISION' for target in node.targets))
    gate=replace_once(gate,'def screen_failure_details(path):',
        'from reverse_start_empty_plan_20261025 import extra_hashes\nREVISION='+repr(revision)+'\n\n\ndef screen_failure_details(path):')
    gate=replace_once(gate,"checker=ROOT/'screen_book_grasp_trial.py'","checker=ROOT/'screen_reverse_start_empty_20261025.py'")
    gate=replace_once(gate,"required={'accepted_for_supervised_grasp_approach':True,'code_unchanged':True,",
        "required={'accepted_for_supervised_empty_preparation':True,'accepted_for_supervised_grasp_approach':False,\n"
        "        'empty_hand_only':True,'full_CAD_tip_contacts':True,'cube_disabled_entire_screen':True,'code_unchanged':True,")
    gate=replace_once(gate,"'cube_contact_verified':False,'placement_checkpoint_required':True,","'cube_contact_verified':False,'placement_checkpoint_required':False,")
    gate=replace_once(gate,"'physical_calibration_verified':False,'initial_thumb_contact_operator_observed':True}",
        "'physical_calibration_verified':False,'initial_thumb_contact_operator_observed':False}")
    gate=replace_once(gate,"static=r.get('static_screens',[]);dynamic=r.get('dynamic_sensitivity',[])",
        "manifests=r.get('model_manifests',[])\n    if len(manifests)!=3 or any(\n"
        "        s.get('fingertip_collision_model')!=REVISION or not s.get('CAD_collision_and_visual_compiled_transforms_equal')\n"
        "        or len(s.get('middle_ring_proxies',[]))!=10 for s in manifests):\n"
        "        raise RuntimeError('Incomplete full CAD geometry evidence; no writes')\n"
        "    static=r.get('static_screens',[]);dynamic=r.get('dynamic_sensitivity',[])")
    (ROOT/'reverse_start_empty_gate_20261025.py').write_text(gate,encoding='utf-8')
    cmd='@echo off\nchcp 65001 >nul\ncd /d "%~dp0"\n"%~dp0.runtime\\python\\python.exe" -X utf8 -B "%~dp0leap_reverse_start_empty_20261025.py" --execute\npause\n'
    (ROOT/'leap_reverse_start_empty.cmd').write_bytes(cmd.replace('\n','\r\n').encode('utf-8'))
    from reverse_start_empty_plan_20261025 import ReverseStartPlan
    from hand_motor_mapping import SIM_TO_MOTOR_IDS
    source=ROOT/'reverse_start_readonly_20261025_040743_585738/result.json';actual=json.loads(source.read_bytes())
    folder=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/fixture_v2';folder.mkdir(exist_ok=True)
    request=dict(plan_version=ReverseStartPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=actual['off_state'],
        development_fixture_only=True,actual_readonly_source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    (folder/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
    plan=ReverseStartPlan(folder/'request.json');print('EMPTY_ENTRY_SOURCES_READY',len(plan.plan(plan.source_position())))

if __name__=='__main__':main()
