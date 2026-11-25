import json,re,hashlib,ast
from pathlib import Path
from placement_alignment_plan_20261030 import ROOT as R,OUT,PROOF,REQUEST,PlacementAlignmentPlan,FILES
from placement_alignment_gate_20261030 import validate_review
expected={
 'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
 'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
 'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
 'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d',
 'horizontal_quarter_motion_runner_20261027.py':'5c8487f134264f5ef4eb9f6bdf6a465e5fb6c69945c4a91407e1b6d75cc760cb',
 'screen_yaw_roundtrip_model_20261030.py':'c9097f13bce11a1cb457e7670b0e916fd1e71c6fce72c175baa874e9e4b72e10',
 'screen_material_checked_reverse_cube_20261029.py':'79adde54e2d3a8e45bbd895b0f50b5c297536c0556ff508e39b54f567e3a3d66',
}
for rel,value in expected.items():assert hashlib.sha256((R/rel).read_bytes()).hexdigest()==value,rel
for name in FILES:
 if name.endswith('.py'):ast.parse((R/name).read_text(encoding='utf-8'),filename=name)
review=validate_review(PlacementAlignmentPlan(REQUEST),PROOF);assert not review['loaded_motion_authorized'] and review['post_placement_goal_writes']==0
for rel in ('right_hand_rotation_20261030/placement_alignment_overlay/operator_guide.html','right_hand_rotation_20261030/yaw_roundtrip/operator_guide.html'):
 p=R/rel;page=p.read_text(encoding='utf-8')
 if p.parent==OUT:
  for link in re.findall(r'(?:src|href)="([^"]+)"',page):assert (p.parent/link).resolve().exists(),link
 else:assert '../placement_alignment_overlay/operator_guide.html' in page
report=dict(status='verified',protected_files_unchanged=list(expected),proof_scope=review['scope'],model_static_cases=201,model_dynamic_cases=9,software_tests=22,servo_writes=0,default_execution_allowed=False)
(OUT/'final_verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
