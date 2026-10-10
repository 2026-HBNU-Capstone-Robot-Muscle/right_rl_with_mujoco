#!/usr/bin/env bash
# mujoco_playground 설치본에 이 프로젝트의 환경 수정을 얹는다.
#
# 왜 필요한가
#   학습 코드(learning/*.py)만으로는 돌지 않는다. 과제 2 는 상류 mujoco_playground
#   에 없는 것들에 의존한다:
#     * face_up.py        - 과제 2 환경 자체(면 맞추기 + 제자리 복귀)
#     * reorient.py       - 연쇄 목표(chain_goals), settle_steps, hyperext_floor,
#                           action_delay_steps 등
#     * leap_rh_mjx.xml   - **엄지 관절 범위 교정**. menagerie 원본은 th_axl/th_mcp
#                           가 한 칸 밀려 있어 th_mcp 상한이 실제 스토퍼보다 31도
#                           더 굽혀졌다. 공식 URDF 및 LEAP SDK 의 LEAPsim_limits 와
#                           대조해 고친 것이다.
#   이 셋이 빠지면 학습은 돌지만 **다른 과제, 다른 손**을 학습한다.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PG=${PG:?PG 를 mujoco_playground 저장소 루트로 지정하세요}
LH="$PG/mujoco_playground/_src/manipulation/leap_hand"
[ -d "$LH" ] || { echo "leap_hand 경로가 없다: $LH" >&2; exit 1; }

STAMP=$(date +%Y%m%d_%H%M%S)
echo "백업 접미사: .bak_$STAMP"
for f in reorient.py face_up.py base.py leap_hand_constants.py; do
  [ -e "$LH/$f" ] && cp "$LH/$f" "$LH/$f.bak_$STAMP"
  cp "$HERE/env/$f" "$LH/$f"
  echo "  env  $f"
done
[ -e "$LH/xmls/leap_rh_mjx.xml" ] && cp "$LH/xmls/leap_rh_mjx.xml" "$LH/xmls/leap_rh_mjx.xml.bak_$STAMP"
cp "$HERE/env/xmls/leap_rh_mjx.xml" "$LH/xmls/leap_rh_mjx.xml"
echo "  xml  leap_rh_mjx.xml"

mkdir -p "$PG/learning"
for f in train_jax_sac.py residual_wrapper.py distill_residual.py eval_chain.py render_policy.py; do
  [ -e "$PG/learning/$f" ] && cp "$PG/learning/$f" "$PG/learning/$f.bak_$STAMP"
  cp "$HERE/learning/$f" "$PG/learning/$f"
  echo "  learn $f"
done

echo ""
echo "확인:"
python - <<'PY'
from brax.training import networks as bn
bn.KERNEL_INITIALIZER.setdefault(None, None)
from mujoco_playground import registry
env = registry.load("LeapCubeFaceUp", config_overrides={
    "impl": "jax", "chain_goals": True, "hyperext_floor": -0.2,
    "settle_steps": 8, "action_scale": 0.2})
c = env._config
print(f"  LeapCubeFaceUp 로드 OK  관측 {env.observation_size['state'][0]}차원 "
      f"행동 {env.action_size}")
print(f"  chain_goals={c.chain_goals} settle={c.settle_steps} "
      f"hyperext={c.hyperext_floor} action_delay={c.action_delay_steps}")
import numpy as np
lo = np.asarray(env._safe_lowers)
assert abs(lo[2] + 0.2) < 1e-9, "hyperext_floor 가 안 먹었다"
print("  pip/dip 하한 -0.2 적용 확인")
PY
echo "설치 완료."
