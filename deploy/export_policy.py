"""증류망을 numpy 단독으로 돌 수 있는 .npz 로 내보낸다.

왜
  실기 PC 에 jax/brax/orbax 를 설치하고 체크포인트 디렉터리를 들고 다니는 것은
  배포 경로를 무겁고 깨지기 쉽게 만든다. 정책은 MLP 하나이므로 가중치와 정규화
  통계만 있으면 numpy 로 충분하다.

무엇이 들어가는가
  * 정규화 통계 mean/std  - **이게 없으면 정책은 무의미한 출력을 낸다.**
    신경망 파라미터만 챙기고 통계를 빠뜨리는 것이 이관에서 가장 흔한 실패다.
  * 각 층의 kernel/bias
  * 결정적 추론은 tanh(mean) 이다. 정책망은 (mean, log_std) 를 이어 내므로
    앞 절반만 쓰고 tanh 를 건다.

검증
  내보낸 뒤 무작위 관측 512개에 대해 jax 추론과 numpy 추론을 대조한다.
"""
import argparse
import functools

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--distilled", required=True, help="증류망 파일")
    ap.add_argument("--norm_from", required=True, help="구조 템플릿용 SAC 체크포인트 디렉터리")
    ap.add_argument("--hidden", default="512,256,128")
    ap.add_argument("--obs_size", type=int, default=108)
    ap.add_argument("--action_size", type=int, default=16)
    ap.add_argument("--out", default="policy_np.npz")
    a = ap.parse_args()

    import jax, flax
    from etils import epath
    from brax.training import networks as bn
    from brax.training.acme import running_statistics
    from brax.training.agents.sac import checkpoint as sac_checkpoint
    from brax.training.agents.sac import networks as sac_networks
    bn.KERNEL_INITIALIZER.setdefault(None, None)
    from mujoco_playground.config import manipulation_params

    cfg = manipulation_params.brax_sac_config("LeapCubeReorient")
    kw = dict(cfg.network_factory)
    kw["hidden_layer_sizes"] = tuple(int(x) for x in a.hidden.split(","))
    net = functools.partial(sac_networks.make_sac_networks, **kw)(
        observation_size=a.obs_size, action_size=a.action_size,
        preprocess_observations_fn=running_statistics.normalize)

    tmpl = {"norm": sac_checkpoint.load(a.norm_from)[0],
            "policy": net.policy_network.init(jax.random.PRNGKey(0))}
    r = flax.serialization.from_bytes(
        tmpl, epath.Path(a.distilled).resolve().read_bytes())
    params = (r["norm"], r["policy"])

    flat = {}
    leaves = jax.tree_util.tree_flatten_with_path(r["policy"])[0]
    for path, v in leaves:
        key = "/".join(str(getattr(p, "key", p)) for p in path)
        flat["W:" + key] = np.asarray(v)
    mean = np.asarray(jax.tree_util.tree_leaves(r["norm"].mean)[0])
    std = np.asarray(jax.tree_util.tree_leaves(r["norm"].std)[0])
    np.savez(a.out, mean=mean, std=std,
             hidden=np.array(kw["hidden_layer_sizes"]),
             action_size=a.action_size, **flat)
    print(f"저장: {a.out}")
    print(f"  정규화 mean/std 차원 {mean.shape} / {std.shape}")
    for k in sorted(flat):
        print(f"  {k:<40} {flat[k].shape}")

    # --- 검증: jax 추론 vs numpy 추론 -----------------------------------
    infer = jax.jit(sac_networks.make_inference_fn(net)(params, deterministic=True))
    import policy_np
    p = policy_np.Policy(a.out)
    rng = np.random.default_rng(0)
    obs = rng.normal(size=(512, a.obs_size)).astype(np.float32)
    ja = np.asarray(jax.vmap(lambda o: infer(o, jax.random.PRNGKey(0))[0])(obs))
    na = np.stack([p(o) for o in obs])
    err = float(np.abs(ja - na).max())
    print(f"\njax 대 numpy 최대 오차 {err:.3e}  ->", "일치" if err < 1e-5 else "불일치")


if __name__ == "__main__":
    main()
