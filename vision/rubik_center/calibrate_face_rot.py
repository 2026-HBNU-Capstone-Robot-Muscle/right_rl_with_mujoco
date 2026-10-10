"""면마다 마커가 몇 번 90도 돌아가 붙었는지 자동으로 알아낸다.

왜 필요한가
  12mm 마커에는 방향 표시를 넣을 여백이 없다(콰이어트 존을 먹는다). 그래서
  붙일 때 방향을 맞추는 대신 **붙인 뒤에 측정**한다.

  그리고 루빅큐브는 면을 돌리면 그 면의 중앙 조각이 제자리에서 90도 회전한다.
  위치와 법선은 그대로라 면 오차(스핀 불변)에는 영향이 없지만, 두 면이 동시에
  보일 때 보드 적합이 어긋난다. 스크램블 뒤에는 이걸 다시 돌려야 한다.

방법
  후보는 4^6 = 4096 개뿐이다. 큐브를 여러 자세로 들고 찍은 프레임들에 대해
  후보마다 보드 자세를 풀고 재투영 오차를 합산해 최소를 고른다.
  **두 면 이상이 보이는 프레임**만 쓴다 - 한 면만 보이면 그 면의 회전이
  자세와 구별되지 않아 정보가 없다.
"""
import itertools

import cv2
import numpy as np

import cube_board as cb


def solve(observations, K, dist, marker_mm, cube_mm=cb.CUBE_MM, verbose=True):
    """observations: [(ids(N,), corners(N,4,2))] - 각 프레임의 검출 결과.

    반환: (best_face_rot, 프레임당 평균 재투영 오차 px)
    """
    obs = [(np.asarray(i).ravel(), np.asarray(c, np.float32).reshape(-1, 4, 2))
           for i, c in observations]
    obs = [(i, c) for i, c in obs if len(set(int(x) // 1 for x in i)) >= 2]
    if not obs:
        raise ValueError("두 면 이상 보이는 프레임이 없다. 큐브를 비스듬히 들고 다시 찍을 것.")
    if verbose:
        print(f"사용 프레임 {len(obs)}개")

    best, best_err = None, np.inf
    for cand in itertools.product(range(4), repeat=6):
        P, I = cb.object_points(cube_mm=cube_mm, marker_mm=marker_mm,
                                layout="centers", face_rot=cand)
        idx = {int(v): k for k, v in enumerate(I)}
        tot, cnt = 0.0, 0
        for ids, cor in obs:
            op = np.concatenate([P[idx[int(i)]] for i in ids]).reshape(-1, 3)
            ip = cor.reshape(-1, 2)
            ok, rv, tv = cv2.solvePnP(op, ip, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                tot += 1e6; cnt += 1; continue
            pr, _ = cv2.projectPoints(op, rv, tv, K, dist)
            tot += float(np.sqrt(((pr.reshape(-1, 2) - ip) ** 2).sum(1)).mean())
            cnt += 1
        e = tot / max(cnt, 1)
        if e < best_err:
            best_err, best = e, cand
    if verbose:
        print(f"face_rot = {best}   평균 재투영 오차 {best_err:.3f} px")
        print("  1px 이하면 신뢰. 크면 프레임을 더 모으거나 캘리브레이션을 다시 할 것.")
    return best, best_err


def collect(cap, det, n=20, min_faces=2, show=False):
    """카메라에서 두 면 이상 보이는 프레임을 n 개 모은다."""
    out = []
    while len(out) < n:
        ok, f = cap.read()
        if not ok:
            break
        c, i, _ = det.detectMarkers(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
        if i is not None and len(i) >= min_faces:
            out.append((i.ravel(), np.array([x.reshape(4, 2) for x in c], np.float32)))
            print(f"  {len(out)}/{n}  면 {sorted(int(v) for v in i.ravel())}")
    return out
