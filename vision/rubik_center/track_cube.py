"""큐브 보드 + 손 마커로 상대 자세를 낸다. 정책 관측에 넣을 값을 만든다.

핵심: 카메라 위치를 정밀하게 고정할 필요가 없다
  우리 관측은 `손바닥 위치 - 큐브 위치` 인 **상대량**이다. 손에도 마커를 붙여
  두 자세를 같은 카메라 프레임에서 재고 빼면, 노트북을 건드려도 값이 유지된다.

프레임을 반드시 맞춰야 한다 (여기서 조용히 틀리기 쉽다)
  solvePnP 가 주는 R_cube/t_cube 는 **카메라 프레임**이다. 그런데 정책이 먹는
  관측(`손바닥 - 큐브`, 큐브 자세)은 시뮬의 **월드 프레임**이다. 카메라 프레임
  값을 그대로 넣으면 카메라를 어디에 놓느냐에 따라 관측이 통째로 회전한다.

  손 마커를 기준으로 돌려서 월드로 옮긴다.

      R_wc  = HAND_TO_WORLD @ R_hand^T          카메라 -> 월드
      palm  = t_hand + R_hand @ PALM_IN_MARKER  카메라 프레임의 손바닥 위치
      d_world     = R_wc @ (palm - t_cube)      = 관측의 `손바닥 - 큐브`
      R_cube_world = R_wc @ R_cube

  PALM_IN_MARKER : 시뮬의 grasp_site 가 손 마커 프레임에서 어디인가 [m].
      자로 재서 넣는다. 0 으로 두면 마커 위치를 손바닥으로 간주하는 것이고,
      그 오차가 그대로 위치 오차가 된다(성공 임계값이 15 mm 다).
  HAND_TO_WORLD  : 손 마커 프레임 -> 시뮬 월드. 마커를 손등에 수평으로,
      마커의 u 축을 시뮬 +x, v 축을 +y 로 맞춰 붙이면 단위행렬이다.
"""
import cv2
import numpy as np

import cube_board as cb

HAND_ID = 40
HAND_MM = 30.0

# 손 마커 프레임에서 본 손바닥(grasp_site) 위치 [m]. **자로 재서 넣을 것.**
PALM_IN_MARKER = np.zeros(3)
# 손 마커 프레임 -> 시뮬 월드 프레임.
HAND_TO_WORLD = np.eye(3)


def build(marker_mm=cb.MARKER_MM, grid=cb.GRID):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    P, I = cb.object_points(marker_mm=marker_mm, grid=grid)
    board = (cv2.aruco.Board(P, d, I) if hasattr(cv2.aruco, "Board")
             else cv2.aruco.Board_create(P, d, I))
    det = (cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
           if hasattr(cv2.aruco, "ArucoDetector") else None)
    return d, board, det


def _pose_from(board, corners, ids, K, dist):
    """보드에 속한 마커만 모아 한 번에 푼다. 실패하면 None."""
    obj, img = board.matchImagePoints(corners, ids)
    if obj is None or len(obj) < 4:
        return None
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    pr, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = float(np.sqrt(((pr.reshape(-1, 2) - img.reshape(-1, 2)) ** 2).sum(1)).mean())
    return cv2.Rodrigues(rvec)[0], tvec.ravel(), err


def step(frame, board, det, d, K, dist):
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if det is not None:
        corners, ids, _ = det.detectMarkers(g)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(g, d)
    if ids is None:
        return None
    ids = ids.ravel()
    cube = [(c, i) for c, i in zip(corners, ids) if i < 40]
    hand = [(c, i) for c, i in zip(corners, ids) if i == HAND_ID]
    out = {"n_cube_markers": len(cube)}
    if cube:
        r = _pose_from(board, [c for c, _ in cube],
                       np.array([i for _, i in cube]).reshape(-1, 1), K, dist)
        if r:
            out["R_cube"], out["t_cube"], out["reproj"] = r
    if hand:
        s = HAND_MM / 1000.0
        obj = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0],
                        [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], np.float32)
        ok, rv, tv = cv2.solvePnP(obj, hand[0][0].reshape(4, 2), K, dist)
        if ok:
            out["R_hand"], out["t_hand"] = cv2.Rodrigues(rv)[0], tv.ravel()
    if "t_cube" in out and "t_hand" in out:
        R_wc = HAND_TO_WORLD @ out["R_hand"].T          # 카메라 -> 월드
        palm_cam = out["t_hand"] + out["R_hand"] @ PALM_IN_MARKER
        out["palm_minus_cube"] = R_wc @ (palm_cam - out["t_cube"])
        out["R_cube_world"] = R_wc @ out["R_cube"]
        out["cube_quat_world"] = mat_to_quat(out["R_cube_world"])
        out["face_up"] = cb.face_index_up(out["R_cube_world"],
                                          np.array([0.0, 0.0, 1.0]))
        # 참고용 원시값(카메라 프레임). 정책 관측에 직접 쓰지 말 것.
        out["rel_cam"] = out["t_cube"] - out["t_hand"]
    return out


def mat_to_quat(R):
    """회전행렬 -> wxyz 쿼터니언."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                         (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                         (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    if i == 1:
        s = np.sqrt(1 - R[0, 0] + R[1, 1] - R[2, 2]) * 2
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                         0.25 * s, (R[1, 2] + R[2, 1]) / s])
    s = np.sqrt(1 - R[0, 0] - R[1, 1] + R[2, 2]) * 2
    return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                     (R[1, 2] + R[2, 1]) / s, 0.25 * s])
