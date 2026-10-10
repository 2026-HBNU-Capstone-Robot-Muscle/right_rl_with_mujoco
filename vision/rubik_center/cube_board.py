"""큐브 ArUco 보드 정의. 우리 시뮬의 면 규약과 좌표계를 그대로 따른다.

왜 보드인가
  면마다 마커 하나를 따로 풀면(solvePnP) 보이는 면이 바뀔 때마다 자세가 튄다.
  cv2.aruco.Board 는 검출된 마커 전부를 **하나의 강체**로 묶어 한 번에 푼다.
  두 면이 동시에 보이면 정확도가 크게 오르고, 한 면만 보여도 동작한다.

면 규약 (face_up.py 의 _up_face_index 와 동일해야 한다)
      0:+z  1:-z  2:+x  3:-x  4:+y  5:-y
  이 순서가 어긋나면 "목표 면"이 통째로 뒤바뀐다.

좌표계
  큐브 중심이 원점. 시뮬의 큐브 지오메트리와 같은 축을 쓴다.
  단위는 **미터**로 둔다(시뮬과 같게). 카메라 캘리브레이션도 같은 단위여야 한다.
"""
import numpy as np

CUBE_MM = 57.0          # 시뮬 큐브 한 변 (reorientation_cube.xml: size .0285 -> 57mm)
MARKER_MM = 20.0        # 2x2 배치 기준. 22mm 는 면 가장자리 여백이 0.89셀로 부족했다
                        # (57mm 면, 사분면 중심 +-14.25mm, 4x4 사전은 6x6 셀).
                        # 20mm 면 가장자리 1.27셀, 마커 사이 2.55셀로 여유가 있다.
GRID = 2                # 1 이면 면당 1개, 2 면 2x2 로 4개

# 면 인덱스 -> (바깥 법선 n, 오른쪽 u, 위 v).  u x v = n 이어야 마커가 바깥에서 정상으로 보인다.
FACES = [
    (np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0])),   # 0 +z
    (np.array([0, 0, -1.0]), np.array([-1.0, 0, 0]), np.array([0, 1.0, 0])),  # 1 -z
    (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])),   # 2 +x
    (np.array([-1.0, 0, 0]), np.array([0, -1.0, 0]), np.array([0, 0, 1.0])),  # 3 -x
    (np.array([0, 1.0, 0]), np.array([0, 0, 1.0]), np.array([1.0, 0, 0])),   # 4 +y
    (np.array([0, -1.0, 0]), np.array([0, 0, -1.0]), np.array([1.0, 0, 0])),  # 5 -y
]


def _rot_uv(u, v, k):
    """면내 기저를 법선 둘레로 90도씩 k 번 돌린다."""
    for _ in range(k % 4):
        u, v = v, -u
    return u, v


def object_points(cube_mm=CUBE_MM, marker_mm=MARKER_MM, grid=GRID,
                  layout="grid", sticker_mm=None, face_rot=(0,) * 6):
    """(N,4,3) float32 objPoints 와 (N,) ids. 단위 m.

    layout="grid"    : 면을 가득 쓰는 grid x grid 배치. 일반 큐브용.
    layout="centers" : **면당 중앙 1개.** 3x3 루빅큐브용.
        루빅큐브에서 코어에 고정된 것은 6개 중앙 조각뿐이다. 코너·엣지는
        면을 돌리면 움직이므로 마커를 붙일 수 없다. 중앙에만 붙이면 어떤
        스크램블에도 보드 정의가 유지된다.

    face_rot : 면마다 마커가 몇 번 90도 돌아가 붙었는지 (0~3).
        중앙 조각은 그 면을 돌리면 **제자리에서 90도씩 회전**한다. 위치와
        법선은 그대로라 면 오차(스핀 불변)에는 영향이 없지만, 두 면이 동시에
        보일 때 보드 적합이 어긋난다. calibrate_face_rot.py 로 추정한다.

    마커 코너 순서는 OpenCV 규약대로 TL, TR, BR, BL (마커를 정면에서 볼 때).
    """
    h = cube_mm / 2000.0
    s = marker_mm / 1000.0
    if layout == "centers":
        centers, per = [(0.0, 0.0)], 1
    elif grid == 1:
        centers, per = [(0.0, 0.0)], 1
    else:
        step = (cube_mm / 1000.0) / grid
        offs = [(i + 0.5) * step - (cube_mm / 2000.0) for i in range(grid)]
        centers = [(a, b) for b in reversed(offs) for a in offs]
        per = grid * grid
    pts, ids = [], []
    for f, (n, u0, v0) in enumerate(FACES):
        u, v = _rot_uv(u0, v0, face_rot[f])
        for k, (cu, cv) in enumerate(centers):
            c = h * n + cu * u + cv * v
            pts.append(np.array([
                c - (s / 2) * u + (s / 2) * v,   # TL
                c + (s / 2) * u + (s / 2) * v,   # TR
                c + (s / 2) * u - (s / 2) * v,   # BR
                c - (s / 2) * u - (s / 2) * v,   # BL
            ], dtype=np.float32))
            ids.append(f * per + k)
    return np.array(pts, dtype=np.float32), np.array(ids, dtype=np.int32)


def face_index_up(R_cube, up=np.array([0, 0, 1.0])):
    """지금 위를 향한 면의 인덱스. face_up.py 의 _up_face_index 와 같은 계산."""
    return int(np.argmax([up @ (R_cube @ n) for n, _, _ in FACES]))


def face_error(R_cube, target_face, up=np.array([0, 0, 1.0])):
    """목표 면 법선이 위에서 벗어난 각도 [rad]. 스핀 불변. 성공 판정과 동일."""
    n_world = R_cube @ FACES[target_face][0]
    return float(np.arccos(np.clip(n_world @ up, -1.0, 1.0)))
