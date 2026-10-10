"""카메라 여러 대에서 손 기준 큐브 자세를 얻는다.

왜 여러 대인가
  3x3 루빅큐브는 중앙 스티커에만 마커를 붙일 수 있어 면당 1개뿐이고, 하필
  손가락이 가장 자주 닿는 자리다. 가림이 지배적인 실패 원인이라 카메라를
  늘리는 것이 가장 효과가 크다. 두 대를 약 90도 벌려 놓는다.

카메라 간 외부 파라미터가 필요 없다
  우리 관측은 `손바닥 - 큐브` 인 상대량이다. 손에도 마커가 있으면 각 카메라가
  독립적으로 손 기준 자세를 내고, 그 값은 카메라와 무관하다. 카메라를 옮기거나
  노트북을 건드려도 그대로 동작한다.

주의
  * 장치는 /dev/v4l/by-id/ 로 지정한다. /dev/video* 번호는 부팅마다 바뀐다.
  * MJPG 를 해상도 설정 **전에** 걸어야 USB 대역폭이 모자라지 않는다.
  * 노출·초점을 수동 고정한다. 자동이면 어두울 때 15fps 로 떨어져 제어 주기보다
    느려진다.
  * read() 를 제어 루프에서 직접 부르면 큐에 쌓인 오래된 프레임이 나온다.
    그래버 스레드로 최신 프레임만 보관한다.
"""
import struct
import threading
import time

import cv2
import numpy as np

import cube_board as cb
import track_cube as tc


def open_camera(path, width=1280, height=720, fps=30, exposure=-6):
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"카메라를 못 열었다: {path}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # 해상도보다 먼저
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)     # 1 = 수동 (드라이버마다 다름)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    fourcc = struct.pack("<I", int(cap.get(cv2.CAP_PROP_FOURCC))).decode(errors="replace")
    got = (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
           cap.get(cv2.CAP_PROP_FPS))
    print(f"  {path}\n    포맷 {fourcc}  {got[0]:.0f}x{got[1]:.0f} @ {got[2]:.0f}fps")
    if fourcc != "MJPG":
        print("    [경고] MJPG 가 아니다. 두 대를 같은 USB 2.0 호스트에 물리면"
              " 대역폭이 모자라 스트림이 안 열린다.")
    return cap


class Grabber(threading.Thread):
    """카메라 하나를 계속 읽어 최신 프레임만 보관한다."""

    def __init__(self, cap, name):
        super().__init__(daemon=True)
        self.cap, self.name = cap, name
        self.lock = threading.Lock()
        self.latest = None
        self.n_read = 0
        self._stop = False

    def run(self):
        while not self._stop:
            ok, f = self.cap.read()
            if ok:
                with self.lock:
                    self.latest = (time.time(), f)
                    self.n_read += 1

    def get(self):
        with self.lock:
            return self.latest

    def stop(self):
        self._stop = True


class MultiCamCube:
    """여러 카메라에서 손 기준 큐브 자세를 낸다.

    cams: [(장치경로, camera.npz 경로)]
    """

    def __init__(self, cams, marker_mm=12.0, face_rot=(0,) * 6,
                 layout="centers", **cap_kw):
        self.K, self.dist, self.grab = [], [], []
        for path, calib in cams:
            z = np.load(calib)
            self.K.append(z["K"]); self.dist.append(z["dist"])
            g = Grabber(open_camera(path, **cap_kw), path)
            g.start(); self.grab.append(g)
        self.d, self.board, self.det = tc.build(marker_mm=marker_mm, grid=1)
        P, I = cb.object_points(marker_mm=marker_mm, layout=layout, face_rot=face_rot)
        self.board = (cv2.aruco.Board(P, self.d, I) if hasattr(cv2.aruco, "Board")
                      else cv2.aruco.Board_create(P, self.d, I))
        self.last = None
        self.n_miss = 0

    def read(self):
        """가장 좋은 추정 하나를 고른다.

        평균을 내면 나쁜 추정 하나가 전체를 끌어내린다. 검출 마커 수가 많은
        쪽을, 같으면 재투영 오차가 작은 쪽을 고른다.
        """
        best, best_key = None, None
        for i, g in enumerate(self.grab):
            fr = g.get()
            if fr is None:
                continue
            r = tc.step(fr[1], self.board, self.det, self.d, self.K[i], self.dist[i])
            if not r or "rel" not in r:
                continue
            key = (r["n_cube_markers"], -r.get("reproj", 0.0))
            if best_key is None or key > best_key:
                best_key, best = key, dict(r, cam=i, stamp=fr[0])
        if best is None:
            self.n_miss += 1
            if self.last is None:
                return None
            # 끊기면 직전 값을 유지하되 **신선도를 함께 알린다.**
            # 이게 없으면 호출부가 몇 초 전 값을 최신인 줄 알고 쓰고,
            # 끊김률 통계도 0 으로 나온다.
            stale = dict(self.last)
            stale["stale"] = True
            stale["age"] = time.time() - self.last["stamp"]
            return stale
        self.n_miss = 0
        best["stale"] = False
        best["age"] = 0.0
        self.last = best
        return best

    def close(self):
        for g in self.grab:
            g.stop()
