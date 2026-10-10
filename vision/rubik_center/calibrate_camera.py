"""체스보드로 카메라 내부 파라미터를 구한다. 1회만 하면 된다.

체스보드는 A4 에 9x6 내부코너(=10x7 칸)로 인쇄하고, **실제 크기 100%** 로
뽑은 뒤 한 칸 변을 자로 재서 SQUARE_MM 에 넣는다. 이 값이 틀리면 이후
모든 거리 추정이 같은 비율로 틀린다.

각도와 위치를 바꿔가며 20~30장을 찍는다. 화면 구석까지 골고루 채워야
왜곡 계수가 제대로 나온다.
"""
import glob
import sys

import cv2
import numpy as np

COLS, ROWS = 9, 6        # 내부 코너 개수
SQUARE_MM = 25.0


def main(pattern="calib/*.jpg", out="camera.npz"):
    objp = np.zeros((ROWS * COLS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * (SQUARE_MM / 1000.0)
    op, ip, shape = [], [], None
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"이미지가 없다: {pattern}");  return
    for f in files:
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        ok, c = cv2.findChessboardCorners(g, (COLS, ROWS), None)
        if not ok:
            print("  코너 못 찾음:", f);  continue
        c = cv2.cornerSubPix(g, c, (11, 11), (-1, -1),
                             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        op.append(objp);  ip.append(c);  shape = g.shape[::-1]
    if len(op) < 8:
        print(f"쓸 수 있는 장수 {len(op)} - 최소 8장 이상 필요");  return
    err, K, dist, _, _ = cv2.calibrateCamera(op, ip, shape, None, None)
    np.savez(out, K=K, dist=dist, rms=err, n=len(op))
    print(f"저장: {out}   장수 {len(op)}   재투영 오차 {err:.3f} px")
    print("  0.5px 이하면 양호, 1.0px 넘으면 다시 찍을 것")
    print("K =\n", K)


if __name__ == "__main__":
    main(*(sys.argv[1:] or []))
