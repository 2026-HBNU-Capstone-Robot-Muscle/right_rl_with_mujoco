"""면마다 마커 4개가 이미 배치된 인쇄 템플릿. 잘라서 큐브 면에 그대로 붙인다.

왜 템플릿인가
  마커를 하나씩 손으로 붙이면 위치와 **방향**이 틀어진다. 위치가 몇 mm 틀리면
  자세가 그만큼 틀리고, 방향이 90도 틀어지면 그 면의 자세 해가 통째로 회전한다.
  ArUco 마커는 회전 비대칭이라 검출기가 코너를 정해진 순서로 돌려주는데,
  그 순서가 cube_board.object_points 의 (u, v) 기저와 맞아야 하기 때문이다.

방향 맞추는 법
  각 면의 **+u,+v 꼭지에 검은 삼각형**을 찍어 둔다. 큐브에 축을 먼저 표시하고
  (한 면을 +z, 인접 두 면을 +x, +y 로 정해 오른손 좌표계를 만든다),
  삼각형이 그 면의 u 축과 v 축이 만나는 꼭지에 오도록 붙이면 된다.

      면 0 (+z): u=+x  v=+y        면 3 (-x): u=-y  v=+z
      면 1 (-z): u=-x  v=+y        면 4 (+y): u=+z  v=+x
      면 2 (+x): u=+y  v=+z        면 5 (-y): u=-z  v=+x
"""
import sys, pathlib
# cube_board 는 rubik_center/ 에 있다(공용). 이 폴더는 대안 경로 전용이다.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "rubik_center"))
import cv2
import numpy as np

import cube_board as cb

DPI = 300
BLEED_MM = 6.0
UV = [("+x", "+y"), ("-x", "+y"), ("+y", "+z"),
      ("-y", "+z"), ("+z", "+x"), ("-z", "+x")]
NAME = ["+z", "-z", "+x", "-x", "+y", "-y"]


def _px(mm):
    return int(round(mm / 25.4 * DPI))


def face_template(f, d, cube_mm=cb.CUBE_MM, marker_mm=cb.MARKER_MM):
    side, bl = _px(cube_mm), _px(BLEED_MM)
    W = side + 2 * bl
    img = np.full((W, W), 255, np.uint8)
    c = W // 2
    off = _px(cube_mm / 4)                      # 사분면 중심 = 면 중심에서 변/4
    s = _px(marker_mm)
    # (du, dv) 부호와 격자칸 k 는 object_points 의 centers 순서와 같아야 한다:
    #   k=0:(-u,+v)  k=1:(+u,+v)  k=2:(-u,-v)  k=3:(+u,-v)
    for k, (du, dv) in enumerate([(-1, 1), (1, 1), (-1, -1), (1, -1)]):
        mid = f * 4 + k
        m = cv2.aruco.generateImageMarker(d, mid, s)
        x = c + du * off - s // 2
        y = c - dv * off - s // 2               # 이미지 y 는 아래로 증가
        img[y:y + s, x:x + s] = m
    # 자르는 선 (면 경계)
    cv2.rectangle(img, (bl, bl), (bl + side - 1, bl + side - 1), 160, 1)
    # +u,+v 꼭지 삼각형 (방향 기준)
    t = _px(3.5)
    x1, y1 = bl + side - _px(1.0), bl + _px(1.0)
    cv2.fillPoly(img, [np.array([[x1, y1], [x1 - t, y1], [x1, y1 + t]])], 0)
    # 여백에 라벨
    u, v = UV[f]
    cv2.putText(img, f"face {f}  {NAME[f]}   u={u} (->)  v={v} (^)  ID {f*4}-{f*4+3}",
                (bl, bl - _px(1.5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, 0, 1, cv2.LINE_AA)
    cv2.putText(img, f"triangle = +{u},+{v} corner", (bl, W - _px(1.5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, 0, 1, cv2.LINE_AA)
    return img


def main(out="faces_A4.png", cube_mm=cb.CUBE_MM, marker_mm=cb.MARKER_MM):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    tiles = [face_template(f, d, cube_mm, marker_mm) for f in range(6)]
    for f, t in enumerate(tiles):
        cv2.imwrite(f"face{f}_{NAME[f].replace('+','p').replace('-','m')}.png", t)
    W, H = _px(210), _px(297)
    sheet = np.full((H, W), 255, np.uint8)
    s = tiles[0].shape[0]
    pad = _px(5)
    x, y = pad, pad
    for t in tiles:
        if x + s + pad > W:
            x, y = pad, y + s + pad
        sheet[y:y + s, x:x + s] = t
        x += s + pad
    cv2.imwrite(out, sheet)
    print(f"저장: {out} + face0~5 개별 파일")
    print(f"  면 {cube_mm}mm, 마커 {marker_mm}mm 2x2")
    print("  인쇄는 실제 크기 100%. 안쪽 선(면 경계)을 따라 자를 것.")


if __name__ == "__main__":
    main()
