"""인쇄용 마커 시트 생성. A4 300dpi, 실제 mm 크기로 나온다.

인쇄 주의
  프린터 대화상자에서 **"실제 크기"/"100%"** 로 인쇄할 것. "페이지에 맞춤"을
  쓰면 크기가 달라져 자세 추정이 그만큼 통째로 틀린다.
  인쇄 후 자로 마커 한 변을 재서 설정값과 일치하는지 반드시 확인한다.
"""
import sys

import sys, pathlib
# cube_board 는 rubik_center/ 에 있다(공용). 이 폴더는 대안 경로 전용이다.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "rubik_center"))
import cv2
import numpy as np

import cube_board as cb

DPI = 300
DICT = cv2.aruco.DICT_4X4_50


def _mm2px(mm):
    return int(round(mm / 25.4 * DPI))


def marker_img(d, mid, mm):
    px = _mm2px(mm)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(d, mid, px)
    return cv2.aruco.drawMarker(d, mid, px)          # OpenCV < 4.7


def main(marker_mm=cb.MARKER_MM, grid=cb.GRID, hand_mm=30.0, out="markers_A4.png"):
    d = cv2.aruco.getPredefinedDictionary(DICT)
    W, H = _mm2px(210), _mm2px(297)
    sheet = np.full((H, W), 255, np.uint8)
    pad = _mm2px(12)                    # 여백 = 콰이어트 존 확보
    x, y, rowh = pad, pad, 0
    n = 6 * grid * grid
    labels = ["+z", "-z", "+x", "-x", "+y", "+y"]
    labels[5] = "-y"
    for mid in list(range(n)) + [40]:   # 40 = 손 기준 마커
        mm = hand_mm if mid == 40 else marker_mm
        img = marker_img(d, mid, mm)
        s = img.shape[0]
        if x + s + pad > W:
            x, y = pad, y + rowh + pad
            rowh = 0
        if y + s + pad > H:
            print("A4 한 장에 안 들어간다. marker_mm 을 줄이거나 나눠 인쇄할 것.")
            break
        sheet[y:y + s, x:x + s] = img
        tag = "hand" if mid == 40 else f"{mid} {labels[mid // (grid * grid)]}"
        cv2.putText(sheet, tag, (x, y + s + _mm2px(4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0, 1, cv2.LINE_AA)
        cv2.rectangle(sheet, (x - 2, y - 2), (x + s + 2, y + s + 2), 200, 1)
        x += s + pad
        rowh = max(rowh, s + _mm2px(6))
    cv2.imwrite(out, sheet)
    print(f"저장: {out}  ({marker_mm}mm x {n}개 + 손 {hand_mm}mm)")
    print("인쇄: 실제 크기 100%, 무광 용지. 인쇄 후 자로 한 변을 재서 확인할 것.")


if __name__ == "__main__":
    a = [float(v) for v in sys.argv[1:3]] or [cb.MARKER_MM, cb.GRID]
    main(marker_mm=a[0], grid=int(a[1]) if len(a) > 1 else cb.GRID)
