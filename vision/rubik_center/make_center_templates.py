"""3x3 루빅큐브 중앙 스티커용 마커 템플릿.

왜 중앙만인가
  루빅큐브에서 코어에 고정된 것은 6개 중앙 조각뿐이다. 코너·엣지는 면을
  돌리면 움직이므로 마커를 붙이면 보드 정의가 깨진다.

방향은 신경 쓰지 않아도 된다
  12mm 급 마커에는 방향 표시를 넣을 여백이 없다(콰이어트 존을 먹는다).
  대신 붙인 뒤 calibrate_face_rot.py 로 면별 90도 회전을 추정한다.
  같은 장치가 **스크램블 후 중앙이 돌아간 것**도 보정한다.

크기
  마커 = 스티커 x 0.75. 4x4 사전은 6x6 셀이라 콰이어트 존 1셀을 확보하려면
  marker + 2*(marker/6) <= sticker, 즉 marker <= 0.75*sticker 다.

출력
  타일 = 스티커 크기 정사각형(마커 중앙 배치). **타일 경계선을 따라 잘라**
  기존 스티커 위에 그대로 덮어 붙인다.
"""
import cv2
import numpy as np

DPI = 300
NAME = ["+z", "-z", "+x", "-x", "+y", "-y"]
STICKERS = (15.0, 16.0, 17.0)      # 흔한 57mm 3x3 의 중앙 스티커 크기


def _px(mm):
    return int(round(mm / 25.4 * DPI))


def tile(d, face, sticker_mm):
    marker_mm = 0.75 * sticker_mm
    S, M = _px(sticker_mm), _px(marker_mm)
    t = np.full((S, S), 255, np.uint8)
    o = (S - M) // 2
    t[o:o + M, o:o + M] = cv2.aruco.generateImageMarker(d, face, M)
    cv2.rectangle(t, (0, 0), (S - 1, S - 1), 170, 1)      # 자르는 선
    return t


def main(out="center_markers_A4.png", stickers=STICKERS, hand_mm=30.0):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    W, H = _px(210), _px(297)
    sheet = np.full((H, W), 255, np.uint8)
    x0, y = _px(15), _px(18)

    # 인쇄 배율 검증용 50mm 눈금자
    cv2.line(sheet, (x0, y), (x0 + _px(50), y), 0, 3)
    for k in range(11):
        xx = x0 + _px(5 * k)
        cv2.line(sheet, (xx, y - _px(2 if k % 2 == 0 else 1)), (xx, y), 0, 2)
    cv2.putText(sheet, "50 mm  <- ruler: print at 100%, measure this first",
                (x0, y + _px(5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, 0, 1, cv2.LINE_AA)
    y += _px(14)

    for st in stickers:
        cv2.putText(sheet, f"sticker {st:.0f} mm   marker {0.75*st:.2f} mm"
                    f"   (cut on the square outline)",
                    (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, 0, 1, cv2.LINE_AA)
        y += _px(4)
        x = x0
        for f in range(6):
            t = tile(d, f, st)
            s = t.shape[0]
            sheet[y:y + s, x:x + s] = t
            cv2.putText(sheet, f"{f} {NAME[f]}", (x, y + s + _px(4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, 0, 1, cv2.LINE_AA)
            x += s + _px(9)
        y += _px(0) + tile(d, 0, st).shape[0] + _px(12)

    # 손 기준 마커 (손등/손목 - 크기 제약 없음)
    cv2.putText(sheet, f"hand reference marker  ID 40   {hand_mm:.0f} mm",
                (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, 0, 1, cv2.LINE_AA)
    y += _px(4)
    hm = _px(hand_mm)
    sheet[y:y + hm, x0:x0 + hm] = cv2.aruco.generateImageMarker(d, 40, hm)
    cv2.imwrite(out, sheet)
    print(f"저장: {out}")
    for st in stickers:
        print(f"  스티커 {st:.0f}mm -> 마커 {0.75*st:.2f}mm")
    print("  1) 100% 로 인쇄  2) 눈금자 50mm 를 자로 확인  3) 큐브 스티커를 재고 맞는 줄만 사용")


if __name__ == "__main__":
    main()
