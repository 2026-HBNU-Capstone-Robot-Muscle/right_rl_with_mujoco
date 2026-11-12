"""Native SVG explanation; keeps the source photograph unchanged."""
import base64
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/rigid_contact_review_20261025'
photo=ROOT/'loaded_quarter_trial_20261025_171607_019752/cube_before_turn.jpg'
data=base64.b64encode(photo.read_bytes()).decode('ascii')
svg=f'''<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="990" viewBox="0 0 1280 990">
<defs><marker id="arrow" markerWidth="10" markerHeight="10" refX="8" refY="3" orient="auto"><path d="M0,0 L0,6 L9,3 z" fill="#be123c"/></marker></defs>
<rect width="1280" height="990" fill="#f7f8fa"/>
<g font-family="Malgun Gothic, sans-serif" fill="#172033">
<text x="30" y="45" font-size="28" font-weight="bold">회전 없이, 큐브의 밑모서리와 실제 받침을 확인하는 촬영</text>
<text x="30" y="81" font-size="20">지난 사진에서는 표시한 부분이 손가락에 가려져 있어요. 새 사진은 카메라를 낮춰 옆에서 봅니다.</text>
<image x="0" y="104" width="960" height="540" href="data:image/jpeg;base64,{data}"/>
<rect x="302" y="353" width="219" height="40" rx="4" stroke="#facc15" stroke-width="4" fill="none"/>
<path d="M1020 350 L528 373" stroke="#be123c" stroke-width="4" fill="none" marker-end="url(#arrow)"/>
<text x="980" y="280" font-size="23" font-weight="bold">이 밑모서리와</text><text x="980" y="316" font-size="23" font-weight="bold">아래 받침이</text><text x="980" y="352" font-size="23" font-weight="bold">함께 보이게</text>
<text x="30" y="685" font-size="23" font-weight="bold">촬영 방향 개념도</text><text x="250" y="685" font-size="18">치수와 구조를 재현한 그림은 아닙니다.</text>
<rect x="770" y="829" width="350" height="70" rx="10" fill="#293341"/><text x="860" y="870" fill="white" font-size="22">로봇 본체</text>
<rect x="570" y="815" width="210" height="35" rx="8" fill="#596678"/><text x="515" y="891" font-size="21">검지와 중지의 받침 구간</text>
<rect x="590" y="703" width="145" height="112" rx="8" fill="#fff" stroke="#172033" stroke-width="3"/><text x="631" y="768" font-size="25">큐브</text>
<line x1="590" y1="815" x2="735" y2="815" stroke="#be123c" stroke-width="7"/>
<rect x="150" y="770" width="115" height="72" rx="9" fill="#2563eb"/><circle cx="207" cy="805" r="21" fill="#172033"/><text x="145" y="880" font-size="22">카메라를 낮게</text>
<path d="M280 805 L565 815" stroke="#be123c" stroke-width="4" fill="none" marker-end="url(#arrow)"/>
<text x="30" y="949" font-size="21">첫 SPACE는 빈손 준비 → PLACE에서 큐브를 놓고 둘째 SPACE로 촬영 → 큐브를 뺀 뒤 셋째 SPACE로 OFF</text>
</g></svg>'''
(OUT/'camera_support_side_guide.svg').write_text(svg,encoding='utf-8')
(OUT/'camera_support_side_guide.html').write_text('<!doctype html><meta charset="utf-8"><title>큐브 받침 촬영</title><style>body{margin:0;background:#f7f8fa}svg{width:100%;height:auto;max-width:1280px;display:block;margin:auto}</style>'+svg,encoding='utf-8')
print('SAVED',OUT/'camera_support_side_guide.html')
