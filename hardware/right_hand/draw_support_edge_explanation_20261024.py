"""Native vector engineering sketch plus a local PNG preview. Not photo editing."""
from pathlib import Path
from html import escape
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024'
OUT.mkdir(parents=True,exist_ok=True)
W,H=1400,900
image=Image.new('RGB',(W,H),'#f5f7fb');draw=ImageDraw.Draw(image)
fontfile=Path('C:/Windows/Fonts/malgun.ttf')
boldfile=Path('C:/Windows/Fonts/malgunbd.ttf')
if not fontfile.exists():
    fontfile=Path('/mnt/c/Windows/Fonts/malgun.ttf')
    boldfile=Path('/mnt/c/Windows/Fonts/malgunbd.ttf')
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
     '<rect width="1400" height="900" fill="#f5f7fb"/>']

def text(x,y,value,size=28,fill='#142338',bold=False):
    font=ImageFont.truetype(str(boldfile if bold else fontfile),size)
    draw.text((x,y),value,font=font,fill=fill)
    svg.append(f'<text x="{x}" y="{y+size}" fill="{fill}" font-family="Malgun Gothic, sans-serif" font-size="{size}" font-weight="{700 if bold else 400}">{escape(value)}</text>')

def rect(box,fill,outline=None,width=1):
    draw.rectangle(box,fill=fill,outline=outline,width=width)
    x0,y0,x1,y1=box
    svg.append(f'<rect x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" fill="{fill}" stroke="{outline or fill}" stroke-width="{width}"/>')

def line(points,fill,width=3,dashed=False):
    if dashed:
        (x0,y0),(x1,y1)=points
        import math
        length=math.hypot(x1-x0,y1-y0)
        for d in range(0,int(length),18):
            a=d/max(length,1);b=min(d+9,length)/max(length,1)
            draw.line((x0+(x1-x0)*a,y0+(y1-y0)*a,x0+(x1-x0)*b,y0+(y1-y0)*b),fill=fill,width=width)
    else:draw.line(points,fill=fill,width=width)
    coords=' '.join(f'{x},{y}' for x,y in points)
    svg.append(f'<polyline points="{coords}" fill="none" stroke="{fill}" stroke-width="{width}"'+(' stroke-dasharray="9 9"' if dashed else '')+'/>')

def arrow(a,b,fill,width=4,both=False):
    import math
    line([a,b],fill,width)
    def tip(p,q):
        angle=math.atan2(p[1]-q[1],p[0]-q[0]);length=16
        points=[p,(p[0]-length*math.cos(angle-.45),p[1]-length*math.sin(angle-.45)),
                (p[0]-length*math.cos(angle+.45),p[1]-length*math.sin(angle+.45))]
        draw.polygon(points,fill=fill)
        svg.append('<polygon points="'+' '.join(f'{x},{y}' for x,y in points)+f'" fill="{fill}"/>')
    tip(b,a)
    if both:tip(a,b)

def badge(x,y,label,fill):
    draw.ellipse((x-25,y-25,x+25,y+25),fill=fill,outline='white',width=3)
    svg.append(f'<circle cx="{x}" cy="{y}" r="25" fill="{fill}" stroke="white" stroke-width="3"/>')
    text(x-14,y-23,label,32,'white',True)

text(65,35,'이 두 끝의 위치를 물어본 거야',44,bold=True)
text(65,104,'옆모습 개념도  |  손가락은 왼쪽, 손목은 오른쪽',25,fill='#53647a')
rect((55,166,1345,790),'white','#d8e0eb',2)
text(105,206,'손가락 쪽',29,bold=True);arrow((305,260),(145,260),'#34455c',4)
text(1125,206,'손목 쪽',29,bold=True);arrow((1100,260),(1260,260),'#34455c',4)

# Same physical order as the user's measured stack: wood / glass / hand.
rect((485,489,1245,525),'#b9e9f3','#218ba3',3)
rect((485,525,1245,690),'#d8ad79','#986a3a',3)
text(1020,545,'나무 받침',31,fill='#4b311a',bold=True)
text(1176,437,'유리판',24,fill='#146c82',bold=True)
arrow((1221,471),(1221,506),'#146c82',3)

# Fixed body edge A is distinct from moving motor housings farther left.
rect((745,321,1175,488),'#263446','#142338',3)
text(855,357,'LEAP 본체',37,fill='white',bold=True)
text(877,411,'손바닥 부분',24,fill='#d9e2ed')
rect((615,344,744,428),'#6d7e91','#34455c',3)
rect((405,352,610,411),'#7f91a5','#34455c',3)
rect((220,363,400,405),'#a4b3c3','#34455c',3)
text(228,292,'손가락과 움직이는 모터',24,fill='#53647a')

badge(745,488,'A','#d64045');badge(485,488,'B','#7356bf')
line([(745,515),(745,748)],'#d64045',3,True)
line([(485,515),(485,748)],'#7356bf',3,True)
arrow((493,729),(737,729),'#142338',4,True)
text(515,686,'이 가로 거리',27,bold=True)
text(500,749,'예전 답변: 약 34mm',22,fill='#53647a')

text(805,265,'A  본체의 손가락 쪽 끝',28,fill='#bd3039',bold=True)
line([(800,304),(768,304),(748,457)],'#d64045',4)
text(95,554,'B  유리와 나무의 앞끝',27,fill='#6849b3',bold=True)
arrow((445,563),(484,520),'#7356bf',4)
text(95,606,'초록 매트의 끝 말고,',23,fill='#53647a')
text(95,643,'단단한 받침의 끝을 봐줘.',23,fill='#53647a')
text(65,815,'그림처럼 B가 A보다 손가락 쪽으로 더 나와 있는지만 알려줘.',28,bold=True)
text(65,858,'실제 비율과 현재 위치를 확정한 그림은 아니야. 정확히 34mm를 다시 잴 필요는 없어.',21,fill='#53647a')
svg.append('</svg>')
(OUT/'support_edges_A_B.svg').write_text('\n'.join(svg),encoding='utf-8')
image.save(OUT/'support_edges_A_B.png')
print(OUT/'support_edges_A_B.png')
