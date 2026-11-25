"""Engineering diagram of the remaining book-front reference, not photo editing."""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/book20_current_review'
im=Image.new('RGB',(1400,820),'#f5f7fb');d=ImageDraw.Draw(im)
regular=Path('/mnt/c/Windows/Fonts/malgun.ttf');bold=Path('/mnt/c/Windows/Fonts/malgunbd.ttf')
def text(x,y,s,size=25,color='#142338',strong=False):d.text((x,y),s,font=ImageFont.truetype(str(bold if strong else regular),size),fill=color)
def arrow(x0,x1,y,color):
    d.line((x0,y,x1,y),fill=color,width=4)
    for x,sign in ((x0,1),(x1,-1)):d.polygon([(x,y),(x+sign*13,y-7),(x+sign*13,y+7)],fill=color)
def badge(x,y,s,col):
    d.ellipse((x-22,y-22,x+22,y+22),fill=col,outline='white',width=3);text(x-12,y-22,s,30,'white',True)
text(50,30,'책 두께는 20mm로 확인했어. 남은 건 책 앞끝 위치야.',35,strong=True)
text(50,89,'옆모습 개념도. A는 움직이지 않는 뿌리 모터가 붙은 본체의 앞끝, B는 책 앞끝.',23,color='#53647a')
for col,(label,offset) in enumerate((('예시 1: 책 끝이 A보다 뒤에 있음',70),('예시 2: 책 끝이 A보다 앞에 있음',-70))):
    ox=45+col*690;y=165
    d.rectangle((ox,y,ox+660,y+548),fill='white',outline='#d8e0eb',width=2)
    text(ox+20,y+20,label,25,strong=True)
    text(ox+20,y+67,'손가락 쪽 ←                         → 손목 쪽',22,color='#53647a')
    a=ox+270;b=a+offset
    d.rectangle((ox+145,y+205,a-2,y+262),fill='#9aabba',outline='#34455c',width=3)
    text(ox+100,y+162,'움직이는 손가락',20,color='#53647a')
    d.rectangle((a,y+182,a+85,y+315),fill='#455468',outline='#142338',width=3)
    d.rectangle((a+85,y+143,ox+615,y+315),fill='#263446',outline='#142338',width=3)
    text(a+100,y+182,'LEAP 본체',28,'white',True)
    text(a+2,y+206,'고정',22,'white');text(a+2,y+241,'부분',22,'white')
    d.rectangle((b,y+316,ox+635,y+393),fill='#eee8d9',outline='#9b8e74',width=3)
    text(ox+465,y+340,'책 20mm',23,strong=True)
    d.rectangle((ox+85,y+395,ox+635,y+419),fill='#b9e9f3',outline='#218ba3',width=3)
    d.rectangle((ox+85,y+421,ox+635,y+465),fill='#d8ad79',outline='#986a3a',width=2)
    badge(a,y+315,'A','#d64045');badge(b,y+315,'B','#7356bf')
    d.line((a,y+337,a,y+508),fill='#d64045',width=2)
    d.line((b,y+337,b,y+508),fill='#7356bf',width=2)
    arrow(min(a,b)+2,max(a,b)-2,y+493,'#142338')
    text(ox+32,y+507,'알려줄 값: A와 B 사이 가로 거리',23,strong=True)
text(50,740,'예: “A보다 뒤로 10mm” 또는 “A보다 앞으로 10mm”',29,strong=True)
text(50,785,'이 그림은 위치 설명용이고 실제 책 앞끝을 측정해서 그린 것은 아니야.',21,color='#53647a')
path=OUT/'book_front_A_B_question.png';im.save(path);print(path)
