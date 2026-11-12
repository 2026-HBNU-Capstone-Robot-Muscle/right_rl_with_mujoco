"""A scientific comparison panel: saved hand photo and simulated placement."""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_043035_review'

def main():
    actual=Image.open(ROOT/'reverse_start_empty_20261025_043035_076434/camera/frame_00938.jpg').convert('RGB')
    sim=Image.open(ROOT/'right_hand_rotation_20261024/original_sequence_clearance/reverse_turn_start_and_finish.png').convert('RGB')
    sim=sim.crop((0,50,600,490))
    panel=Image.new('RGB',(1280,650),(246,247,249));draw=ImageDraw.Draw(panel)
    font=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',25);small=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',21)
    draw.text((20,10),'새 시작 자세의 큐브 위치',font=font,fill=(20,25,35))
    draw.text((20,52),'왼쪽: 방금 촬영한 실제 손',font=small,fill=(20,25,35))
    draw.text((700,52),'오른쪽: 큐브를 놓는 모델 예시',font=small,fill=(20,25,35))
    panel.paste(actual.resize((680,383)),(0,92));panel.paste(sim.resize((560,411)),(700,92))
    draw=ImageDraw.Draw(panel)
    draw.text((22,525),'검지와 중지의 평평한 링크 윗면에 큐브를 걸쳐 놓기',font=font,fill=(20,25,35))
    draw.text((22,568),'초록은 위로, 주황은 손목 쪽으로. 손을 떼어도 자연스럽게 받쳐져야 해.',font=small,fill=(20,25,35))
    draw.text((22,605),'다음 실행에서 PLACE 안내가 나온 뒤에만 놓아줘. 이번에는 사진만 저장해.',font=small,fill=(20,25,35))
    panel.save(OUT/'placement_guide.png')
    print(OUT/'placement_guide.png')

if __name__=='__main__':main()
