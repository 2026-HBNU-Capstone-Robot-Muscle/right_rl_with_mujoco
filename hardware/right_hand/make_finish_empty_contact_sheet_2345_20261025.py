import json
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'quarter_finish_empty_20261025_234555_682505';OUT=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/finish_empty_completion_2345'
def main():
    frames=json.loads((SOURCE/'camera/frames.json').read_bytes())['frames'];selected=[]
    for phase,count in (('stage_5_empty_start_hold',2),('stage_6_empty_quarter_reference',4),('stage_7_empty_root_finish',2),('stage_8_empty_observation',1)):
        rows=[r for r in frames if r['phase']==phase]
        if rows:
            for j in range(count):selected.append(rows[round((len(rows)-1)*j/max(1,count-1))])
    fontpath='C:/Windows/Fonts/malgun.ttf' if Path('C:/Windows/Fonts/malgun.ttf').exists() else '/mnt/c/Windows/Fonts/malgun.ttf'
    sheet=Image.new('RGB',(1440,3*300),(24,28,33));font=ImageFont.truetype(fontpath,15)
    for i,row in enumerate(selected):
        im=Image.open(SOURCE/'camera'/row['image']).convert('RGB');im.thumbnail((480,270));x=i%3*480;y=i//3*300;sheet.paste(im,(x,y+30));ImageDraw.Draw(sheet).text((x+6,y+5),row['phase'],font=font,fill='white')
    p=OUT/'actual_empty_reference_finish_sheet.jpg';sheet.save(p,quality=92);print(p)
if __name__=='__main__':main()
