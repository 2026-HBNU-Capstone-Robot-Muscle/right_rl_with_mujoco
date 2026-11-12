import pathlib, struct, zlib, re, sys, json
import cv2
import numpy as np
sys.stdout.reconfigure(encoding='utf-8')
out = pathlib.Path('capstone_evaluation_evidence')
out.mkdir(exist_ok=True)
home = pathlib.Path.home()
p = home / 'Documents' / '카카오톡 받은 파일' / '(양식)캡스톤디자인 과제 결과보고서.hwp'
b = p.read_bytes()
ss = 1 << struct.unpack_from('<H', b, 30)[0]
mss = 1 << struct.unpack_from('<H', b, 32)[0]
def sec(n): return b[(n+1)*ss:(n+2)*ss]
def words(d): return struct.unpack('<'+'I'*(len(d)//4),d)
fatsectors = list(struct.unpack_from('<109I',b,76))
dif = struct.unpack_from('<I',b,68)[0]
for _ in range(struct.unpack_from('<I',b,72)[0]):
    w=words(sec(dif)); fatsectors.extend(w[:-1]); dif=w[-1]
fat=[]
for n in fatsectors:
    if n < 0xfffffffa: fat.extend(words(sec(n)))
def chain(n,tab,fn):
    data=[]; seen=set()
    while n < 0xfffffffa and n not in seen:
        seen.add(n); data.append(fn(n)); n=tab[n]
    return b''.join(data)
directory=chain(struct.unpack_from('<I',b,48)[0],fat,sec)
entries=[]
for i in range(0,len(directory),128):
    d=directory[i:i+128]; nl=struct.unpack_from('<H',d,64)[0]
    if nl < 2: continue
    entries.append((d[:nl-2].decode('utf-16le'),d[66],struct.unpack_from('<I',d,116)[0],struct.unpack_from('<Q',d,120)[0]))
root=next(x for x in entries if x[1]==5)
mini=chain(root[2],fat,sec)
mf=chain(struct.unpack_from('<I',b,60)[0],fat,sec)
minifat=words(mf)
def stream(e):
    name,typ,start,size=e
    return (chain(start,minifat,lambda n:mini[n*mss:(n+1)*mss]) if size<4096 else chain(start,fat,sec))[:size]
print('STREAMS',[(e[0],e[3]) for e in entries])
header=stream(next(e for e in entries if e[0]=='FileHeader'))
compressed=bool(struct.unpack_from('<I',header,36)[0]&1)
texts=[]
for e in entries:
    if not re.match(r'Section\d+$',e[0]): continue
    data=stream(e)
    if compressed: data=zlib.decompress(data,-15)
    pos=0
    while pos+4<=len(data):
        tag=struct.unpack_from('<I',data,pos)[0];pos+=4
        size=tag>>20
        if size==4095: size=struct.unpack_from('<I',data,pos)[0];pos+=4
        payload=data[pos:pos+size];pos+=size
        if tag&1023==67:
            arr=struct.unpack('<'+'H'*(len(payload)//2),payload)
            chars=[];j=0
            while j<len(arr):
                c=arr[j]
                if c in [1,2,3,11,12,14,15,16,17,18,21,22,23]: j+=8;continue
                if c==9: chars.append('\t');j+=8;continue
                if c in [10,13]: chars.append('\n')
                elif c>=32: chars.append(chr(c))
                j+=1
            texts.append(''.join(chars))
text='\n'.join(texts).encode('utf-16le', errors='surrogatepass').decode('utf-16le', errors='replace')
(out/'report_template.txt').write_text(text,encoding='utf-8')
print('TEMPLATE TEXT\n',text)
v=home/'Downloads'/'ROQET_04_발표영상.mp4'
cap=cv2.VideoCapture(str(v));fps=cap.get(cv2.CAP_PROP_FPS);count=cap.get(cv2.CAP_PROP_FRAME_COUNT)
duration=count/fps
print('VIDEO',json.dumps({'fps':fps,'frames':count,'duration_sec':duration,'width':cap.get(3),'height':cap.get(4)}))
times=np.linspace(0,max(0,duration-1),30)
for batch in range(3):
    tiles=[]
    for idx,t in enumerate(times[batch*10:(batch+1)*10]):
        cap.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,frame=cap.read()
        if not ok: continue
        cv2.imwrite(str(out/f'frame_{batch*10+idx:02d}_{t:.1f}s.jpg'),frame)
        small=cv2.resize(frame,(640,360))
        small=cv2.copyMakeBorder(small,28,0,0,0,cv2.BORDER_CONSTANT,value=(255,255,255))
        cv2.putText(small,f'{t:.1f} sec',(10,20),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,0),1)
        tiles.append(small)
    if tiles:
        while len(tiles)<10: tiles.append(np.zeros_like(tiles[0]))
        montage=np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,10,2)])
        cv2.imwrite(str(out/f'montage_{batch}.jpg'),montage)
cap.release()
