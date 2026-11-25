"""Read HWP compound streams using the standard library; do not modify source."""
from pathlib import Path
import struct,zlib,re,sys,json,hashlib

SOURCE=Path('c:/Users/김준서/Documents/카카오톡 받은 파일/2. 제15회 AI+X 작품전시회_족자(페널) 제작 양식(쾌남근육맨).hwp')
OUT=Path(__file__).resolve().parent

def main():
    blob=SOURCE.read_bytes()
    if blob[:8]!=bytes.fromhex('d0cf11e0a1b11ae1'):raise ValueError('Not a compound HWP file')
    u32=lambda offset:struct.unpack_from('<I',blob,offset)[0]
    sector_size=1<<struct.unpack_from('<H',blob,30)[0]
    mini_size=1<<struct.unpack_from('<H',blob,32)[0]
    def sector(sid):return blob[(sid+1)*sector_size:(sid+2)*sector_size]
    def ints(data):return list(struct.unpack('<'+'I'*(len(data)//4),data))
    valid=lambda sid:0<=sid<0xfffffffa
    difat=ints(blob[76:512]);next_difat=u32(68)
    for _ in range(u32(72)):
        data=ints(sector(next_difat));difat.extend(data[:-1]);next_difat=data[-1]
    fat=[]
    for sid in [v for v in difat if valid(v)][:u32(44)]:fat.extend(ints(sector(sid)))
    def chain(start,table,read):
        chunks=[];seen=set();sid=start
        while valid(sid):
            if sid in seen or sid>=len(table):raise ValueError('Invalid compound stream chain')
            seen.add(sid);chunks.append(read(sid));sid=table[sid]
        return b''.join(chunks)
    directory=chain(u32(48),fat,sector);entries=[]
    for pos in range(0,len(directory),128):
        e=directory[pos:pos+128]
        length=struct.unpack_from('<H',e,64)[0]
        if e[66] and length>=2:
            entries.append(dict(name=e[:length-2].decode('utf-16le'),kind=e[66],
                start=struct.unpack_from('<I',e,116)[0],size=struct.unpack_from('<Q',e,120)[0]))
    root=next(e for e in entries if e['kind']==5)
    mini_stream=chain(root['start'],fat,sector)[:root['size']]
    mini_fat=ints(chain(u32(60),fat,sector)[:u32(64)*sector_size]) if u32(64) else []
    def read(e):
        if e['size']<u32(56):
            data=chain(e['start'],mini_fat,lambda sid:mini_stream[sid*mini_size:(sid+1)*mini_size])
        else:data=chain(e['start'],fat,sector)
        return data[:e['size']]
    header=read(next(e for e in entries if e['name']=='FileHeader'))
    compressed=bool(struct.unpack_from('<I',header,36)[0]&1)
    if '--export-images' in sys.argv:
        folder=OUT/'poster_original_images';folder.mkdir(exist_ok=True);items=[]
        for entry in entries:
            if entry['kind']!=2 or not entry['name'].lower().startswith('bin'):continue
            data=read(entry)
            if not data.startswith((b'\x89PNG',b'\xff\xd8\xff',b'BM')):
                try:data=zlib.decompress(data,-15)
                except zlib.error:continue
            extension='.png' if data.startswith(b'\x89PNG') else '.jpg' if data.startswith(b'\xff\xd8\xff') else '.bmp' if data.startswith(b'BM') else None
            if extension is None:continue
            target=folder/(Path(entry['name']).stem+extension);target.write_bytes(data)
            items.append(dict(stream=entry['name'],file=str(target),bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))
        (folder/'manifest.json').write_text(json.dumps(items,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(items,ensure_ascii=False));return
    sections=sorted([e for e in entries if e['kind']==2 and re.fullmatch(r'Section\d+',e['name'])],key=lambda e:int(e['name'][7:]))
    paragraphs=[]
    for e in sections:
        data=read(e);data=zlib.decompress(data,-15) if compressed else data;pos=0
        while pos<len(data):
            if pos+4>len(data):raise ValueError('Truncated HWP record')
            value=struct.unpack_from('<I',data,pos)[0];pos+=4;tag=value&1023;size=value>>20
            if size==4095:size=struct.unpack_from('<I',data,pos)[0];pos+=4
            record=data[pos:pos+size]
            if len(record)!=size:raise ValueError('Truncated HWP record payload')
            pos+=size
            if tag==67:
                text=record.decode('utf-16le')
                text=re.sub(r'[\x00-\x08\x0b-\x1f]','',text).strip()
                if text:paragraphs.append(text)
    result='\n'.join(paragraphs)
    (OUT/'hwp_text.txt').write_text(result,encoding='utf-8')
    print('SOURCE:',SOURCE,'\nCOMPRESSED:',compressed,'SECTIONS:',len(sections),'PARAGRAPHS:',len(paragraphs))
    print(result)

if __name__=='__main__':main()
