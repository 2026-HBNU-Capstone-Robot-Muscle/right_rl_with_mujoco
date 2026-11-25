"""Read supplied HWP/XLSX forms without modifying the originals."""
import importlib.util
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import zipfile
import zlib

ROOT=Path(__file__).resolve().parent
SOURCE=Path('C:/Users/김준서/Documents/카카오톡 받은 파일/2026 작품전시회 제출 양식/2026 작품전시회 제출 양식/10월 23일 까지')
OUT=ROOT/'exhibition_20261023'


def hwp_text(path):
    raw=path.read_bytes()
    if raw[:8]!=bytes.fromhex('d0cf11e0a1b11ae1'):raise ValueError('Not an OLE HWP file')
    sector=1<<struct.unpack_from('<H',raw,30)[0]
    mini_sector=1<<struct.unpack_from('<H',raw,32)[0]
    def sec(n):return raw[(n+1)*sector:(n+2)*sector]
    difat=list(struct.unpack_from('<109I',raw,76))
    next_difat,count_difat=struct.unpack_from('<II',raw,68)
    for _ in range(count_difat):
        words=struct.unpack('<%dI'%(sector//4),sec(next_difat))
        difat.extend(words[:-1]);next_difat=words[-1]
    count_fat=struct.unpack_from('<I',raw,44)[0]
    fat=[]
    for n in [i for i in difat if i<0xfffffffa][:count_fat]:
        fat.extend(struct.unpack('<%dI'%(sector//4),sec(n)))
    def chain(start,table,read):
        chunks=[];seen=set();n=start
        while n<0xfffffffa:
            if n in seen or n>=len(table):raise ValueError('Invalid sector chain')
            seen.add(n);chunks.append(read(n));n=table[n]
        return b''.join(chunks)
    directory=chain(struct.unpack_from('<I',raw,48)[0],fat,sec)
    entries=[]
    for offset in range(0,len(directory),128):
        row=directory[offset:offset+128]
        if len(row)<128:break
        name_length=struct.unpack_from('<H',row,64)[0]
        if row[66] in (2,5) and 2<=name_length<=64:
            entries.append(dict(name=row[:name_length-2].decode('utf-16le'),kind=row[66],
                start=struct.unpack_from('<I',row,116)[0],size=struct.unpack_from('<Q',row,120)[0]))
    root=next(e for e in entries if e['kind']==5)
    mini_stream=chain(root['start'],fat,sec)[:root['size']]
    mini_fat_bytes=chain(struct.unpack_from('<I',raw,60)[0],fat,sec)
    mini_fat=list(struct.unpack('<%dI'%(len(mini_fat_bytes)//4),mini_fat_bytes))
    cutoff=struct.unpack_from('<I',raw,56)[0]
    streams={}
    for row in entries:
        if row['kind']!=2:continue
        if row['size']<cutoff:
            data=chain(row['start'],mini_fat,lambda n:mini_stream[n*mini_sector:(n+1)*mini_sector])
        else:data=chain(row['start'],fat,sec)
        streams[row['name']]=data[:row['size']]
    header=streams['FileHeader'];flags=struct.unpack_from('<I',header,36)[0]
    if flags&2:raise ValueError('Password-protected HWP')
    paragraphs=[]
    for name,data in sorted(streams.items()):
        if not name.startswith('Section'):continue
        if flags&1:data=zlib.decompress(data,-15)
        pos=0
        while pos+4<=len(data):
            record=struct.unpack_from('<I',data,pos)[0];pos+=4
            tag=record&1023;length=record>>20
            if length==4095:length=struct.unpack_from('<I',data,pos)[0];pos+=4
            payload=data[pos:pos+length];pos+=length
            if tag!=67:continue
            units=struct.unpack('<%dH'%(len(payload)//2),payload)
            text=[];i=0
            while i<len(units):
                value=units[i]
                if value in (10,13):text.append('\n');i+=1
                elif 1<=value<=23:
                    if value==9:text.append('\t')
                    i+=8
                else:
                    if value>=32:text.append(chr(value))
                    i+=1
            paragraphs.append(''.join(text))
    return dict(streams=list(streams),compressed=bool(flags&1),
                preview=streams.get('PrvText',b'').decode('utf-16le',errors='replace'),
                paragraphs=paragraphs)


def workbook_text(path):
    ns={'s':'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
    with zipfile.ZipFile(path) as z:
        strings=[]
        if 'xl/sharedStrings.xml' in z.namelist():
            for si in ET.fromstring(z.read('xl/sharedStrings.xml')).findall('s:si',ns):
                strings.append(''.join(t.text or '' for t in si.findall('.//s:t',ns)))
        sheets=[]
        for name in z.namelist():
            if not (name.startswith('xl/worksheets/sheet') and name.endswith('.xml')):continue
            data=ET.fromstring(z.read(name));cells=[]
            for cell in data.findall('.//s:sheetData/s:row/s:c',ns):
                value=cell.find('s:v',ns)
                text=value.text if value is not None else ''
                if cell.get('t')=='s' and text:text=strings[int(text)]
                if cell.get('t')=='inlineStr':text=''.join(t.text or '' for t in cell.findall('.//s:t',ns))
                if text:cells.append(dict(cell=cell.get('r'),text=text))
            sheets.append(dict(part=name,cells=cells,merges=[m.get('ref') for m in data.findall('.//s:mergeCell',ns)]))
        return sheets


def main():
    OUT.mkdir(exist_ok=True)
    modules={name:bool(importlib.util.find_spec(name)) for name in ('olefile','hwp5','openpyxl','win32com')}
    print('MODULES',json.dumps(modules))
    result=dict(source=str(SOURCE),files=[])
    for path in sorted(SOURCE.iterdir()):
        if path.suffix.lower() not in ('.hwp','.xlsx'):continue
        row=dict(name=path.name,bytes=path.stat().st_size)
        print('FILE',path.name,'bytes',row['bytes'])
        if path.suffix.lower()=='.xlsx':
            row['sheets']=workbook_text(path)
            print(json.dumps(row['sheets'],ensure_ascii=False,indent=2))
        else:
            row['signature_hex']=path.read_bytes()[:16].hex()
            row.update(hwp_text(path))
            print('PREVIEW',row['preview'])
            print('BODY','\n'.join(row['paragraphs']))
        result['files'].append(row)
    (OUT/'forms_extracted.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    for name in ('latest_goal_alignment.md',):
        print('GOAL',name,(ROOT/'goal_presentation_review'/name).read_text(encoding='utf-8'))


if __name__=='__main__':main()
