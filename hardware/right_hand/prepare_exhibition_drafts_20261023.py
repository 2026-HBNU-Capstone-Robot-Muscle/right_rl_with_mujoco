"""Prepare copy-ready text and a workbook COPY, leaving supplied forms intact."""
import hashlib
from pathlib import Path
import re
from xml.sax.saxutils import escape
import xml.etree.ElementTree as ET
import zipfile

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'exhibition_20261023'
SOURCE=Path('C:/Users/김준서/Documents/카카오톡 받은 파일/2026 작품전시회 제출 양식/2026 작품전시회 제출 양식/10월 23일 까지')
TITLE='인공 근육 기반 구동 기술 강화학습'
SUBTITLE='양손 로봇을 이용한 3×3×3 큐브 조작'
OVERVIEW='''본 작품은 서로 다른 구조의 두 로봇 손을 활용하여 3×3×3 루빅스 큐브를 조작하는 시스템을 개발하는 프로젝트이다. 오른손은 큐브를 지지하고 원하는 면이 위를 향하도록 방향을 바꾸며, 왼손은 상단 레이어를 회전시키는 역할을 목표로 한다. 카메라 기반 큐브 자세 추정과 관절 상태 정보를 활용하고, MuJoCo 시뮬레이션과 강화학습을 통해 조작 동작을 연구한다. 현재 실물 로봇 손의 준비 동작과 큐브 파지 접근을 구현했으며, 목표 방향 회전과 양손 협조 동작을 검증하고 있다.'''
PROCESS=[
    '로봇 손 구성 및 제어 환경 구축: LEAP Hand 기반 오른손과 자체 설계 왼손을 활용하는 시스템을 설계하고, 관절 구동 및 상태 수집 환경을 구성하였다.',
    '비전·시뮬레이션 환경 구성: 카메라와 마커를 이용한 큐브 자세 추정 기능을 구성하고, MuJoCo에서 손·큐브·받침 구조의 접촉과 동작을 실험하였다.',
    '학습·실물 동작 검증: 시뮬레이션 기반 강화학습 제어를 연구하고 실물의 준비 자세 이동과 큐브 파지 접근을 확인하였다. 목표 방향 회전과 양손 동작 연계를 후속 검증 과제로 진행하고 있다.',
]
EFFECTS=[
    '다관절 로봇 손의 파지·자세 변경과 양손 역할 분담을 연구하여 복잡한 물체 조작을 위한 기초 기술을 축적할 수 있다.',
    '시뮬레이션과 실물의 영상·관절 상태를 비교하는 실험 환경을 구축하여 강화학습 제어 및 로봇 교육·연구에 활용할 수 있다.',
]
REQUEST='전시용 테이블 및 PC·모니터·로봇 전원장치용 전원 콘센트 필요'


def set_inline_cell(xml,reference,text):
    """Keep all worksheet namespaces, styles and other ZIP parts unchanged."""
    pattern=re.compile(r'<c\b(?=[^>]*\br="'+re.escape(reference)+r'")[^>]*(?:/>|>.*?</c>)',re.S)
    match=pattern.search(xml)
    if match:
        opening=match.group(0).split('>',1)[0].rstrip('/')
        opening=re.sub(r'\s+t="[^"]*"','',opening)
        replacement=opening+' t="inlineStr"><is><t>'+escape(text)+'</t></is></c>'
        return xml[:match.start()]+replacement+xml[match.end():]
    row_number=re.search(r'\d+$',reference).group(0)
    row=re.search(r'<row\b(?=[^>]*\br="'+row_number+r'")[^>]*>.*?</row>',xml,re.S)
    if row is None:raise ValueError('Missing template row '+row_number)
    # P4 is omitted entirely in the supplied blank template. It is the last
    # column of this form, so add it at the end of row 4 without rewriting XML.
    if reference!='P4':raise ValueError('Unexpected omitted cell '+reference)
    replacement='<c r="'+reference+'" t="inlineStr"><is><t>'+escape(text)+'</t></is></c>'
    insert=row.end()-len('</row>')
    return xml[:insert]+replacement+xml[insert:]


def main():
    OUT.mkdir(exist_ok=True)
    draft=f'''# 작품전시회 기입 문구

신청서·족자·작품현황의 제목은 기존 등록명으로 통일한다.
작품명: {TITLE}
설명용 부제: {SUBTITLE}
팀명: 쾌남근육맨 / 학과: 컴퓨터공학과 / 지도교수: 이상금
참여학생: 권우현, 정민성, 전지환, 김준서, 추상훈, 남기용 (6명)

## 1. 참가 신청서

기존 팀원·학번·연락처·지도교수 기입 사항은 유지한다.

- 전기사용여부: 유(○)
- LAN 사용: 필요 없음(○) — 현재처럼 PC에서 로컬 제어하고 저장 영상을 사용하는 전시 기준
- 작품크기: 공란 유지 (사용자 요청; 설치 공간 확정 후 기입)
- 기타요청사항: {REQUEST}

인터넷 없이도 현재 로컬 제어 방식으로 손을 천천히 움직이는 시연을 계획할 수 있다. 이는 전체 큐브 자동 해결이나 실물 SAC 정책 실행 완료를 뜻하지 않는다.

## 2. 족자(패널)

### 작품명

{TITLE}
({SUBTITLE})

### 작품 개요

{OVERVIEW}

### 작품 추진과정

'''+''.join(f'{n}. {text}\n\n' for n,text in enumerate(PROCESS,1))+'''4. 사진: 로봇 손 실물 사진과 MuJoCo 시뮬레이션 화면을 함께 배치한다.

### 기대효과

'''+''.join(f'- {text}\n' for text in EFFECTS)+'''
### 완성품

사진 설명: LEAP Hand 기반 로봇 손과 카메라·관절 제어 시스템 시제품
시뮬레이션 사진 설명: MuJoCo 기반 큐브 파지·재배향 실험 환경

전시 시연 계획: 로봇 손을 천천히 움직이며 준비 자세 이동과 큐브 파지 접근 과정을 시연하고, 시뮬레이션 화면과 개발 영상을 함께 제시한다.

실물 사진은 현재 시제품 사진을 사용한다. 목표 방향 회전·연속 재배향·양손 전체 솔빙은 아직 검증 중이므로 완료 기능으로 소개하지 않는다. 왼손 실물 사진과 최신 구현 범위는 왼손 담당자가 확인한 자료를 사용한다.

## 3. 참가 작품 현황(엑셀)

- 기존 작품명·학과·교수·팀·팀원 정보 유지
- 전기사용: 유(J4)의 기존 표시 유지
- LAN 사용: 필요없음(N4)에 ○
- 작품크기(O4): 공란 유지
- 기타요청(P4): '''+REQUEST+'''

`3_참가작품현황_작성안.xlsx`는 위 두 빈 항목(N4/P4)을 반영한 복사본이다. 원본은 수정하지 않았다. 제목과 팀원 자료는 제공된 양식에 있는 내용을 사용했다.
'''
    (OUT/'작품전시회_기입문구.md').write_text(draft,encoding='utf-8')
    panel='\n'.join([TITLE,'('+SUBTITLE+')','',
        '팀명: 쾌남근육맨','지도교수: 이상금',
        '참여학생: 권우현, 정민성, 전지환, 김준서, 추상훈, 남기용','',
        '작품 개요',OVERVIEW,'','작품 추진과정',
        *[f'- {text}' for text in PROCESS],
        '- 사진: 로봇 손 실물 및 MuJoCo 시뮬레이션 화면','',
        '기대효과',*[f'- {text}' for text in EFFECTS],'','완성품',
        '- LEAP Hand 기반 로봇 손과 카메라·관절 제어 시스템 시제품',
        '- MuJoCo 기반 큐브 파지·재배향 실험 환경',
        '- 시연 계획: 준비 자세 이동과 큐브 파지 접근을 느린 동작으로 시연'])
    (OUT/'족자_붙여넣기.txt').write_text(panel,encoding='utf-8-sig')
    original=next(SOURCE.glob('3.*.xlsx'));blob=original.read_bytes();original_hash=hashlib.sha256(blob).hexdigest()
    destination=OUT/'3_참가작품현황_작성안.xlsx'
    with zipfile.ZipFile(original) as z,zipfile.ZipFile(destination,'w') as out:
        for info in z.infolist():
            data=z.read(info.filename)
            if info.filename=='xl/worksheets/sheet2.xml':
                xml=data.decode('utf-8')
                for cell,text in (('N4','○'),('P4',REQUEST)):xml=set_inline_cell(xml,cell,text)
                ET.fromstring(xml);data=xml.encode('utf-8')
            out.writestr(info,data)
    assert hashlib.sha256(original.read_bytes()).hexdigest()==original_hash
    with zipfile.ZipFile(original) as before,zipfile.ZipFile(destination) as after:
        assert before.namelist()==after.namelist()
        assert after.testzip() is None
        for name in before.namelist():
            if name!='xl/worksheets/sheet2.xml':assert before.read(name)==after.read(name)
        ns={'s':'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
        sheet=ET.fromstring(after.read('xl/worksheets/sheet2.xml'))
        for ref,text in (('N4','○'),('P4',REQUEST)):
            node=sheet.find('.//s:c[@r="'+ref+'"]',ns)
            assert ''.join(t.text or '' for t in node.findall('.//s:t',ns))==text
        size=sheet.find('.//s:c[@r="O4"]',ns)
        assert size is None or not ''.join(size.itertext()).strip()
    print('Saved copy-ready text and validated workbook copy:',destination)


if __name__=='__main__':main()
