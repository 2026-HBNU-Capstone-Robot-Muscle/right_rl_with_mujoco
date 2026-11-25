import importlib.util, json, pathlib, sys
sys.stdout.reconfigure(encoding='utf-8')
print('DOCX',bool(importlib.util.find_spec('docx')))
sales=333605938; cogs=202235513; profit=43601051; ni=45206805
ar=(51127642+43623073)/2
inv=(52636828+51754865)/2
ap=(13039380+12370177)/2
assets=(566942110+514531948)/2
equity=(436320337+402192070)/2
values={
 'lecture_AR':230/34, 'lecture_AR_days':365/(230/34),
 'lecture_INV':147/29.5,'lecture_INV_days':365/(147/29.5),
 'lecture_AP':140/24.5,'lecture_AP_days':365/(140/24.5),
 'lecture_cycle':365/(230/34)+365/(147/29.5),
 'AR_avg':ar,'INV_avg':inv,'AP_avg':ap,'A_avg':assets,
 'AR_turn':sales/ar,'AR_days':365/(sales/ar),
 'INV_turn':cogs/inv,'INV_days':365/(cogs/inv),
 'AP_turn_cogs_proxy':cogs/ap,'AP_days_cogs_proxy':365/(cogs/ap),
 'A_turn':sales/assets,'OPM':profit/sales*100,'NPM':ni/sales*100,
 'ROA_avg':ni/assets*100,'ROE_total_avg':ni/equity*100,
 'ROE_owners_avg':44260956/((424313255+391687603)/2)*100,
 'WC':247684612-106411348,'current_ratio':247684612/106411348*100,
 'DE':130621773/436320337*100,
 'operating_cycle':365/(sales/ar)+365/(cogs/inv),
}
print(json.dumps(values,ensure_ascii=False,indent=2))
pathlib.Path(__file__).with_name('verified_calculations.json').write_text(json.dumps(values,ensure_ascii=False,indent=2),encoding='utf-8')
import re
folder=pathlib.Path(__file__).parent
problems=(folder/'경제성분석_중간고사_예상문제.md').read_text(encoding='utf-8')
ox=problems.split('## A.')[1].split('## B.')[0]
mc=problems.split('## B.')[1].split('## C.')[0]
ox_answers=problems.split('## C.')[1].split('## D.')[0]
mc_answers=problems.split('## D.')[1]
print('PROBLEM_COUNTS',len(re.findall(r'^\d+\.',ox,re.M)),len(re.findall(r'^\d+\.',mc,re.M)))
print('ANSWER_COUNTS',len(re.findall(r'^\| \d+ \|',ox_answers,re.M)),len(re.findall(r'^\| \d+ \|',mc_answers,re.M)))
print('EXTRACTED_LESSONS',len(list(folder.glob('lesson_*_extracted.txt'))))
print('STUDY_FILES', [(p.name,p.stat().st_size) for p in folder.glob('*.md')])
