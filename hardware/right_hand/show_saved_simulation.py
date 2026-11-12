"""Open existing local simulation replay; never starts training or hardware."""
from pathlib import Path
import os,sys
sys.stdout.reconfigure(encoding='utf-8')
root=Path(__file__).resolve().parent
page=root/'leap_simulation_view.html'
for name in ('sac_forward_support_reference_v1_20k/sac_corrected_flip.gif',
             'sac_forward_support_reference_v1_20k/sac_corrected_flip_comparison.jpg'):
    if not (root/name).is_file():raise FileNotFoundError(name)
os.startfile(str(page))
print('저장된 시뮬레이션 재생 화면 열기 요청 완료:',page)
print('모터/카메라/학습 실행 없음. 브라우저의 실제 표시 여부는 직접 확인 필요.')
