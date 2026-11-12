"""Anonymous GET of the public URL supplied by the user; no credentials."""
import json,urllib.request,urllib.error
from html.parser import HTMLParser
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261104/shared_discussion';O.mkdir(exist_ok=True);url='https://chatgpt.com/share/6aea39e5-46b0-83ee-87f1-c7d20bccf7a9'
class Visible(HTMLParser):
 def __init__(self):super().__init__();self.skip=0;self.parts=[]
 def handle_starttag(self,t,a):
  if t in ('script','style','noscript'):self.skip+=1
 def handle_endtag(self,t):
  if t in ('script','style','noscript'):self.skip=max(0,self.skip-1)
 def handle_data(self,s):
  if not self.skip and s.strip():self.parts.append(s.strip())
report=dict(url=url,authenticated=False)
try:
 request=urllib.request.Request(url,headers={'User-Agent':'Mozilla/5.0','Accept':'text/html'})
 with urllib.request.urlopen(request,timeout=20) as response:
  body=response.read(6000000);report.update(status=response.status,bytes=len(body),content_type=response.headers.get('Content-Type'))
 (O/'public_page.html').write_bytes(body);parser=Visible();parser.feed(body.decode('utf-8','replace'));text='\n'.join(parser.parts);(O/'visible_text.txt').write_text(text,encoding='utf-8');report.update(visible_characters=len(text),preview=text[:1000])
except urllib.error.HTTPError as e:report.update(status=e.code,error=e.reason)
except Exception as e:report.update(error=str(e))
(O/'fetch_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,ensure_ascii=False))
