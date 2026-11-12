# Codex 웹에서 시작하기

개인 비공개 저장소: `star1209042/xlerobot` / 사용할 브랜치: `main`.

코드와 모델 및 인계문은 GitHub에 있다. 실제 클라우드 환경 생성/게시와 클라우드 계산은 별도로 확인해야 한다. 로컬 GitHub CLI 로그인과 Codex 웹의 GitHub 연결은 별도 설정이다.

1. 같은 ChatGPT 계정으로 웹에 로그인한다. 새 작업에서 **Work in → Cloud → Select environment → Create environment**를 선택한다. **Settings → Codex Cloud → Environments → Create environment**에서도 만들 수 있다.
2. GitHub 연결이 필요하면 개인 계정의 `xlerobot` 저장소를 연결하고 선택한다. 팀 조직의 추가 접근 권한은 이번 작업에 필요하지 않다.
3. **Get started**로 환경 준비를 시작한다. 아래 요청문을 사용한다.

```text
이 저장소는 LEAP 오른손 큐브 조작 프로젝트의 선별한 인계본입니다.
AGENTS.md와 CURRENT_STATE.md를 읽고, 저장소 루트에서 bash cloud/setup.sh로
가상환경과 의존성을 준비해 주세요. 원본 3,519개 파일과 결합 원본 170개의
해시 및 MuJoCo 저장 모델의 기본 검사를 확인해 주세요.
모터 명령, .cmd 실행, 카메라 촬영은 하지 마세요.
전체 과거 학습/모델 생성 환경이 이식됐다고 가정하지 말고
이 최소 환경에서 가능한 범위를 설치 결과에 기록해 주세요.
```

4. 준비 결과에서 `cloud/smoke.py` 검사 성공과 파일 접근을 확인한다. 실패한 설치나 필요한 접근은 해결한 뒤 **Publish**를 선택한다.
5. **Environment published**가 나온 뒤 새 작업을 시작하고 `cloud/START_PROMPT.txt`를 붙여 넣는다. 같은 작업을 다른 PC에서 다시 열어 이어간다.

전체 기존 대화 맥락을 IDE에서 새 클라우드 작업으로 넘기려면, 게시된 환경을 VS Code 입력창 아래 클라우드 아이콘에서 선택하고 다음 요청을 보낸다. 해당 선택 항목이 보이는지는 실제 설치 화면에서 확인한다. 이 기능과 모든 로컬 대화 기록의 자동 동기화를 혼동하지 않는다.

현재 저장소의 검사는 원본 Windows와 Linux WSL에서 수행했다. 실제 Codex Cloud에서 수행한 검사로 기록하지 않는다. 원래 PC의 로봇 USB와 웹캠은 이 이전만으로 클라우드에 연결되지 않는다.

공식 안내: [클라우드 환경 만들기](https://learn.chatgpt.com/docs/environments/cloud-environments), [IDE에서 맥락을 넘기는 방식](https://learn.chatgpt.com/docs/prompting).
