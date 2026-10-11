# duaro-rdk-x5

z축(Feetech STS-3215 리프트) + 턴테이블(NEMA23 + DM542T)을 RDK X5 보드에서
조그 / 상태 기록 / 재현하는 코드.

## 폴더 구성
```
z_axis_lift/     z축 드라이버 (feetech_sts, lift_axis)
turn_table/      턴테이블 드라이버 (turn_axis, 상대 회전 전용)
config/          setup.py 가 만드는 z축 캘리브레이션·상태 JSON — git 미추적
poses.csv        jog_record.py 가 쌓는 상태 목록 — git 미추적
common.py        경로 / 축 열기 / CSV / 터미널 / 조그 공용
setup.py         ① z축 캘리브레이션
jog_record.py    ② 상태 기록
jog.py           ③ 조그 + 상태 재현
```

## 하드웨어 연결
- z축: CH343 USB 어댑터 → `/dev/ttyACM0`, 서보 ID 11, 12V 전원
- 턴테이블: DM542T +Vdc/GND ← 24V 어댑터, 신호 스위치 5V 위치,
  모터 검정→A+ 초록→A− 빨강→B+ 파랑→B−,
  RDK X5 핀31→PUL+, 핀33→DIR+, 핀39→PUL−, 핀34→DIR− (3.3V 직접 구동),
  DIP: 전류 2.84A(SW1 on, SW2 on, SW3 off), SW4 off, 1600 pulse/rev(SW5 off, SW6 off, SW7 on, SW8 on)

## 사용 순서
```bash
# ① z축 캘리브레이션 (처음 한 번: 영점 → 높이A → 높이B → 상한)
python3 setup.py            # 다시 하려면 --recalib

# ② 상태 기록 (한 번 실행 = 한 상태)
python3 jog_record.py       # 상태명 → z축 조그+Enter → 턴테이블 조그+Enter → 순서(1/2)

# ③ 조그 + 재현 (Ctrl-C 로 종료)
python3 jog.py              # n → 상태명 입력 → 이동
```

## 키
| 축 | 키 |
|---|---|
| z축 | `u`/`k` (↑/↓) 상하, `+`/`-` 속도, `g` mm 입력 이동, `h` 하드스톱 호밍, `z` 현재=0mm, `t` 토크 ON/OFF |
| 턴테이블 | `a`/`d` (←/→) 좌/우 회전, `w`/`e` 왼쪽/오른쪽 90도, `,`/`.` 속도 |
| 공통 | `space` 정지, `n` 상태로 이동(jog.py), `l` 상태 목록(jog.py), `Enter` 기록(jog_record.py) |

## 상태 형식 (poses.csv)
```
name,z_mm,turn_deg,order
상태1,100.0,-180.0,1
상태2,200.0,180.0,2
```
- `z_mm`: 절대 높이. `turn_deg`: 지금 위치에서 돌릴 상대 각도 (+ = 오른쪽).
- `order`: 1 = z축 먼저, 2 = 턴테이블 먼저.
- 같은 이름으로 다시 기록하면 그 줄만 덮어쓴다.