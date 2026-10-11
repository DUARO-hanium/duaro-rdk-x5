"""
turn_axis.py — 턴테이블(스테퍼) 상대 회전 컨트롤러

하드웨어 (2026-08-28 동작 확인):
  * 모터 23HS22-2804S1 + 드라이버 DM542T
    DIP: 전류 2.84A(SW1 on, SW2 on, SW3 off), SW4 off, 1600 pulse/rev(SW5 off, SW6 off, SW7 on, SW8 on)
  * 모터-테이블 직결 → 1600펄스 = 360도
  * RDK X5 3.3V 직접 구동: 핀31→PUL+, 핀33→DIR+, 핀39→PUL−, 핀34→DIR− (active-high)
  * 모터 검정→A+ 초록→A− 빨강→B+ 파랑→B−

위치 센서가 없으므로 절대 각도·영점은 두지 않고 상대 회전만 한다.
+deg = DIR HIGH = 오른쪽(시계). 프로세스 시작(또는 mark) 이후 보낸 펄스는 세어 둔다.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import Hobot.GPIO as GPIO


@dataclass
class TurnConfig:
    pul_pin: int = 31              # BOARD 번호
    dir_pin: int = 33
    steps_per_rev: int = 1600      # DM542T DIP 와 일치해야 함
    delay: float = 0.0005          # 기본 순항 펄스 반주기
    delay_min: float = 0.0001      # 최고 속도
    delay_max: float = 0.01        # 최저 속도 (한 바퀴 약 32초)
    ramp_start: float = 0.0008     # 출발 시 이 delay 부터 서서히 가속 (탈조 방지)
    ramp_factor: float = 0.93      # 배치(조그)/펄스(회전)마다 delay 를 이 비율로 줄임
    batch: int = 40                # 조그 한 번에 보내는 펄스 수
    dir_setup_s: float = 0.002     # DIR 변경 후 대기


class TurnAxis:
    def __init__(self, cfg: TurnConfig | None = None):
        self.cfg = cfg or TurnConfig()
        self.delay = self.cfg.delay
        self._ramp = self.cfg.ramp_start
        self._dir_cw: bool | None = None
        self._steps = 0
        self._open = False

    def open(self) -> "TurnAxis":
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.cfg.pul_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.cfg.dir_pin, GPIO.OUT, initial=GPIO.LOW)
        self._open = True
        return self

    def close(self) -> None:
        if self._open:
            GPIO.cleanup((self.cfg.pul_pin, self.cfg.dir_pin))
            self._open = False

    # ---------------- 속도 ----------------
    def speed_up(self) -> None:
        self.delay = max(self.cfg.delay_min, self.delay / 1.5)

    def speed_down(self) -> None:
        self.delay = min(self.cfg.delay_max, self.delay * 1.5)

    # ---------------- 상대 각도 ----------------
    def rel_deg(self) -> float:
        """mark() 이후 돌린 각도 (+ = 오른쪽)."""
        return self._steps * 360.0 / self.cfg.steps_per_rev

    def mark(self) -> None:
        self._steps = 0

    # ---------------- 펄스 ----------------
    def _set_dir(self, cw: bool) -> None:
        if self._dir_cw != cw:
            GPIO.output(self.cfg.dir_pin, GPIO.HIGH if cw else GPIO.LOW)
            time.sleep(self.cfg.dir_setup_s)
            self._dir_cw = cw

    def _pulse(self, half_period: float) -> None:
        GPIO.output(self.cfg.pul_pin, GPIO.HIGH)
        time.sleep(half_period)
        GPIO.output(self.cfg.pul_pin, GPIO.LOW)
        time.sleep(half_period)

    def reset_ramp(self) -> None:
        self._ramp = max(self.cfg.ramp_start, self.delay)

    def jog_batch(self, direction: int) -> None:
        """조그용: batch 펄스. 연속 호출하면 ramp_start 에서 delay 까지 서서히 가속."""
        cw = direction > 0
        self._set_dir(cw)
        self._ramp = max(self.delay, self._ramp * self.cfg.ramp_factor)
        for _ in range(self.cfg.batch):
            self._pulse(self._ramp)
        self._steps += self.cfg.batch if cw else -self.cfg.batch

    def rotate_deg(self, deg: float) -> float:
        """상대 회전 (블로킹). + = 오른쪽. 실제 돌린 각도 반환."""
        n = round(deg * self.cfg.steps_per_rev / 360.0)
        if n == 0:
            return 0.0
        self._set_dir(n > 0)
        cur = self.cfg.ramp_start
        for _ in range(abs(n)):
            cur = max(self.delay, cur * self.cfg.ramp_factor)
            self._pulse(cur)
        self._steps += n
        return n * 360.0 / self.cfg.steps_per_rev
