"""
common.py — 경로 / 축 열기 / 포즈 CSV / 터미널 / 조그 공용

폴더:
  z_axis_lift/  turn_table/   축 모듈
  config/                     setup.py 가 만드는 z축 캘리브레이션·상태 JSON (git 미추적)
  poses.csv                   jog_record.py 가 쌓는 상태 목록
"""
from __future__ import annotations

import csv
import json
import os
import select
import sys
import termios
import time
import tty

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(HERE, "config")
LIFT_CONFIG = os.path.join(CONFIG_DIR, "lift_config.json")
LIFT_STATE = os.path.join(CONFIG_DIR, "lift_state.json")
POSES_CSV = os.path.join(HERE, "poses.csv")

for _sub in ("z_axis_lift", "turn_table"):
    _p = os.path.join(HERE, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

DEFAULT_PORT = os.environ.get("LIFT_PORT", "/dev/ttyACM0")
DEFAULT_BAUD = 1_000_000

HOLD_S = 0.6            # 마지막 키 입력 후 이 시간 지나면 정지 (터미널 auto-repeat 지연보다 길게)
GOTO_TIMEOUT_S = 60.0

Z_FIRST, TURN_FIRST = "1", "2"
ORDER_LABEL = {Z_FIRST: "z축 먼저", TURN_FIRST: "턴테이블 먼저"}

LIFT_KEYS = "u/k(↑/↓)=상하  +/-=속도  g=mm 입력 이동  h=호밍  z=영점  t=토크 ON/OFF"
TURN_KEYS = "a/d(←/→)=좌/우 회전  w/e=왼쪽/오른쪽 90도  ,/.=속도"


# ---------------------------------------------------------------- 축 열기
def lift_calibrated() -> bool:
    return os.path.exists(LIFT_CONFIG)


def open_lift(port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD):
    """(bus, LiftAxis). config/lift_config.json 있으면 그 값, 없으면 기본값."""
    from feetech_sts import FeetechBus
    from lift_axis import LiftAxis, LiftAxisConfig

    os.makedirs(CONFIG_DIR, exist_ok=True)
    cfg = LiftAxisConfig.from_json(LIFT_CONFIG) if lift_calibrated() else LiftAxisConfig()
    bus = FeetechBus(port, baudrate=baud).open()
    if not bus.ping(cfg.motor_id):
        bus.close()
        raise RuntimeError(f"z축 서보 id {cfg.motor_id} 응답 없음 (port={port}). 12V 전원/케이블 확인.")
    lift = LiftAxis(bus, cfg, state_path=LIFT_STATE)
    lift.configure(resume=True)
    return bus, lift


def open_turn():
    from turn_axis import TurnAxis
    return TurnAxis().open()


def park_lift(lift, tag: str, enabled: bool = True) -> None:
    """캘리브레이션·영점이 있으면 z축을 맨 밑(soft_min, 하강 하한 적용)으로 내린다."""
    if not (enabled and lift_calibrated() and lift.homed):
        return
    print(f"[{tag}] z축 맨 밑으로 이동...")
    try:
        h = lift.goto_mm(lift.cfg.soft_min_mm, timeout_s=GOTO_TIMEOUT_S)
        print(f"[{tag}] z = {h:.1f} mm")
    except RuntimeError as e:
        print(f"[{tag}] 이동 실패: {e}")


def save_lift_config(lift) -> None:
    tmp = LIFT_CONFIG + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(lift.config_dict(), f, indent=2, ensure_ascii=False)
    os.replace(tmp, LIFT_CONFIG)


# ---------------------------------------------------------------- poses.csv
CSV_FIELDS = ("name", "z_mm", "turn_deg", "order")


def load_poses(path: str = POSES_CSV) -> dict:
    """{name: {"z_mm": float, "turn_deg": float, "order": "1"|"2"}}  (파일 순서 유지)"""
    if not os.path.exists(path):
        return {}
    with open(path, newline="", encoding="utf-8") as f:
        return {r["name"]: {"z_mm": float(r["z_mm"]), "turn_deg": float(r["turn_deg"]), "order": r["order"]}
                for r in csv.DictReader(f)}


def save_pose(name: str, z_mm: float, turn_deg: float, order: str, path: str = POSES_CSV) -> bool:
    """같은 이름이면 그 줄을 덮어쓰고, 없으면 끝에 추가. 이미 있었으면 True."""
    if order not in ORDER_LABEL:
        raise ValueError(f"order must be {Z_FIRST} or {TURN_FIRST}")
    poses = load_poses(path)
    existed = name in poses
    poses[name] = {"z_mm": round(z_mm, 1), "turn_deg": round(turn_deg, 2), "order": order}
    tmp = path + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        for n, p in poses.items():
            w.writerow({"name": n, **p})
    os.replace(tmp, path)
    return existed


def format_pose(name: str, p: dict) -> str:
    return f"{name}: z={p['z_mm']} mm, turn={p['turn_deg']:+} deg ({ORDER_LABEL[p['order']]})"


# ---------------------------------------------------------------- 터미널
class RawTerminal:
    """cbreak 모드 키 입력. Ctrl-C 는 그대로 KeyboardInterrupt."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def getch(self, timeout: float = 0.0):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None

    def ask(self, prompt: str) -> str:
        """잠시 일반 모드로 돌아가 한 줄 입력."""
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        try:
            return input(prompt).strip()
        finally:
            tty.setcbreak(self.fd)

    def ask_float(self, prompt: str) -> float:
        while True:
            try:
                return float(self.ask(prompt))
            except ValueError:
                print("숫자만 입력")


ARROWS = {"A": "u", "B": "k", "C": "d", "D": "a"}   # ESC [ A/B/C/D


# ---------------------------------------------------------------- 조그
class Jog:
    """
    키 → 두 축 조그. 루프마다 key(ch) 로 입력을 넘기고 tick() 을 한 번 부른다.
    lift / turn 중 None 인 축의 키는 무시한다.
    """

    def __init__(self, term: RawTerminal, lift=None, turn=None, guard: bool = True):
        self.term, self.lift, self.turn, self.guard = term, lift, turn, guard
        self.v_cmd, self.lift_deadline = 0.0, 0.0
        self.turn_dir, self.turn_deadline = 0, 0.0
        self.torque_on = True
        self.set_speed_range()

    def set_speed_range(self) -> None:
        """v_max 기준으로 조그 속도/증감폭 설정. 캘리브레이션으로 v_max 가 바뀌면 다시 부른다."""
        v_max = self.lift.cfg.v_max if self.lift else 0
        self.speed_step = max(100, v_max // 10)
        self.speed = max(self.speed_step, v_max // 2)

    def poll_timeout(self) -> float:
        if self.turn_dir and time.monotonic() < self.turn_deadline:
            return 0.0
        return 0.02 if self.v_cmd else 0.05

    def stop(self) -> None:
        self.v_cmd, self.turn_dir = 0.0, 0
        if self.lift:
            self.lift.stop()

    # ---- 키 ----
    def key(self, ch: str) -> bool:
        """처리했으면 True."""
        if ch == "\x1b":
            seq = (self.term.getch(0.01) or "") + (self.term.getch(0.01) or "")
            ch = ARROWS.get(seq[-1:] if seq[:1] == "[" else "", "")
            if not ch:
                return True
        if ch == " ":
            self.stop()
            return True
        now = time.monotonic()
        if self.lift and self._lift_key(ch, now):
            return True
        if self.turn and self._turn_key(ch, now):
            return True
        return False

    def _lift_key(self, ch: str, now: float) -> bool:
        lift = self.lift
        if ch == "u":
            self.v_cmd, self.lift_deadline = +self.speed, now + HOLD_S
        elif ch == "k":
            self.v_cmd, self.lift_deadline = -self.speed, now + HOLD_S
        elif ch in ("+", "="):
            self.speed = min(lift.cfg.v_max, self.speed + self.speed_step)
            print(f"\n[z] 속도 {self.speed} tick/s")
        elif ch == "-":
            self.speed = max(self.speed_step, self.speed - self.speed_step)
            print(f"\n[z] 속도 {self.speed} tick/s")
        elif ch == "g":
            self.stop()
            s = self.term.ask("\n[z] goto mm: ")
            if s:
                try:
                    self.goto_mm(float(s))
                except ValueError:
                    print("[z] 숫자 아님")
        elif ch == "h":
            self.stop()
            print("\n[z] 하드스톱 호밍...")
            lift.home(log=None)
            print("[z] 영점 저장 (0 mm)")
        elif ch == "z":
            self.stop()
            lift.zero_here()
            print("\n[z] 영점 저장 (0 mm)")
        elif ch == "t":
            self.stop()
            self.torque_on = not self.torque_on
            (lift.bus.torque_on if self.torque_on else lift.bus.torque_off)(lift.id)
            print(f"\n[z] 토크 {'ON' if self.torque_on else 'OFF'}")
        else:
            return False
        return True

    def _turn_key(self, ch: str, now: float) -> bool:
        turn = self.turn
        if ch == "d":
            self.turn_dir, self.turn_deadline = +1, now + HOLD_S
        elif ch == "a":
            self.turn_dir, self.turn_deadline = -1, now + HOLD_S
        elif ch in ("e", "w"):
            self.turn_dir = 0
            deg = 90.0 if ch == "e" else -90.0
            print(f"\n[turn] {'오른쪽' if deg > 0 else '왼쪽'} 90도...")
            turn.rotate_deg(deg)
        elif ch == ".":
            turn.speed_up()
            print(f"\n[turn] 속도 up (delay={turn.delay * 1000:.2f} ms)")
        elif ch == ",":
            turn.speed_down()
            print(f"\n[turn] 속도 down (delay={turn.delay * 1000:.2f} ms)")
        else:
            return False
        return True

    # ---- 동작 ----
    def goto_mm(self, mm: float) -> float | None:
        """z축 절대 이동. 영점 없으면 거부, 과전류면 토크 OFF 상태로 남긴다."""
        if not self.lift.homed:
            print("[z] 영점 없음 (h 또는 z 먼저)")
            return None
        try:
            return self.lift.goto_mm(mm, timeout_s=GOTO_TIMEOUT_S)
        except RuntimeError as e:
            self.torque_on = False
            print(f"\n[SAFETY] {e} → 토크 OFF. t 로 다시 켜기")
            return None

    def tick(self) -> None:
        now = time.monotonic()
        if self.lift:
            if self.v_cmd and now > self.lift_deadline:
                self.v_cmd = 0.0
            if self.torque_on:
                if self.lift.check_overcurrent():
                    self.torque_on = False
                    self.v_cmd = 0.0
                    print(f"\n[SAFETY] z축 과전류 {self.lift.cfg.cutoff_current_ma:.0f} mA 초과 → 토크 OFF. t 로 다시 켜기")
                else:
                    self.lift.set_velocity(self.v_cmd, guard=self.guard)
        if self.turn:
            if self.turn_dir and now < self.turn_deadline:
                self.turn.jog_batch(self.turn_dir)
            else:
                self.turn_dir = 0
                self.turn.reset_ramp()

    def status(self) -> str:
        parts = []
        if self.lift:
            parts.append(f"z={self.lift.get_height_mm():7.2f} mm (v={self.speed}, "
                         f"{'영점 OK' if self.lift.homed else '영점 없음'})")
        if self.turn:
            parts.append(f"turn={self.turn.rel_deg():+8.2f} deg (delay={self.turn.delay * 1000:.2f} ms)")
        return " | ".join(parts)


def jog_loop(term: RawTerminal, jog: Jog, on_key, status_every: float = 0.2) -> None:
    """
    조그 루프. 모든 키는 먼저 jog 가 보고, 그 다음 on_key(ch, handled) 가 본다.
    on_key 가 True 를 돌려주면 축을 멈추고 루프를 끝낸다.
    """
    last = 0.0
    while True:
        first = True
        while True:                       # 쌓인 입력 전부 소화 (space 가 뒤에 묻히지 않게)
            ch = term.getch(jog.poll_timeout() if first else 0.0)
            first = False
            if ch is None:
                break
            handled = jog.key(ch)
            if on_key(ch, handled):
                jog.stop()
                return
        jog.tick()
        now = time.monotonic()
        if now - last > status_every:
            last = now
            print("\r" + jog.status() + "   ", end="", flush=True)
