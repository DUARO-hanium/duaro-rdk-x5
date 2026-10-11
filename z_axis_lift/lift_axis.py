"""
lift_axis.py — z축(리프트) 컨트롤러

lerobot_alohamini 의 src/lerobot/robots/alohamini/lift_axis.py (LiftAxis) 를
FeetechBus(feetech_sts.py) 위로 이식한 것. 제어 방식은 AlohaMini 와 동일:

  * 서보를 VELOCITY 모드로 두고
  * Present_Position(0..4095, 1회전에 wrap)을 주기적으로 읽어 다회전 누적 tick 을 추적
  * 누적 tick → 각도 → mm 로 환산 (lead_mm_per_rev, output_gear_ratio)
  * 목표 높이(mm)는 P 제어로 속도 명령을 만들어 폐루프 추종
  * 하드스톱 호밍: 아래로 내리다 스톨(전류↑ 또는 정지)하면 그 위치를 0 mm 로 설정

추가된 것:
  * 상태 파일(state_path)에 누적 tick / 영점 저장 → CLI 를 여러 번 나눠 실행해도 높이 기준 유지
    (프로세스가 꺼져 있는 동안 서보가 반 바퀴(2048 tick) 이상 돌면 기준이 틀어짐 → 그땐 다시 home)
  * 과전류 컷오프(cutoff_current_ma) — examples/debug/axis.py 의 안전장치
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, asdict, fields

from feetech_sts import (
    FeetechBus, FeetechError, MODE_VELOCITY, TICKS_PER_REV, CURRENT_MA_PER_UNIT,
)


@dataclass
class LiftAxisConfig:
    # --- 서보 ---
    motor_id: int = 11               # AlohaMini 리프트 서보 ID (assembly_guide.md)
    motor_model: str = "sts3215"

    # --- 기구 환산 (★ 커스텀 z축이면 반드시 실측해서 맞출 것) ---
    lead_mm_per_rev: float = 84.0    # 서보 1회전당 z 이동량 [mm]  (AlohaMini1 STS3215 리프트 = 84)
    output_gear_ratio: float = 1.0   # 서보 각도 → 출력축 각도 비 (감속 없으면 1.0)
    dir_sign: int = -1               # 서보 + 방향 회전이 "내려감"이면 -1, "올라감"이면 +1

    # --- 소프트 리밋 ---
    soft_min_mm: float = 0.0
    soft_max_mm: float = 600.0       # 리프트 스트로크 [mm]  (★ 실제 스트로크로 수정)
    descent_floor_mm: float = 5.0    # 이 높이 이하에서는 하강 명령 무시 (스톨 방지)

    # --- 호밍 (아래로 내려 하드스톱 → 그 자리를 0mm) ---
    home_speed: int = 1300           # 호밍 속도 [tick/s]
    home_stall_current_ma: float = 300.0
    home_timeout_s: float = 30.0

    # --- 속도 폐루프 ---
    kp_vel: float = 300.0            # [tick/s per mm]
    v_max: int = 1300                # 속도 제한 [tick/s]  (STS3215 무부하 최대 ≈ 3400)
    on_target_mm: float = 1.0
    step_mm: float = 2.0             # jog 1회 이동량 [mm]

    # --- 안전 ---
    cutoff_current_ma: float = 1000.0   # 연속 cutoff_samples 회 초과 시 토크 차단
    cutoff_samples: int = 2

    @classmethod
    def from_json(cls, path: str) -> "LiftAxisConfig":
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
        allowed = {f.name for f in fields(cls)}
        unknown = set(raw) - allowed
        if unknown:
            raise ValueError(f"unknown config keys in {path}: {sorted(unknown)}")
        return cls(**raw)


class LiftAxis:
    def __init__(self, bus: FeetechBus, cfg: LiftAxisConfig, state_path: str | None = None):
        self.bus = bus
        self.cfg = cfg
        self.id = cfg.motor_id
        self.state_path = state_path

        self._deg_per_tick = 360.0 / TICKS_PER_REV
        self._mm_per_deg = (cfg.lead_mm_per_rev * cfg.output_gear_ratio) / 360.0

        self._last_tick: float = 0.0
        self._extended_ticks: float = 0.0
        self._z0_deg: float = 0.0
        self._homed: bool = False
        self._configured = False
        self._over_cnt = 0

    # ---------------- 상태 저장/복원 ----------------
    def save_state(self) -> None:
        if not self.state_path:
            return
        data = {
            "motor_id": self.id,
            "last_tick": self._last_tick,
            "extended_ticks": self._extended_ticks,
            "z0_deg": self._z0_deg,
            "homed": self._homed,
            "saved_at": time.time(),
        }
        tmp = self.state_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f)
        os.replace(tmp, self.state_path)

    def _load_state(self) -> bool:
        if not self.state_path or not os.path.exists(self.state_path):
            return False
        try:
            with open(self.state_path, "r", encoding="utf-8") as f:
                d = json.load(f)
        except (OSError, json.JSONDecodeError):
            return False
        if d.get("motor_id") != self.id:
            return False
        self._last_tick = float(d["last_tick"])
        self._extended_ticks = float(d["extended_ticks"])
        self._z0_deg = float(d["z0_deg"])
        self._homed = bool(d.get("homed", False))
        return True

    def clear_state(self) -> None:
        if self.state_path and os.path.exists(self.state_path):
            os.remove(self.state_path)

    # ---------------- 초기화 ----------------
    def configure(self, resume: bool = True) -> None:
        """VELOCITY 모드 설정 + 토크 ON. resume=True 면 상태 파일에서 누적 tick/영점 복원."""
        if self._configured:
            return
        mode = self.bus.read(self.id, "Operating_Mode")
        if mode != MODE_VELOCITY:
            self.bus.set_operating_mode(self.id, MODE_VELOCITY)   # torque off + lock 0 + write
        else:
            self.bus.torque_off(self.id)
        self.bus.write(self.id, "Goal_Velocity", 0)
        self.bus.torque_on(self.id)

        restored = resume and self._load_state()
        if restored:
            # 꺼져 있던 동안의 이동분(<½회전) 반영
            self._update_extended_ticks()
        else:
            self._last_tick = float(self.bus.read(self.id, "Present_Position"))
            self._extended_ticks = 0.0
            self._z0_deg = 0.0
            self._homed = False
        self._configured = True

    @property
    def homed(self) -> bool:
        return self._homed

    # ---------------- 위치 추적 ----------------
    def _update_extended_ticks(self) -> None:
        cur = float(self.bus.read(self.id, "Present_Position")) % TICKS_PER_REV
        delta = cur - self._last_tick
        half = TICKS_PER_REV * 0.5
        if delta > half:
            delta -= TICKS_PER_REV
        elif delta < -half:
            delta += TICKS_PER_REV
        self._extended_ticks += delta
        self._last_tick = cur

    def _extended_deg(self) -> float:
        return self.cfg.dir_sign * self._extended_ticks * self._deg_per_tick

    def get_height_mm(self) -> float:
        self._update_extended_ticks()
        return (self._extended_deg() - self._z0_deg) * self._mm_per_deg

    def raw_ticks(self) -> tuple[float, float]:
        """(현재 raw tick 0..4095, 누적 tick)"""
        self._update_extended_ticks()
        return self._last_tick, self._extended_ticks

    def read_current_ma(self) -> float:
        try:
            return abs(self.bus.read(self.id, "Present_Current")) * CURRENT_MA_PER_UNIT
        except FeetechError:
            return 0.0

    # ---------------- 명령 ----------------
    def _write_vel_user(self, v_user: float) -> None:
        """사용자 프레임 속도(+ = 위) → 서보 raw 속도."""
        v = int(max(-self.cfg.v_max, min(self.cfg.v_max, v_user)))
        self.bus.write(self.id, "Goal_Velocity", v * self.cfg.dir_sign)

    def _guard(self, v_user: float, cur_mm: float) -> float:
        c = self.cfg
        if v_user < 0 and cur_mm <= c.descent_floor_mm:
            return 0.0
        if (cur_mm >= c.soft_max_mm and v_user > 0) or (cur_mm <= c.soft_min_mm and v_user < 0):
            return 0.0
        return v_user

    def check_overcurrent(self) -> bool:
        """과전류가 연속 감지되면 정지+토크 OFF 후 True 반환."""
        ma = self.read_current_ma()
        if ma > self.cfg.cutoff_current_ma:
            self._over_cnt += 1
        else:
            self._over_cnt = 0
        if self._over_cnt >= self.cfg.cutoff_samples:
            self.emergency_stop()
            return True
        return False

    def set_velocity(self, v_user: float, guard: bool = True) -> float:
        """속도 명령 [tick/s], + = 위. 소프트리밋 적용. 실제 쓴 값 반환."""
        if guard:
            v_user = self._guard(v_user, self.get_height_mm())
        self._write_vel_user(v_user)
        return v_user

    def stop(self) -> None:
        self.bus.write(self.id, "Goal_Velocity", 0)

    def emergency_stop(self) -> None:
        try:
            self.stop()
        finally:
            self.bus.torque_off(self.id)

    def step_towards(self, target_mm: float) -> tuple[float, float, bool]:
        """P 제어 한 스텝. (현재 mm, 쓴 속도, 도달 여부) 반환."""
        cur = self.get_height_mm()
        err = target_mm - cur
        if abs(err) <= self.cfg.on_target_mm:
            self._write_vel_user(0)
            return cur, 0.0, True
        v = max(-self.cfg.v_max, min(self.cfg.v_max, self.cfg.kp_vel * err))
        v = self._guard(v, cur)
        self._write_vel_user(v)
        return cur, v, v == 0.0  # 가드로 0 이 됐으면 더 못 가므로 종료

    def goto_mm(self, target_mm: float, timeout_s: float = 30.0, period_s: float = 0.03,
                on_step=None, settle_s: float = 0.5) -> float:
        """목표 높이까지 블로킹 이동. 도달 후 settle_s 동안 관성 오버슈트를 보정하고 정지, 현재 높이 반환."""
        target_mm = max(self.cfg.soft_min_mm, self.cfg.descent_floor_mm, min(self.cfg.soft_max_mm, target_mm))
        t_end = time.monotonic() + timeout_s
        t_done = None
        try:
            while time.monotonic() < t_end:
                if self.check_overcurrent():
                    raise RuntimeError("overcurrent cutoff during goto")
                cur, v, done = self.step_towards(target_mm)
                if on_step:
                    on_step(cur, v)
                if done and t_done is None:
                    t_done = time.monotonic()
                if t_done is not None and time.monotonic() - t_done >= settle_s:
                    break
                time.sleep(period_s)
        finally:
            self.stop()
            self.save_state()
        return self.get_height_mm()

    def zero_here(self) -> None:
        """현재 위치를 0 mm 로 설정 (호밍 없이 수동 영점)."""
        self._update_extended_ticks()
        self._z0_deg = self._extended_deg()
        self._homed = True
        self.save_state()

    def home(self, log=print) -> float:
        """아래로 내려 하드스톱 스톨 감지 → 영점. AlohaMini LiftAxis.home 과 동일 로직."""
        c = self.cfg
        self._write_vel_user(-abs(c.home_speed))
        stuck = 0
        period = 0.1
        min_move = max(3.0, abs(c.home_speed) * period * 0.25)   # 기대 이동량의 25% 미만이면 "안 움직임"
        last_ext = self._extended_ticks
        t0 = time.monotonic()
        t_end = t0 + c.home_timeout_s
        try:
            while time.monotonic() < t_end:
                time.sleep(period)
                self._update_extended_ticks()
                moved = abs(self._extended_ticks - last_ext) > min_move
                last_ext = self._extended_ticks
                ma = self.read_current_ma()
                if log:
                    log(f"[home] tick={self._last_tick:.0f} ext={self._extended_ticks:.0f} I={ma:.0f}mA")
                if time.monotonic() - t0 < 0.5:        # 출발 가속 구간은 판정 제외
                    continue
                if ma >= c.home_stall_current_ma or not moved:
                    stuck += 1
                else:
                    stuck = 0
                if stuck >= 3:
                    break
        finally:
            self.stop()
        # AlohaMini 는 여기서 토크를 잠깐 풀어 기구 응력을 뺀 뒤 영점 설정
        self.bus.torque_off(self.id)
        time.sleep(1.0)
        self._update_extended_ticks()
        self._z0_deg = self._extended_deg()
        self._homed = True
        self.bus.torque_on(self.id)
        self.save_state()
        h = self.get_height_mm()
        if log:
            log(f"[home] done. z0_deg={self._z0_deg:.1f}, height={h:.2f} mm")
        return h

    def set_lead(self, lead_mm_per_rev: float, max_mm_s: float = 40.0) -> dict:
        """리드 갱신 + 최고속도(max_mm_s)에 맞춰 v_max/kp_vel 재산정. 갱신된 값 dict 반환."""
        self.cfg.lead_mm_per_rev = lead_mm_per_rev
        self._mm_per_deg = (lead_mm_per_rev * self.cfg.output_gear_ratio) / 360.0
        self.cfg.v_max = int(max_mm_s * TICKS_PER_REV / lead_mm_per_rev)   # tick/s
        self.cfg.kp_vel = round(self.cfg.v_max / 6.0, 1)                     # 6 mm 남았을 때부터 감속
        self.cfg.home_speed = int(15.0 * TICKS_PER_REV / lead_mm_per_rev)   # 호밍(하드스톱 충돌)은 15 mm/s
        return {"lead_mm_per_rev": round(lead_mm_per_rev, 3), "v_max": self.cfg.v_max,
                "kp_vel": self.cfg.kp_vel, "home_speed": self.cfg.home_speed}

    def config_dict(self) -> dict:
        return asdict(self.cfg)
