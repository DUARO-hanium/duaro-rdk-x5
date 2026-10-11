#!/usr/bin/env python3
"""
setup.py — z축 캘리브레이션 (처음 한 번, 또는 기구가 바뀌었을 때)

  python3 setup.py                # config/lift_config.json 없으면 마법사 시작
  python3 setup.py --recalib      # 캘리브레이션 다시
  python3 setup.py --port /dev/ttyACM0

절차:
  [1/4] 영점     : u/k 로 최저점까지 내린 뒤  z (여기가 0 mm)  또는  h (하드스톱 호밍)
  [2/4] 높이 A   : 리프트의 한 지점을 자로 잰 높이 입력
  [3/4] 높이 B   : u 로 넉넉히 올린 뒤 Enter → 같은 지점의 높이 입력 → 환산비 계산·저장
  [4/4] 상한     : u 로 위 끝까지 올린 뒤 Enter → soft_max 저장
결과: config/lift_config.json (환산비·속도·상한), config/lift_state.json (누적 tick·영점)
턴테이블은 캘리브레이션 없음 (상대 회전만 사용).
"""
from __future__ import annotations

import argparse

import common
from common import LIFT_KEYS, Jog, RawTerminal, jog_loop, lift_calibrated, open_lift, park_lift, save_lift_config

STEPS = {
    "zero": "[1/4] u/k 로 z축을 최저점까지 내린 뒤  z (여기가 0)  또는  h (하드스톱 호밍)",
    "a":    "[2/4] 리프트의 한 지점을 정해 자로 잰 높이 A 입력",
    "b":    "[3/4] u 로 넉넉히 올린 뒤 Enter → 같은 지점의 높이 B 입력 (환산비 자동 계산)",
    "top":  "[4/4] u 로 위 끝(더 올리면 안 되는 지점)까지 올린 뒤 Enter (상한 저장)",
    None:   "캘리브레이션 완료 상태. z/h 로 영점만 다시 잡을 수 있음. q 로 종료.",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=common.DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=common.DEFAULT_BAUD)
    ap.add_argument("--recalib", action="store_true", help="z축 캘리브레이션 다시")
    args = ap.parse_args()

    bus, lift = open_lift(args.port, args.baud)
    park_lift(lift, "setup")
    jog = None
    st = {"calib": "zero" if (args.recalib or not lift_calibrated()) else None,
          "a_mm": 0.0, "ext_a": 0.0}

    print("키:", LIFT_KEYS, " space=정지  q=종료")
    print(STEPS[st["calib"]])
    try:
        with RawTerminal() as term:
            jog = Jog(term, lift=lift, guard=st["calib"] is None)   # 캘리브레이션 중엔 소프트리밋 끔

            def on_key(ch, handled):
                c = st["calib"]
                if ch == "q":
                    return True
                if handled and ch in ("z", "h") and c == "zero":
                    print(STEPS["a"])
                    st["a_mm"] = term.ask_float("      높이 A [mm]: ")
                    _, st["ext_a"] = lift.raw_ticks()
                    st["calib"] = "b"
                    print(STEPS["b"])
                elif ch in ("\r", "\n") and c in ("b", "top"):
                    jog.stop()
                    if c == "b":
                        _, ext_b = lift.raw_ticks()
                        ext = ext_b - st["ext_a"]
                        b_mm = term.ask_float("\n      높이 B [mm]: ")
                        moved = abs(b_mm - st["a_mm"])
                        if abs(ext) < 100 or moved <= 0:
                            print("      이동량이 너무 작음. 더 올린 뒤 다시 Enter")
                        else:
                            lead = moved * 4096.0 / abs(ext)
                            upd = lift.set_lead(lead)
                            save_lift_config(lift)
                            jog.set_speed_range()
                            print(f"      ext={ext:.0f} tick, 이동 {moved:.1f} mm → 환산비 {lead:.3f} mm/회전, {upd} 저장")
                            st["calib"] = "top"
                            print(STEPS["top"])
                    else:
                        top = round(lift.get_height_mm(), 1)
                        lift.cfg.soft_max_mm = top
                        save_lift_config(lift)
                        jog.guard = True
                        st["calib"] = None
                        print(f"      상한 soft_max_mm={top} 저장 → {common.LIFT_CONFIG}")
                        print("[setup] 캘리브레이션 완료! q 로 종료하세요.")
                return False

            jog_loop(term, jog, on_key)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        lift.stop()
        park_lift(lift, "setup", enabled=jog is None or jog.torque_on)
        lift.save_state()
        bus.close()
        print("[setup] 종료")


if __name__ == "__main__":
    main()
