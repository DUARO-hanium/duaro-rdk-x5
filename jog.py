#!/usr/bin/env python3
"""
jog.py — 두 축 조그 + 상태 재현 (Ctrl-C 로만 종료)

  python3 jog.py

z축:      u/k(↑/↓)=상하  +/-=속도  g=mm 입력 이동  h=호밍  z=영점  t=토크 ON/OFF
턴테이블: a/d(←/→)=좌/우 회전  w/e=왼쪽/오른쪽 90도  ,/.=속도
공통:     space=정지  n=상태명 입력 → poses.csv 의 상태로 이동  l=상태 목록

n: z축은 저장된 높이로 절대 이동, 턴테이블은 저장된 각도만큼 지금 위치에서 상대 회전.
   순서는 기록할 때 정한 대로 (1 = z축 먼저, 2 = 턴테이블 먼저).
사전 조건: python3 setup.py (z축 캘리브레이션), 상태는 python3 jog_record.py 로 기록
"""
from __future__ import annotations

import argparse
import sys

import common
from common import (LIFT_KEYS, TURN_KEYS, Z_FIRST, ORDER_LABEL, Jog, RawTerminal, format_pose, jog_loop,
                    lift_calibrated, load_poses, open_lift, open_turn, park_lift)


def run_pose(jog: Jog, name: str, p: dict) -> None:
    print(f"\n[n] {format_pose(name, p)}")
    for axis in ("z", "turn") if p["order"] == Z_FIRST else ("turn", "z"):
        if axis == "z":
            h = jog.goto_mm(p["z_mm"])
            if h is None:
                print("[n] z축 이동 실패 → 중단")
                return
            print(f"    z축 → {h:.1f} mm")
        else:
            d = jog.turn.rotate_deg(p["turn_deg"])
            print(f"    턴테이블 {d:+.1f} deg 회전")
    print("[n] 완료")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=common.DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=common.DEFAULT_BAUD)
    args = ap.parse_args()

    if not lift_calibrated():
        sys.exit("[jog] z축 캘리브레이션 없음 → 먼저 python3 setup.py")
    bus, lift = open_lift(args.port, args.baud)
    if not lift.homed:
        print("[jog] 주의: z축 영점 없음 → h 또는 z 로 영점을 잡기 전엔 n/g 사용 불가")
    park_lift(lift, "jog")
    turn = open_turn()
    jog = None

    print(f"z축:      {LIFT_KEYS}")
    print(f"턴테이블: {TURN_KEYS}")
    print("공통:     space=정지  n=상태로 이동  l=상태 목록  Ctrl-C=종료")
    try:
        with RawTerminal() as term:
            jog = Jog(term, lift=lift, turn=turn)

            def on_key(ch, handled):
                if ch == "n":
                    jog.stop()
                    name = term.ask("\n상태의 명을 입력하시오: ")
                    if not name:
                        return False
                    poses = load_poses()
                    if name not in poses:
                        print(f"'{name}' 없음. 등록된 상태: {list(poses) or '없음'}")
                    else:
                        run_pose(jog, name, poses[name])
                elif ch == "l":
                    poses = load_poses()
                    print("\n[poses]" + (" 없음" if not poses else ""))
                    for k, v in poses.items():
                        print("  " + format_pose(k, v))
                return False

            jog_loop(term, jog, on_key)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        lift.stop()
        park_lift(lift, "jog", enabled=jog is None or jog.torque_on)
        lift.save_state()
        turn.close()
        bus.close()
        print("[jog] 종료")


if __name__ == "__main__":
    main()
