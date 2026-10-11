#!/usr/bin/env python3
"""
jog_record.py — 상태 기록 (한 번 실행 = 한 상태)

  python3 jog_record.py

  1. 상태명 입력
  2. z축 조그 (u/k, +/-, g, space) → Enter : 그때의 높이(mm, 절대) 기록
  3. 턴테이블 조그 (a/d, w/e, ,/., space) → Enter : 이 단계에서 돌린 각도(상대, + = 오른쪽) 기록
  4. 재현 순서 입력 (1 = z축 먼저, 2 = 턴테이블 먼저)
  → poses.csv 에 추가. 같은 상태명이 있으면 그 줄만 덮어씀.

사전 조건: python3 setup.py (z축 캘리브레이션 + 영점)
"""
from __future__ import annotations

import argparse
import sys
import time

import common
from common import (LIFT_KEYS, TURN_KEYS, ORDER_LABEL, Jog, RawTerminal, format_pose, jog_loop,
                    lift_calibrated, open_lift, open_turn, park_lift, save_pose)


def enter(ch, handled):
    return ch in ("\r", "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=common.DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=common.DEFAULT_BAUD)
    args = ap.parse_args()

    if not lift_calibrated():
        sys.exit("[record] z축 캘리브레이션 없음 → 먼저 python3 setup.py")
    name = ""
    while not name:
        name = input("상태명: ").strip()

    bus, lift = open_lift(args.port, args.baud)
    if not lift.homed:
        bus.close()
        sys.exit("[record] z축 영점 없음 → 먼저 python3 setup.py (z 또는 h)")
    park_lift(lift, "record")
    turn = open_turn()
    jog_z = None
    try:
        with RawTerminal() as term:
            print(f"[1/3] z축 이동 후 Enter 로 기록   키: {LIFT_KEYS}  space=정지")
            jog_z = Jog(term, lift=lift)
            jog_loop(term, jog_z, enter)
            time.sleep(0.3)
            z_mm = round(lift.get_height_mm(), 1)
            print(f"\n      z = {z_mm} mm 기록")

            print(f"[2/3] 턴테이블 회전 후 Enter 로 기록   키: {TURN_KEYS}  space=정지")
            turn.mark()
            jog_loop(term, Jog(term, turn=turn), enter)
            turn_deg = round(turn.rel_deg(), 2)
            print(f"\n      turn = {turn_deg:+} deg 기록 (이 단계에서 돌린 양)")

            order = ""
            while order not in ORDER_LABEL:
                order = term.ask("[3/3] 재현 순서 (1 = z축 먼저, 2 = 턴테이블 먼저): ")
        existed = save_pose(name, z_mm, turn_deg, order)
        print(f"[record] {'덮어씀' if existed else '추가'}: {format_pose(name, {'z_mm': z_mm, 'turn_deg': turn_deg, 'order': order})}")
        print(f"         → {common.POSES_CSV}")
    except KeyboardInterrupt:
        print("\n[record] 중단 (저장 안 함)")
    finally:
        print()
        lift.stop()
        park_lift(lift, "record", enabled=jog_z is None or jog_z.torque_on)
        lift.save_state()
        turn.close()
        bus.close()


if __name__ == "__main__":
    main()
