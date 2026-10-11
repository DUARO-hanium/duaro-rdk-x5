"""
feetech_sts.py — Feetech STS/SMS 시리즈(STS-3215 등) 최소 드라이버 (pyserial 전용)

lerobot 의 FeetechMotorsBus(protocol 0)와 동일한 프로토콜/레지스터/부호 인코딩을
외부 의존성 없이 구현한 것. RDK X5(Ubuntu 22.04, python3-serial 기본 탑재)에서
lerobot 전체를 설치하지 않고 z축 서보를 제어하기 위해 사용한다.

패킷 형식 (protocol 0):
  TX: FF FF ID LEN INST P1..PN CHK      LEN = N + 2
  RX: FF FF ID LEN ERR P1..PN CHK       CHK = ~(ID+LEN+INST/ERR+ΣP) & 0xFF
2바이트 값은 little-endian (low byte first) — STS 시리즈 기준.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import serial

DEFAULT_BAUDRATE = 1_000_000

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

# STS-3215 control table: name -> (address, byte length)
# (lerobot/motors/feetech/tables.py STS_SMS_SERIES_CONTROL_TABLE 발췌)
REG = {
    "Model_Number": (3, 2),
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Return_Delay_Time": (7, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Max_Temperature_Limit": (13, 1),
    "Max_Voltage_Limit": (14, 1),
    "Min_Voltage_Limit": (15, 1),
    "Max_Torque_Limit": (16, 2),
    "Operating_Mode": (33, 1),   # EEPROM: Lock=0 상태에서만 쓰기 가능
    "Protective_Torque": (34, 1),
    "Protection_Time": (35, 1),
    "Overload_Torque": (36, 1),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Time": (44, 2),
    "Goal_Velocity": (46, 2),
    "Torque_Limit": (48, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Status": (65, 1),
    "Moving": (66, 1),
    "Present_Current": (69, 2),
}

# sign-magnitude 인코딩의 부호 비트 위치 (lerobot STS_SMS_SERIES_ENCODINGS_TABLE + Present_Current)
SIGN_BIT = {
    "Goal_Position": 15,
    "Goal_Velocity": 15,
    "Present_Position": 15,
    "Present_Velocity": 15,
    "Present_Load": 10,
    "Present_Current": 15,
}

# Operating_Mode 값
MODE_POSITION = 0
MODE_VELOCITY = 1
MODE_PWM = 2
MODE_STEP = 3

# 단위 환산
TICKS_PER_REV = 4096
CURRENT_MA_PER_UNIT = 6.5      # Present_Current 1 unit = 6.5 mA
VOLTAGE_V_PER_UNIT = 0.1       # Present_Voltage 1 unit = 0.1 V

STATUS_BITS = {
    0: "voltage", 1: "sensor", 2: "temperature", 3: "current", 4: "angle", 5: "overload",
}


class FeetechError(Exception):
    pass


class FeetechTimeout(FeetechError):
    pass


def encode_sign_magnitude(value: int, sign_bit: int) -> int:
    max_mag = (1 << sign_bit) - 1
    mag = abs(int(value))
    if mag > max_mag:
        raise ValueError(f"magnitude {mag} exceeds {max_mag} (sign_bit={sign_bit})")
    return ((1 if value < 0 else 0) << sign_bit) | mag


def decode_sign_magnitude(raw: int, sign_bit: int) -> int:
    mag = raw & ((1 << sign_bit) - 1)
    return -mag if (raw >> sign_bit) & 1 else mag


@dataclass
class Status:
    error: int

    def names(self) -> list[str]:
        return [n for b, n in STATUS_BITS.items() if (self.error >> b) & 1]


class FeetechBus:
    """단일 시리얼 포트에 물린 Feetech 버스. 여러 ID의 서보를 다룰 수 있다."""

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser: serial.Serial | None = None
        self.last_error: int = 0

    # ---------------- 연결 ----------------
    def open(self) -> "FeetechBus":
        self._ser = serial.Serial(
            self.port, baudrate=self.baudrate, timeout=self.timeout, write_timeout=self.timeout,
        )
        self._ser.reset_input_buffer()
        return self

    def close(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    @property
    def ser(self) -> serial.Serial:
        if self._ser is None:
            raise FeetechError("bus not open — call open() first")
        return self._ser

    # ---------------- 저수준 패킷 ----------------
    @staticmethod
    def _checksum(payload: bytes) -> int:
        return (~sum(payload)) & 0xFF

    def _tx(self, motor_id: int, inst: int, params: bytes = b"") -> None:
        body = bytes([motor_id, len(params) + 2, inst]) + params
        pkt = b"\xff\xff" + body + bytes([self._checksum(body)])
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self.ser.flush()

    def _rx(self, expect_id: int) -> tuple[int, bytes]:
        """상태 패킷 수신 → (error, params)."""
        ser = self.ser
        deadline = time.monotonic() + self.timeout * 4
        # 헤더 FF FF 동기화
        prev = None
        while True:
            b = ser.read(1)
            if not b:
                if time.monotonic() > deadline:
                    raise FeetechTimeout(f"no response from id {expect_id}")
                continue
            if prev == 0xFF and b[0] == 0xFF:
                break
            prev = b[0]
        head = ser.read(2)
        if len(head) < 2:
            raise FeetechTimeout(f"truncated header from id {expect_id}")
        rid, length = head[0], head[1]
        rest = ser.read(length)
        if len(rest) < length:
            raise FeetechTimeout(f"truncated body from id {expect_id}")
        err, params, chk = rest[0], rest[1:-1], rest[-1]
        calc = self._checksum(bytes([rid, length, err]) + params)
        if chk != calc:
            raise FeetechError(f"checksum mismatch from id {rid}: got {chk:#x}, calc {calc:#x}")
        if rid != expect_id:
            raise FeetechError(f"unexpected id in response: {rid} (expected {expect_id})")
        self.last_error = err
        return err, bytes(params)

    def _txrx(self, motor_id: int, inst: int, params: bytes = b"", retries: int = 2) -> tuple[int, bytes]:
        last: Exception | None = None
        for _ in range(retries + 1):
            try:
                self._tx(motor_id, inst, params)
                return self._rx(motor_id)
            except FeetechError as e:
                last = e
                time.sleep(0.005)
        raise last  # type: ignore[misc]

    # ---------------- 고수준 API ----------------
    def ping(self, motor_id: int) -> bool:
        try:
            self._tx(motor_id, INST_PING)
            self._rx(motor_id)
            return True
        except FeetechError:
            return False

    def scan(self, ids=range(0, 254), per_id_timeout: float = 0.01) -> list[int]:
        old_ser_t, old_t = self.ser.timeout, self.timeout
        self.ser.timeout = per_id_timeout
        self.timeout = per_id_timeout
        found = []
        try:
            for i in ids:
                if self.ping(i):
                    found.append(i)
        finally:
            self.ser.timeout = old_ser_t
            self.timeout = old_t
        return found

    def read(self, motor_id: int, name: str) -> int:
        addr, length = REG[name]
        _, data = self._txrx(motor_id, INST_READ, bytes([addr, length]))
        if len(data) != length:
            raise FeetechError(f"read {name}: expected {length} bytes, got {len(data)}")
        raw = int.from_bytes(data, "little")
        if name in SIGN_BIT:
            return decode_sign_magnitude(raw, SIGN_BIT[name])
        return raw

    def write(self, motor_id: int, name: str, value: int) -> None:
        addr, length = REG[name]
        v = int(value)
        if name in SIGN_BIT:
            v = encode_sign_magnitude(v, SIGN_BIT[name])
        if v < 0 or v >= (1 << (8 * length)):
            raise ValueError(f"write {name}: value {value} out of range for {length} byte(s)")
        self._txrx(motor_id, INST_WRITE, bytes([addr]) + v.to_bytes(length, "little"))

    # 편의 함수 (lerobot enable_torque/disable_torque 와 동일한 순서)
    def torque_on(self, motor_id: int) -> None:
        self.write(motor_id, "Torque_Enable", 1)
        self.write(motor_id, "Lock", 1)

    def torque_off(self, motor_id: int) -> None:
        self.write(motor_id, "Torque_Enable", 0)
        self.write(motor_id, "Lock", 0)

    def set_operating_mode(self, motor_id: int, mode: int) -> None:
        """EEPROM 영역이므로 Lock 해제 후 기록. 호출 후 토크는 꺼진 상태."""
        self.torque_off(motor_id)
        self.write(motor_id, "Operating_Mode", mode)

    def set_id(self, motor_id: int, new_id: int) -> None:
        if not (0 <= new_id <= 253):
            raise ValueError("new id must be 0..253")
        self.torque_off(motor_id)
        self.write(motor_id, "ID", new_id)
        time.sleep(0.05)
        self.write(new_id, "Lock", 1)

    def status(self, motor_id: int) -> Status:
        return Status(self.read(motor_id, "Status"))

    def snapshot(self, motor_id: int) -> dict:
        """디버그용 주요 레지스터 일괄 읽기."""
        keys = [
            "Model_Number", "Operating_Mode", "Torque_Enable", "Lock", "Acceleration",
            "Goal_Velocity", "Present_Position", "Present_Velocity", "Present_Load",
            "Present_Voltage", "Present_Temperature", "Present_Current", "Moving", "Status",
        ]
        out = {}
        for k in keys:
            try:
                out[k] = self.read(motor_id, k)
            except FeetechError as e:
                out[k] = f"ERR({e})"
        if isinstance(out.get("Present_Current"), int):
            out["Present_Current_mA"] = round(out["Present_Current"] * CURRENT_MA_PER_UNIT, 1)
        if isinstance(out.get("Present_Voltage"), int):
            out["Present_Voltage_V"] = round(out["Present_Voltage"] * VOLTAGE_V_PER_UNIT, 1)
        if isinstance(out.get("Status"), int):
            out["Status_flags"] = Status(out["Status"]).names()
        return out
