"""通信协议层：帧编解码、CRC16、数据包定义。

与 mcu_bridge 100% 复用，基于通信协议设计 v1.0。
"""

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


# ============================================================
# CRC16
# ============================================================

def crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT (同 CRC-16/MODBUS)，多项式 0x1021。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


# ============================================================
# 帧常量
# ============================================================

SOF1 = 0x5A
SOF2 = 0xA5

DIR_UPLOAD = 0x00   # MCU → 上位机
DIR_COMMAND = 0x80  # 上位机 → MCU


class PacketID(IntEnum):
    """上传数据包 ID (0x00~0x0F)。"""
    FAST_STATUS = 0x01
    TOFSENSE_FULL = 0x02
    IMU_ATTITUDE = 0x03
    FOC_FEEDBACK = 0x04
    TRIWHEEL_FEEDBACK = 0x05
    DRV8701_FEEDBACK = 0x06
    CHASSIS_FULL = 0x07
    FOC_CONFIG = 0x08
    EVENT_NOTIFY = 0x09
    ACK = 0x0A


class CmdID(IntEnum):
    """命令数据包 ID (0x10~0x1F)。"""
    HEARTBEAT = 0x10
    EMERGENCY_STOP = 0x11
    MODE_CONTROL = 0x12
    CHASSIS_VELOCITY = 0x13
    CHASSIS_TORQUE = 0x14
    TRIWHEEL_ANGLE = 0x15
    FOC_DIRECT = 0x16
    FOC_GAINS = 0x17
    DRV8701_DIRECT = 0x18
    POLL_REQUEST = 0x19
    CALIBRATION = 0x1A


class ChassisMode(IntEnum):
    SAFE = 0
    OPEN_LOOP = 1
    CLOSED_LOOP = 2
    SLAVE = 3


class EventType(IntEnum):
    MODE_CHANGE = 0x01
    ERROR = 0x02
    ERROR_CLEAR = 0x03
    DBUS = 0x04
    ESTOP = 0x05
    TOF_ANOMALY = 0x06


# 错误标志位
ERROR_FLAG_NAMES = {
    0: "急停",
    1: "IMU故障",
    2: "FOC故障",
    3: "DRV故障",
    4: "TOF故障",
    5: "DBUS断连",
    6: "过温",
    7: "电压异常",
    8: "三角轮故障",
}

CHASSIS_MODE_NAMES = {0: "安全", 1: "开环", 2: "闭环", 3: "从机"}

TOF_STATUS_NAMES = {0: "无效", 1: "有效", 2: "弱信号", 3: "超量程", 0xFF: "未知"}

EVENT_TYPE_NAMES = {
    0x01: "模式切换",
    0x02: "故障",
    0x03: "故障恢复",
    0x04: "DBUS",
    0x05: "急停",
    0x06: "TOF异常",
}


# ============================================================
# 解析后帧结构
# ============================================================

@dataclass
class ParsedFrame:
    packet_id: int
    seq_num: int
    payload: dict
    is_command: bool = False


# ============================================================
# 上传数据包 payload 解析
# ============================================================

def _parse_fast_status(data: bytes) -> dict:
    if len(data) < 12:
        return {}
    chassis_mode = data[0]
    heartbeat_echo = data[1]
    error_flags = struct.unpack_from('<H', data, 2)[0]
    tof_min = struct.unpack_from('<H', data, 4)[0]
    tof_max = struct.unpack_from('<H', data, 6)[0]
    tof_valid = data[8]
    imu_pitch = data[9] / 10.0 if data[9] < 128 else (data[9] - 256) / 10.0
    imu_roll = data[10] / 10.0 if data[10] < 128 else (data[10] - 256) / 10.0
    chassis_pitch = data[11] / 10.0 if data[11] < 128 else (data[11] - 256) / 10.0
    return {
        "chassis_mode": chassis_mode,
        "heartbeat_echo": heartbeat_echo,
        "error_flags": error_flags,
        "tof_min_distance_mm": tof_min,
        "tof_max_distance_mm": tof_max,
        "tof_valid_count": tof_valid,
        "imu_pitch_deg": imu_pitch,
        "imu_roll_deg": imu_roll,
        "chassis_pitch_deg": chassis_pitch,
    }


def _parse_tofsense_full(data: bytes) -> dict:
    sensors = []
    for s in range(4):
        offset = s * 4
        if offset + 4 > len(data):
            break
        dist = struct.unpack_from('<H', data, offset)[0]
        status = data[offset + 2]
        signal = data[offset + 3]
        sensors.append({
            "distance_mm": dist,
            "status": status,
            "signal": signal,
        })
    return {"tof": sensors}


def _parse_imu_attitude(data: bytes) -> dict:
    if len(data) < 44:
        return {}
    quat_w, quat_x, quat_y, quat_z = struct.unpack_from('<ffff', data, 0)
    gyro_x, gyro_y, gyro_z = struct.unpack_from('<fff', data, 16)
    accel_x, accel_y, accel_z = struct.unpack_from('<fff', data, 28)
    temp = struct.unpack_from('<f', data, 40)[0]
    return {
        "quat_w": quat_w, "quat_x": quat_x, "quat_y": quat_y, "quat_z": quat_z,
        "gyro_x_dps": gyro_x, "gyro_y_dps": gyro_y, "gyro_z_dps": gyro_z,
        "accel_x_g": accel_x, "accel_y_g": accel_y, "accel_z_g": accel_z,
        "temperature_c": temp,
    }


def _parse_foc_feedback(data: bytes) -> dict:
    motors = []
    for m in range(6):
        offset = m * 7
        if offset + 7 > len(data):
            break
        online = data[offset]
        vel_raw = struct.unpack_from('<h', data, offset + 1)[0]
        torque_raw = struct.unpack_from('<h', data, offset + 3)[0]
        fb_hz = struct.unpack_from('<H', data, offset + 5)[0]
        motors.append({
            "online": online,
            "velocity_rpm": vel_raw / 10.0,
            "torque": torque_raw / 1000.0,
            "feedback_hz": fb_hz,
        })
    return {"foc_motors": motors}


def _parse_triwheel_feedback(data: bytes) -> dict:
    wheels = []
    for w in range(4):
        offset = w * 8
        if offset + 8 > len(data):
            break
        angle, filtered = struct.unpack_from('<ff', data, offset)
        wheels.append({
            "angle_deg": angle,
            "filtered_angle_deg": filtered,
        })
    chassis_pitch = 0.0
    if len(data) >= 36:
        chassis_pitch = struct.unpack_from('<f', data, 32)[0]
    return {"triwheels": wheels, "chassis_pitch_deg": chassis_pitch}


def _parse_drv8701_feedback(data: bytes) -> dict:
    motors = []
    for m in range(6):
        offset = m * 7
        if offset + 7 > len(data):
            break
        enabled = data[offset]
        mode = data[offset + 1]
        direction_raw = data[offset + 2]
        direction = -1 if direction_raw > 127 else direction_raw
        duty_raw = struct.unpack_from('<h', data, offset + 3)[0]
        current_raw = struct.unpack_from('<h', data, offset + 5)[0]
        motors.append({
            "enabled": enabled,
            "mode": mode,
            "direction": direction,
            "duty": duty_raw / 1000.0,
            "current_ma": current_raw,
        })
    return {"drv_motors": motors}


def _parse_event_notify(data: bytes) -> dict:
    if len(data) < 6:
        return {}
    event_type = data[0]
    event_code = data[1]
    event_data = struct.unpack_from('<I', data, 2)[0]
    return {
        "event_type": event_type,
        "event_code": event_code,
        "event_data": event_data,
    }


def _parse_ack(data: bytes) -> dict:
    if len(data) < 3:
        return {}
    return {
        "ack_packet_id": data[0],
        "error_code": data[1],
    }


# payload 解析器映射
_PARSERS = {
    PacketID.FAST_STATUS: _parse_fast_status,
    PacketID.TOFSENSE_FULL: _parse_tofsense_full,
    PacketID.IMU_ATTITUDE: _parse_imu_attitude,
    PacketID.FOC_FEEDBACK: _parse_foc_feedback,
    PacketID.TRIWHEEL_FEEDBACK: _parse_triwheel_feedback,
    PacketID.DRV8701_FEEDBACK: _parse_drv8701_feedback,
    PacketID.EVENT_NOTIFY: _parse_event_notify,
    PacketID.ACK: _parse_ack,
}


# ============================================================
# 帧解码器（状态机）
# ============================================================

class FrameDecoder:
    """SOF 状态机帧解码器。

    用法:
        decoder = FrameDecoder()
        for byte in serial_stream:
            frame = decoder.feed(byte)
            if frame:
                handle(frame)
    """

    STATE_IDLE = 0
    STATE_GOT_5A = 1
    STATE_GOT_A5 = 2
    STATE_READING = 3

    def __init__(self):
        self._state = self.STATE_IDLE
        self._buf = bytearray()
        self._payload_len = 0
        self._crc_errors = 0
        self._total_frames = 0

    def feed(self, byte: int) -> Optional[ParsedFrame]:
        """喂入单个字节，完整帧到达时返回 ParsedFrame，否则返回 None。"""
        b = byte & 0xFF

        if self._state == self.STATE_IDLE:
            if b == SOF1:
                self._state = self.STATE_GOT_5A
            return None

        if self._state == self.STATE_GOT_5A:
            if b == SOF2:
                self._state = self.STATE_GOT_A5
                self._buf = bytearray()
            else:
                self._state = self.STATE_IDLE
            return None

        if self._state == self.STATE_GOT_A5:
            # b = Frame_Type
            self._buf.append(b)
            self._state = self.STATE_READING
            self._header_read = 1
            return None

        # STATE_READING
        self._buf.append(b)
        self._header_read = getattr(self, '_header_read', 0) + 1

        if self._header_read == 3:
            # Payload_Len (2 bytes) just completed
            self._payload_len = struct.unpack_from('<H', self._buf, 1)[0]
        elif self._header_read >= 4 and len(self._buf) >= 3 + self._payload_len + 2:
            # Got full frame: header(1B type + 2B len + 1B seq) + payload + crc(2B)
            return self._finalize()

        return None

    def _finalize(self) -> Optional[ParsedFrame]:
        header_end = 4  # type(1) + len(2) + seq(1)
        payload_end = header_end + self._payload_len
        frame_type = self._buf[0]
        payload_len = struct.unpack_from('<H', self._buf, 1)[0]
        seq_num = self._buf[3]
        payload = bytes(self._buf[header_end:payload_end])
        crc_received = struct.unpack_from('<H', self._buf, payload_end)[0]

        # Verify CRC over frame_type..payload (bytes 0..payload_end-1)
        crc_expected = crc16_ccitt(bytes(self._buf[:payload_end]))
        self._total_frames += 1

        if crc_received != crc_expected:
            self._crc_errors += 1
            self._state = self.STATE_IDLE
            return None

        # Reset for next frame
        self._state = self.STATE_IDLE

        is_command = bool(frame_type & 0x80)
        packet_id = frame_type & 0x7F

        # Parse payload
        parsed_payload = {}
        if not is_command and packet_id in _PARSERS:
            try:
                parsed_payload = _PARSERS[packet_id](payload)
            except Exception:
                pass

        return ParsedFrame(
            packet_id=packet_id,
            seq_num=seq_num,
            payload=parsed_payload,
            is_command=is_command,
        )

    @property
    def crc_errors(self) -> int:
        return self._crc_errors

    @property
    def total_frames(self) -> int:
        return self._total_frames

    def reset(self):
        self._state = self.STATE_IDLE
        self._buf = bytearray()


# ============================================================
# 帧编码器（上位机 → MCU）
# ============================================================

class FrameEncoder:
    """构建命令帧字节序列。"""

    def __init__(self):
        self._seq_num = 0

    def build_frame(self, packet_id: int, payload: bytes) -> bytes:
        """构建完整帧（含 SOF + CRC16）。"""
        frame_type = packet_id | DIR_COMMAND
        seq = self._seq_num
        self._seq_num = (self._seq_num + 1) & 0xFF

        header = bytes([frame_type])
        header += struct.pack('<H', len(payload))
        header += bytes([seq])

        crc_input = header + payload
        crc = crc16_ccitt(crc_input)

        frame = bytes([SOF1, SOF2]) + crc_input + struct.pack('<H', crc)
        return frame

    def build_heartbeat(self, timestamp_ms: int) -> bytes:
        return self.build_frame(CmdID.HEARTBEAT, struct.pack('<I', timestamp_ms & 0xFFFFFFFF))

    def build_emergency_stop(self, stop_type: int = 0x00) -> bytes:
        return self.build_frame(CmdID.EMERGENCY_STOP, bytes([stop_type, 0, 0, 0]))

    def build_mode_control(self, target_mode: int, enable_mask: int = 0) -> bytes:
        return self.build_frame(CmdID.MODE_CONTROL, struct.pack('<BI', target_mode, enable_mask))
