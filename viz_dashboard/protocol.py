"""通信协议层：FlexWire 寄存器读写 + CRC16。

基于 TI FlexWire 协议改进，取消设备地址段，增加读取长度。
寄存器地址 + 内容形式，每个寄存器 32-bit。
"""

import struct
from enum import IntEnum
from typing import Optional, List, Tuple

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
# 协议常量
# ============================================================

SYNC = 0x55          # UART 同步字节
SOF_HOST = 0x5A      # 主机→MCU 帧起始
SOF_MCU = 0xA5       # MCU→主机 帧起始

# ============================================================
# 寄存器地址定义
# ============================================================

class RegAddr(IntEnum):
    """寄存器地址枚举。

    Excel 中使用 "0x" 前缀 + 十进制数字的记法，如 0x10=10, 0x20=20。
    所有地址均为十进制整数值。
    """
    # ---- 控制寄存器 (r/w) ----
    CTRL = 0                # 0x00
    DRIVE_SPEED = 1         # 0x01
    DRIVE_ACCEL = 2         # 0x02
    LEFT_DRIVE_PID = 3      # 0x03
    RIGHT_DRIVE_PID = 4     # 0x04
    DRIVE_FF_TORQUE = 5     # 0x05
    OPENLOOP_TORQUE = 6     # 0x06
    TRIWHEEL_ANGLE_FRONT = 7   # 0x07
    TRIWHEEL_ANGLE_REAR = 8    # 0x08
    TRIWHEEL_DUTY_FRONT = 9    # 0x09
    TRIWHEEL_DUTY_REAR = 10    # 0x10

    # ---- TOF 传感器 (r) ----
    TOF1 = 11               # 0x11
    TOF2 = 12               # 0x12
    TOF3 = 13               # 0x13
    TOF4 = 14               # 0x14

    # ---- IMU (r) ----
    IMU_QUAT_W = 15         # 0x15
    IMU_QUAT_X = 16         # 0x16
    IMU_QUAT_Y = 17         # 0x17
    IMU_QUAT_Z = 18         # 0x18
    IMU_YAW = 19            # 0x19
    IMU_PITCH = 20          # 0x20
    IMU_ROLL = 21           # 0x21
    IMU_GYRO_YAW = 22       # 0x22
    IMU_GYRO_PITCH = 23     # 0x23
    IMU_GYRO_ROLL = 24      # 0x24
    IMU_ACCEL_X = 25        # 0x25
    IMU_ACCEL_Y = 26        # 0x26
    IMU_ACCEL_Z = 27        # 0x27
    IMU_TEMP = 28           # 0x28

    # ---- 驱动电机反馈 (r) ----
    MOTOR_L3_TORQUE_SPEED = 29  # 0x29
    MOTOR_L3_ANGLE = 30         # 0x30
    MOTOR_L4_TORQUE_SPEED = 31  # 0x31
    MOTOR_L4_ANGLE = 32         # 0x32
    MOTOR_L5_TORQUE_SPEED = 33  # 0x33
    MOTOR_L5_ANGLE = 34         # 0x34
    MOTOR_R0_TORQUE_SPEED = 35  # 0x35
    MOTOR_R0_ANGLE = 36         # 0x36
    MOTOR_R1_TORQUE_SPEED = 37  # 0x37
    MOTOR_R1_ANGLE = 38         # 0x38
    MOTOR_R2_TORQUE_SPEED = 39  # 0x39
    MOTOR_R2_ANGLE = 40         # 0x40

    # ---- 三角轮反馈 (r) ----
    TRIWHEEL_ANGLE_CUR_FRONT = 41   # 0x41
    TRIWHEEL_ANGLE_CUR_REAR = 42    # 0x42
    TRIWHEEL_DUTY_CUR_FRONT = 43    # 0x43
    TRIWHEEL_DUTY_CUR_REAR = 44     # 0x44

    # ---- 角加速度计 (r) ----
    ACCEL_LF_X = 45         # 0x45
    ACCEL_LF_Y = 46         # 0x46
    ACCEL_LF_Z = 47         # 0x47
    ACCEL_RF_X = 48         # 0x48
    ACCEL_RF_Y = 49         # 0x49
    ACCEL_RF_Z = 50         # 0x50
    ACCEL_LR_X = 51         # 0x51
    ACCEL_LR_Y = 52         # 0x52
    ACCEL_LR_Z = 53         # 0x53
    ACCEL_RR_X = 54         # 0x54
    ACCEL_RR_Y = 55         # 0x55
    ACCEL_RR_Z = 56         # 0x56

    # ---- 在线状态 (r) ----
    ONLINE_STATUS = 57      # 0x57


# ============================================================
# 寄存器字段定义
# ============================================================

def _hf16(hi: int, lo: int) -> Tuple[int, int, int]:
    """半寄存器有符号 16-bit 字段: (bit_offset, bit_len, signed)"""
    return (lo, hi - lo + 1, 1)

def _hfu16(hi: int, lo: int) -> Tuple[int, int, int]:
    """半寄存器无符号 16-bit 字段"""
    return (lo, hi - lo + 1, 0)

def _f32() -> Tuple[int, int, str]:
    """全寄存器 float32"""
    return (0, 32, 'f')

def _bit(pos: int) -> Tuple[int, int, int]:
    """单个 bit 字段"""
    return (pos, 1, 0)

def _bits(hi: int, lo: int) -> Tuple[int, int, int]:
    """多 bit 无符号字段"""
    return (lo, hi - lo + 1, 0)


REGISTER_DEFS = {
    # ---- 控制寄存器 ----
    RegAddr.CTRL: {
        "chassis_mode": _bits(31, 29),
        "foc_enable": _bit(28),
        "brushed_enable": _bit(27),
        "foc_ctrl_mode": _bit(26),
        "brushed_ctrl_mode": _bit(25),
        "tri_lf_zero": _bit(24),
        "tri_rf_zero": _bit(23),
        "tri_lr_zero": _bit(22),
        "tri_rr_zero": _bit(21),
        "drive_id0_clear": _bit(20),
        "drive_id1_clear": _bit(19),
        "drive_id2_clear": _bit(18),
        "drive_id3_clear": _bit(17),
        "drive_id4_clear": _bit(16),
        "drive_id5_clear": _bit(15),
    },

    # ---- 驱动轮速度/加速度 (半寄存器对) ----
    RegAddr.DRIVE_SPEED: {
        "left_speed": _hf16(31, 16),
        "right_speed": _hf16(15, 0),
    },
    RegAddr.DRIVE_ACCEL: {
        "left_accel": _hf16(31, 16),
        "right_accel": _hf16(15, 0),
    },
    RegAddr.LEFT_DRIVE_PID: {
        "left_kp": _hf16(31, 16),
        "left_kd": _hf16(15, 0),
    },
    RegAddr.RIGHT_DRIVE_PID: {
        "right_kp": _hf16(31, 16),
        "right_kd": _hf16(15, 0),
    },
    RegAddr.DRIVE_FF_TORQUE: {
        "left_ff_torque": _hf16(31, 16),
        "right_ff_torque": _hf16(15, 0),
    },
    RegAddr.OPENLOOP_TORQUE: {
        "left_ol_torque": _hf16(31, 16),
        "right_ol_torque": _hf16(15, 0),
    },
    RegAddr.TRIWHEEL_ANGLE_FRONT: {
        "tri_lf_angle_target": _hf16(31, 16),
        "tri_rf_angle_target": _hf16(15, 0),
    },
    RegAddr.TRIWHEEL_ANGLE_REAR: {
        "tri_lr_angle_target": _hf16(31, 16),
        "tri_rr_angle_target": _hf16(15, 0),
    },
    RegAddr.TRIWHEEL_DUTY_FRONT: {
        "tri_lf_duty_target": _hf16(31, 16),
        "tri_rf_duty_target": _hf16(15, 0),
    },
    RegAddr.TRIWHEEL_DUTY_REAR: {
        "tri_lr_duty_target": _hf16(31, 16),
        "tri_rr_duty_target": _hf16(15, 0),
    },

    # ---- TOF ----
    RegAddr.TOF1: {
        "distance_mm": _hfu16(31, 16),
        "signal": _bits(15, 8),
        "status": _bits(7, 5),
        "fault": _bit(4),
    },
    RegAddr.TOF2: {
        "distance_mm": _hfu16(31, 16),
        "signal": _bits(15, 8),
        "status": _bits(7, 5),
        "fault": _bit(4),
    },
    RegAddr.TOF3: {
        "distance_mm": _hfu16(31, 16),
        "signal": _bits(15, 8),
        "status": _bits(7, 5),
        "fault": _bit(4),
    },
    RegAddr.TOF4: {
        "distance_mm": _hfu16(31, 16),
        "signal": _bits(15, 8),
        "status": _bits(7, 5),
        "fault": _bit(4),
    },

    # ---- IMU float32 ----
    RegAddr.IMU_QUAT_W: {"quat_w": _f32()},
    RegAddr.IMU_QUAT_X: {"quat_x": _f32()},
    RegAddr.IMU_QUAT_Y: {"quat_y": _f32()},
    RegAddr.IMU_QUAT_Z: {"quat_z": _f32()},
    RegAddr.IMU_YAW: {"yaw_deg": _f32()},
    RegAddr.IMU_PITCH: {"pitch_deg": _f32()},
    RegAddr.IMU_ROLL: {"roll_deg": _f32()},
    RegAddr.IMU_GYRO_YAW: {"gyro_yaw_dps": _f32()},
    RegAddr.IMU_GYRO_PITCH: {"gyro_pitch_dps": _f32()},
    RegAddr.IMU_GYRO_ROLL: {"gyro_roll_dps": _f32()},
    RegAddr.IMU_ACCEL_X: {"accel_x_g": _f32()},
    RegAddr.IMU_ACCEL_Y: {"accel_y_g": _f32()},
    RegAddr.IMU_ACCEL_Z: {"accel_z_g": _f32()},
    RegAddr.IMU_TEMP: {"temperature_c": _f32()},

    # ---- 驱动电机反馈 ----
    RegAddr.MOTOR_L3_TORQUE_SPEED: {"torque": _hf16(31, 16), "speed": _hf16(15, 0)},
    RegAddr.MOTOR_L3_ANGLE: {"total_angle_rad": _f32()},
    RegAddr.MOTOR_L4_TORQUE_SPEED: {"torque": _hf16(31, 16), "speed": _hf16(15, 0)},
    RegAddr.MOTOR_L4_ANGLE: {"total_angle_rad": _f32()},
    RegAddr.MOTOR_L5_TORQUE_SPEED: {"torque": _hf16(31, 16), "speed": _hf16(15, 0)},
    RegAddr.MOTOR_L5_ANGLE: {"total_angle_rad": _f32()},
    RegAddr.MOTOR_R0_TORQUE_SPEED: {"torque": _hf16(31, 16), "speed": _hf16(15, 0)},
    RegAddr.MOTOR_R0_ANGLE: {"total_angle_rad": _f32()},
    RegAddr.MOTOR_R1_TORQUE_SPEED: {"torque": _hf16(31, 16), "speed": _hf16(15, 0)},
    RegAddr.MOTOR_R1_ANGLE: {"total_angle_rad": _f32()},
    RegAddr.MOTOR_R2_TORQUE_SPEED: {"torque": _hf16(31, 16), "speed": _hf16(15, 0)},
    RegAddr.MOTOR_R2_ANGLE: {"total_angle_rad": _f32()},

    # ---- 三角轮反馈 ----
    RegAddr.TRIWHEEL_ANGLE_CUR_FRONT: {
        "tri_lf_angle": _hf16(31, 16), "tri_rf_angle": _hf16(15, 0),
    },
    RegAddr.TRIWHEEL_ANGLE_CUR_REAR: {
        "tri_lr_angle": _hf16(31, 16), "tri_rr_angle": _hf16(15, 0),
    },
    RegAddr.TRIWHEEL_DUTY_CUR_FRONT: {
        "tri_lf_duty": _hf16(31, 16), "tri_rf_duty": _hf16(15, 0),
    },
    RegAddr.TRIWHEEL_DUTY_CUR_REAR: {
        "tri_lr_duty": _hf16(31, 16), "tri_rr_duty": _hf16(15, 0),
    },

    # ---- 角加速度计 ----
    RegAddr.ACCEL_LF_X: {"accel_lf_x": _f32()},
    RegAddr.ACCEL_LF_Y: {"accel_lf_y": _f32()},
    RegAddr.ACCEL_LF_Z: {"accel_lf_z": _f32()},
    RegAddr.ACCEL_RF_X: {"accel_rf_x": _f32()},
    RegAddr.ACCEL_RF_Y: {"accel_rf_y": _f32()},
    RegAddr.ACCEL_RF_Z: {"accel_rf_z": _f32()},
    RegAddr.ACCEL_LR_X: {"accel_lr_x": _f32()},
    RegAddr.ACCEL_LR_Y: {"accel_lr_y": _f32()},
    RegAddr.ACCEL_LR_Z: {"accel_lr_z": _f32()},
    RegAddr.ACCEL_RR_X: {"accel_rr_x": _f32()},
    RegAddr.ACCEL_RR_Y: {"accel_rr_y": _f32()},
    RegAddr.ACCEL_RR_Z: {"accel_rr_z": _f32()},

    # ---- 在线状态 ----
    RegAddr.ONLINE_STATUS: {
        "motor_id0_online": _bit(0),
        "motor_id1_online": _bit(1),
        "motor_id2_online": _bit(2),
        "motor_id3_online": _bit(3),
        "motor_id4_online": _bit(4),
        "motor_id5_online": _bit(5),
        "accel_lf_online": _bit(6),
        "accel_rf_online": _bit(7),
        "accel_lr_online": _bit(8),
        "accel_rr_online": _bit(9),
        "tof1_online": _bit(10),
        "tof2_online": _bit(11),
        "tof3_online": _bit(12),
        "tof4_online": _bit(13),
        "dbus_online": _bit(14),
    },
}


# ============================================================
# 帧构建（主机 → MCU）
# ============================================================

def build_read_request(reg_addr: int, length: int) -> bytes:
    """构建读寄存器请求帧。

    SYNC(0x55) | SOF(0x5A) | REG_ADDR(1B) | LEN(1B) | CRC16(2B)
    """
    header = struct.pack('<BB', reg_addr, length)
    crc = crc16_ccitt(header)
    return bytes([SYNC, SOF_HOST]) + header + struct.pack('<H', crc)


def build_write_request(reg_addr: int, data: bytes) -> bytes:
    """构建写寄存器请求帧。

    SYNC(0x55) | SOF(0x5A) | REG_ADDR(1B) | LEN(1B) | DATA(4B×LEN) | CRC16(2B)
    """
    length = len(data) // 4
    header = struct.pack('<BB', reg_addr, length)
    crc_input = header + data
    crc = crc16_ccitt(crc_input)
    return bytes([SYNC, SOF_HOST]) + crc_input + struct.pack('<H', crc)


def build_write_register(reg_addr: int, value: int) -> bytes:
    """构建单个寄存器写请求（value 为 32-bit 整数值）。"""
    return build_write_request(reg_addr, struct.pack('<I', value))


# ============================================================
# 响应解码（MCU → 主机）
# ============================================================

class ResponseDecoder:
    """MCU 响应帧解码器（状态机）。

    用法:
        decoder = ResponseDecoder(expected_data_len=8)
        for byte in serial_stream:
            data = decoder.feed(byte)
            if data is not None:
                handle(data)
    """

    STATE_IDLE = 0
    STATE_READING = 1

    def __init__(self):
        self._state = self.STATE_IDLE
        self._buf = bytearray()
        self._expected = 0
        self._crc_errors = 0
        self._total_responses = 0

    def set_expected(self, data_len: int):
        """设置期望的数据字节数（不含 CRC）。"""
        self._expected = data_len

    def feed(self, byte: int) -> Optional[bytes]:
        """喂入单个字节，完整响应到达时返回 data bytes，否则返回 None。"""
        b = byte & 0xFF

        if self._state == self.STATE_IDLE:
            if b == SOF_MCU:
                self._state = self.STATE_READING
                self._buf = bytearray()
            return None

        # STATE_READING
        self._buf.append(b)
        needed = self._expected + 2  # data + CRC16

        if len(self._buf) >= needed:
            data = bytes(self._buf[:self._expected])
            crc_received = struct.unpack_from('<H', self._buf, self._expected)[0]
            self._total_responses += 1
            self._state = self.STATE_IDLE

            crc_expected = crc16_ccitt(data)
            if crc_received != crc_expected:
                self._crc_errors += 1
                return None

            return data

        return None

    @property
    def crc_errors(self) -> int:
        return self._crc_errors

    @property
    def total_responses(self) -> int:
        return self._total_responses

    def reset(self):
        self._state = self.STATE_IDLE
        self._buf = bytearray()


# ============================================================
# 寄存器数据解析
# ============================================================

def parse_register(reg_addr: int, raw_value: int) -> dict:
    """将 32-bit 寄存器原始值按字段定义解析为 dict。"""
    fields = REGISTER_DEFS.get(reg_addr)
    if fields is None:
        return {"_raw": raw_value}

    result = {}
    for name, (offset, length, kind) in fields.items():
        mask = (1 << length) - 1
        raw_bits = (raw_value >> offset) & mask

        if kind == 'f':
            # float32
            result[name] = struct.unpack('<f', struct.pack('<I', raw_value))[0]
        elif kind == 1:
            # signed integer (int16 for half-register)
            if raw_bits & (1 << (length - 1)):
                raw_bits -= (1 << length)
            result[name] = raw_bits
        else:
            # unsigned integer or bit
            result[name] = raw_bits

    return result


def parse_response(reg_addr: int, data: bytes) -> List[dict]:
    """解析读响应数据，返回寄存器数据列表（每个元素一个 dict）。"""
    results = []
    num_regs = len(data) // 4
    for i in range(num_regs):
        raw = struct.unpack_from('<I', data, i * 4)[0]
        addr = reg_addr + i
        results.append({
            "addr": addr,
            **parse_register(addr, raw),
        })
    return results


# ============================================================
# 寄存器枚举值
# ============================================================

CHASSIS_MODE_NAMES = {0: "安全", 1: "开环", 2: "闭环", 3: "从机"}

TOF_STATUS_NAMES = {0: "无效", 1: "有效", 2: "弱信号", 3: "超量程"}

# ============================================================
# 便捷：获取寄存器值
# ============================================================

def pack_register(reg_addr: int, fields: dict) -> int:
    """将字段 dict 打包为 32-bit 寄存器值。"""
    defs = REGISTER_DEFS.get(reg_addr)
    if defs is None:
        return 0

    value = 0
    for name, (offset, length, kind) in defs.items():
        if name not in fields:
            continue
        field_val = fields[name]

        if kind == 'f':
            packed = struct.unpack('<I', struct.pack('<f', float(field_val)))[0]
            value |= (packed & ((1 << 32) - 1))
        else:
            mask = (1 << length) - 1
            if kind == 1 and field_val < 0:
                field_val = (field_val + (1 << length)) & mask
            value |= (int(field_val) & mask) << offset

    return value
