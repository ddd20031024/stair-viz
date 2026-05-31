"""通信协议层:FlexWire 寄存器读写 + CRC8。

基于 TI FlexWire 协议改进，取消设备地址段，增加读取长度。
寄存器地址 + 内容形式，每个寄存器 32-bit。
"""

import struct
from enum import IntEnum
from typing import Optional, List, Tuple


# ============================================================
# CRC8
# ============================================================

def crc8(data: bytes) -> int:
    """CRC-8，多项式 0x07。"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


def crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT (同 CRC-16/MODBUS)，多项式 0x1021。（保留，新协议不再使用。）"""
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
CMD_READ = 0x01      # 读命令
CMD_WRITE = 0x02     # 写命令


# ============================================================
# 寄存器地址定义（十进制整数值）
# ============================================================

class RegAddr(IntEnum):
    """Register addresses from read_notes/上下位机通信.xlsx."""

    CTRL = 0x00
    DRIVE_SPEED = 0x01
    DRIVE_ACCEL = 0x02
    LEFT_DRIVE_PID = 0x03
    RIGHT_DRIVE_PID = 0x04
    DRIVE_FF_TORQUE = 0x05
    OPENLOOP_TORQUE = 0x06
    TRIWHEEL_ANGLE_FRONT = 0x07
    TRIWHEEL_ANGLE_REAR = 0x08
    TRIWHEEL_DUTY_FRONT = 0x09
    TRIWHEEL_DUTY_REAR = 0x10

    GIMBAL_DUTY = 0x11
    RESERVED_12 = 0x12
    RESERVED_13 = 0x13
    RESERVED_14 = 0x14
    RESERVED_15 = 0x15
    RESERVED_16 = 0x16

    TOF1 = 0x17
    TOF2 = 0x18
    TOF3 = 0x19
    TOF4 = 0x20

    IMU_QUAT_W = 0x21
    IMU_QUAT_X = 0x22
    IMU_QUAT_Y = 0x23
    IMU_QUAT_Z = 0x24
    IMU_YAW = 0x25
    IMU_PITCH = 0x26
    IMU_ROLL = 0x27
    IMU_GYRO_YAW = 0x28
    IMU_GYRO_PITCH = 0x29
    IMU_GYRO_ROLL = 0x30
    IMU_ACCEL_X = 0x31
    IMU_ACCEL_Y = 0x32
    IMU_ACCEL_Z = 0x33
    IMU_TEMP = 0x34

    MOTOR_L3_TORQUE_SPEED = 0x35
    MOTOR_L3_ANGLE = 0x36
    MOTOR_L4_TORQUE_SPEED = 0x37
    MOTOR_L4_ANGLE = 0x38
    MOTOR_L5_TORQUE_SPEED = 0x39
    MOTOR_L5_ANGLE = 0x40
    MOTOR_R0_TORQUE_SPEED = 0x41
    MOTOR_R0_ANGLE = 0x42
    MOTOR_R1_TORQUE_SPEED = 0x43
    MOTOR_R1_ANGLE = 0x44
    MOTOR_R2_TORQUE_SPEED = 0x45
    MOTOR_R2_ANGLE = 0x46

    TRIWHEEL_ANGLE_CUR_FRONT = 0x47
    TRIWHEEL_ANGLE_CUR_REAR = 0x48
    TRIWHEEL_DUTY_CUR_FRONT = 0x49
    TRIWHEEL_DUTY_CUR_REAR = 0x50

    ACCEL_LF_X = 0x51
    ACCEL_LF_Y = 0x52
    ACCEL_LF_Z = 0x53
    ACCEL_RF_X = 0x54
    ACCEL_RF_Y = 0x55
    ACCEL_RF_Z = 0x56
    ACCEL_LR_X = 0x57
    ACCEL_LR_Y = 0x58
    ACCEL_LR_Z = 0x59
    ACCEL_RR_X = 0x60
    ACCEL_RR_Y = 0x61
    ACCEL_RR_Z = 0x62

    GIMBAL_QUAT_W = 0x63
    GIMBAL_QUAT_X = 0x64
    GIMBAL_QUAT_Y = 0x65
    GIMBAL_QUAT_Z = 0x66
    GIMBAL_YAW = 0x67
    GIMBAL_PITCH = 0x68
    GIMBAL_ROLL = 0x69
    GIMBAL_GYRO_YAW = 0x70
    GIMBAL_GYRO_PITCH = 0x71
    GIMBAL_GYRO_ROLL = 0x72
    GIMBAL_ACCEL_X = 0x73
    GIMBAL_ACCEL_Y = 0x74
    GIMBAL_ACCEL_Z = 0x75
    GIMBAL_DUTY_CUR = 0x76

    RESERVED_77 = 0x77
    RESERVED_78 = 0x78
    RESERVED_79 = 0x79
    RESERVED_80 = 0x80
    RESERVED_81 = 0x81
    ONLINE_STATUS = 0x82

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


def _i32() -> Tuple[int, int, str]:
    """全寄存器 int32"""
    return (0, 32, 'i')


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
        "triwheel_ctrl_mode": _bit(25),
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
    RegAddr.GIMBAL_DUTY: {
        "gimbal_x_duty_target": _hf16(31, 16),
        "gimbal_y_duty_target": _hf16(15, 0),
    },

    # ---- TOF 传感器 ----
    RegAddr.TOF1: {
        "distance_mm": _hfu16(31, 16),
        "signal": _bits(15, 8),
        "status": _bits(7, 4),
        "fault": _bits(3, 0),
    },
    RegAddr.TOF2: {
        "distance_mm": _hfu16(31, 16),
        "signal": _bits(15, 8),
        "status": _bits(7, 4),
        "fault": _bits(3, 0),
    },
    RegAddr.TOF3: {
        "distance_mm": _hfu16(31, 16),
        "signal": _bits(15, 8),
        "status": _bits(7, 4),
        "fault": _bits(3, 0),
    },
    RegAddr.TOF4: {
        "distance_mm": _hfu16(31, 16),
        "signal": _bits(15, 8),
        "status": _bits(7, 4),
        "fault": _bits(3, 0),
    },

    # ---- 底盘 IMU float32 ----
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
    RegAddr.IMU_ACCEL_X: {"accel_x_ms2": _f32()},
    RegAddr.IMU_ACCEL_Y: {"accel_y_ms2": _f32()},
    RegAddr.IMU_ACCEL_Z: {"accel_z_ms2": _f32()},
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

    # ---- 四轮加速度计 (int32, mg) ----
    RegAddr.ACCEL_LF_X: {"accel_lf_x_mg": _i32()},
    RegAddr.ACCEL_LF_Y: {"accel_lf_y_mg": _i32()},
    RegAddr.ACCEL_LF_Z: {"accel_lf_z_mg": _i32()},
    RegAddr.ACCEL_RF_X: {"accel_rf_x_mg": _i32()},
    RegAddr.ACCEL_RF_Y: {"accel_rf_y_mg": _i32()},
    RegAddr.ACCEL_RF_Z: {"accel_rf_z_mg": _i32()},
    RegAddr.ACCEL_LR_X: {"accel_lr_x_mg": _i32()},
    RegAddr.ACCEL_LR_Y: {"accel_lr_y_mg": _i32()},
    RegAddr.ACCEL_LR_Z: {"accel_lr_z_mg": _i32()},
    RegAddr.ACCEL_RR_X: {"accel_rr_x_mg": _i32()},
    RegAddr.ACCEL_RR_Y: {"accel_rr_y_mg": _i32()},
    RegAddr.ACCEL_RR_Z: {"accel_rr_z_mg": _i32()},

    # ---- 云台 IMU float32 ----
    RegAddr.GIMBAL_QUAT_W: {"gimbal_quat_w": _f32()},
    RegAddr.GIMBAL_QUAT_X: {"gimbal_quat_x": _f32()},
    RegAddr.GIMBAL_QUAT_Y: {"gimbal_quat_y": _f32()},
    RegAddr.GIMBAL_QUAT_Z: {"gimbal_quat_z": _f32()},
    RegAddr.GIMBAL_YAW: {"gimbal_yaw_deg": _f32()},
    RegAddr.GIMBAL_PITCH: {"gimbal_pitch_deg": _f32()},
    RegAddr.GIMBAL_ROLL: {"gimbal_roll_deg": _f32()},
    RegAddr.GIMBAL_GYRO_YAW: {"gimbal_gyro_yaw_dps": _f32()},
    RegAddr.GIMBAL_GYRO_PITCH: {"gimbal_gyro_pitch_dps": _f32()},
    RegAddr.GIMBAL_GYRO_ROLL: {"gimbal_gyro_roll_dps": _f32()},
    RegAddr.GIMBAL_ACCEL_X: {"gimbal_accel_x_g": _f32()},
    RegAddr.GIMBAL_ACCEL_Y: {"gimbal_accel_y_g": _f32()},
    RegAddr.GIMBAL_ACCEL_Z: {"gimbal_accel_z_g": _f32()},
    RegAddr.GIMBAL_DUTY_CUR: {
        "gimbal_x_duty": _hf16(31, 16), "gimbal_y_duty": _hf16(15, 0),
    },

    # ---- 在线状态位图 (MSB 对齐) ----
    RegAddr.ONLINE_STATUS: {
        "motor_id0_online": _bit(31),
        "motor_id1_online": _bit(30),
        "motor_id2_online": _bit(29),
        "motor_id3_online": _bit(28),
        "motor_id4_online": _bit(27),
        "motor_id5_online": _bit(26),
        "accel_lf_online": _bit(25),
        "accel_rf_online": _bit(24),
        "accel_lr_online": _bit(23),
        "accel_rr_online": _bit(22),
        "tof1_online": _bit(21),
        "tof2_online": _bit(20),
        "tof3_online": _bit(19),
        "tof4_online": _bit(18),
        "dbus_online": _bit(17),
    },
}


# ============================================================
# 帧构建（主机 → MCU）
# ============================================================

def build_read_request(reg_addr: int, length: int) -> bytes:
    """构建读寄存器请求帧。

    SYNC(0x55) | CMD_READ(0x01) | REG_ADDR(1B) | LEN(1B) | CRC8(1B)
    """
    header = struct.pack('<BBBB', SYNC, CMD_READ, reg_addr, length)
    crc = crc8(header)
    return header + bytes([crc])


def build_write_request(reg_addr: int, data: bytes) -> bytes:
    """构建写寄存器请求帧。

    SYNC(0x55) | CMD_WRITE(0x02) | REG_ADDR(1B) | LEN(1B) | DATA(4B×LEN) | CRC8(1B)
    """
    length = len(data) // 4
    header = struct.pack('<BBBB', SYNC, CMD_WRITE, reg_addr, length)
    crc = crc8(header + data)
    return header + data + bytes([crc])


def build_write_register(reg_addr: int, value: int) -> bytes:
    """构建单个寄存器写请求（value 为 32-bit 整数值）。"""
    return build_write_request(reg_addr, struct.pack('<I', value))


# ============================================================
# 响应解码（MCU → 主机）
# ============================================================

class ResponseDecoder:
    """MCU 响应帧解码器。

    新协议响应无帧头，格式为: DATA(4B×N) | CRC8(1B)。
    发送请求后直接读取 4*LEN+1 字节即可，无需状态机。
    """

    def __init__(self):
        self._crc_errors = 0
        self._total_responses = 0

    def decode(self, data: bytes) -> Optional[bytes]:
        """解码响应：校验 CRC8 并返回数据部分，失败返回 None。"""
        if len(data) < 2:
            return None
        payload = data[:-1]
        crc_received = data[-1]
        crc_expected = crc8(payload)
        self._total_responses += 1
        if crc_received != crc_expected:
            self._crc_errors += 1
            return None
        return payload

    @property
    def crc_errors(self) -> int:
        return self._crc_errors

    @property
    def total_responses(self) -> int:
        return self._total_responses

    def reset(self):
        pass


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
            result[name] = struct.unpack('<f', struct.pack('<I', raw_value))[0]
        elif kind == 'i':
            result[name] = struct.unpack('<i', struct.pack('<I', raw_value))[0]
        elif kind == 1:
            if raw_bits & (1 << (length - 1)):
                raw_bits -= (1 << length)
            result[name] = raw_bits
        else:
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
# 便捷：寄存器值打包
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
            value |= packed
        elif kind == 'i':
            packed = struct.unpack('<I', struct.pack('<i', int(field_val)))[0]
            value |= packed
        else:
            mask = (1 << length) - 1
            if kind == 1 and field_val < 0:
                field_val = (field_val + (1 << length)) & mask
            value |= (int(field_val) & mask) << offset

    return value
