"""模拟串口数据源 —— MCU 寄存器仿真器。

接收主机读请求，返回模拟的寄存器数据。
当无下位机硬件时，用于前端开发调试。
"""

import math
import random
import struct
import time
import threading
from collections import deque

from viz_dashboard.protocol import (
    SYNC, SOF_HOST, SOF_MCU,
    crc16_ccitt,
    RegAddr,
    parse_register,
    pack_register,
)


def _build_read_response(reg_addr: int, values: list) -> bytes:
    """构建读响应帧: SOF(0xA5) + DATA(4B×N) + CRC16(2B)"""
    data = b''.join(struct.pack('<I', v) for v in values)
    crc = crc16_ccitt(data)
    return bytes([SOF_MCU]) + data + struct.pack('<H', crc)


class MockSerial:
    """伪装成 pyserial.Serial，write() 接收请求，read() 返回响应字节。"""

    def __init__(self):
        self.is_open = True
        self._read_buf = bytearray()
        self._lock = threading.Lock()
        self._stop = False

        # 模拟时间
        self._t = 0.0

        # 模拟错误标志（随机翻转）
        self._error_flags = 0

        # 模拟 TOF 基线
        self._tof_bases = [850, 820, 910, 230]

        # 在线状态
        self._online = 0x7FFF  # bits 0-14 all 1

        # 请求解析状态机
        self._req_state = 0       # 0=idle, 1=got_55, 2=got_5A, 3=reading
        self._req_buf = bytearray()
        self._req_payload_len = 0

        self._thread = threading.Thread(target=self._tick_loop, daemon=True)
        self._thread.start()

    def read(self, n: int = 1) -> bytes:
        while True:
            with self._lock:
                if len(self._read_buf) >= n:
                    result = bytes(self._read_buf[:n])
                    del self._read_buf[:n]
                    return result
            time.sleep(0.0005)

    def write(self, data: bytes):
        """接收主机发来的字节，解析请求并生成响应。"""
        for byte in data:
            self._feed_request(byte)

    def close(self):
        self._stop = True
        self.is_open = False

    def _enqueue_response(self, data: bytes):
        with self._lock:
            self._read_buf.extend(data)

    # ------------------------------------------------------------
    # 请求解析状态机
    # ------------------------------------------------------------

    def _feed_request(self, byte: int):
        """解析主机请求帧: 0x55 0x5A ADDR LEN CRC16"""
        b = byte & 0xFF

        if self._req_state == 0:
            if b == SYNC:
                self._req_state = 1
            return

        if self._req_state == 1:
            if b == SOF_HOST:
                self._req_state = 2
                self._req_buf = bytearray()
            else:
                self._req_state = 0
            return

        if self._req_state == 2:
            # REG_ADDR
            self._req_buf.append(b)
            self._req_state = 3
            return

        # state == 3: LEN + CRC16
        self._req_buf.append(b)
        if len(self._req_buf) >= 4:
            # REG_ADDR(1) + LEN(1) + CRC16(2)
            reg_addr = self._req_buf[0]
            length = self._req_buf[1]
            crc_received = struct.unpack_from('<H', self._req_buf, 2)[0]
            self._req_state = 0

            crc_expected = crc16_ccitt(bytes(self._req_buf[:2]))
            if crc_received == crc_expected:
                self._handle_read_request(reg_addr, length)

    def _handle_read_request(self, reg_addr: int, length: int):
        """生成模拟寄存器数据并返回响应。"""
        values = []
        for i in range(length):
            addr = reg_addr + i
            val = self._gen_register(addr)
            values.append(val)
        resp = _build_read_response(reg_addr, values)
        self._enqueue_response(resp)

    # ------------------------------------------------------------
    # 模拟数据生成
    # ------------------------------------------------------------

    def _tick_loop(self):
        """后台更新模拟状态。"""
        tick = 0
        while not self._stop:
            self._t += 0.005

            # 偶尔翻转错误位
            if tick > 0 and tick % 600 == 0:
                self._error_flags ^= (1 << random.randint(0, 8))

            tick += 1
            if tick >= 1000:
                tick = 0
            time.sleep(0.005)  # 200Hz

    def _gen_register(self, addr: int) -> int:
        t = self._t

        if addr == RegAddr.CTRL:
            return pack_register(RegAddr.CTRL, {
                "chassis_mode": 2,  # 闭环
                "foc_enable": 1,
                "brushed_enable": 0,
                "foc_ctrl_mode": 0,
                "brushed_ctrl_mode": 0,
            })

        if addr == RegAddr.ONLINE_STATUS:
            return self._online

        # ---- TOF ----
        if RegAddr.TOF1 <= addr <= RegAddr.TOF4:
            idx = addr - RegAddr.TOF1
            base = self._tof_bases[idx]
            dist = max(0, int(base + random.gauss(0, 15)))
            signal = max(0, min(255, int([200, 180, 210, 90][idx] + random.gauss(0, 5))))
            status = 1 if dist > 50 else 0
            fault = 0 if dist > 30 else 1
            return (dist << 16) | (signal << 8) | (status << 5) | (fault << 4)

        # ---- IMU ----
        pitch = math.sin(t * 2.0) * 3.0
        roll = math.cos(t * 1.7) * 1.5
        yaw = t * 10.0
        cy = math.cos(math.radians(yaw) / 2)
        sy = math.sin(math.radians(yaw) / 2)
        cp = math.cos(math.radians(pitch) / 2)
        sp = math.sin(math.radians(pitch) / 2)
        cr = math.cos(math.radians(roll) / 2)
        sr = math.sin(math.radians(roll) / 2)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        imu_map = {
            RegAddr.IMU_QUAT_W: qw,
            RegAddr.IMU_QUAT_X: qx,
            RegAddr.IMU_QUAT_Y: qy,
            RegAddr.IMU_QUAT_Z: qz,
            RegAddr.IMU_YAW: math.degrees(yaw) % 360,
            RegAddr.IMU_PITCH: pitch,
            RegAddr.IMU_ROLL: roll,
            RegAddr.IMU_GYRO_YAW: random.gauss(0, 0.05),
            RegAddr.IMU_GYRO_PITCH: random.gauss(0, 0.05),
            RegAddr.IMU_GYRO_ROLL: random.gauss(0, 0.05),
            RegAddr.IMU_ACCEL_X: random.gauss(0, 0.02),
            RegAddr.IMU_ACCEL_Y: random.gauss(0, 0.02),
            RegAddr.IMU_ACCEL_Z: 0.98 + random.gauss(0, 0.02),
            RegAddr.IMU_TEMP: 35.0 + random.gauss(0, 0.5),
        }
        if addr in imu_map:
            return struct.unpack('<I', struct.pack('<f', imu_map[addr]))[0]

        # ---- 驱动电机反馈 (0x29-0x34) ----
        # 电机顺序: L3, L4, L5, R0, R1, R2
        motor_index_map = {
            RegAddr.MOTOR_L3_TORQUE_SPEED: 0,
            RegAddr.MOTOR_L4_TORQUE_SPEED: 1,
            RegAddr.MOTOR_L5_TORQUE_SPEED: 2,
            RegAddr.MOTOR_R0_TORQUE_SPEED: 3,
            RegAddr.MOTOR_R1_TORQUE_SPEED: 4,
            RegAddr.MOTOR_R2_TORQUE_SPEED: 5,
        }

        if addr in motor_index_map:
            # speed: rad/s × 16 (电角度速度), ~120 rad/s → ~1920 raw → ~1146 rpm
            speed_rads = 120.0 + random.gauss(0, 3)
            speed_raw = int(speed_rads * 16)
            torque_raw = int(random.gauss(0, 100))
            speed_raw = max(-32768, min(32767, speed_raw))
            torque_raw = max(-32768, min(32767, torque_raw))
            return ((torque_raw & 0xFFFF) << 16) | (speed_raw & 0xFFFF)

        # Motor angle registers
        motor_angle_regs = {
            RegAddr.MOTOR_L3_ANGLE: 0, RegAddr.MOTOR_L4_ANGLE: 1,
            RegAddr.MOTOR_L5_ANGLE: 2, RegAddr.MOTOR_R0_ANGLE: 3,
            RegAddr.MOTOR_R1_ANGLE: 4, RegAddr.MOTOR_R2_ANGLE: 5,
        }
        if addr in motor_angle_regs:
            idx = motor_angle_regs[addr]
            angle = t * (12.0 + idx * 0.5)
            return struct.unpack('<I', struct.pack('<f', angle))[0]

        # ---- 三角轮反馈 ----
        if addr == RegAddr.TRIWHEEL_ANGLE_CUR_FRONT:
            la = int((45.0 + random.gauss(0, 0.5)) * 100)
            ra = int((-45.0 + random.gauss(0, 0.5)) * 100)
            return ((la & 0xFFFF) << 16) | (ra & 0xFFFF)
        if addr == RegAddr.TRIWHEEL_ANGLE_CUR_REAR:
            la = int((45.0 + random.gauss(0, 0.5)) * 100)
            ra = int((-45.0 + random.gauss(0, 0.5)) * 100)
            return ((la & 0xFFFF) << 16) | (ra & 0xFFFF)

        if addr == RegAddr.TRIWHEEL_DUTY_CUR_FRONT:
            ld = int(random.uniform(-0.5, 0.5) * 1000)
            rd = int(random.uniform(-0.5, 0.5) * 1000)
            return ((ld & 0xFFFF) << 16) | (rd & 0xFFFF)
        if addr == RegAddr.TRIWHEEL_DUTY_CUR_REAR:
            ld = int(random.uniform(-0.5, 0.5) * 1000)
            rd = int(random.uniform(-0.5, 0.5) * 1000)
            return ((ld & 0xFFFF) << 16) | (rd & 0xFFFF)

        # ---- 角加速度计 ----
        accel_map = {
            RegAddr.ACCEL_LF_X: (0.02, 0.02), RegAddr.ACCEL_LF_Y: (0.02, 0.02), RegAddr.ACCEL_LF_Z: (0.98, 0.02),
            RegAddr.ACCEL_RF_X: (0.02, 0.02), RegAddr.ACCEL_RF_Y: (0.02, 0.02), RegAddr.ACCEL_RF_Z: (0.98, 0.02),
            RegAddr.ACCEL_LR_X: (0.02, 0.02), RegAddr.ACCEL_LR_Y: (0.02, 0.02), RegAddr.ACCEL_LR_Z: (0.98, 0.02),
            RegAddr.ACCEL_RR_X: (0.02, 0.02), RegAddr.ACCEL_RR_Y: (0.02, 0.02), RegAddr.ACCEL_RR_Z: (0.98, 0.02),
        }
        if addr in accel_map:
            mean, std = accel_map[addr]
            return struct.unpack('<I', struct.pack('<f', mean + random.gauss(0, std)))[0]

        # ---- 未定义的寄存器 ----
        return 0
