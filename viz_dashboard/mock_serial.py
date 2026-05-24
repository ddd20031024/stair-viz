"""模拟串口数据源 —— MCU 寄存器仿真器。

接收主机读请求，返回模拟的寄存器数据。
当无下位机硬件时，用于前端开发调试。
"""

import math
import random
import struct
import time
import threading

from viz_dashboard.protocol import (
    SYNC, CMD_READ, CMD_WRITE,
    crc8,
    RegAddr,
    parse_register,
    pack_register,
)


def _build_read_response(reg_addr: int, values: list) -> bytes:
    """构建读响应帧: DATA(4B×N) | CRC8(1B)（新协议无帧头）。"""
    data = b''.join(struct.pack('<I', v) for v in values)
    crc = crc8(data)
    return data + bytes([crc])


class MockSerial:
    """伪装成 pyserial.Serial，write() 接收请求，read() 返回响应字节。"""

    def __init__(self):
        self.is_open = True
        self._read_buf = bytearray()
        self._lock = threading.Lock()
        self._stop = False

        self._t = 0.0

        # 模拟错误标志（随机翻转）
        self._error_flags = 0

        # 模拟 TOF 基线
        self._tof_bases = [850, 820, 910, 230]

        # 在线状态 (MSB 对齐: bits 31..17)
        self._online = 0
        for bit in [31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17]:
            self._online |= (1 << bit)

        # 请求解析: SYNC(0x55) | CMD(1B) | REG_ADDR(1B) | LEN(1B) | CRC8(1B)
        self._req_state = 0       # 0=SYNC, 1=CMD, 2=ADDR, 3=LEN, 4=CRC
        self._req_cmd = 0
        self._req_addr = 0

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
    # 请求解析状态机: 0x55 | CMD | ADDR | LEN | CRC8
    # ------------------------------------------------------------

    def _feed_request(self, byte: int):
        b = byte & 0xFF

        if self._req_state == 0:
            if b == SYNC:
                self._req_state = 1
            return

        if self._req_state == 1:
            self._req_cmd = b
            self._req_state = 2
            return

        if self._req_state == 2:
            self._req_addr = b
            self._req_state = 3
            return

        if self._req_state == 3:
            self._req_len = b
            self._req_state = 4
            return

        # state == 4: CRC8
        self._req_state = 0
        crc_received = b
        header = bytes([SYNC, self._req_cmd, self._req_addr, self._req_len])
        crc_expected = crc8(header)
        if crc_received == crc_expected:
            if self._req_cmd == CMD_READ:
                self._handle_read_request(self._req_addr, self._req_len)

    def _handle_read_request(self, reg_addr: int, length: int):
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
        tick = 0
        while not self._stop:
            self._t += 0.005

            if tick > 0 and tick % 600 == 0:
                self._error_flags ^= (1 << random.randint(0, 8))

            tick += 1
            if tick >= 1000:
                tick = 0
            time.sleep(0.005)

    def _gen_register(self, addr: int) -> int:
        t = self._t

        if addr == RegAddr.CTRL:
            return pack_register(RegAddr.CTRL, {
                "chassis_mode": 2,
                "foc_enable": 1,
                "brushed_enable": 0,
                "foc_ctrl_mode": 0,
                "triwheel_ctrl_mode": 0,
            })

        if addr == RegAddr.ONLINE_STATUS:
            return self._online

        # ---- 云台控制 ----
        if addr == RegAddr.GIMBAL_DUTY:
            return 0

        # ---- TOF (0x11-0x14) ----
        if RegAddr.TOF1 <= addr <= RegAddr.TOF4:
            idx = addr - RegAddr.TOF1
            base = self._tof_bases[idx]
            dist = max(0, int(base + random.gauss(0, 15)))
            signal = max(0, min(255, int([200, 180, 210, 90][idx] + random.gauss(0, 5))))
            status = 1 if dist > 50 else 0
            fault = 0 if dist > 30 else random.choice([0, 1])
            return (dist << 16) | (signal << 8) | (status << 4) | fault

        # ---- 底盘 IMU (0x15-0x22) ----
        pitch = math.sin(t * 2.0) * 3.0
        roll = math.cos(t * 1.7) * 1.5
        yaw = (t * 10.0) % 360
        yaw_rad = math.radians(yaw)
        cy = math.cos(yaw_rad / 2)
        sy = math.sin(yaw_rad / 2)
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
            RegAddr.IMU_YAW: yaw,
            RegAddr.IMU_PITCH: pitch,
            RegAddr.IMU_ROLL: roll,
            RegAddr.IMU_GYRO_YAW: random.gauss(0, 0.05),
            RegAddr.IMU_GYRO_PITCH: random.gauss(0, 0.05),
            RegAddr.IMU_GYRO_ROLL: random.gauss(0, 0.05),
            RegAddr.IMU_ACCEL_X: random.gauss(0, 0.2),
            RegAddr.IMU_ACCEL_Y: random.gauss(0, 0.2),
            RegAddr.IMU_ACCEL_Z: 9.8 + random.gauss(0, 0.2),
            RegAddr.IMU_TEMP: 35.0 + random.gauss(0, 0.5),
        }
        if addr in imu_map:
            return struct.unpack('<I', struct.pack('<f', imu_map[addr]))[0]

        # ---- 驱动电机反馈 ----
        motor_ts_addrs = [
            RegAddr.MOTOR_L3_TORQUE_SPEED, RegAddr.MOTOR_L4_TORQUE_SPEED,
            RegAddr.MOTOR_L5_TORQUE_SPEED, RegAddr.MOTOR_R0_TORQUE_SPEED,
            RegAddr.MOTOR_R1_TORQUE_SPEED, RegAddr.MOTOR_R2_TORQUE_SPEED,
        ]
        if addr in motor_ts_addrs:
            speed_rads = 120.0 + random.gauss(0, 3)
            speed_raw = max(-32768, min(32767, int(speed_rads * 16)))
            torque_raw = max(-32768, min(32767, int(random.gauss(0, 100))))
            return ((torque_raw & 0xFFFF) << 16) | (speed_raw & 0xFFFF)

        motor_angle_addrs = [
            RegAddr.MOTOR_L3_ANGLE, RegAddr.MOTOR_L4_ANGLE, RegAddr.MOTOR_L5_ANGLE,
            RegAddr.MOTOR_R0_ANGLE, RegAddr.MOTOR_R1_ANGLE, RegAddr.MOTOR_R2_ANGLE,
        ]
        if addr in motor_angle_addrs:
            idx = motor_angle_addrs.index(addr)
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

        # ---- 四轮加速度计 (int32, mg) ----
        accel_mg = [
            (RegAddr.ACCEL_LF_X, 20), (RegAddr.ACCEL_LF_Y, 20), (RegAddr.ACCEL_LF_Z, 980),
            (RegAddr.ACCEL_RF_X, 20), (RegAddr.ACCEL_RF_Y, 20), (RegAddr.ACCEL_RF_Z, 980),
            (RegAddr.ACCEL_LR_X, 20), (RegAddr.ACCEL_LR_Y, 20), (RegAddr.ACCEL_LR_Z, 980),
            (RegAddr.ACCEL_RR_X, 20), (RegAddr.ACCEL_RR_Y, 20), (RegAddr.ACCEL_RR_Z, 980),
        ]
        for a_addr, baseline in accel_mg:
            if addr == a_addr:
                val = int(baseline + random.gauss(0, 10))
                return struct.unpack('<I', struct.pack('<i', val))[0]

        # ---- 云台 IMU ----
        gimbal_imu = {
            RegAddr.GIMBAL_QUAT_W: 1.0, RegAddr.GIMBAL_QUAT_X: 0.0,
            RegAddr.GIMBAL_QUAT_Y: 0.0, RegAddr.GIMBAL_QUAT_Z: 0.0,
            RegAddr.GIMBAL_YAW: 0.0,
            RegAddr.GIMBAL_PITCH: math.sin(t * 1.5) * 5.0,
            RegAddr.GIMBAL_ROLL: math.cos(t * 1.3) * 2.0,
            RegAddr.GIMBAL_GYRO_YAW: random.gauss(0, 0.02),
            RegAddr.GIMBAL_GYRO_PITCH: random.gauss(0, 0.02),
            RegAddr.GIMBAL_GYRO_ROLL: random.gauss(0, 0.02),
            RegAddr.GIMBAL_ACCEL_X: random.gauss(0, 0.01),
            RegAddr.GIMBAL_ACCEL_Y: random.gauss(0, 0.01),
            RegAddr.GIMBAL_ACCEL_Z: -1.0 + random.gauss(0, 0.01),
        }
        if addr in gimbal_imu:
            return struct.unpack('<I', struct.pack('<f', gimbal_imu[addr]))[0]

        if addr == RegAddr.GIMBAL_DUTY_CUR:
            gx = int(random.uniform(-0.3, 0.3) * 1000)
            gy = int(random.uniform(-0.3, 0.3) * 1000)
            return ((gx & 0xFFFF) << 16) | (gy & 0xFFFF)

        # ---- 保留寄存器 ----
        return 0
