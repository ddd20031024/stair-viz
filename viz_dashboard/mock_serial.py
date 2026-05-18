"""模拟串口数据源 —— 生成合理但随机的传感器数据。

当无下位机硬件时，用于前端开发调试。
数据遵循通信协议设计 v1.0 的帧格式。
"""

import math
import random
import struct
import time
import threading

from viz_dashboard.protocol import (
    SOF1, SOF2, DIR_UPLOAD, crc16_ccitt,
    PacketID, ChassisMode,
)


def _build_frame(packet_id: int, payload: bytes, seq: int) -> bytes:
    frame_type = packet_id  # upload direction, bit7=0
    header = bytes([frame_type]) + struct.pack('<H', len(payload)) + bytes([seq])
    crc = crc16_ccitt(header + payload)
    return bytes([SOF1, SOF2]) + header + payload + struct.pack('<H', crc)


class MockSerial:
    """伪装成 pyserial.Serial，read(1) 返回模拟帧字节。"""

    def __init__(self):
        self.is_open = True
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._seq = 0
        self._stop = False
        self._thread = threading.Thread(target=self._generator, daemon=True)

        # 模拟状态
        self._t = 0.0
        self._mode = ChassisMode.CLOSED_LOOP
        self._error_flags = 0
        self._tof_distances = [850, 820, 910, 230]   # mm
        self._tof_status = [1, 1, 1, 2]
        self._tof_signals = [200, 180, 210, 90]

        self._thread.start()

    def read(self, n: int = 1) -> bytes:
        while True:
            with self._lock:
                if len(self._buf) >= n:
                    result = bytes(self._buf[:n])
                    del self._buf[:n]
                    return result
            time.sleep(0.001)

    def close(self):
        self._stop = True
        self.is_open = False

    def _enqueue(self, data: bytes):
        with self._lock:
            self._buf.extend(data)

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def _generator(self):
        """按调度表生成模拟帧。"""
        tick = 0
        while not self._stop:
            try:
                if tick % 10 == 0:
                    self._gen_fast_status()
                if tick % 20 == 5:
                    self._gen_tofsense_full()
                if tick % 20 == 15:
                    self._gen_imu_attitude()
                if tick % 50 == 10:
                    self._gen_foc_feedback()
                if tick % 50 == 35:
                    self._gen_triwheel_feedback()
                if tick % 100 == 50:
                    self._gen_drv8701_feedback()

                # 模拟偶尔的事件
                if tick > 0 and tick % 500 == 0:
                    self._gen_event()

                tick += 1
                if tick >= 100:
                    tick = 0
                self._t += 0.001

                time.sleep(0.001)   # ~1kHz 调度
            except Exception:
                break

    def _gen_fast_status(self):
        d0 = self._tof_distances
        valid_distances = [d for d, s in zip(d0, self._tof_status) if s == 1 and d > 0]
        tof_valid = len(valid_distances)
        tof_min = min(valid_distances) if valid_distances else 0
        tof_max = max(valid_distances) if valid_distances else 0

        # 模拟小幅波动
        pitch = round(math.sin(self._t * 2.0) * 3.0, 1)
        roll = round(math.cos(self._t * 1.7) * 1.5, 1)
        chassis_p = round(pitch * 0.9, 1)

        payload = struct.pack('<BBHHHBBB',
            self._mode,         # chassis_mode
            0x42,               # heartbeat_echo
            self._error_flags,  # error_flags
            tof_min,
            tof_max,
            tof_valid,
            int(pitch * 10) & 0xFF,
            int(roll * 10) & 0xFF,
            int(chassis_p * 10) & 0xFF,
        )
        payload += b'\x00' * 11   # reserved[11] + pad to 24B
        self._enqueue(_build_frame(PacketID.FAST_STATUS, payload, self._next_seq()))

    def _gen_tofsense_full(self):
        # 添加噪声
        noisy = []
        for i in range(4):
            base = self._tof_distances[i]
            dist = max(0, int(base + random.gauss(0, 15)))
            status = self._tof_status[i]
            signal = max(0, min(255, int(self._tof_signals[i] + random.gauss(0, 5))))
            noisy.append((dist, status, signal))

        payload = bytearray()
        for dist, status, signal in noisy:
            payload += struct.pack('<HBB', dist, status, signal)
        self._enqueue(_build_frame(PacketID.TOFSENSE_FULL, bytes(payload), self._next_seq()))

    def _gen_imu_attitude(self):
        pitch = math.sin(self._t * 2.0) * 3.0
        roll = math.cos(self._t * 1.7) * 1.5
        yaw = self._t * 10.0

        # 欧拉 → 四元数
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

        payload = struct.pack('<fffffffffff',
            qw, qx, qy, qz,
            random.gauss(0, 0.05), random.gauss(0, 0.05), random.gauss(0, 0.05),
            random.gauss(0, 0.02), random.gauss(0, 0.02), random.gauss(0.98, 0.02),
            35.0 + random.gauss(0, 0.5),
        )
        self._enqueue(_build_frame(PacketID.IMU_ATTITUDE, payload, self._next_seq()))

    def _gen_foc_feedback(self):
        payload = bytearray()
        for m in range(6):
            online = 1
            vel = int((1200 + random.gauss(0, 30)) * 10)  # scaled ×10
            torque = int(random.gauss(0, 100))
            fb_hz = 900 + int(random.gauss(0, 20))
            payload += struct.pack('<BhHH',
                online,
                vel,
                max(-1000, min(1000, torque)),
                max(0, min(2000, fb_hz)),
            )
        self._enqueue(_build_frame(PacketID.FOC_FEEDBACK, bytes(payload), self._next_seq()))

    def _gen_triwheel_feedback(self):
        payload = bytearray()
        chassis_pitch_deg = math.sin(self._t * 2.0) * 3.0
        for w in range(4):
            base_angle = 45.0 if w in (0, 2) else -45.0
            angle = base_angle + random.gauss(0, 0.5)
            filtered = angle + random.gauss(0, 0.1)
            payload += struct.pack('<ff', angle, filtered)
        payload += struct.pack('<f', chassis_pitch_deg)
        self._enqueue(_build_frame(PacketID.TRIWHEEL_FEEDBACK, bytes(payload), self._next_seq()))

    def _gen_drv8701_feedback(self):
        payload = bytearray()
        for m in range(6):
            enabled = 1 if m < 4 else 0
            mode = 0
            direction = -1 if m in (0, 2) else 1
            duty = int(random.uniform(0.3, 0.7) * 1000)
            current = int(random.gauss(500, 100))
            payload += struct.pack('<BBbhh',
                enabled, mode, direction,
                max(0, min(1000, duty)),
                max(0, min(5000, current)),
            )
        self._enqueue(_build_frame(PacketID.DRV8701_FEEDBACK, bytes(payload), self._next_seq()))

    def _gen_event(self):
        # 随机生成一个事件
        event_type = random.choice([1, 5])
        if event_type == 1:
            payload = struct.pack('<BBI', 1, self._mode, 0)
        else:
            payload = struct.pack('<BBI', 5, 0, 0)
        self._enqueue(_build_frame(PacketID.EVENT_NOTIFY, payload, self._next_seq()))
