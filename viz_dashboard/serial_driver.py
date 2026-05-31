"""串口 I/O 层：主动轮询寄存器 + 响应解析 + 连接监控。

若 serial_port=None,则使用内置模拟数据源(无需硬件)。
"""

import threading
import time
from typing import Optional, Callable, List

from viz_dashboard.protocol import (
    crc8,
    build_read_request, parse_response,
    RegAddr,
)


class SerialDriver:
    """串口驱动，后台线程轮询寄存器，解析响应并更新共享状态。"""

    def __init__(self, port: Optional[str] = None, baud_rate: int = 921600):
        self._port = port
        self._baud_rate = baud_rate
        self._serial = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._last_response_time = 0.0
        self._connected = False

        self._crc_errors = 0
        self._total_responses = 0

        # 回调
        self.on_registers = None   # callable(reg_addr, results_list)
        self.on_connected_changed = None  # callable(bool)

    # --------------------------------------------------------
    # 属性
    # --------------------------------------------------------

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @connected.setter
    def connected(self, val: bool):
        changed = False
        with self._lock:
            if self._connected != val:
                self._connected = val
                changed = True
        if changed and self.on_connected_changed:
            self.on_connected_changed(val)

    @property
    def crc_errors(self) -> int:
        with self._lock:
            return self._crc_errors

    @property
    def total_responses(self) -> int:
        with self._lock:
            return self._total_responses

    # --------------------------------------------------------
    # 启停
    # --------------------------------------------------------

    def start(self):
        if self._running:
            return
        self._running = True

        if self._port:
            self._start_serial()
        else:
            self._start_mock()

        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def send(self, frame: bytes):
        """发送命令帧到串口（线程安全）。"""
        if self._serial and self._serial.is_open:
            with self._lock:
                try:
                    self._serial.write(frame)
                except Exception:
                    pass

    # --------------------------------------------------------
    # 串口 / 模拟
    # --------------------------------------------------------

    def _start_serial(self):
        import serial
        self._serial = serial.Serial(
            port=self._port,
            baudrate=self._baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01,
        )
        self._is_mock = False

    def _start_mock(self):
        from viz_dashboard.mock_serial import MockSerial
        self._serial = MockSerial()
        self._is_mock = True

    # --------------------------------------------------------
    # 发送读请求 + 收响应
    # --------------------------------------------------------

    def _send_read_request(self, reg_addr: int, length: int) -> int:
        """发送读请求，返回期望的响应数据字节数。"""
        frame = build_read_request(reg_addr, length)
        try:
            self._serial.write(frame)
        except Exception:
            return -1
        return length * 4

    def _read_response(self, expected_data_len: int, timeout: float = 0.05) -> Optional[bytes]:
        """从串口读取响应帧，返回 data bytes 或 None。

        新协议响应: DATA(4BxN) | CRC8(1B)，无帧头。
        """
        to_read = expected_data_len + 1  # data + CRC8
        data_buf = bytearray()
        deadline = time.time() + timeout + 0.05

        while len(data_buf) < to_read and time.time() < deadline:
            chunk = self._serial.read(to_read - len(data_buf))
            if chunk:
                data_buf.extend(chunk)

        if len(data_buf) < to_read:
            return None

        payload = bytes(data_buf[:expected_data_len])
        crc_received = data_buf[expected_data_len]
        crc_expected = crc8(payload)

        if crc_received != crc_expected:
            with self._lock:
                self._crc_errors += 1
            return None

        return payload

    # --------------------------------------------------------
    # 后台轮询循环
    # --------------------------------------------------------

    # 轮询调度表: (周期 ticks, 寄存器地址, 读取长度, 名称)
    # tick = 5ms, 2 ticks = 100Hz, 4 = 50Hz, 10 = 20Hz
    _POLL_SCHEDULE = [
        (2, RegAddr.CTRL, 1, "ctrl"),                              # 0x00
        (2, RegAddr.ONLINE_STATUS, 1, "online"),                   # 0x52
        (4, RegAddr.TOF1, 4, "tof"),                               # 0x11-0x14
        (4, RegAddr.IMU_PITCH, 2, "imu_euler"),                    # 0x1A-0x1B
        (10, RegAddr.MOTOR_L3_TORQUE_SPEED, 12, "motors"),         # 0x23-0x2E
        (10, RegAddr.TRIWHEEL_ANGLE_CUR_FRONT, 4, "triwheels"),    # 0x2F-0x32
        (10, RegAddr.IMU_QUAT_W, 14, "imu_full"),                  # 0x15-0x22
    ]

    def _poll_loop(self):
        tick = 0
        while self._running:
            loop_start = time.time()

            for period, reg_addr, length, name in self._POLL_SCHEDULE:
                if tick % period != 0:
                    continue

                if not self._serial or not self._serial.is_open:
                    break

                expected = self._send_read_request(reg_addr, length)
                if expected < 0:
                    break

                data = self._read_response(expected)
                if data is not None:
                    self._last_response_time = time.time()
                    with self._lock:
                        self._total_responses += 1

                    if not self.connected:
                        self.connected = True

                    results = parse_response(reg_addr, data)
                    if self.on_registers:
                        self.on_registers(reg_addr, results)
                else:
                    if self.connected and (time.time() - self._last_response_time) > 0.3:
                        self.connected = False

                time.sleep(0.001)

            elapsed = time.time() - loop_start
            sleep_time = 0.005 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            tick += 1
            if tick >= 100:
                tick = 0

    