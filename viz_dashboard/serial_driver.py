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
        self._timeout_count = 0
        self._write_errors = 0
        self._last_success = None
        self._last_timeout = None
        self._last_write_error = None
        self._last_request = None

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

    def diagnostics(self) -> dict:
        with self._lock:
            return {
                "port": self._port,
                "baud_rate": self._baud_rate,
                "is_mock": getattr(self, "_is_mock", False),
                "running": self._running,
                "serial_open": bool(self._serial and self._serial.is_open),
                "connected": self._connected,
                "total_responses": self._total_responses,
                "crc_errors": self._crc_errors,
                "timeout_count": self._timeout_count,
                "write_errors": self._write_errors,
                "last_response_age_s": (
                    round(time.time() - self._last_response_time, 3)
                    if self._last_response_time
                    else None
                ),
                "last_success": dict(self._last_success) if self._last_success else None,
                "last_timeout": dict(self._last_timeout) if self._last_timeout else None,
                "last_write_error": dict(self._last_write_error) if self._last_write_error else None,
                "last_request": dict(self._last_request) if self._last_request else None,
            }

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
        with self._lock:
            self._last_request = {
                "timestamp": time.time(),
                "reg_addr": int(reg_addr),
                "length": length,
                "hex": frame.hex(" "),
            }
        try:
            self._serial.write(frame)
        except Exception:
            with self._lock:
                self._write_errors += 1
                self._last_write_error = {
                    "timestamp": time.time(),
                    "reg_addr": reg_addr,
                    "length": length,
                }
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
            return None, bytes(data_buf)

        payload = bytes(data_buf[:expected_data_len])
        crc_received = data_buf[expected_data_len]
        crc_expected = crc8(payload)

        if crc_received != crc_expected:
            with self._lock:
                self._crc_errors += 1
            return None, bytes(data_buf)

        return payload, bytes(data_buf)

    # --------------------------------------------------------
    # 后台轮询循环
    # --------------------------------------------------------

    # 轮询调度表: (周期 ticks, 寄存器地址, 读取长度, 名称)
    # tick = 5ms, 2 ticks = 100Hz, 4 = 50Hz, 10 = 20Hz
    _POLL_SCHEDULE = [
        (2, RegAddr.CTRL, 1, "ctrl"),                              # 0x00
        (2, RegAddr.ONLINE_STATUS, 1, "online"),                   # 0x82
        (4, RegAddr.TOF1, 1, "tof1"),
        (4, RegAddr.TOF2, 1, "tof2"),
        (4, RegAddr.TOF3, 1, "tof3"),
        (4, RegAddr.TOF4, 1, "tof4"),
        (4, RegAddr.IMU_PITCH, 1, "imu_pitch"),
        (4, RegAddr.IMU_ROLL, 1, "imu_roll"),
        (10, RegAddr.MOTOR_L3_TORQUE_SPEED, 1, "motor_l3"),
        (10, RegAddr.MOTOR_L4_TORQUE_SPEED, 1, "motor_l4"),
        (10, RegAddr.MOTOR_L5_TORQUE_SPEED, 1, "motor_l5"),
        (10, RegAddr.MOTOR_R0_TORQUE_SPEED, 1, "motor_r0"),
        (10, RegAddr.MOTOR_R1_TORQUE_SPEED, 1, "motor_r1"),
        (10, RegAddr.MOTOR_R2_TORQUE_SPEED, 1, "motor_r2"),
        (10, RegAddr.TRIWHEEL_ANGLE_CUR_FRONT, 1, "triwheel_front_angle"),
        (10, RegAddr.TRIWHEEL_ANGLE_CUR_REAR, 1, "triwheel_rear_angle"),
        (10, RegAddr.TRIWHEEL_DUTY_CUR_FRONT, 1, "triwheel_front_duty"),
        (10, RegAddr.TRIWHEEL_DUTY_CUR_REAR, 1, "triwheel_rear_duty"),
        (10, RegAddr.IMU_QUAT_W, 1, "imu_quat_w"),
        (10, RegAddr.IMU_QUAT_X, 1, "imu_quat_x"),
        (10, RegAddr.IMU_QUAT_Y, 1, "imu_quat_y"),
        (10, RegAddr.IMU_QUAT_Z, 1, "imu_quat_z"),
        (10, RegAddr.IMU_GYRO_YAW, 1, "imu_gyro_yaw"),
        (10, RegAddr.IMU_GYRO_PITCH, 1, "imu_gyro_pitch"),
        (10, RegAddr.IMU_GYRO_ROLL, 1, "imu_gyro_roll"),
        (10, RegAddr.IMU_ACCEL_X, 1, "imu_accel_x"),
        (10, RegAddr.IMU_ACCEL_Y, 1, "imu_accel_y"),
        (10, RegAddr.IMU_ACCEL_Z, 1, "imu_accel_z"),
        (10, RegAddr.IMU_TEMP, 1, "imu_temp"),
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

                data, raw_response = self._read_response(expected)
                if data is not None:
                    self._last_response_time = time.time()
                    with self._lock:
                        self._total_responses += 1
                        self._last_success = {
                            "timestamp": self._last_response_time,
                            "reg_addr": int(reg_addr),
                            "length": length,
                            "name": name,
                            "bytes": len(data),
                        }

                    if not self.connected:
                        self.connected = True

                    results = parse_response(reg_addr, data)
                    if self.on_registers:
                        self.on_registers(reg_addr, results)
                else:
                    with self._lock:
                        self._timeout_count += 1
                        self._last_timeout = {
                            "timestamp": time.time(),
                            "reg_addr": int(reg_addr),
                            "length": length,
                            "name": name,
                            "expected_bytes": expected,
                            "raw_bytes": len(raw_response),
                            "raw_hex": raw_response.hex(" "),
                        }
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

    
