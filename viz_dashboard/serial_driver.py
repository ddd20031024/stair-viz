"""串口 I/O 层：后台读线程 + 帧状态机 + 连接监控。

若 serial_port=None，则使用内置模拟数据源（无需硬件）。
"""

import threading
import time
from typing import Optional


class SerialDriver:
    """串口驱动，后台线程读取并解析帧，更新共享状态。"""

    def __init__(self, port: Optional[str] = None, baud_rate: int = 921600):
        self._port = port
        self._baud_rate = baud_rate
        self._serial = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._last_frame_time = 0.0
        self._connected = False

        # 累积统计
        self._crc_errors = 0
        self._total_frames = 0

        # 最新帧回调（在 read 线程中调用）
        self.on_frame = None   # callable(parsed_frame)
        self.on_connected_changed = None  # callable(bool)

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
    def total_frames(self) -> int:
        with self._lock:
            return self._total_frames

    def start(self):
        if self._running:
            return
        self._running = True

        if self._port:
            self._start_serial()
        else:
            self._start_mock()

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
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

    def _start_serial(self):
        import serial
        self._serial = serial.Serial(
            port=self._port,
            baudrate=self._baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
        )

    def _start_mock(self):
        from viz_dashboard.mock_serial import MockSerial
        self._serial = MockSerial()
        self._is_mock = True

    def _read_loop(self):
        from viz_dashboard.protocol import FrameDecoder
        decoder = FrameDecoder()

        while self._running:
            try:
                if self._serial and self._serial.is_open:
                    byte = self._serial.read(1)
                    if byte:
                        frame = decoder.feed(byte[0])
                        if frame:
                            self._last_frame_time = time.time()
                            self._total_frames = decoder.total_frames
                            self._crc_errors = decoder.crc_errors
                            if not self.connected:
                                self.connected = True
                            if self.on_frame:
                                self.on_frame(frame)
                    else:
                        # 超时，检查连接
                        if self.connected and (time.time() - self._last_frame_time) > 0.3:
                            self.connected = False
                else:
                    time.sleep(0.1)
            except Exception:
                if self.connected:
                    self.connected = False
                time.sleep(0.5)
