"""上位机可视化服务入口。

启动方式:
    python -m viz_dashboard.viz_server                    # 模拟数据
    python -m viz_dashboard.viz_server --port /dev/ttyACM0  # 真实硬件

浏览器打开 http://localhost:8080 即可看到仪表盘。
"""

import argparse
import asyncio
import json
import os
import time

# ============================================================
# 全局状态（串口读线程写入，asyncio 协程读取）
# ============================================================

mcu_state = {
    "status": {
        "chassis_mode": 0,
        "heartbeat_echo": 0,
        "error_flags": 0,
        "tof_min_distance_mm": 0,
        "tof_max_distance_mm": 0,
        "tof_valid_count": 0,
        "imu_pitch_deg": 0.0,
        "imu_roll_deg": 0.0,
        "chassis_pitch_deg": 0.0,
    },
    "tof": [
        {"distance_mm": 0, "status": 0, "signal": 0} for _ in range(4)
    ],
    "imu": {
        "quat_w": 1.0, "quat_x": 0.0, "quat_y": 0.0, "quat_z": 0.0,
        "gyro_x_dps": 0.0, "gyro_y_dps": 0.0, "gyro_z_dps": 0.0,
        "accel_x_g": 0.0, "accel_y_g": 0.0, "accel_z_g": 0.0,
        "temperature_c": 0.0,
    },
    "foc_motors": [
        {"online": 0, "velocity_rpm": 0.0, "torque": 0.0, "feedback_hz": 0} for _ in range(6)
    ],
    "triwheels": [
        {"angle_deg": 0.0, "filtered_angle_deg": 0.0} for _ in range(4)
    ],
    "chassis_pitch_from_triwheel": 0.0,
    "drv_motors": [
        {"enabled": 0, "mode": 0, "direction": 0, "duty": 0.0, "current_ma": 0} for _ in range(6)
    ],
    "mcu_connected": False,
    "crc_errors": 0,
    "total_frames": 0,
}

event_queue = []   # (timestamp, event_dict)

import threading
_state_lock = threading.Lock()


def _update_state(frame):
    """串口读线程回调：将解析后的帧写入全局状态。"""
    pid = frame.packet_id
    payload = frame.payload
    with _state_lock:
        if pid == 0x01 and "chassis_mode" in payload:
            mcu_state["status"].update(payload)
        elif pid == 0x02 and "tof" in payload:
            for i, s in enumerate(payload["tof"]):
                if i < 4:
                    mcu_state["tof"][i].update(s)
        elif pid == 0x03:
            mcu_state["imu"].update(payload)
        elif pid == 0x04 and "foc_motors" in payload:
            for i, m in enumerate(payload["foc_motors"]):
                if i < 6:
                    mcu_state["foc_motors"][i].update(m)
        elif pid == 0x05:
            if "triwheels" in payload:
                for i, w in enumerate(payload["triwheels"]):
                    if i < 4:
                        mcu_state["triwheels"][i].update(w)
            if "chassis_pitch_deg" in payload:
                mcu_state["chassis_pitch_from_triwheel"] = payload["chassis_pitch_deg"]
        elif pid == 0x06 and "drv_motors" in payload:
            for i, m in enumerate(payload["drv_motors"]):
                if i < 6:
                    mcu_state["drv_motors"][i].update(m)
        elif pid == 0x09:
            event_queue.append((time.time(), payload))


def _update_connection(connected: bool):
    with _state_lock:
        mcu_state["mcu_connected"] = connected


# ============================================================
# 模拟数据源（无硬件时）
# ============================================================

def _start_mock():
    """在后台线程运行模拟串口数据源，直接更新 mcu_state。"""
    import threading
    import time
    import math
    import random

    def _run():
        tick = 0
        while True:
            with _state_lock:
                s = mcu_state["status"]
                t = tick * 0.001
                s["chassis_mode"] = 2
                s["error_flags"] = 0
                s["tof_valid_count"] = 4
                s["tof_min_distance_mm"] = 220 + int(random.gauss(0, 10))
                s["tof_max_distance_mm"] = 920 + int(random.gauss(0, 15))
                s["imu_pitch_deg"] = round(math.sin(t * 2.0) * 3.0, 1)
                s["imu_roll_deg"] = round(math.cos(t * 1.7) * 1.5, 1)
                s["chassis_pitch_deg"] = round(s["imu_pitch_deg"] * 0.9, 1)

                for i in range(4):
                    tof = mcu_state["tof"][i]
                    base = [850, 820, 910, 230][i]
                    tof["distance_mm"] = max(0, int(base + random.gauss(0, 15)))
                    tof["status"] = 1 if tof["distance_mm"] > 50 else 0
                    tof["signal"] = max(0, min(255, int([200, 180, 210, 90][i] + random.gauss(0, 5))))

                imu = mcu_state["imu"]
                pitch_r = math.radians(s["imu_pitch_deg"])
                roll_r = math.radians(s["imu_roll_deg"])
                imu["quat_w"] = math.cos(pitch_r/2) * math.cos(roll_r/2)
                imu["quat_x"] = math.sin(roll_r/2) * math.cos(pitch_r/2)
                imu["quat_y"] = math.sin(pitch_r/2) * math.cos(roll_r/2)
                imu["quat_z"] = math.sin(pitch_r/2) * math.sin(roll_r/2)
                imu["gyro_x_dps"] = random.gauss(0, 0.05)
                imu["gyro_y_dps"] = random.gauss(0, 0.05)
                imu["gyro_z_dps"] = random.gauss(0, 0.05)
                imu["accel_z_g"] = 0.98 + random.gauss(0, 0.02)
                imu["temperature_c"] = 35.0 + random.gauss(0, 0.3)

                for m in mcu_state["foc_motors"]:
                    m["online"] = 1
                    m["velocity_rpm"] = round(1200 + random.gauss(0, 30), 1)
                    m["torque"] = round(random.gauss(0, 0.1), 3)
                    m["feedback_hz"] = 900 + int(random.gauss(0, 20))

                for i, w in enumerate(mcu_state["triwheels"]):
                    base = 45.0 if i in (0, 2) else -45.0
                    w["angle_deg"] = round(base + random.gauss(0, 0.5), 1)
                    w["filtered_angle_deg"] = round(w["angle_deg"] + random.gauss(0, 0.1), 1)

                mcu_state["chassis_pitch_from_triwheel"] = round(s["imu_pitch_deg"] * 0.85, 1)

                for i, d in enumerate(mcu_state["drv_motors"]):
                    d["enabled"] = 1 if i < 4 else 0
                    d["duty"] = round(random.uniform(0.3, 0.7), 3)
                    d["current_ma"] = int(random.gauss(500, 100))

                mcu_state["mcu_connected"] = True
                mcu_state["total_frames"] += 1

            tick += 1
            if tick >= 100:
                tick = 0
            time.sleep(0.01)   # ~100Hz

    t = threading.Thread(target=_run, daemon=True)
    t.start()


# ============================================================
# FastAPI 应用
# ============================================================

from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")


@asynccontextmanager
async def lifespan(app: FastAPI):
    asyncio.create_task(_push_loop())
    yield


app = FastAPI(title="楼梯检测机器人 · 上位机监控", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
async def index():
    html_path = os.path.join(STATIC_DIR, "index.html")
    with open(html_path, "r", encoding="utf-8") as f:
        return HTMLResponse(f.read())


# WebSocket 广播
_ws_clients: list = []


async def _push_loop():
    """每 20ms 将 mcu_state 快照推送给所有 WebSocket 客户端。"""
    seq = 0
    while True:
        await asyncio.sleep(0.02)
        if not _ws_clients:
            continue

        with _state_lock:
            snapshot = json.dumps({
                "type": "state_snapshot",
                "seq": seq,
                "timestamp": time.time(),
                "data": {
                    "status": dict(mcu_state["status"]),
                    "tof": [dict(t) for t in mcu_state["tof"]],
                    "imu": dict(mcu_state["imu"]),
                    "foc_motors": [dict(m) for m in mcu_state["foc_motors"]],
                    "triwheels": [dict(w) for w in mcu_state["triwheels"]],
                    "chassis_pitch_from_triwheel": mcu_state["chassis_pitch_from_triwheel"],
                    "drv_motors": [dict(d) for d in mcu_state["drv_motors"]],
                    "mcu_connected": mcu_state["mcu_connected"],
                    "crc_errors": mcu_state["crc_errors"],
                    "total_frames": mcu_state["total_frames"],
                },
            }, ensure_ascii=False)

        # 处理事件队列
        events_snapshot = []
        while event_queue:
            ts, evt = event_queue.pop(0)
            events_snapshot.append({"timestamp": ts, **evt})

        stale = []
        for ws in _ws_clients:
            try:
                await ws.send_text(snapshot)
                for evt in events_snapshot:
                    await ws.send_text(json.dumps({
                        "type": "event",
                        "seq": seq,
                        "timestamp": evt.pop("timestamp"),
                        "data": evt,
                    }, ensure_ascii=False))
            except Exception:
                stale.append(ws)
        for ws in stale:
            _ws_clients.remove(ws)

        seq += 1


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    _ws_clients.append(ws)
    try:
        while True:
            await ws.receive_text()  # 保持连接，处理前端发来的消息
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        if ws in _ws_clients:
            _ws_clients.remove(ws)


# ============================================================
# 入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="上位机可视化服务")
    parser.add_argument("--port", type=str, default=None, help="串口设备路径（如 /dev/ttyACM0），不指定则使用模拟数据")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--host", type=str, default="0.0.0.0")
    parser.add_argument("--web-port", type=int, default=8080)
    args = parser.parse_args()

    if args.port:
        # 真实串口模式
        from viz_dashboard.serial_driver import SerialDriver
        driver = SerialDriver(port=args.port, baud_rate=args.baud)
        driver.on_frame = _update_state
        driver.on_connected_changed = _update_connection
        driver.start()
        print(f"[串口模式] 已连接 {args.port} @ {args.baud}")
    else:
        # 模拟数据模式
        _start_mock()
        print("[模拟模式] 使用内置模拟数据源，打开 http://localhost:8080")

    import uvicorn
    uvicorn.run(app, host=args.host, port=args.web_port, log_level="info")


if __name__ == "__main__":
    main()
