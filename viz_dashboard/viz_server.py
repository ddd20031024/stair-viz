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
import threading
import math
from typing import List

from viz_dashboard.protocol import RegAddr

# ============================================================
# 全局状态（串口读线程写入，asyncio 协程读取）
# ============================================================

mcu_state = {
    "status": {
        "chassis_mode": 0,
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
    "total_responses": 0,
}

event_queue = []   # (timestamp, event_dict)

_state_lock = threading.Lock()


# ============================================================
# 寄存器数据 → mcu_state 更新
# ============================================================

# 电机映射: Excel 中 L3/L4/L5/R0/R1/R2 → 前端 motors[0..5]
_MOTOR_ORDER = ["L3", "L4", "L5", "R0", "R1", "R2"]

# 三角轮映射: Excel 中 lf/rf/lr/rr → 前端 triwheels[0..3]
_TRIWHEEL_ORDER = ["lf", "rf", "lr", "rr"]

# 电机扭矩/速度寄存器地址 (torque+speed pairs) — 新协议地址
_MOTOR_TORQUE_SPEED_ADDRS = [
    RegAddr.MOTOR_L3_TORQUE_SPEED,  # 35
    RegAddr.MOTOR_L4_TORQUE_SPEED,  # 37
    RegAddr.MOTOR_L5_TORQUE_SPEED,  # 39
    RegAddr.MOTOR_R0_TORQUE_SPEED,  # 41
    RegAddr.MOTOR_R1_TORQUE_SPEED,  # 43
    RegAddr.MOTOR_R2_TORQUE_SPEED,  # 45
]
_MOTOR_ANGLE_ADDRS = [
    RegAddr.MOTOR_L3_ANGLE,  # 36
    RegAddr.MOTOR_L4_ANGLE,  # 38
    RegAddr.MOTOR_L5_ANGLE,  # 40
    RegAddr.MOTOR_R0_ANGLE,  # 42
    RegAddr.MOTOR_R1_ANGLE,  # 44
    RegAddr.MOTOR_R2_ANGLE,  # 46
]

# 三角轮角度/占空比寄存器
_TRIWHEEL_ANGLE_FRONT_ADDR = RegAddr.TRIWHEEL_ANGLE_CUR_FRONT   # 47
_TRIWHEEL_ANGLE_REAR_ADDR = RegAddr.TRIWHEEL_ANGLE_CUR_REAR     # 48
_TRIWHEEL_DUTY_FRONT_ADDR = RegAddr.TRIWHEEL_DUTY_CUR_FRONT     # 49
_TRIWHEEL_DUTY_REAR_ADDR = RegAddr.TRIWHEEL_DUTY_CUR_REAR       # 50

_prev_chassis_mode = 0
_prev_tof_faults = [0, 0, 0, 0]


def _update_state(reg_addr: int, results: List[dict]):
    """串口读线程回调：将解析后的寄存器数据写入全局状态。"""
    global _prev_chassis_mode, _prev_tof_faults

    with _state_lock:
        mcu_state["total_responses"] += 1
        for reg in results:
            addr = reg.get("addr", reg_addr)

            # ---- CTRL 寄存器 ----
            if addr == RegAddr.CTRL:
                mode = reg.get("chassis_mode", 0)
                if mode != _prev_chassis_mode and _prev_chassis_mode is not None:
                    event_queue.append((time.time(), {
                        "event_type": 1,  # 模式切换
                        "event_code": mode,
                        "event_data": _prev_chassis_mode,
                    }))
                _prev_chassis_mode = mode
                mcu_state["status"]["chassis_mode"] = mode

                # 构建错误标志（从控制位推断）
                error_flags = 0
                if not reg.get("foc_enable", 0):
                    error_flags |= (1 << 2)  # FOC故障
                mcu_state["status"]["error_flags"] = error_flags

            # ---- TOF 寄存器 ----
            elif RegAddr.TOF1 <= addr <= RegAddr.TOF4:
                idx = addr - RegAddr.TOF1
                if idx < 4:
                    dist = reg.get("distance_mm", 0)
                    status = reg.get("status", 0)
                    signal = reg.get("signal", 0)
                    fault = reg.get("fault", 0)
                    mcu_state["tof"][idx].update({
                        "distance_mm": dist,
                        "status": status,
                        "signal": signal,
                    })
                    # TOF 故障事件
                    if fault != _prev_tof_faults[idx]:
                        if fault:
                            event_queue.append((time.time(), {
                                "event_type": 6,
                                "event_code": idx,
                                "event_data": fault,
                            }))
                    _prev_tof_faults[idx] = fault

                # 更新 TOF 聚合统计
                tofs = mcu_state["tof"]
                valid = [t for t in tofs if t["status"] == 1 and t["distance_mm"] > 0]
                mcu_state["status"]["tof_valid_count"] = len(valid)
                mcu_state["status"]["tof_min_distance_mm"] = min((t["distance_mm"] for t in valid), default=0)
                mcu_state["status"]["tof_max_distance_mm"] = max((t["distance_mm"] for t in valid), default=0)

            # ---- IMU euler ----
            elif addr == RegAddr.IMU_PITCH:
                mcu_state["status"]["imu_pitch_deg"] = reg.get("pitch_deg", 0.0)
            elif addr == RegAddr.IMU_ROLL:
                mcu_state["status"]["imu_roll_deg"] = reg.get("roll_deg", 0.0)

            # ---- IMU full data ----
            elif addr == RegAddr.IMU_QUAT_W:
                mcu_state["imu"]["quat_w"] = reg.get("quat_w", 0.0)
            elif addr == RegAddr.IMU_QUAT_X:
                mcu_state["imu"]["quat_x"] = reg.get("quat_x", 0.0)
            elif addr == RegAddr.IMU_QUAT_Y:
                mcu_state["imu"]["quat_y"] = reg.get("quat_y", 0.0)
            elif addr == RegAddr.IMU_QUAT_Z:
                mcu_state["imu"]["quat_z"] = reg.get("quat_z", 0.0)
            elif addr == RegAddr.IMU_GYRO_YAW:
                mcu_state["imu"]["gyro_z_dps"] = reg.get("gyro_yaw_dps", 0.0)
            elif addr == RegAddr.IMU_GYRO_PITCH:
                mcu_state["imu"]["gyro_y_dps"] = reg.get("gyro_pitch_dps", 0.0)
            elif addr == RegAddr.IMU_GYRO_ROLL:
                mcu_state["imu"]["gyro_x_dps"] = reg.get("gyro_roll_dps", 0.0)
            elif addr == RegAddr.IMU_ACCEL_X:
                mcu_state["imu"]["accel_x_g"] = reg.get("accel_x_ms2", 0.0)
            elif addr == RegAddr.IMU_ACCEL_Y:
                mcu_state["imu"]["accel_y_g"] = reg.get("accel_y_ms2", 0.0)
            elif addr == RegAddr.IMU_ACCEL_Z:
                mcu_state["imu"]["accel_z_g"] = reg.get("accel_z_ms2", 0.0)
            elif addr == RegAddr.IMU_TEMP:
                mcu_state["imu"]["temperature_c"] = reg.get("temperature_c", 0.0)

            # ---- 驱动电机 ----
            elif addr in _MOTOR_TORQUE_SPEED_ADDRS:
                idx = _MOTOR_TORQUE_SPEED_ADDRS.index(addr)
                if idx < 6:
                    speed_raw = reg.get("speed", 0)
                    torque_raw = reg.get("torque", 0)
                    # speed: rad/s × 16 (int16) → rpm
                    speed_rads = speed_raw / 16.0
                    speed_rpm = speed_rads * 60.0 / (2.0 * math.pi)
                    mcu_state["foc_motors"][idx].update({
                        "velocity_rpm": round(speed_rpm, 1),
                        "torque": round(torque_raw / 1000.0, 3),
                        "feedback_hz": 0,
                    })
            elif addr in _MOTOR_ANGLE_ADDRS:
                idx = _MOTOR_ANGLE_ADDRS.index(addr)
                # total angle is available but not displayed in current UI
                pass

            # ---- 三角轮 ----
            elif addr == _TRIWHEEL_ANGLE_FRONT_ADDR:
                lf_raw = reg.get("tri_lf_angle", 0)
                rf_raw = reg.get("tri_rf_angle", 0)
                # angle: int16 × 100 → degrees
                mcu_state["triwheels"][0].update({
                    "angle_deg": round(lf_raw / 100.0, 1),
                    "filtered_angle_deg": round(lf_raw / 100.0, 1),
                })
                mcu_state["triwheels"][1].update({
                    "angle_deg": round(rf_raw / 100.0, 1),
                    "filtered_angle_deg": round(rf_raw / 100.0, 1),
                })
            elif addr == _TRIWHEEL_ANGLE_REAR_ADDR:
                lr_raw = reg.get("tri_lr_angle", 0)
                rr_raw = reg.get("tri_rr_angle", 0)
                mcu_state["triwheels"][2].update({
                    "angle_deg": round(lr_raw / 100.0, 1),
                    "filtered_angle_deg": round(lr_raw / 100.0, 1),
                })
                mcu_state["triwheels"][3].update({
                    "angle_deg": round(rr_raw / 100.0, 1),
                    "filtered_angle_deg": round(rr_raw / 100.0, 1),
                })

            # ---- 在线状态 ----
            elif addr == RegAddr.ONLINE_STATUS:
                online_bits = [
                    reg.get("motor_id3_online", 0),  # L3
                    reg.get("motor_id4_online", 0),  # L4
                    reg.get("motor_id5_online", 0),  # L5
                    reg.get("motor_id0_online", 0),  # R0
                    reg.get("motor_id1_online", 0),  # R1
                    reg.get("motor_id2_online", 0),  # R2
                ]
                for i, bit in enumerate(online_bits):
                    mcu_state["foc_motors"][i]["online"] = 1 if bit else 0

            # ---- IMU euler 补充 (pitch for chassis) ----
            if addr == RegAddr.IMU_PITCH:
                pitch = reg.get("pitch_deg", 0.0)
                mcu_state["status"]["chassis_pitch_deg"] = pitch * 0.9
                mcu_state["chassis_pitch_from_triwheel"] = pitch * 0.85


def _update_connection(connected: bool):
    with _state_lock:
        mcu_state["mcu_connected"] = connected


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
                    "total_frames": mcu_state["total_responses"],
                },
            }, ensure_ascii=False)

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
            await ws.receive_text()
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

    from viz_dashboard.serial_driver import SerialDriver
    driver = SerialDriver(port=args.port, baud_rate=args.baud)
    driver.on_registers = _update_state
    driver.on_connected_changed = _update_connection
    driver.start()

    if args.port:
        print(f"[串口模式] 已连接 {args.port} @ {args.baud}")
    else:
        print("[模拟模式] 使用内置模拟数据源，打开 http://localhost:8080")

    import uvicorn
    uvicorn.run(app, host=args.host, port=args.web_port, log_level="info")


if __name__ == "__main__":
    main()
