# stair_viz — 楼梯检测机器人上位机可视化

基于 FastAPI + WebSocket 的轻量级上位机监控仪表盘，用于楼梯检测机器人的实时数据可视化。

## 1. 项目说明

本项目是 [ros2-stairs-detection](../stair_chapter2) 的上位机可视化配套项目。通过 USB 虚拟串口读取 STM32 下位机上传的传感器数据，解析通信协议帧后在 Web 浏览器中实时展示。

**核心特点**：
- 零构建前端：单 HTML 文件，浏览器打开即用
- 轻量依赖：仅需 `fastapi`、`uvicorn`、`pyserial`
- 暗色仪表盘主题，6 个数据面板
- 支持多浏览器同时监控

## 2. 通信协议

本项目的串口通信协议与 [通信协议设计.md](../stair_chapter2/project_data/通信协议设计.md) 完全一致。

| 项目 | 说明 |
|---|---|
| 物理层 | USB CDC ACM (虚拟串口)，识别为 `/dev/ttyACM0` |
| 波特率 | 921600 |
| 帧结构 | SOF(0x5A 0xA5) + Frame_Type + Packet_ID + Payload_Len + Seq_Num + Payload + CRC16 |
| 字节序 | Little-Endian |
| 上传带宽 | ≈ 9.8 KB/s（持续流式） |

## 3. 可视化面板

| 面板 | 数据来源 | 刷新率 | 内容 |
|---|---|---|---|
| 安全状态 | FAST_STATUS | 100Hz | 底盘模式、连接状态、错误标志位、TOF 距离摘要 |
| TOF 测距 | TOFSENSE_FULL | 50Hz | 4 路 TOF 距离柱状图、信号强度、有效状态 |
| IMU 姿态 | FAST_STATUS + IMU_ATTITUDE | 100/50Hz | Pitch/Roll 人工地平线、底盘俯仰、温度 |
| FOC 电机 | FOC_FEEDBACK | 20Hz | 6 路电机在线/速度/扭矩/反馈频率 |
| 三角轮 | TRIWHEEL_FEEDBACK | 20Hz | 4 轮原始角度 + 滤波角度、底盘俯仰估计 |
| 事件日志 | EVENT_NOTIFY | 事件驱动 | 模式切换、故障、急停等事件流 |

## 4. 目录结构

```
stair_viz/
  venv/                      ← Python 虚拟环境
  viz_dashboard/
    __init__.py
    protocol.py              ← 帧编解码 + CRC16（与 mcu_bridge 100% 复用）
    serial_driver.py         ← 串口 I/O + 帧状态机
    viz_server.py            ← FastAPI + WebSocket 服务入口
    static/
      index.html             ← 单页前端仪表盘
  params/
    viz_dashboard.params.yaml
  launch/
    viz_dashboard.launch.py  ← ROS2 launch 文件（可选）
  requirements.txt
  README.md
```

## 5. 环境要求

- Ubuntu 22.04 / 24.04
- Python 3.10+
- USB 串口设备（如 `/dev/ttyACM0`）
- 浏览器（Chrome / Firefox / Edge）

## 6. 快速开始

```bash
# 1. 激活虚拟环境
cd /home/xpy/stair_viz
source venv/bin/activate

# 2. 安装依赖（首次）
pip install -r requirements.txt

# 3. 直接启动（开发模式）
python viz_dashboard/viz_server.py --port /dev/ttyACM0

# 4. 打开浏览器
# http://localhost:8080
```

**ROS2 launch 方式**（可选）：

```bash
cd /home/xpy/stair_chapter2
source install/setup.bash
ros2 launch viz_dashboard viz_dashboard.launch.py
```

## 7. 配置参数

`params/viz_dashboard.params.yaml`：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | 串口设备路径 |
| `baud_rate` | `921600` | 虚拟串口波特率 |
| `web_host` | `0.0.0.0` | Web 服务监听地址 |
| `web_port` | `8080` | Web 服务端口 |
| `push_interval_ms` | `20` | WebSocket 推送间隔（毫秒） |
| `heartbeat_interval_ms` | `100` | 心跳包发送间隔（毫秒） |

## 8. 不仿真联调（无下位机时）

若暂无 STM32 硬件，可运行模拟数据源进行前端开发调试：

```bash
python viz_dashboard/mock_serial.py
```

模拟数据源会生成随机但合理的传感器数据，通过 WebSocket 推送到前端。

## 9. 后续扩展

- [ ] 历史数据曲线（TOF/电机最近 60s 趋势）
- [ ] 数据录制与回放（JSONL 格式）
- [ ] 上位机→下位机命令面板（急停、模式切换）
- [ ] 接入 mcu_bridge ROS2 话题（当前直读串口）
- [ ] 移动端响应式布局

## 10. 相关文档

- [通信协议设计](../stair_chapter2/project_data/通信协议设计.md)
- [必要数据挑选清单](../stair_chapter2/project_data/必要数据挑选清单.md)
- [上位机可视化规划设计](../stair_chapter2/project_data/上位机可视化规划设计.md)
