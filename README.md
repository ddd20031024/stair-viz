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

通信协议采用**寄存器地址 + 内容**形式，参考 TI FlexWire 协议改进（取消设备地址段，增加读取长度）。

| 项目 | 说明 |
|---|---|
| 物理层 | USB CDC ACM (虚拟串口)，识别为 `/dev/ttyACM0` |
| 波特率 | 921600 |
| 读请求 | SYNC(0x55) + SOF(0x5A) + REG_ADDR(1B) + LEN(1B) + CRC16(2B) |
| 读响应 | SOF(0xA5) + DATA(4B×LEN) + CRC16(2B) |
| 写请求 | SYNC(0x55) + SOF(0x5A) + REG_ADDR(1B) + LEN(1B) + DATA(4B×LEN) + CRC16(2B) |
| 字节序 | Little-Endian |
| CRC | CRC-16-CCITT（多项式 0x1021，同 MODBUS） |
| 寄存器位宽 | 32-bit / 寄存器 |
| 轮询模式 | 主机主动轮询，MCU 响应（非持续流式推送） |

完整寄存器列表见 `read_notes/上下位机通信.xlsx`。

## 3. 可视化面板

| 面板 | 数据来源（寄存器） | 刷新率 | 内容 |
|---|---|---|---|
| 安全状态 | CTRL(0x00), ONLINE(0x57), TOF(0x11-0x14) | 100Hz | 底盘模式、连接状态、错误标志位、TOF 距离摘要 |
| TOF 测距 | TOF1-4(0x11-0x14) | 50Hz | 4 路 TOF 距离柱状图、信号强度、有效状态 |
| IMU 姿态 | IMU_PITCH/ROLL(0x20-0x21) + IMU 全量(0x15-0x28) | 50/20Hz | Pitch/Roll 人工地平线、底盘俯仰、温度 |
| FOC 电机 | MOTOR_L3~R2(0x29-0x40) | 20Hz | 6 路电机在线/速度/扭矩 |
| 三角轮 | TRIWHEEL_CUR(0x41-0x44) | 20Hz | 4 轮角度 + 占空比、底盘俯仰估计 |
| 事件日志 | 寄存器变化检测 | 事件驱动 | 模式切换、故障、TOF 异常等事件流 |

## 4. 目录结构

```
stair_viz/
  venv/                      ← Python 虚拟环境
  viz_dashboard/
    __init__.py
    protocol.py              ← FlexWire 寄存器读写 + CRC16 + 寄存器字段定义
    serial_driver.py         ← 串口 I/O + 主动寄存器轮询
    mock_serial.py           ← MCU 寄存器仿真器（响应读请求）
    viz_server.py            ← FastAPI + WebSocket 服务入口
    static/
      index.html             ← 单页前端仪表盘
  read_notes/                ← 通信协议规范与沟通记录
    上下位机通信.xlsx
    通信.txt
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

# 3. 模拟模式启动（无需硬件）
python -m viz_dashboard.viz_server

# 4. 真实串口模式
python -m viz_dashboard.viz_server --port /dev/ttyACM0

# 5. 打开浏览器
# http://localhost:8080
```

## 7. 配置参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `serial_port` | 无（模拟模式） | 串口设备路径，如 `/dev/ttyACM0` |
| `baud_rate` | `921600` | 虚拟串口波特率 |
| `web_host` | `0.0.0.0` | Web 服务监听地址 |
| `web_port` | `8080` | Web 服务端口 |
| 寄存器轮询周期 | 5ms/tick | 100Hz(CTRL/Online), 50Hz(TOF/IMU Euler), 20Hz(Motors/Triwheels/IMU) |

## 8. 模拟模式（无下位机时）

若暂无 STM32 硬件，直接不带 `--port` 参数启动即可使用内置 MCU 寄存器仿真器：

```bash
python -m viz_dashboard.viz_server
```

仿真器会响应主机的寄存器读请求，返回带噪声的模拟传感器数据（TOF ~230-910mm、IMU ±3° 摆动、电机 ~1200rpm），通过 WebSocket 推送到前端。

## 9. 后续扩展

- [ ] 历史数据曲线（TOF/电机最近 60s 趋势）
- [ ] 数据录制与回放（JSONL 格式）
- [ ] 上位机→下位机命令面板（急停、模式切换）
- [ ] 接入 mcu_bridge ROS2 话题（当前直读串口）
- [ ] 移动端响应式布局

## 10. 相关文档

- [通信协议规范](read_notes/上下位机通信.xlsx)
- [上下位机通信说明](read_notes/通信.txt)
- [项目详解](PROJECT_DETAIL.md)
