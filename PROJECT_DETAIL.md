# stair_viz 项目详解

## 一、项目定位

`stair_viz` 是楼梯检测机器人（[ros2-stairs-detection](../stair_chapter2)）的上位机可视化配套项目。通过 USB 虚拟串口接收 STM32 下位机上传的全部传感器数据，筛选出必要数据进行实时可视化展示。

**核心目标**：
- 接收全部 10 种 MCU 上传数据包，完整解析并落地
- 可视化 6 类必要数据（安全状态、TOF 测距、IMU 姿态、FOC 电机、三角轮、事件日志）
- 上位机→下位机命令接口**已预留但暂不实际发送**（仅心跳维持连接）
- 前端单 HTML 文件零构建，后端单 Python 文件即可启动

---

## 二、接收的数据

### 2.1 数据来源

MCU 通过 USB CDC ACM（虚拟串口 `/dev/ttyACM0`，921600 波特率）上传二进制帧，帧格式为：

```
SOF(0x5A 0xA5) + Frame_Type(1B) + Packet_ID(1B) + Payload_Len(2B) + Seq_Num(1B) + Payload(N) + CRC16(2B)
```

### 2.2 全部接收的 10 种上传数据包

| Packet_ID | 名称 | 载荷 | 上传频率 | 优先级 | 可视化 |
|---|---|---|---|---|---|
| 0x01 | FAST_STATUS | 24B | 100Hz | 关键 | **是** — 安全状态面板 + 部分 IMU 面板 |
| 0x02 | TOFSENSE_FULL | 16B | 50Hz | 关键 | **是** — TOF 测距面板 |
| 0x03 | IMU_ATTITUDE | 44B | 50Hz | 高 | **是** — IMU 姿态面板 |
| 0x04 | FOC_FEEDBACK | 42B | 20Hz | 中 | **是** — FOC 电机面板 |
| 0x05 | TRIWHEEL_FEEDBACK | 36B | 20Hz | 中 | **是** — 三角轮面板 |
| 0x06 | DRV8701_FEEDBACK | 42B | 5Hz | 低 | **否** — 解析入库，不显示 |
| 0x07 | CHASSIS_FULL | 76B | 按需 | 低 | **否** — 调试时按需查询 |
| 0x08 | FOC_CONFIG | 138B | 按需 | 低 | **否** — 调试时按需查询 |
| 0x09 | EVENT_NOTIFY | 6B | 事件驱动 | 关键 | **是** — 事件日志面板 |
| 0x0A | ACK | 3B | 响应 | — | **否** — 协议层自动处理 |

### 2.3 每种数据包的字段详情

#### FAST_STATUS (0x01) — 100Hz

安全快照，包含跨域关键字段：

| 字段 | 类型 | 单位 | 说明 | 可视化 |
|---|---|---|---|---|
| chassis_mode | uint8 | 枚举 | 0=安全 1=开环 2=闭环 3=从机 | 安全面板 · 模式标签 |
| heartbeat_echo | uint8 | 计数 | 心跳回显 | 内部使用 |
| error_flags | uint16 | 位域 | 9 种错误标志 | 安全面板 · 错误图标阵列 |
| tof_min_distance_mm | uint16 | mm | 4路TOF 最小距离 | 安全面板 · 最小距离数字 |
| tof_max_distance_mm | uint16 | mm | 4路TOF 最大距离 | IMU 面板 · 最大距离数字 |
| tof_valid_count | uint8 | 个 | 有效 TOF 数量 (0~4) | 安全面板 · 有效数指示 |
| imu_pitch_deg | int8/10 | ° | IMU 俯仰角 | IMU 面板 · 人工地平线 + 数字 |
| imu_roll_deg | int8/10 | ° | IMU 横滚角 | IMU 面板 · 人工地平线 + 数字 |
| chassis_pitch_deg | int8/10 | ° | 底盘俯仰角 | IMU 面板 · 底盘俯仰数字 |

#### TOFSENSE_FULL (0x02) — 50Hz

4 路 TOF 传感器的完整读数：

| 字段 | 类型 | 说明 |
|---|---|---|
| distance_mm[4] | uint16 | 每路测距距离 (mm)，0=无效/超量程 |
| distance_status[4] | uint8 | 0=无效 1=有效 2=弱信号 3=超量程 0xFF=未知 |
| signal_strength[4] | uint8 | 信号强度 0~255 |

#### IMU_ATTITUDE (0x03) — 50Hz

| 字段 | 类型 | 说明 |
|---|---|---|
| quat_w/x/y/z | float32 | 四元数姿态表示 |
| gyro_x/y/z_dps | float32 | 角速度 (deg/s) |
| accel_x/y/z_g | float32 | 加速度 (g) |
| temperature_c | float32 | 温度 (℃) |

#### FOC_FEEDBACK (0x04) — 20Hz

| 字段 | 类型 | 说明 |
|---|---|---|
| online[6] | uint8 | 0=离线 1=在线 |
| velocity_rpm[6] | int16/10 | 速度 (RPM) |
| torque[6] | int16/1000 | 归一化扭矩 (-1.0 ~ 1.0) |
| feedback_hz[6] | uint16 | CAN 反馈频率 (Hz) |

#### TRIWHEEL_FEEDBACK (0x05) — 20Hz

| 字段 | 类型 | 说明 |
|---|---|---|
| angle_to_horizontal_deg[4] | float32 | 4轮相对水平面角度 (°) |
| filtered_angle_deg[4] | float32 | 低通滤波后角度 (°) |
| chassis_pitch_deg | float32 | 底盘整体俯仰角 (°) |

#### EVENT_NOTIFY (0x09) — 事件驱动

| 字段 | 类型 | 说明 |
|---|---|---|
| event_type | uint8 | 事件类别：模式切换/故障/故障恢复/DBUS/急停/TOF异常 |
| event_code | uint8 | 具体事件码 |
| event_data | uint32 | 事件附加数据 |

### 2.4 接收但不显示的数据

| 数据 | 原因 |
|---|---|
| DRV8701 6路完整状态 | 5Hz 低频，非楼梯安全关键 |
| IMU 四元数/角速度/加速度完整值 | 前端只展示 pitch/roll 聚合结果 |
| CHASSIS_FULL / FOC_CONFIG | 按需查询的调试数据 |
| heartbeat_echo / seq_num | 内部协议管理字段 |

---

## 三、项目文件结构

```
stair_viz/
  venv/                       Python 3.10 虚拟环境
  viz_dashboard/
    __init__.py               空文件，标识为 Python 包
    protocol.py               通信协议帧编解码层（⭐ 核心）
    serial_driver.py          串口 I/O 驱动层
    viz_server.py             FastAPI + WebSocket 服务入口（⭐ 主程序）
    mock_serial.py            模拟串口数据源（无硬件时使用）
    static/
      index.html              前端仪表盘单页面
  params/                     预留：ROS2 参数文件目录
  launch/                     预留：ROS2 launch 文件目录
  requirements.txt            pip 依赖清单
  README.md                   快速开始文档
  PROJECT_DETAIL.md           本文件
```

---

## 四、每个脚本的作用

### 4.1 `protocol.py` —— 通信协议层

**职责**：帧编解码、CRC16 校验、数据包定义。与未来的 `mcu_bridge` ROS2 节点 **100% 复用**。

| 组件 | 作用 |
|---|---|
| `crc16_ccitt(data)` | CRC-16-CCITT 校验计算（多项式 0x1021，同 MODBUS） |
| `PacketID` / `CmdID` / `ChassisMode` / `EventType` | 枚举常量定义 |
| `ERROR_FLAG_NAMES` / `MODE_NAMES` 等 | 中文名称映射字典 |
| `ParsedFrame` | dataclass：解析后的帧结构 `{packet_id, seq_num, payload, is_command}` |
| `FrameDecoder` | **SOF 状态机帧解码器**：喂入字节流，完整帧到达时返回 ParsedFrame |
| `FrameEncoder` | 命令帧构建器：`build_heartbeat()` / `build_emergency_stop()` / `build_mode_control()` |
| `_parse_fast_status()` ~ `_parse_ack()` | 8 个 payload 解析函数，将二进制转为 dict |

**数据流**：

```
串口字节流 → FrameDecoder.feed(byte) → 状态机识别 SOF→读取帧头→读取payload→CRC校验
  → ParsedFrame(packet_id=0x01, payload={"chassis_mode": 2, "tof_min_distance_mm": 850, ...})
```

**当前状态**：✅ 完成，8 种上传包解析已全部实现。

### 4.2 `serial_driver.py` —— 串口驱动层

**职责**：封装 pyserial，后台线程读取 + 连接监控。

| 组件 | 作用 |
|---|---|
| `SerialDriver(port, baud_rate)` | 串口驱动主类 |
| `driver.start()` | 启动后台读线程（如果是真实串口）或模拟数据线程 |
| `driver.stop()` | 停止线程，关闭串口 |
| `driver.send(frame)` | 发送命令帧（线程安全） |
| `driver.on_frame` | 回调：收到解析帧时调用，传入 `ParsedFrame` |
| `driver.on_connected_changed` | 回调：连接状态变化时调用，传入 `bool` |
| `driver.connected` | 属性：当前连接状态（300ms 无帧 → 断连） |

**工作流**：

```
后台线程循环:
  byte = serial.read(1)
  if byte:
      frame = decoder.feed(byte[0])
      if frame:
          on_frame(frame)          ← 更新全局 mcu_state
          connected = True
  else:
      if idle > 300ms:
          connected = False       ← 断连通知
```

**当前状态**：✅ 完成。

### 4.3 `mock_serial.py` —— 模拟数据源

**职责**：当无 STM32 硬件时，生成符合协议格式的模拟传感器数据。

| 组件 | 作用 |
|---|---|
| `MockSerial` 类 | 伪装成 `pyserial.Serial`，提供 `read(1)` 和 `is_open` |
| `_generator()` | 后台线程按 1kHz 调度表生成帧字节，模拟 100Hz/50Hz/20Hz/5Hz 多频数据 |
| `_gen_fast_status()` ~ `_gen_event()` | 各类数据包的模拟生成函数，添加高斯噪声模拟真实波动 |

**模拟的数据特征**：
- TOF 距离以 [850, 820, 910, 230] mm 为基线，叠加 15mm 标准差噪声
- IMU pitch 以 sin 波 ±3° 摆动，roll 以 cos 波 ±1.5° 摆动
- 电机速度以 1200rpm 为基线，30rpm 标准差
- 三角轮角度 ±45° 基线，0.5° 标准差
- 5 秒一次随机事件（模式切换 或 急停）

**当前状态**：✅ 完成。

### 4.4 `viz_server.py` —— FastAPI 服务入口

**职责**：Web 服务 + WebSocket 实时推送 + 全局状态管理。**主程序入口**。

| 组件 | 作用 |
|---|---|
| `mcu_state` (全局 dict) | 全部 MCU 数据的内存存储，线程安全（`threading.Lock` 保护） |
| `_update_state(frame)` | 串口读线程回调：将 `ParsedFrame` 更新到 `mcu_state` |
| `_update_connection(connected)` | 连接状态变化回调 |
| `_start_mock()` | 内置简化模拟数据源（比 mock_serial.py 更轻量，直接写 mcu_state） |
| `_push_loop()` | asyncio 协程：每 20ms 快照 mcu_state → JSON → WebSocket 广播 |
| `FastAPI app` | 路由：`GET /` 返回仪表盘 HTML，`WS /ws` 推送实时数据 |
| `main()` | CLI 入口：解析 `--port` / `--baud` / `--web-port` 参数 |

**启动方式**：

```bash
# 模拟模式（无需硬件）
python -m viz_dashboard.viz_server

# 真实串口模式
python -m viz_dashboard.viz_server --port /dev/ttyACM0

# 指定 Web 端口
python -m viz_dashboard.viz_server --web-port 9090
```

**当前状态**：✅ 完成。

### 4.5 `static/index.html` —— 前端仪表盘

**职责**：单文件 Web 前端，零构建工具，浏览器直接打开。

**6 个数据面板**：

| 面板 | 位置 | 数据源 | 更新频率 | 实现方式 |
|---|---|---|---|---|
| 安全状态 | 左上 | FAST_STATUS | 100Hz (每 20ms 快照) | DOM 更新 + CSS 类切换 |
| TOF 测距 | 中上 | TOFSENSE_FULL | 50Hz | 4 根动态高度柱状图（CSS transition） |
| IMU 姿态 | 右上 | FAST_STATUS + IMU_ATTITUDE | 100/50Hz | Canvas 绘制人工地平线 + DOM 数值 |
| FOC 电机 | 全宽横条 | FOC_FEEDBACK | 20Hz | 6 个电机卡片 + 速度进度条 |
| 三角轮 | 左下 | TRIWHEEL_FEEDBACK | 20Hz | 4 轮角度数值 |
| 事件日志 | 右下 | EVENT_NOTIFY | 事件驱动 | 滚动列表，最新在上 |

**技术细节**：

- WebSocket 连接：`ws://<host>/ws`，自动重连
- 暗色仪表盘主题，CSS Grid 响应式布局
- FPS 计数器：基于时间戳差值估算帧率
- 错误标志以彩色标签网格展示，active 状态红色高亮
- TOF 柱颜色随状态变化：绿=有效 / 黄=弱信号 / 红=无效 / 灰=超量程
- 人工地平线：Canvas 2D 绘制，天空(蓝)/地面(棕) 分界随 pitch 偏移，十字线随 roll 旋转
- 事件日志：最多保留 100 条，自动截断

**当前状态**：✅ 完成。

---

## 五、当前完成进度

### 已完成 ✅

| 模块 | 进度 | 说明 |
|---|---|---|
| protocol.py | 100% | 8 种上传包解析 + 3 种命令帧构建 + CRC16 |
| serial_driver.py | 100% | 串口 I/O + 帧状态机 + 连接监控 + 模拟数据切换 |
| mock_serial.py | 100% | 按 1kHz 调度表生成全部 7 种流式数据包 + 事件 |
| viz_server.py | 100% | FastAPI + WebSocket + 全局状态管理 + CLI 入口 |
| index.html | 100% | 6 面板仪表盘 + 暗色主题 + 响应式布局 |
| requirements.txt | 100% | fastapi, uvicorn, pyserial |
| 虚拟环境 | 100% | Python 3.10, 所有依赖已安装 |

### 待完成 🔲

| 任务 | 优先级 | 说明 |
|---|---|---|
| 真实硬件联调 | P0 | 接入 STM32 串口，验证协议解析正确性 |
| 上位机→MCU 命令面板 | P1 | 前端增加急停按钮、模式选择器，后端实际发送命令帧 |
| 历史曲线 | P2 | TOF 距离/电机转速最近 60s 趋势图（Chart.js line chart） |
| 数据录制 | P2 | "开始录制"按钮，WebSocket 流保存为 JSONL |
| ROS2 launch 文件 | P3 | 写入 params + launch 目录，集成到 ros2-stairs-detection |
| mcu_bridge 接入 | P3 | viz_server 改为订阅 ROS2 话题而非直读串口 |
| 移动端适配 | P4 | CSS @media 手机竖屏单列布局 |
| 三维姿态球 | P4 | 替换二维人工地平线为 Three.js 3D 姿态球 |

---

## 六、下一步工作

### 6.1 当前最关键：验证协议解析

**操作**：用模拟数据模式确认 6 个面板都能正常显示、数据合理。

```bash
cd /home/xpy/stair_viz
source venv/bin/activate
python -m viz_dashboard.viz_server
# 浏览器打开 http://localhost:8080
```

预期看到：
- 顶部连接状态绿灯，模式显示"闭环"
- TOF 4 根柱状图持续跳动（S3 明显偏低）
- 人工地平线随 sin/cos 缓慢摆动
- 6 个电机 RPM ~1200，扭矩在 0 附近波动
- 三角轮 4 个角度显示 ±45° 附近
- 事件日志每隔约 5 秒弹出一条

### 6.2 接入真实硬件

1. 确认 STM32 固件按协议发送帧
2. 确认 USB 枚举为 `/dev/ttyACM0`
3. 运行 `python -m viz_dashboard.viz_server --port /dev/ttyACM0`
4. 对比各面板数据与实际传感器读数

### 6.3 补全命令发送功能

在 `viz_server.py` 中增加 WebSocket 消息处理（前端→后端），将前端的急停/模式切换指令通过 `FrameEncoder` 构建帧并调用 `driver.send()`。

### 6.4 集成到 ROS2 工作区

将 `stair_viz` 以符号链接或子模块形式放入 `stair_chapter2/src/`，编写 `setup.py` + `package.xml` + launch 文件。

---

## 七、架构总图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        硬件层                                        │
│  STM32 MCU ──USB CDC──▶ /dev/ttyACM0                                │
│  (100Hz/50Hz/20Hz/5Hz 多频上传)                                      │
└─────────────────────────────┬───────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    serial_driver.py                                  │
│  串口读线程: serial.read() → FrameDecoder.feed() → ParsedFrame       │
│  连接监控:   300ms 无帧 → connected=false                            │
└─────────────────────────────┬───────────────────────────────────────┘
                              │ on_frame 回调
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    viz_server.py                                     │
│  _update_state(): ParsedFrame → mcu_state dict (线程安全锁)          │
│  _push_loop():    每 20ms 快照 mcu_state → JSON                     │
│                   事件单独推送（不合并到快照）                          │
└─────────────────────────────┬───────────────────────────────────────┘
                              │ WebSocket (ws://host:8080/ws)
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    index.html (浏览器)                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                          │
│  │ 安全状态  │  │ TOF 测距 │  │ IMU 姿态 │   ← 顶部 3 栏             │
│  │ 模式/错误 │  │ 4路柱状图│  │ 人工地平线│                          │
│  └──────────┘  └──────────┘  └──────────┘                          │
│  ┌─────────────────────────────────────────┐                        │
│  │  FOC 电机 ×6 (RPM / 扭矩 / 反馈频率)     │  ← 全宽横条            │
│  └─────────────────────────────────────────┘                        │
│  ┌──────────────────┐  ┌──────────────────┐                        │
│  │ 三角轮角度 ×4     │  │ 事件日志 (滚动)   │  ← 底部 2 栏           │
│  └──────────────────┘  └──────────────────┘                        │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 八、数据流时序

以 FAST_STATUS (100Hz) + TOFSENSE_FULL (50Hz) 为例：

```
时间轴 (ms)    MCU                         viz_server                 浏览器
─────────────────────────────────────────────────────────────────────────────
t=0          发送 FAST_STATUS #1  ────▶  FrameDecoder.feed()
                                        CRC 校验通过
                                        _update_state() 写 mcu_state
                                        _push_loop() 20ms 定时器未到

t=5          发送 TOFSENSE_FULL #1 ────▶ _update_state() 更新 tof 数组

t=10         发送 FAST_STATUS #2  ────▶ _update_state() 更新 status

...          ...

t=20                                   _push_loop() 触发
                                        lock → snapshot mcu_state
                                        json.dumps → WebSocket send ──▶ onmessage
                                                                          updateUI(snapshot)
                                                                            updateSafety()
                                                                            updateTOF()
                                                                            updateIMU()
                                                                            ...（所有面板）
```

每个 20ms 周期内，浏览器收到的 JSON 快照包含所有字段的最新值，不会出现"安全面板是 15ms 前数据、TOF 面板是 2ms 前数据"的不一致。
