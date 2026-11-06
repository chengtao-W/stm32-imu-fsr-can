# STM32 IMU 与足底压力 CAN 系统

该项目包含 STM32H723 下位机和 Python 图形上位机。下位机采集 IMU 与墨现足底压力传感器数据，通过 CAN1 周期发送；上位机完成 CAN 分片重组、CRC 校验、实时曲线、压力热力图和 CSV 记录。

当前版本只处理传感器数据，不包含电机初始化、电机控制、电机反馈、AO 控制或力矩指令。

## 功能

- 通过 CAN2 接收最多两路 IMU 数据。
- 解析加速度、角速度、欧拉角和四元数。
- 通过 USB Host CDC 接收墨现 20 点足底压力数据。
- 支持左右鞋垫、总压力和步态相位计算。
- 将完整传感器快照封装为带 CRC16 的应用层消息。
- 通过 CAN1 以 200 Hz 向上位机发送传感器快照。
- 分别判断两路 IMU 是否在 500 ms 内收到数据；四元数过期时只将对应姿态字段置零。
- 2 号 IMU 有其他测量却缺少四元数时，限次请求其以 10 ms 周期发送四元数。
- 上位机显示两路 IMU 曲线和左右足底 20 点压力热力图。
- 支持 Windows CAN 适配器和 Jetson SocketCAN。
- 支持 CSV 记录、悬停读数和无硬件演示模式。
- USART1 提供 `STATUS`、`FSR` 和 `HELP` 调试命令。

## 系统结构

```text
IMU 1/2 -- CAN2 500 kbit/s --+
                              |
墨现鞋垫 -- USB Host CDC -----+--> STM32H723 --> CAN1 1 Mbit/s --> Python上位机
                                      |                              |
                                      +--> USART1 调试输出            +--> 曲线/热力图/CSV
```

## 数据流

1. `IMU_Poll_RX()`轮询 CAN2 接收 FIFO 并更新两路 IMU 状态。
2. USB Host CDC 回调将数据送入足底压力帧解析器。
3. `FSR_DetectGaitPhase()`根据左右足底压力更新接触状态和步态相位。
4. TIM4 每 5 ms 产生发送标志；站起/坐下判断仍每 100 ms 执行一次。
5. `ExoTelemetry_Push()`生成 196 字节传感器快照。
6. 通信层增加 `EB 90` 帧头和 CRC16，并拆分为 26 个经典 CAN 帧发送。

200 Hz 是固件遥测调度频率，不代表每路 IMU 的每个测量字段都以 200 Hz 更新；IMU 仍按自身配置的频率提供新样本。
串口只用于调试命令和状态输出；传感器快照通过 CAN1 发布。

## CAN 接口

| 用途 | 外设 | 波特率 | CAN ID |
| --- | --- | ---: | ---: |
| IMU 输入 | FDCAN2 | 500 kbit/s | 由 IMU 协议定义 |
| 上位机输出 | FDCAN1 | 1 Mbit/s | `0x621` |
| 上位机输入 | FDCAN1 | 1 Mbit/s | `0x620`，当前不注册控制处理器 |

传感器消息 ID 为 `0x05`。载荷包含：

- 两路 IMU 的三轴加速度、三轴角速度、欧拉角和四元数；
- 左右足底的有效点数、总压力、步态相位；
- 左右足底各 20 个原始压力值。

完整字节布局、CRC 和重组方法见 [CAN 协议文档](docs/CAN_PROTOCOL.md)。

## 目录

```text
Core/          应用逻辑、传感器解析和通信封装
Drivers/       STM32H7 HAL 与 CMSIS
Middlewares/   STM32 USB Host CDC 中间件
USB_HOST/      USB Host 应用与底层配置
User/          CAN BSP、IMU 协议解析和延时模块
MDK-ARM/       Keil 工程与启动文件
docs/          通信协议文档
host/          Python CAN 图形上位机
CtrBoard.ioc   STM32CubeMX 工程
```

## 关键文件

- `Core/Src/main.c`：初始化和主循环。
- `Core/Src/imu.c`：IMU CAN 数据接收与解析。
- `Core/Src/fsr.c`：墨现鞋垫 USB CDC 帧解析和步态相位。
- `Core/Src/exo_telemetry.c`：传感器快照组装。
- `Core/Src/robot_comm.c`：应用层帧、CRC16 和 CAN 分片发送。
- `User/bsp_fdcan.c`：FDCAN 配置与收发接口。
- `host/protocol.py`：CAN 字节流重组、CRC16 和传感器载荷解码。
- `host/sensor_monitor.py`：IMU 曲线、足底热力图和 CSV 图形界面。

## 编译

### Keil MDK

1. 安装 Keil MDK-ARM 和 STM32H7 Device Family Pack。
2. 打开 `MDK-ARM/CtrBoard.uvprojx`。
3. 选择目标 `CtrBoard`。
4. 执行 Build。

已验证工具链：ARM Compiler 5.06 update 1。当前仓库中的最小工程编译结果为 `0 errors, 0 warnings`。

## 上位机

安装依赖：

```bash
cd host
python -m pip install -r requirements.txt
```

连接 CAN 硬件后运行：

```bash
python sensor_monitor.py
```

不连接硬件检查界面：

```bash
python sensor_monitor.py --demo
```

Windows PCAN 默认使用 `pcan / PCAN_USBBUS1`；Jetson 默认使用 `socketcan / can0`。详细说明见 [上位机文档](host/README.md)。

## Jetson 接收

配置 SocketCAN：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
candump can0,621:7FF
```

Jetson 端不能把每个 CAN 帧当作独立样本。接收程序需要按顺序拼接 ID `0x621` 的数据，搜索 `EB 90`，读取长度字段，收齐载荷和 CRC 后再发布 ROS 消息。

## 当前限制

- 经典 CAN 单帧最多 8 字节，一个传感器样本需要 26 帧。
- 协议当前没有逐 CAN 帧序号；发生丢帧时，应丢弃本条消息并搜索下一组 `EB 90` 帧头。
- 未连接的第二路 IMU 或第二只鞋垫字段保持为零。
- 仓库不包含电机控制功能，也不接受电机控制命令。
