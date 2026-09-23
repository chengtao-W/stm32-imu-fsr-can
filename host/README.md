# 传感器 CAN 上位机

该上位机只接收并显示当前下位机发送的 IMU 与足底压力消息，不包含电机控制功能。

## 功能

- 重组 CAN ID `0x621` 的连续经典 CAN 帧。
- 校验应用层 CRC16，并在丢帧或 CRC 错误后自动重新同步。
- 显示两路 IMU 的欧拉角、角速度和加速度曲线。
- 显示左右鞋垫各 20 个压力点的热力图、总压力和步态相位。
- 鼠标悬停在曲线或压力点上时显示对应数值。
- 将完整传感器样本记录为 CSV。
- 提供演示模式，无 CAN 硬件也能检查界面。

## 安装

需要 Python 3.10 或更高版本。Tkinter 随 Windows 官方 Python 安装；Ubuntu/Jetson 可能需要安装 `python3-tk`。

```bash
python -m pip install -r requirements.txt
```

## Windows

以 PCAN 为例，在界面中选择：

```text
CAN 后端: pcan
通道: PCAN_USBBUS1
波特率: 1000000
```

其他适配器可根据 python-can 驱动选择 `vector` 或 `slcan`。Windows 可以双击 `run_monitor.bat` 启动。

## Jetson

先配置 SocketCAN：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

启动上位机：

```bash
python sensor_monitor.py
```

界面选择 `socketcan`、`can0` 和 `1000000`。

## 演示模式

```bash
python sensor_monitor.py --demo
```

演示数据只用于检查图像、热力图和 CSV，不代表真实人体运动。

## 测试

```bash
python -m unittest test_protocol.py
```

协议模块只使用 Python 标准库，测试覆盖载荷解码、8 字节分片重组以及 CRC 错误后的重新同步。
