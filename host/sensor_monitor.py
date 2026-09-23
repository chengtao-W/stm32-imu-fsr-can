#!/usr/bin/env python3
"""IMU and plantar-pressure CAN monitor."""

from __future__ import annotations

import argparse
import colorsys
import csv
from collections import deque
from datetime import datetime
import math
import platform
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import can
except ImportError:
    can = None

from protocol import (
    CAN_SENSOR_ID,
    FootSample,
    ImuSample,
    MSG_SENSOR_STATE,
    SensorSample,
    StreamParser,
    decode_sensor_payload,
)

COLORS = ("#3b82f6", "#22c55e", "#f97316")
PHASE_NAMES = {0: "未知", 1: "着地", 2: "支撑", 3: "离地", 4: "摆动"}


class CanReceiver:
    def __init__(self, events: queue.Queue):
        self.events = events
        self.parser = StreamParser()
        self.bus = None
        self.running = False
        self.thread = None
        self.can_frames = 0

    def connect(self, interface: str, channel: str, bitrate: int) -> None:
        if can is None:
            raise RuntimeError("缺少 python-can，请先安装 requirements.txt")
        self.disconnect()
        self.parser = StreamParser()
        self.can_frames = 0
        filters = [{"can_id": CAN_SENSOR_ID, "can_mask": 0x7FF, "extended": False}]
        self.bus = can.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            can_filters=filters,
        )
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def disconnect(self) -> None:
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.8)
        self.thread = None
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        self.bus = None

    def _read_loop(self) -> None:
        self.events.put(("status", "CAN 已连接"))
        try:
            while self.running and self.bus is not None:
                message = self.bus.recv(timeout=0.2)
                if message is None:
                    continue
                if message.is_extended_id or message.arbitration_id != CAN_SENSOR_ID:
                    continue
                self.can_frames += 1
                for message_id, payload in self.parser.feed(message.data):
                    if message_id != MSG_SENSOR_STATE:
                        continue
                    try:
                        self.events.put(("sample", decode_sensor_payload(payload)))
                    except ValueError as exc:
                        self.events.put(("log", f"载荷错误: {exc}"))
                self.events.put(("stats", self.can_frames, self.parser.frames_ok, self.parser.crc_errors))
        except Exception as exc:
            self.events.put(("status", f"CAN 错误: {exc}"))
        finally:
            self.running = False


class DemoSource:
    def __init__(self, events: queue.Queue):
        self.events = events
        self.running = False
        self.thread = None

    def start(self) -> None:
        self.stop()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None

    def _loop(self) -> None:
        start = time.monotonic()
        self.events.put(("status", "演示模式"))
        while self.running:
            t = time.monotonic() - start
            phase = 2.0 * math.pi * 0.65 * t
            imu1 = ImuSample(
                (0.05 * math.sin(phase), 0.08 * math.cos(phase), 0.98),
                (25.0 * math.cos(phase), 3.0 * math.sin(phase), 1.0),
                (20.0 * math.sin(phase), 6.0 * math.cos(phase), 15.0),
                (1.0, 0.0, 0.0, 0.0),
            )
            imu2 = ImuSample(
                (0.08 * math.sin(phase + 0.7), 0.04, 0.96),
                (35.0 * math.cos(phase + 0.7), 2.0, 0.5),
                (38.0 * math.sin(phase + 0.7), 4.0, 14.0),
                (1.0, 0.0, 0.0, 0.0),
            )
            left_level = max(0.0, math.sin(phase))
            right_level = max(0.0, -math.sin(phase))
            left_values = tuple(int(left_level * (50 + 9 * i)) for i in range(20))
            right_values = tuple(int(right_level * (50 + 9 * (19 - i))) for i in range(20))
            left = FootSample(2 if left_level > 0.1 else 4, 20, sum(left_values), left_values)
            right = FootSample(2 if right_level > 0.1 else 4, 20, sum(right_values), right_values)
            self.events.put(("sample", SensorSample(int(t * 1000), imu1, imu2, left, right)))
            time.sleep(0.1)


class LineChart(tk.Canvas):
    def __init__(self, parent, title: str, unit: str, labels=("X", "Y", "Z")):
        super().__init__(parent, bg="#101418", highlightthickness=0, height=160)
        self.title = title
        self.unit = unit
        self.labels = labels
        self.times = deque(maxlen=240)
        self.values = [deque(maxlen=240) for _ in labels]
        self.y_min = -1.0
        self.y_max = 1.0
        self.hover_x = None
        self.bind("<Configure>", lambda _event: self.redraw())
        self.bind("<Motion>", self._on_motion)
        self.bind("<Leave>", lambda _event: self._clear_hover())

    def push(self, timestamp: float, values) -> None:
        self.times.append(timestamp)
        for series, value in zip(self.values, values):
            series.append(float(value))
        self._update_range()
        self.redraw()

    def clear(self) -> None:
        self.times.clear()
        for series in self.values:
            series.clear()
        self.redraw()

    def _update_range(self) -> None:
        visible = [v for series in self.values for v in series]
        if not visible:
            return
        target_min, target_max = min(visible), max(visible)
        span = max(target_max - target_min, 0.5)
        target_min -= span * 0.12
        target_max += span * 0.12
        self.y_min += (target_min - self.y_min) * 0.18
        self.y_max += (target_max - self.y_max) * 0.18

    def _on_motion(self, event) -> None:
        self.hover_x = event.x
        self.redraw()

    def _clear_hover(self) -> None:
        self.hover_x = None
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        width, height = max(self.winfo_width(), 10), max(self.winfo_height(), 10)
        left, top, right, bottom = 48, 28, width - 12, height - 24
        self.create_text(10, 8, text=f"{self.title} ({self.unit})", anchor="nw", fill="#e5e7eb", font=("Segoe UI", 10, "bold"))
        for i, (label, color) in enumerate(zip(self.labels, COLORS)):
            self.create_line(width - 160 + i * 52, 16, width - 142 + i * 52, 16, fill=color, width=2)
            self.create_text(width - 138 + i * 52, 16, text=label, anchor="w", fill="#d1d5db")
        for i in range(5):
            y = top + (bottom - top) * i / 4
            value = self.y_max - (self.y_max - self.y_min) * i / 4
            self.create_line(left, y, right, y, fill="#28313a")
            self.create_text(left - 5, y, text=f"{value:.1f}", anchor="e", fill="#94a3b8", font=("Segoe UI", 8))
        if len(self.times) < 2:
            return
        t_end = self.times[-1]
        t_start = max(self.times[0], t_end - 20.0)
        span_t = max(t_end - t_start, 0.1)
        span_y = max(self.y_max - self.y_min, 1e-6)
        visible_indices = [i for i, value in enumerate(self.times) if value >= t_start]
        for series, color in zip(self.values, COLORS):
            points = []
            for i in visible_indices:
                x = left + (self.times[i] - t_start) / span_t * (right - left)
                y = bottom - (series[i] - self.y_min) / span_y * (bottom - top)
                points.extend((x, y))
            if len(points) >= 4:
                self.create_line(*points, fill=color, width=2, smooth=True)
        self.create_text(right, bottom + 5, text=f"{span_t:.1f} s", anchor="ne", fill="#94a3b8", font=("Segoe UI", 8))
        if self.hover_x is not None and left <= self.hover_x <= right:
            ratio = (self.hover_x - left) / max(right - left, 1)
            target_t = t_start + ratio * span_t
            index = min(visible_indices, key=lambda i: abs(self.times[i] - target_t))
            x = left + (self.times[index] - t_start) / span_t * (right - left)
            self.create_line(x, top, x, bottom, fill="#cbd5e1", dash=(3, 3))
            text = "  ".join(f"{label}:{series[index]:.2f}" for label, series in zip(self.labels, self.values))
            self.create_text(min(x + 6, right - 145), top + 4, text=text, anchor="nw", fill="#f8fafc", font=("Consolas", 9))


class PressureMap(tk.Canvas):
    def __init__(self, parent):
        super().__init__(parent, bg="#101418", highlightthickness=0)
        self.left = (0,) * 20
        self.right = (0,) * 20
        self.hover = None
        self.cells = []
        self.bind("<Configure>", lambda _event: self.redraw())
        self.bind("<Motion>", self._on_motion)
        self.bind("<Leave>", lambda _event: self._clear_hover())

    def update_values(self, left, right) -> None:
        self.left, self.right = tuple(left), tuple(right)
        self.redraw()

    def _color(self, value: int) -> str:
        ratio = max(0.0, min(float(value) / 255.0, 1.0))
        red, green, blue = colorsys.hsv_to_rgb(0.34 * (1.0 - ratio), 0.82, 0.82)
        return f"#{int(red * 255):02x}{int(green * 255):02x}{int(blue * 255):02x}"

    def _on_motion(self, event) -> None:
        self.hover = next(((side, index, value, box) for side, index, value, box in self.cells
                           if box[0] <= event.x <= box[2] and box[1] <= event.y <= box[3]), None)
        self.redraw()

    def _clear_hover(self) -> None:
        self.hover = None
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        self.cells.clear()
        width, height = max(self.winfo_width(), 20), max(self.winfo_height(), 20)
        margin, gap = 30, 52
        foot_width = (width - margin * 2 - gap) / 2
        cell_gap = 5
        cell_w = (foot_width - cell_gap * 3) / 4
        cell_h = max(38, min((height - 80 - cell_gap * 4) / 5, 100))
        for side_index, (title, values) in enumerate((("左足", self.left), ("右足", self.right))):
            x0 = margin + side_index * (foot_width + gap)
            self.create_text(x0, 18, text=title, anchor="w", fill="#e5e7eb", font=("Segoe UI", 12, "bold"))
            for index, value in enumerate(values):
                row, column = divmod(index, 4)
                x1 = x0 + column * (cell_w + cell_gap)
                y1 = 44 + row * (cell_h + cell_gap)
                box = (x1, y1, x1 + cell_w, y1 + cell_h)
                self.create_rectangle(*box, fill=self._color(value), outline="#334155", width=2)
                self.create_text((box[0] + box[2]) / 2, (box[1] + box[3]) / 2,
                                 text=f"{index}\n{value}", fill="#081014", font=("Segoe UI", 10, "bold"))
                self.cells.append((title, index, value, box))
        if self.hover:
            side, index, value, box = self.hover
            self.create_rectangle(*box, outline="#ffffff", width=3)
            self.create_text(width / 2, height - 12, text=f"{side} 点 {index}: {value}",
                             fill="#f8fafc", font=("Segoe UI", 10, "bold"))


class SensorMonitor(tk.Tk):
    def __init__(self, start_demo: bool = False):
        super().__init__()
        self.title("IMU 与足底压力监测")
        self.geometry("1280x820")
        self.minsize(980, 680)
        self.events = queue.Queue()
        self.receiver = CanReceiver(self.events)
        self.demo = DemoSource(self.events)
        self.started_at = None
        self.csv_file = None
        self.csv_writer = None
        self.sample_count = 0
        self._build_ui()
        self.after(50, self._poll_events)
        self.protocol("WM_DELETE_WINDOW", self._close)
        if start_demo:
            self.after(150, self.start_demo)

    def _build_ui(self) -> None:
        style = ttk.Style(self)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        top = ttk.Frame(self, padding=(10, 8))
        top.pack(fill="x")
        ttk.Label(top, text="CAN 后端").pack(side="left")
        default_interface = "socketcan" if platform.system() == "Linux" else "pcan"
        self.interface = ttk.Combobox(top, width=11, state="readonly", values=("socketcan", "pcan", "vector", "slcan"))
        self.interface.set(default_interface)
        self.interface.pack(side="left", padx=(5, 12))
        ttk.Label(top, text="通道").pack(side="left")
        self.channel = ttk.Entry(top, width=16)
        self.channel.insert(0, "can0" if default_interface == "socketcan" else "PCAN_USBBUS1")
        self.channel.pack(side="left", padx=(5, 12))
        ttk.Label(top, text="波特率").pack(side="left")
        self.bitrate = ttk.Combobox(top, width=10, state="readonly", values=("1000000", "500000"))
        self.bitrate.set("1000000")
        self.bitrate.pack(side="left", padx=(5, 12))
        ttk.Button(top, text="连接", command=self.connect_can).pack(side="left", padx=3)
        ttk.Button(top, text="断开", command=self.disconnect).pack(side="left", padx=3)
        ttk.Button(top, text="演示模式", command=self.start_demo).pack(side="left", padx=3)
        ttk.Button(top, text="清空图像", command=self.clear_charts).pack(side="left", padx=(14, 3))
        self.record_button = ttk.Button(top, text="开始记录 CSV", command=self.toggle_recording)
        self.record_button.pack(side="left", padx=3)
        self.status = tk.StringVar(value="未连接")
        ttk.Label(top, textvariable=self.status, foreground="#0f766e").pack(side="right")

        summary = ttk.Frame(self, padding=(10, 0, 10, 8))
        summary.pack(fill="x")
        self.stats = tk.StringVar(value="CAN帧 0 | 样本 0 | CRC错误 0")
        self.imu_summary = tk.StringVar(value="IMU: 等待数据")
        self.pressure_summary = tk.StringVar(value="足底压力: 等待数据")
        ttk.Label(summary, textvariable=self.stats).pack(side="left")
        ttk.Label(summary, textvariable=self.imu_summary).pack(side="left", padx=24)
        ttk.Label(summary, textvariable=self.pressure_summary).pack(side="left", padx=24)

        tabs = ttk.Notebook(self)
        tabs.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        imu_page, pressure_page, log_page = ttk.Frame(tabs), ttk.Frame(tabs), ttk.Frame(tabs)
        tabs.add(imu_page, text="IMU 数据")
        tabs.add(pressure_page, text="足底压力")
        tabs.add(log_page, text="通信日志")

        imu_tabs = ttk.Notebook(imu_page)
        imu_tabs.pack(fill="both", expand=True)
        self.charts = []
        self.imu_charts = []
        for name in ("IMU1 大腿", "IMU2 小腿"):
            page = ttk.Frame(imu_tabs)
            imu_tabs.add(page, text=name)
            group = (
                LineChart(page, "欧拉角", "deg", ("Roll", "Pitch", "Yaw")),
                LineChart(page, "角速度", "deg/s", ("GX", "GY", "GZ")),
                LineChart(page, "加速度", "g", ("AX", "AY", "AZ")),
            )
            for chart in group:
                chart.pack(fill="both", expand=True, pady=1)
                self.charts.append(chart)
            self.imu_charts.append(group)

        pressure_header = ttk.Frame(pressure_page, padding=8)
        pressure_header.pack(fill="x")
        self.left_label = tk.StringVar(value="左足: --")
        self.right_label = tk.StringVar(value="右足: --")
        ttk.Label(pressure_header, textvariable=self.left_label, font=("Segoe UI", 11, "bold")).pack(side="left", padx=12)
        ttk.Label(pressure_header, textvariable=self.right_label, font=("Segoe UI", 11, "bold")).pack(side="left", padx=30)
        ttk.Label(pressure_header, text="色阶 0-255").pack(side="right", padx=12)
        self.pressure_map = PressureMap(pressure_page)
        self.pressure_map.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        self.log_text = tk.Text(log_page, bg="#101418", fg="#d1d5db", insertbackground="white", wrap="word", font=("Consolas", 10))
        self.log_text.pack(fill="both", expand=True, padx=6, pady=6)

    def connect_can(self) -> None:
        try:
            self.demo.stop()
            interface = self.interface.get()
            channel_text = self.channel.get().strip()
            channel = int(channel_text) if interface == "vector" and channel_text.isdigit() else channel_text
            self.receiver.connect(interface, channel, int(self.bitrate.get()))
            self.started_at = time.monotonic()
            self._log(f"连接 {interface} / {channel_text} / {self.bitrate.get()}")
        except Exception as exc:
            messagebox.showerror("CAN 连接失败", str(exc), parent=self)
            self.status.set("连接失败")

    def disconnect(self) -> None:
        self.receiver.disconnect()
        self.demo.stop()
        self.status.set("已断开")
        self._log("连接已断开")

    def start_demo(self) -> None:
        self.receiver.disconnect()
        self.started_at = time.monotonic()
        self.demo.start()
        self._log("演示模式已启动")

    def _poll_events(self) -> None:
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "sample":
                    self._handle_sample(event[1])
                elif event[0] == "status":
                    self.status.set(event[1])
                    self._log(event[1])
                elif event[0] == "stats":
                    self.stats.set(f"CAN帧 {event[1]} | 样本 {event[2]} | CRC错误 {event[3]}")
                elif event[0] == "log":
                    self._log(event[1])
        except queue.Empty:
            pass
        self.after(50, self._poll_events)

    def _handle_sample(self, sample: SensorSample) -> None:
        self.sample_count += 1
        timestamp = time.monotonic() - (self.started_at or time.monotonic())
        for imu, charts in ((sample.imu1, self.imu_charts[0]), (sample.imu2, self.imu_charts[1])):
            charts[0].push(timestamp, imu.euler)
            charts[1].push(timestamp, imu.angular_rate)
            charts[2].push(timestamp, imu.acceleration)
        self.pressure_map.update_values(sample.left.pressure, sample.right.pressure)
        self.left_label.set(f"左足: 总压力 {sample.left.total} | {PHASE_NAMES.get(sample.left.phase, sample.left.phase)} | {sample.left.point_count}点")
        self.right_label.set(f"右足: 总压力 {sample.right.total} | {PHASE_NAMES.get(sample.right.phase, sample.right.phase)} | {sample.right.point_count}点")
        self.imu_summary.set(f"IMU1 R/P/Y {sample.imu1.euler[0]:.1f}/{sample.imu1.euler[1]:.1f}/{sample.imu1.euler[2]:.1f} deg")
        self.pressure_summary.set(f"压力 L/R {sample.left.total}/{sample.right.total}")
        if not self.receiver.running:
            self.stats.set(f"演示样本 {self.sample_count}")
        self._write_csv(sample)

    def clear_charts(self) -> None:
        for chart in self.charts:
            chart.clear()
        self.pressure_map.update_values((0,) * 20, (0,) * 20)
        self.sample_count = 0

    def toggle_recording(self) -> None:
        if self.csv_file:
            self._stop_recording()
            return
        path = filedialog.asksaveasfilename(
            parent=self,
            title="保存传感器数据",
            defaultextension=".csv",
            initialfile=f"sensor_{datetime.now():%Y%m%d_%H%M%S}.csv",
            filetypes=(("CSV", "*.csv"),),
        )
        if not path:
            return
        self.csv_file = open(path, "w", newline="", encoding="utf-8-sig")
        self.csv_writer = csv.writer(self.csv_file)
        header = ["pc_time", "mcu_ms"]
        for prefix in ("imu1", "imu2"):
            header += [f"{prefix}_{name}" for name in (
                "ax", "ay", "az", "gx", "gy", "gz", "roll", "pitch", "yaw", "qw", "qx", "qy", "qz")]
        header += ["left_phase", "right_phase", "left_total", "right_total"]
        header += [f"left_{i}" for i in range(20)] + [f"right_{i}" for i in range(20)]
        self.csv_writer.writerow(header)
        self.record_button.configure(text="停止记录 CSV")
        self._log(f"开始记录: {path}")

    def _write_csv(self, sample: SensorSample) -> None:
        if not self.csv_writer:
            return
        row = [datetime.now().isoformat(timespec="milliseconds"), sample.timestamp_ms]
        for imu in (sample.imu1, sample.imu2):
            row += [*imu.acceleration, *imu.angular_rate, *imu.euler, *imu.quaternion]
        row += [sample.left.phase, sample.right.phase, sample.left.total, sample.right.total]
        row += [*sample.left.pressure, *sample.right.pressure]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def _stop_recording(self) -> None:
        if self.csv_file:
            self.csv_file.close()
        self.csv_file = self.csv_writer = None
        self.record_button.configure(text="开始记录 CSV")
        self._log("CSV 记录已停止")

    def _log(self, text: str) -> None:
        self.log_text.insert("end", f"[{datetime.now():%H:%M:%S}] {text}\n")
        self.log_text.see("end")

    def _close(self) -> None:
        self.receiver.disconnect()
        self.demo.stop()
        self._stop_recording()
        self.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="IMU and plantar-pressure CAN monitor")
    parser.add_argument("--demo", action="store_true", help="start without CAN hardware")
    args = parser.parse_args()
    SensorMonitor(start_demo=args.demo).mainloop()


if __name__ == "__main__":
    main()
