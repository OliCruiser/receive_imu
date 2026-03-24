import json
import math
import queue
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk
from typing import Optional

import serial


FRAME_HEADER = b"\xAA\x55"
FRAME_TAIL = b"\x55\xAA"
DEFAULT_PORT = "COM19"
DEFAULT_BAUDRATE = 115200
READ_TIMEOUT = 1.0
CALIBRATION_FILE = Path("imu_calibration.json")
COMPLEMENTARY_ALPHA = 0.98


@dataclass
class SensorSample:
    frame_id: int
    payload_frame_id: int
    timestamp_ms: Optional[int]
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: float
    my: float
    mz: float


@dataclass
class Calibration:
    gyro_bias: dict[str, float]
    accel_bias: dict[str, float]
    accel_scale: dict[str, float]
    mag_bias: dict[str, float]
    mag_scale: dict[str, float]


@dataclass
class CorrectedSample:
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: float
    my: float
    mz: float


@dataclass
class OrientationState:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    forward_x: float
    forward_y: float
    forward_z: float


def calc_xor_checksum(data: bytes) -> int:
    chk = 0
    for byte in data:
        chk ^= byte
    return chk


def load_calibration(path: Path) -> Calibration:
    raw = json.loads(path.read_text(encoding="utf-8"))
    return Calibration(
        gyro_bias=raw.get("gyro_bias", {"x": 0.0, "y": 0.0, "z": 0.0}),
        accel_bias=raw.get("accel_bias") or raw.get("accel_offset_horizontal") or {"x": 0.0, "y": 0.0, "z": 0.0},
        accel_scale=raw.get("accel_scale") or {"x": 1.0, "y": 1.0, "z": 1.0},
        mag_bias=raw.get("mag_bias") or {"x": 0.0, "y": 0.0, "z": 0.0},
        mag_scale=raw.get("mag_scale") or {"x": 1.0, "y": 1.0, "z": 1.0},
    )


def parse_frame(frame: bytes) -> tuple[int, str]:
    if len(frame) < 10:
        raise ValueError("frame too short")
    if not frame.startswith(FRAME_HEADER):
        raise ValueError("invalid header")
    if not frame.endswith(FRAME_TAIL):
        raise ValueError("invalid tail")

    payload_len = frame[2]
    expected_len = 2 + 1 + 4 + payload_len + 1 + 2
    if len(frame) != expected_len:
        raise ValueError(f"length mismatch: expected {expected_len}, got {len(frame)}")

    frame_id = struct.unpack("<I", frame[3:7])[0]
    payload = frame[7:7 + payload_len]
    recv_chk = frame[7 + payload_len]
    calc_chk = calc_xor_checksum(payload)
    if recv_chk != calc_chk:
        raise ValueError(f"checksum mismatch: expected 0x{calc_chk:02X}, got 0x{recv_chk:02X}")
    return frame_id, payload.decode("utf-8")


def parse_payload(frame_id: int, payload_text: str) -> SensorSample:
    parts = payload_text.strip().split(",")
    if len(parts) not in (10, 11):
        raise ValueError(f"expected 10 or 11 CSV fields, got {len(parts)}")

    values = [float(part) for part in parts]
    payload_frame_id = int(values[0])
    if len(parts) == 11:
        timestamp_ms = int(values[1])
        base = 2
    else:
        timestamp_ms = None
        base = 1

    return SensorSample(
        frame_id=frame_id,
        payload_frame_id=payload_frame_id,
        timestamp_ms=timestamp_ms,
        ax=values[base + 0],
        ay=values[base + 1],
        az=values[base + 2],
        gx=values[base + 3],
        gy=values[base + 4],
        gz=values[base + 5],
        mx=values[base + 6],
        my=values[base + 7],
        mz=values[base + 8],
    )


def read_one_frame(ser: serial.Serial) -> Optional[bytes]:
    buffer = bytearray(getattr(ser, "_orientation_rx_buffer", b""))
    while True:
        if len(buffer) < 3:
            chunk = ser.read(max(1, ser.in_waiting or 1))
            if not chunk:
                setattr(ser, "_orientation_rx_buffer", bytes(buffer))
                return None
            buffer.extend(chunk)

        header_index = buffer.find(FRAME_HEADER)
        if header_index < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            continue

        if header_index > 0:
            del buffer[:header_index]

        if len(buffer) < 3:
            continue

        payload_len = buffer[2]
        frame_len = 2 + 1 + 4 + payload_len + 1 + 2
        while len(buffer) < frame_len:
            chunk = ser.read(frame_len - len(buffer))
            if not chunk:
                setattr(ser, "_orientation_rx_buffer", bytes(buffer))
                return None
            buffer.extend(chunk)

        frame = bytes(buffer[:frame_len])
        del buffer[:frame_len]
        setattr(ser, "_orientation_rx_buffer", bytes(buffer))
        return frame


def read_one_sample(ser: serial.Serial) -> Optional[SensorSample]:
    frame = read_one_frame(ser)
    if frame is None:
        return None
    frame_id, payload_text = parse_frame(frame)
    return parse_payload(frame_id, payload_text)


def correct_sample(sample: SensorSample, calibration: Calibration) -> CorrectedSample:
    return CorrectedSample(
        ax=(sample.ax - calibration.accel_bias["x"]) * calibration.accel_scale["x"],
        ay=(sample.ay - calibration.accel_bias["y"]) * calibration.accel_scale["y"],
        az=(sample.az - calibration.accel_bias["z"]) * calibration.accel_scale["z"],
        gx=sample.gx - calibration.gyro_bias["x"],
        gy=sample.gy - calibration.gyro_bias["y"],
        gz=sample.gz - calibration.gyro_bias["z"],
        mx=(sample.mx - calibration.mag_bias["x"]) * calibration.mag_scale["x"],
        my=(sample.my - calibration.mag_bias["y"]) * calibration.mag_scale["y"],
        mz=(sample.mz - calibration.mag_bias["z"]) * calibration.mag_scale["z"],
    )


def wrap_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def compute_accel_roll_pitch(sample: CorrectedSample) -> tuple[float, float]:
    roll = math.degrees(math.atan2(sample.ay, sample.az))
    pitch = math.degrees(math.atan2(-sample.ax, math.sqrt(sample.ay ** 2 + sample.az ** 2)))
    return roll, pitch


def compute_tilt_compensated_yaw(sample: CorrectedSample, roll_deg: float, pitch_deg: float) -> float:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)

    mx = sample.mx
    my = sample.my
    mz = sample.mz

    mag_x = mx * math.cos(pitch) + mz * math.sin(pitch)
    mag_y = (
        mx * math.sin(roll) * math.sin(pitch)
        + my * math.cos(roll)
        - mz * math.sin(roll) * math.cos(pitch)
    )
    yaw = math.degrees(math.atan2(mag_y, mag_x))
    return wrap_angle_deg(yaw)


class OrientationFilter:
    def __init__(self) -> None:
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.last_time_s: Optional[float] = None
        self.initialized = False

    def update(self, sample: CorrectedSample, sample_time_s: float) -> OrientationState:
        accel_roll_deg, accel_pitch_deg = compute_accel_roll_pitch(sample)
        mag_yaw_deg = compute_tilt_compensated_yaw(sample, accel_roll_deg, accel_pitch_deg)

        if not self.initialized:
            self.roll_deg = accel_roll_deg
            self.pitch_deg = accel_pitch_deg
            self.yaw_deg = mag_yaw_deg
            self.initialized = True
        else:
            dt = sample_time_s - self.last_time_s if self.last_time_s is not None else 0.0
            if dt < 0.0 or dt > 1.0:
                dt = 0.0

            gyro_roll = self.roll_deg + sample.gx * dt
            gyro_pitch = self.pitch_deg + sample.gy * dt
            gyro_yaw = wrap_angle_deg(self.yaw_deg + sample.gz * dt)

            self.roll_deg = COMPLEMENTARY_ALPHA * gyro_roll + (1.0 - COMPLEMENTARY_ALPHA) * accel_roll_deg
            self.pitch_deg = COMPLEMENTARY_ALPHA * gyro_pitch + (1.0 - COMPLEMENTARY_ALPHA) * accel_pitch_deg

            yaw_error = wrap_angle_deg(mag_yaw_deg - gyro_yaw)
            self.yaw_deg = wrap_angle_deg(gyro_yaw + (1.0 - COMPLEMENTARY_ALPHA) * yaw_error)

        self.last_time_s = sample_time_s

        pitch_rad = math.radians(self.pitch_deg)
        yaw_rad = math.radians(self.yaw_deg)
        forward_x = math.cos(yaw_rad) * math.cos(pitch_rad)
        forward_y = math.sin(yaw_rad) * math.cos(pitch_rad)
        forward_z = -math.sin(pitch_rad)
        return OrientationState(
            roll_deg=self.roll_deg,
            pitch_deg=self.pitch_deg,
            yaw_deg=self.yaw_deg,
            forward_x=forward_x,
            forward_y=forward_y,
            forward_z=forward_z,
        )


class OrientationGuiApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("姿态方向查看器")
        self.root.geometry("980x760")

        self.message_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.worker_thread: threading.Thread | None = None
        self.stop_event = threading.Event()

        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        self.status_var = tk.StringVar(value="就绪")
        self.calibration_var = tk.StringVar(value=str(CALIBRATION_FILE.resolve()))

        self.roll_var = tk.StringVar(value="0.00°")
        self.pitch_var = tk.StringVar(value="0.00°")
        self.yaw_var = tk.StringVar(value="0.00°")
        self.forward_var = tk.StringVar(value="(1.000, 0.000, 0.000)")
        self.raw_accel_var = tk.StringVar(value="(0.000, 0.000, 0.000)")
        self.raw_gyro_var = tk.StringVar(value="(0.000, 0.000, 0.000)")
        self.raw_mag_var = tk.StringVar(value="(0.000, 0.000, 0.000)")
        self.corr_accel_var = tk.StringVar(value="(0.000, 0.000, 0.000)")
        self.corr_gyro_var = tk.StringVar(value="(0.000, 0.000, 0.000)")
        self.corr_mag_var = tk.StringVar(value="(0.000, 0.000, 0.000)")

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.process_queue)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(3, weight=1)

        control = ttk.Frame(self.root, padding=12)
        control.grid(row=0, column=0, sticky="ew")
        control.columnconfigure(7, weight=1)

        ttk.Label(control, text="串口").grid(row=0, column=0, sticky="w")
        ttk.Entry(control, textvariable=self.port_var, width=10).grid(row=0, column=1, padx=(6, 12), sticky="w")
        ttk.Label(control, text="波特率").grid(row=0, column=2, sticky="w")
        ttk.Entry(control, textvariable=self.baud_var, width=10).grid(row=0, column=3, padx=(6, 12), sticky="w")
        ttk.Button(control, text="开始", command=self.start_stream).grid(row=0, column=4, padx=4)
        ttk.Button(control, text="停止", command=self.stop_stream).grid(row=0, column=5, padx=4)
        ttk.Button(control, text="重新加载标定", command=self.reload_calibration).grid(row=0, column=6, padx=4)

        ttk.Label(control, text="标定文件").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Label(control, textvariable=self.calibration_var).grid(row=1, column=1, columnspan=7, sticky="w", pady=(10, 0))

        status = ttk.Frame(self.root, padding=(12, 0, 12, 12))
        status.grid(row=1, column=0, sticky="ew")
        ttk.Label(status, text="状态").grid(row=0, column=0, sticky="w")
        ttk.Label(status, textvariable=self.status_var).grid(row=0, column=1, sticky="w")

        orientation_frame = ttk.LabelFrame(self.root, text="姿态结果", padding=12)
        orientation_frame.grid(row=2, column=0, sticky="ew", padx=12, pady=(0, 12))
        orientation_frame.columnconfigure(1, weight=1)
        orientation_frame.columnconfigure(3, weight=1)

        ttk.Label(orientation_frame, text="Roll").grid(row=0, column=0, sticky="w")
        ttk.Label(orientation_frame, textvariable=self.roll_var).grid(row=0, column=1, sticky="w")
        ttk.Label(orientation_frame, text="Pitch").grid(row=0, column=2, sticky="w")
        ttk.Label(orientation_frame, textvariable=self.pitch_var).grid(row=0, column=3, sticky="w")
        ttk.Label(orientation_frame, text="Yaw").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(orientation_frame, textvariable=self.yaw_var).grid(row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Label(orientation_frame, text="方向向量").grid(row=1, column=2, sticky="w", pady=(8, 0))
        ttk.Label(orientation_frame, textvariable=self.forward_var).grid(row=1, column=3, sticky="w", pady=(8, 0))

        viz_frame = ttk.LabelFrame(self.root, text="ENU 图形方向", padding=12)
        viz_frame.grid(row=3, column=0, sticky="ew", padx=12, pady=(0, 12))
        viz_frame.columnconfigure(0, weight=1)
        viz_frame.columnconfigure(1, weight=1)

        self.top_canvas = tk.Canvas(viz_frame, width=420, height=280, bg="#f7f7f2", highlightthickness=1, highlightbackground="#c6c6c6")
        self.top_canvas.grid(row=0, column=0, padx=(0, 8), sticky="nsew")
        self.side_canvas = tk.Canvas(viz_frame, width=420, height=280, bg="#f7f7f2", highlightthickness=1, highlightbackground="#c6c6c6")
        self.side_canvas.grid(row=0, column=1, padx=(8, 0), sticky="nsew")

        data_frame = ttk.LabelFrame(self.root, text="传感器数据", padding=12)
        data_frame.grid(row=4, column=0, sticky="nsew", padx=12, pady=(0, 12))
        data_frame.columnconfigure(1, weight=1)
        data_frame.columnconfigure(3, weight=1)

        ttk.Label(data_frame, text="原始加速度").grid(row=0, column=0, sticky="w")
        ttk.Label(data_frame, textvariable=self.raw_accel_var).grid(row=0, column=1, sticky="w")
        ttk.Label(data_frame, text="校正后加速度").grid(row=0, column=2, sticky="w")
        ttk.Label(data_frame, textvariable=self.corr_accel_var).grid(row=0, column=3, sticky="w")

        ttk.Label(data_frame, text="原始角速度").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(data_frame, textvariable=self.raw_gyro_var).grid(row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Label(data_frame, text="校正后角速度").grid(row=1, column=2, sticky="w", pady=(8, 0))
        ttk.Label(data_frame, textvariable=self.corr_gyro_var).grid(row=1, column=3, sticky="w", pady=(8, 0))

        ttk.Label(data_frame, text="原始磁力计").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Label(data_frame, textvariable=self.raw_mag_var).grid(row=2, column=1, sticky="w", pady=(8, 0))
        ttk.Label(data_frame, text="校正后磁力计").grid(row=2, column=2, sticky="w", pady=(8, 0))
        ttk.Label(data_frame, textvariable=self.corr_mag_var).grid(row=2, column=3, sticky="w", pady=(8, 0))

        log_frame = ttk.LabelFrame(self.root, text="日志", padding=12)
        log_frame.grid(row=5, column=0, sticky="nsew", padx=12, pady=(0, 12))
        self.root.rowconfigure(5, weight=1)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(log_frame, wrap="word", height=14, font=("Consolas", 10))
        self.log_text.grid(row=0, column=0, sticky="nsew")
        self.log_text.configure(state="disabled")
        self.draw_enu_canvases(1.0, 0.0, 0.0)

    def draw_arrow(self, canvas: tk.Canvas, start: tuple[float, float], end: tuple[float, float], color: str, width: int = 3) -> None:
        canvas.create_line(
            start[0],
            start[1],
            end[0],
            end[1],
            fill=color,
            width=width,
            arrow=tk.LAST,
            arrowshape=(12, 14, 5),
        )

    def draw_enu_canvases(self, east: float, north: float, up: float) -> None:
        self.top_canvas.delete("all")
        self.side_canvas.delete("all")

        top_w = int(self.top_canvas["width"])
        top_h = int(self.top_canvas["height"])
        side_w = int(self.side_canvas["width"])
        side_h = int(self.side_canvas["height"])

        cx_top = top_w / 2
        cy_top = top_h / 2
        cx_side = side_w / 2
        cy_side = side_h / 2
        radius = min(top_w, top_h) * 0.34

        self.top_canvas.create_text(16, 16, text="俯视图: EN 平面", anchor="w", fill="#2b2b2b", font=("Microsoft YaHei UI", 11, "bold"))
        self.top_canvas.create_oval(cx_top - radius, cy_top - radius, cx_top + radius, cy_top + radius, outline="#b0b0b0", width=2)
        self.draw_arrow(self.top_canvas, (cx_top, cy_top), (cx_top + radius * 0.95, cy_top), "#cc4b37", 2)
        self.draw_arrow(self.top_canvas, (cx_top, cy_top), (cx_top, cy_top - radius * 0.95), "#2a7d46", 2)
        self.top_canvas.create_text(cx_top + radius + 18, cy_top, text="E", fill="#cc4b37", font=("Consolas", 12, "bold"))
        self.top_canvas.create_text(cx_top, cy_top - radius - 14, text="N", fill="#2a7d46", font=("Consolas", 12, "bold"))
        self.draw_arrow(
            self.top_canvas,
            (cx_top, cy_top),
            (cx_top + east * radius, cy_top - north * radius),
            "#1f5aa6",
            4,
        )
        self.top_canvas.create_text(
            cx_top,
            top_h - 20,
            text=f"East={east:.3f}  North={north:.3f}",
            fill="#2b2b2b",
            font=("Consolas", 11),
        )

        self.side_canvas.create_text(16, 16, text="侧视图: N-U 平面", anchor="w", fill="#2b2b2b", font=("Microsoft YaHei UI", 11, "bold"))
        self.side_canvas.create_line(30, cy_side, side_w - 30, cy_side, fill="#b0b0b0", width=2)
        self.side_canvas.create_line(cx_side, side_h - 30, cx_side, 30, fill="#b0b0b0", width=2)
        self.draw_arrow(self.side_canvas, (cx_side, cy_side), (cx_side, cy_side - radius * 0.95), "#d48b00", 2)
        self.draw_arrow(self.side_canvas, (cx_side, cy_side), (cx_side + radius * 0.95, cy_side), "#2a7d46", 2)
        self.side_canvas.create_text(cx_side, 18, text="U", fill="#d48b00", font=("Consolas", 12, "bold"))
        self.side_canvas.create_text(side_w - 18, cy_side, text="N", fill="#2a7d46", font=("Consolas", 12, "bold"))
        self.draw_arrow(
            self.side_canvas,
            (cx_side, cy_side),
            (cx_side + north * radius, cy_side - up * radius),
            "#7a3db8",
            4,
        )
        self.side_canvas.create_text(
            cx_side,
            side_h - 20,
            text=f"North={north:.3f}  Up={up:.3f}",
            fill="#2b2b2b",
            font=("Consolas", 11),
        )

    def append_log(self, message: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def emit(self, kind: str, payload: object) -> None:
        self.message_queue.put((kind, payload))

    def apply_serial_settings(self) -> tuple[str, int] | None:
        try:
            return self.port_var.get().strip() or DEFAULT_PORT, int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("波特率无效", "波特率必须是整数。")
            return None

    def worker_running(self) -> bool:
        return self.worker_thread is not None and self.worker_thread.is_alive()

    def reload_calibration(self) -> None:
        if not CALIBRATION_FILE.exists():
            messagebox.showerror("缺少标定文件", f"找不到 {CALIBRATION_FILE.resolve()}")
            return
        self.calibration_var.set(str(CALIBRATION_FILE.resolve()))
        self.append_log("已重新加载标定文件。")

    def start_stream(self) -> None:
        settings = self.apply_serial_settings()
        if settings is None:
            return
        if self.worker_running():
            messagebox.showinfo("任务进行中", "当前已经在读取数据了。")
            return

        if not CALIBRATION_FILE.exists():
            messagebox.showerror("缺少标定文件", f"找不到 {CALIBRATION_FILE.resolve()}")
            return

        self.stop_event.clear()
        self.status_var.set("连接中...")
        port, baudrate = settings
        self.append_log(f"开始读取姿态，串口 {port}，波特率 {baudrate}")
        self.worker_thread = threading.Thread(
            target=self.stream_worker,
            args=(port, baudrate),
            daemon=True,
        )
        self.worker_thread.start()

    def stop_stream(self) -> None:
        self.stop_event.set()
        if self.worker_running():
            self.status_var.set("停止中...")
            self.append_log("已请求停止。")
        else:
            self.status_var.set("就绪")

    def process_queue(self) -> None:
        while True:
            try:
                kind, payload = self.message_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "log":
                self.append_log(str(payload))
            elif kind == "status":
                self.status_var.set(str(payload))
            elif kind == "sample":
                raw_sample, corrected, orientation = payload
                self.raw_accel_var.set(f"({raw_sample.ax:.3f}, {raw_sample.ay:.3f}, {raw_sample.az:.3f})")
                self.raw_gyro_var.set(f"({raw_sample.gx:.3f}, {raw_sample.gy:.3f}, {raw_sample.gz:.3f})")
                self.raw_mag_var.set(f"({raw_sample.mx:.3f}, {raw_sample.my:.3f}, {raw_sample.mz:.3f})")
                self.corr_accel_var.set(f"({corrected.ax:.3f}, {corrected.ay:.3f}, {corrected.az:.3f})")
                self.corr_gyro_var.set(f"({corrected.gx:.3f}, {corrected.gy:.3f}, {corrected.gz:.3f})")
                self.corr_mag_var.set(f"({corrected.mx:.3f}, {corrected.my:.3f}, {corrected.mz:.3f})")
                self.roll_var.set(f"{orientation.roll_deg:.2f}°")
                self.pitch_var.set(f"{orientation.pitch_deg:.2f}°")
                self.yaw_var.set(f"{orientation.yaw_deg:.2f}°")
                self.forward_var.set(
                    f"({orientation.forward_x:.3f}, {orientation.forward_y:.3f}, {orientation.forward_z:.3f})"
                )
                self.draw_enu_canvases(
                    orientation.forward_x,
                    orientation.forward_y,
                    orientation.forward_z,
                )
            elif kind == "error":
                self.status_var.set("错误")
                messagebox.showerror("运行错误", str(payload))

        self.root.after(100, self.process_queue)

    def stream_worker(self, port: str, baudrate: int) -> None:
        calibration = load_calibration(CALIBRATION_FILE)
        filter_state = OrientationFilter()
        fallback_start = time.monotonic()

        try:
            with serial.Serial(port, baudrate, timeout=READ_TIMEOUT) as ser:
                self.emit("status", f"已连接 {port}")
                while not self.stop_event.is_set():
                    try:
                        sample = read_one_sample(ser)
                    except ValueError as exc:
                        self.emit("log", f"坏帧已跳过：{exc}")
                        continue

                    if sample is None:
                        continue

                    corrected = correct_sample(sample, calibration)
                    if sample.timestamp_ms is not None:
                        sample_time_s = sample.timestamp_ms / 1000.0
                    else:
                        sample_time_s = time.monotonic() - fallback_start

                    orientation = filter_state.update(corrected, sample_time_s)
                    self.emit("sample", (sample, corrected, orientation))
        except serial.SerialException as exc:
            self.emit("error", f"无法打开 {port}：{exc}")
        finally:
            self.emit("status", "就绪")

    def on_close(self) -> None:
        self.stop_stream()
        self.root.after(200, self.root.destroy)


def main() -> None:
    root = tk.Tk()
    style = ttk.Style()
    if "clam" in style.theme_names():
        style.theme_use("clam")
    app = OrientationGuiApp(root)
    app.append_log("姿态方向查看器已启动。")
    root.mainloop()


if __name__ == "__main__":
    main()
