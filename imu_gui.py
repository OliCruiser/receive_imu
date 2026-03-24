import json
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk

import serial

import receive


ACCEL6_PROGRESS_FILE = Path("accel6_progress.json")


class ImuGuiApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("IMU 标定工具")
        self.root.geometry("980x720")

        self.message_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.worker_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.face_ready_event = threading.Event()

        self.current_face_name: str | None = None
        self.face_prompt_active = False

        self.port_var = tk.StringVar(value=receive.PORT)
        self.baud_var = tk.StringVar(value=str(receive.BAUDRATE))
        self.mode_var = tk.StringVar(value="空闲")
        self.status_var = tk.StringVar(value="就绪")
        self.progress_var = tk.StringVar(value="当前没有采样任务")

        self.raw_accel_var = tk.StringVar(value="(0.0000, 0.0000, 0.0000)")
        self.corr_accel_var = tk.StringVar(value="(0.0000, 0.0000, 0.0000)")
        self.raw_gyro_var = tk.StringVar(value="(0.0000, 0.0000, 0.0000)")
        self.corr_gyro_var = tk.StringVar(value="(0.0000, 0.0000, 0.0000)")
        self.mag_var = tk.StringVar(value="(0.00, 0.00, 0.00)")
        self.face_name_var = tk.StringVar(value="当前没有等待的面")
        self.face_instruction_var = tk.StringVar(
            value="开始六面标定后，这里会显示当前该摆放的方向。"
        )

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
        ttk.Entry(control, textvariable=self.port_var, width=10).grid(
            row=0, column=1, padx=(6, 12), sticky="w"
        )

        ttk.Label(control, text="波特率").grid(row=0, column=2, sticky="w")
        ttk.Entry(control, textvariable=self.baud_var, width=10).grid(
            row=0, column=3, padx=(6, 12), sticky="w"
        )

        ttk.Button(control, text="开始接收", command=self.start_stream).grid(
            row=0, column=4, padx=4
        )
        ttk.Button(control, text="静止标定", command=self.start_static_calibration).grid(
            row=0, column=5, padx=4
        )
        ttk.Button(control, text="加速度计六面标定", command=self.start_accel6).grid(
            row=0, column=6, padx=4
        )
        ttk.Button(control, text="磁力计标定", command=self.start_mag_calibration).grid(
            row=0, column=7, padx=4
        )
        ttk.Button(control, text="停止", command=self.stop_worker).grid(
            row=0, column=8, padx=4, sticky="e"
        )

        status = ttk.Frame(self.root, padding=(12, 0, 12, 12))
        status.grid(row=1, column=0, sticky="ew")
        status.columnconfigure(1, weight=1)
        status.columnconfigure(3, weight=1)

        ttk.Label(status, text="模式").grid(row=0, column=0, sticky="w")
        ttk.Label(status, textvariable=self.mode_var).grid(row=0, column=1, sticky="w")
        ttk.Label(status, text="状态").grid(row=0, column=2, sticky="w", padx=(20, 0))
        ttk.Label(status, textvariable=self.status_var).grid(row=0, column=3, sticky="w")

        ttk.Label(status, text="进度").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(status, textvariable=self.progress_var).grid(
            row=1, column=1, columnspan=3, sticky="w", pady=(8, 0)
        )

        data_panel = ttk.LabelFrame(self.root, text="实时数据", padding=12)
        data_panel.grid(row=2, column=0, sticky="nsew", padx=12, pady=(0, 12))
        data_panel.columnconfigure(1, weight=1)
        data_panel.columnconfigure(3, weight=1)

        ttk.Label(data_panel, text="原始加速度").grid(row=0, column=0, sticky="w")
        ttk.Label(data_panel, textvariable=self.raw_accel_var).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(data_panel, text="校正后加速度").grid(
            row=0, column=2, sticky="w", padx=(20, 0)
        )
        ttk.Label(data_panel, textvariable=self.corr_accel_var).grid(
            row=0, column=3, sticky="w"
        )

        ttk.Label(data_panel, text="原始角速度").grid(row=1, column=0, sticky="w")
        ttk.Label(data_panel, textvariable=self.raw_gyro_var).grid(
            row=1, column=1, sticky="w"
        )
        ttk.Label(data_panel, text="校正后角速度").grid(
            row=1, column=2, sticky="w", padx=(20, 0)
        )
        ttk.Label(data_panel, textvariable=self.corr_gyro_var).grid(
            row=1, column=3, sticky="w"
        )

        ttk.Label(data_panel, text="磁力计").grid(row=2, column=0, sticky="w")
        ttk.Label(data_panel, textvariable=self.mag_var).grid(row=2, column=1, sticky="w")

        face_frame = ttk.LabelFrame(data_panel, text="六面标定引导", padding=12)
        face_frame.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(12, 0))
        face_frame.columnconfigure(1, weight=1)

        ttk.Label(face_frame, text="当前面").grid(row=0, column=0, sticky="w")
        ttk.Label(face_frame, textvariable=self.face_name_var).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(face_frame, text="操作提示").grid(
            row=1, column=0, sticky="w", pady=(8, 0)
        )
        ttk.Label(
            face_frame,
            textvariable=self.face_instruction_var,
            wraplength=760,
            justify="left",
        ).grid(row=1, column=1, sticky="w", pady=(8, 0))

        self.face_button = ttk.Button(
            face_frame,
            text="这一面已放稳",
            command=self.confirm_face_ready,
            state="disabled",
        )
        self.face_button.grid(row=2, column=1, sticky="w", pady=(12, 0))

        log_frame = ttk.LabelFrame(self.root, text="日志", padding=12)
        log_frame.grid(row=3, column=0, sticky="nsew", padx=12, pady=(0, 12))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(
            log_frame, wrap="word", height=16, font=("Consolas", 10)
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")
        self.log_text.configure(state="disabled")

    def append_log(self, message: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def emit(self, kind: str, payload: object) -> None:
        self.message_queue.put((kind, payload))

    def update_status(self, mode: str, status: str) -> None:
        self.mode_var.set(mode)
        self.status_var.set(status)

    def set_progress(self, text: str) -> None:
        self.progress_var.set(text)

    def apply_serial_settings(self) -> bool:
        try:
            receive.PORT = self.port_var.get().strip() or receive.PORT
            receive.BAUDRATE = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("波特率无效", "波特率必须是整数。")
            return False
        return True

    def worker_running(self) -> bool:
        return self.worker_thread is not None and self.worker_thread.is_alive()

    def start_stream(self) -> None:
        if self.apply_serial_settings():
            self.start_worker("接收模式", self.stream_worker)

    def start_static_calibration(self) -> None:
        if self.apply_serial_settings():
            self.start_worker("静止标定", self.static_calibration_worker)

    def start_accel6(self) -> None:
        if self.apply_serial_settings():
            self.start_worker("加速度计六面标定", self.accel6_worker)

    def start_mag_calibration(self) -> None:
        if self.apply_serial_settings():
            self.start_worker("磁力计标定", self.mag_calibration_worker)

    def start_worker(self, mode_name: str, target) -> None:
        if self.worker_running():
            messagebox.showinfo("任务进行中", "当前已有任务在运行，请先停止。")
            return

        self.stop_event.clear()
        self.face_ready_event.clear()
        self.face_prompt_active = False
        self.face_button.configure(state="disabled")
        self.face_name_var.set("当前没有等待的面")
        self.face_instruction_var.set("等待任务开始...")
        self.update_status(mode_name, "启动中...")
        self.set_progress("准备中")
        self.append_log(f"开始{mode_name}，串口 {receive.PORT}，波特率 {receive.BAUDRATE}")

        self.worker_thread = threading.Thread(target=target, daemon=True)
        self.worker_thread.start()

    def stop_worker(self) -> None:
        self.stop_event.set()
        self.face_ready_event.set()
        if self.worker_running():
            self.update_status(self.mode_var.get(), "停止中...")
            self.set_progress("正在停止任务")
            self.append_log("已请求停止当前任务。")
        else:
            self.update_status("空闲", "就绪")
            self.set_progress("当前没有采样任务")

    def confirm_face_ready(self) -> None:
        if not self.face_prompt_active:
            return
        self.face_prompt_active = False
        self.face_button.configure(state="disabled")
        self.face_instruction_var.set("已确认，正在等待稳定并开始采样。")
        self.append_log(f"{self.current_face_name} 已确认放稳。")
        self.face_ready_event.set()

    def process_queue(self) -> None:
        while True:
            try:
                kind, payload = self.message_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "log":
                self.append_log(str(payload))
            elif kind == "status":
                mode, status = payload
                self.update_status(mode, status)
            elif kind == "progress":
                self.set_progress(str(payload))
            elif kind == "sample":
                sample, corrected = payload
                self.raw_accel_var.set(f"({sample.ax:.4f}, {sample.ay:.4f}, {sample.az:.4f})")
                self.corr_accel_var.set(
                    f"({corrected.ax:.4f}, {corrected.ay:.4f}, {corrected.az:.4f})"
                )
                self.raw_gyro_var.set(f"({sample.gx:.4f}, {sample.gy:.4f}, {sample.gz:.4f})")
                self.corr_gyro_var.set(
                    f"({corrected.gx:.4f}, {corrected.gy:.4f}, {corrected.gz:.4f})"
                )
                self.mag_var.set(f"({sample.mx:.2f}, {sample.my:.2f}, {sample.mz:.2f})")
            elif kind == "face_prompt":
                face_name, instruction = payload
                self.current_face_name = face_name
                self.face_prompt_active = True
                self.face_ready_event.clear()
                self.face_name_var.set(face_name)
                self.face_instruction_var.set(
                    f"{instruction}。请把 IMU 摆到这一面并保持静止，然后点击“这一面已放稳”。"
                )
                self.face_button.configure(state="normal")
                self.set_progress(f"等待 {face_name} 摆放")
                self.append_log(f"等待摆放 {face_name}：{instruction}")
            elif kind == "face_done":
                face_name, avg = payload
                self.face_name_var.set(face_name)
                self.face_instruction_var.set("这一面已采样完成，正在进入下一面。")
                self.append_log(
                    f"{face_name} 平均加速度=({avg['ax']:.5f}, {avg['ay']:.5f}, {avg['az']:.5f})"
                )
            elif kind == "finished":
                mode_name, status = payload
                self.face_prompt_active = False
                self.face_button.configure(state="disabled")
                self.update_status(mode_name, status)
                if mode_name == "空闲":
                    self.set_progress("当前没有采样任务")
            elif kind == "error":
                self.face_prompt_active = False
                self.face_button.configure(state="disabled")
                self.update_status(self.mode_var.get(), "错误")
                self.set_progress("任务异常结束")
                messagebox.showerror("任务错误", str(payload))

        self.root.after(100, self.process_queue)

    def open_serial(self) -> serial.Serial:
        return serial.Serial(receive.PORT, receive.BAUDRATE, timeout=receive.READ_TIMEOUT)

    def load_calibration_safe(self) -> receive.CalibrationResult | None:
        if not receive.CALIBRATION_FILE.exists():
            self.emit("error", f"找不到标定文件：{Path(receive.CALIBRATION_FILE).resolve()}")
            return None
        return receive.load_calibration(receive.CALIBRATION_FILE)

    def load_accel6_progress(self) -> dict[str, dict[str, float]]:
        if not ACCEL6_PROGRESS_FILE.exists():
            return {}
        data = json.loads(ACCEL6_PROGRESS_FILE.read_text(encoding="utf-8"))
        return data.get("face_means", {})

    def save_accel6_progress(self, face_means: dict[str, dict[str, float]]) -> None:
        payload = {
            "port": receive.PORT,
            "baudrate": receive.BAUDRATE,
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "face_means": face_means,
        }
        ACCEL6_PROGRESS_FILE.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    def emit_bad_frame(self, message: str) -> None:
        self.emit("log", message.replace("Bad frame skipped:", "坏帧已跳过："))

    def clear_accel6_progress(self) -> None:
        if ACCEL6_PROGRESS_FILE.exists():
            ACCEL6_PROGRESS_FILE.unlink()

    def finalize_accel6(self, face_means: dict[str, dict[str, float]]) -> None:
        for face_name, _instruction in receive.FACE_ORDER:
            validation = receive.validate_face_samples(face_name, face_means[face_name])
            if not validation.ok:
                raise ValueError(
                    f"{face_name} 面历史数据不合格：{validation.message}"
                )

        bias, scale = receive.solve_accel_six_face(face_means)
        if receive.CALIBRATION_FILE.exists():
            calibration = receive.load_calibration(receive.CALIBRATION_FILE)
        else:
            calibration = receive.CalibrationResult(
                port=receive.PORT,
                baudrate=receive.BAUDRATE,
                sample_count=0,
                created_at=time.strftime("%Y-%m-%d %H:%M:%S"),
                gyro_bias={"x": 0.0, "y": 0.0, "z": 0.0},
                gyro_std={"x": 0.0, "y": 0.0, "z": 0.0},
                accel_mean={"x": 0.0, "y": 0.0, "z": 0.0},
                accel_offset_horizontal={"x": 0.0, "y": 0.0, "z": 0.0},
                accel_std={"x": 0.0, "y": 0.0, "z": 0.0},
                accel_norm_mean=0.0,
                mag_mean={"x": 0.0, "y": 0.0, "z": 0.0},
                mag_std={"x": 0.0, "y": 0.0, "z": 0.0},
                mag_norm_mean=0.0,
                east_reference_heading_deg=0.0,
                notes=[],
            )

        calibration.created_at = time.strftime("%Y-%m-%d %H:%M:%S")
        calibration.accel_bias = bias
        calibration.accel_scale = scale
        calibration.accel_six_face = face_means
        note = "Accelerometer six-face calibration computed bias and per-axis scale from +/-X, +/-Y, +/-Z static poses."
        if note not in calibration.notes:
            calibration.notes.append(note)
        receive.save_calibration(calibration, receive.CALIBRATION_FILE)
        self.clear_accel6_progress()

        self.emit(
            "log",
            f"加速度计 bias x={bias['x']:.6f} y={bias['y']:.6f} z={bias['z']:.6f}",
        )
        self.emit(
            "log",
            f"加速度计 scale x={scale['x']:.6f} y={scale['y']:.6f} z={scale['z']:.6f}",
        )
        self.emit("log", f"已保存到 {Path(receive.CALIBRATION_FILE).resolve()}")
        self.emit("progress", "六面标定完成")

    def stream_worker(self) -> None:
        calibration = self.load_calibration_safe()
        if calibration is None:
            self.emit("finished", ("空闲", "就绪"))
            return

        try:
            with self.open_serial() as ser:
                self.emit("status", ("接收模式", f"已连接 {receive.PORT}"))
                self.emit("progress", "正在持续接收数据")
                while not self.stop_event.is_set():
                    try:
                        sample = receive.read_one_sample(ser)
                    except ValueError as exc:
                        self.emit("log", f"坏帧已跳过：{exc}")
                        continue

                    if sample is None:
                        continue

                    corrected = receive.correct_sample(sample, calibration)
                    self.emit("sample", (sample, corrected))
        except serial.SerialException as exc:
            self.emit("error", f"无法打开 {receive.PORT}：{exc}")
        finally:
            self.emit("finished", ("空闲", "就绪"))

    def mag_calibration_worker(self) -> None:
        try:
            with self.open_serial() as ser:
                self.emit(
                    "status",
                    ("磁力计标定", "请手持模块缓慢转动，尽量覆盖各个方向..."),
                )
                self.emit(
                    "log",
                    "开始磁力计标定，请缓慢翻转和旋转模块，尽量让各个方向都经过一次。",
                )
                self.emit("progress", f"已采集 0/{receive.MAG_CALIBRATION_SAMPLES}")
                samples = receive.collect_samples(
                    ser,
                    receive.MAG_CALIBRATION_SAMPLES,
                    progress_callback=lambda current, total: self.emit(
                        "progress", f"已采集 {current}/{total}"
                    ),
                    error_callback=self.emit_bad_frame,
                )
                if self.stop_event.is_set():
                    self.emit("log", "磁力计标定已停止。")
                    return

                bias, scale, min_values, max_values = receive.solve_magnetometer_calibration(samples)
                if receive.CALIBRATION_FILE.exists():
                    calibration = receive.load_calibration(receive.CALIBRATION_FILE)
                else:
                    calibration = receive.CalibrationResult(
                        port=receive.PORT,
                        baudrate=receive.BAUDRATE,
                        sample_count=0,
                        created_at=time.strftime("%Y-%m-%d %H:%M:%S"),
                        gyro_bias={"x": 0.0, "y": 0.0, "z": 0.0},
                        gyro_std={"x": 0.0, "y": 0.0, "z": 0.0},
                        accel_mean={"x": 0.0, "y": 0.0, "z": 0.0},
                        accel_offset_horizontal={"x": 0.0, "y": 0.0, "z": 0.0},
                        accel_std={"x": 0.0, "y": 0.0, "z": 0.0},
                        accel_norm_mean=0.0,
                        mag_mean={"x": 0.0, "y": 0.0, "z": 0.0},
                        mag_std={"x": 0.0, "y": 0.0, "z": 0.0},
                        mag_norm_mean=0.0,
                        east_reference_heading_deg=0.0,
                        notes=[],
                    )

                calibration.created_at = time.strftime("%Y-%m-%d %H:%M:%S")
                calibration.mag_bias = bias
                calibration.mag_scale = scale
                calibration.mag_min = min_values
                calibration.mag_max = max_values
                note = "Magnetometer calibration computed hard-iron bias and per-axis scale from multi-orientation samples."
                if note not in calibration.notes:
                    calibration.notes.append(note)
                receive.save_calibration(calibration, receive.CALIBRATION_FILE)

                self.emit(
                    "log",
                    f"磁力计 bias x={bias['x']:.6f} y={bias['y']:.6f} z={bias['z']:.6f}",
                )
                self.emit(
                    "log",
                    f"磁力计 scale x={scale['x']:.6f} y={scale['y']:.6f} z={scale['z']:.6f}",
                )
                self.emit(
                    "log",
                    f"磁力计范围 min=({min_values['x']:.2f}, {min_values['y']:.2f}, {min_values['z']:.2f}) "
                    f"max=({max_values['x']:.2f}, {max_values['y']:.2f}, {max_values['z']:.2f})",
                )
                self.emit("log", f"已保存到 {Path(receive.CALIBRATION_FILE).resolve()}")
                self.emit("progress", "磁力计标定完成")
        except (serial.SerialException, TimeoutError, ValueError) as exc:
            self.emit("error", str(exc))
        finally:
            self.emit("finished", ("空闲", "就绪"))

    def static_calibration_worker(self) -> None:
        try:
            with self.open_serial() as ser:
                self.emit("status", ("静止标定", "正在采集静止数据..."))
                self.emit("progress", f"已采集 0/{receive.CALIBRATION_SAMPLES}")
                samples = receive.collect_samples(
                    ser,
                    receive.CALIBRATION_SAMPLES,
                    progress_callback=lambda current, total: self.emit(
                        "progress", f"已采集 {current}/{total}"
                    ),
                    error_callback=self.emit_bad_frame,
                )
                if self.stop_event.is_set():
                    self.emit("log", "静止标定已停止。")
                    return

                result = receive.build_calibration(samples)
                receive.save_calibration(result, receive.CALIBRATION_FILE)
                self.emit("log", "静止标定完成，结果已保存。")
                self.emit(
                    "log",
                    f"陀螺仪零偏 x={result.gyro_bias['x']:.6f} y={result.gyro_bias['y']:.6f} z={result.gyro_bias['z']:.6f}",
                )
                self.emit(
                    "log",
                    f"加速度计偏置 x={result.accel_offset_horizontal['x']:.6f} y={result.accel_offset_horizontal['y']:.6f} z={result.accel_offset_horizontal['z']:.6f}",
                )
                self.emit("progress", "静止标定完成")
        except (serial.SerialException, TimeoutError) as exc:
            self.emit("error", str(exc))
        finally:
            self.emit("finished", ("空闲", "就绪"))

    def wait_for_face_ready(self) -> bool:
        while not self.stop_event.is_set():
            if self.face_ready_event.wait(timeout=0.1):
                self.face_ready_event.clear()
                return True
        return False

    def accel6_worker(self) -> None:
        face_means = self.load_accel6_progress()
        completed_faces = [face for face, _ in receive.FACE_ORDER if face in face_means]
        if completed_faces:
            self.emit("log", f"检测到上次进度，将从未完成的面继续：{', '.join(completed_faces)} 已完成")

        try:
            with self.open_serial() as ser:
                self.emit("status", ("加速度计六面标定", f"已连接 {receive.PORT}"))

                for face_name, instruction in receive.FACE_ORDER:
                    if face_name in face_means:
                        self.emit("log", f"跳过 {face_name}，已存在历史采样结果。")
                        continue

                    if self.stop_event.is_set():
                        self.emit("log", "六面标定已停止。")
                        return

                    self.emit("face_prompt", (face_name, instruction))
                    if not self.wait_for_face_ready():
                        self.emit("log", "六面标定已停止。")
                        return

                    self.emit("log", f"{face_name} 正在等待稳定...")
                    self.emit("progress", f"{face_name} 稳定等待 2 秒")
                    deadline = time.time() + 2.0
                    while time.time() < deadline:
                        if self.stop_event.is_set():
                            self.emit("log", "六面标定已停止。")
                            return
                        time.sleep(0.1)

                    self.emit(
                        "status",
                        ("加速度计六面标定", f"正在为 {face_name} 采集 {receive.FACE_SAMPLES} 组数据"),
                    )
                    self.emit("progress", f"{face_name} 已采集 0/{receive.FACE_SAMPLES}")
                    samples = receive.collect_samples(
                        ser,
                        receive.FACE_SAMPLES,
                        progress_callback=lambda current, total, face=face_name: self.emit(
                            "progress", f"{face} 已采集 {current}/{total}"
                        ),
                        error_callback=self.emit_bad_frame,
                    )
                    avg = receive.average_sample(samples)
                    validation = receive.validate_face_samples(face_name, avg)
                    self.emit("log", validation.message)
                    if not validation.ok:
                        raise ValueError(
                            f"{face_name} 面采样不合格，请重新摆放后重试。"
                        )
                    face_means[face_name] = avg
                    self.save_accel6_progress(face_means)
                    self.emit("face_done", (face_name, avg))

                self.finalize_accel6(face_means)
        except (serial.SerialException, TimeoutError, ValueError) as exc:
            self.save_accel6_progress(face_means)
            self.emit(
                "log",
                f"当前六面进度已保存到 {ACCEL6_PROGRESS_FILE.resolve()}，下次可继续。",
            )
            self.emit("error", str(exc))
        finally:
            self.emit("finished", ("空闲", "就绪"))

    def on_close(self) -> None:
        self.stop_worker()
        self.root.after(200, self.root.destroy)


def main() -> None:
    root = tk.Tk()
    style = ttk.Style()
    if "clam" in style.theme_names():
        style.theme_use("clam")
    app = ImuGuiApp(root)
    app.append_log("界面已就绪，请选择一个模式开始。")
    root.mainloop()


if __name__ == "__main__":
    main()
