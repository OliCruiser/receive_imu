import json
import math
import os
import struct
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import mean, pstdev
from typing import Callable, Optional

import serial


FRAME_HEADER = b"\xAA\x55"
FRAME_TAIL = b"\x55\xAA"
PORT = "COM19"
BAUDRATE = 115200
READ_TIMEOUT = 1.0
CALIBRATION_SAMPLES = 150
FACE_SAMPLES = 120
MAG_CALIBRATION_SAMPLES = 600
CALIBRATION_FILE = Path("imu_calibration.json")
DEFAULT_MODE = "stream"
FACE_ORDER = [
    ("+X", "将 +X 轴朝上放置"),
    ("-X", "将 -X 轴朝上放置"),
    ("+Y", "将 +Y 轴朝上放置"),
    ("-Y", "将 -Y 轴朝上放置"),
    ("+Z", "将 +Z 轴朝上放置"),
    ("-Z", "将 -Z 轴朝上放置"),
]


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
class CalibrationResult:
    port: str
    baudrate: int
    sample_count: int
    created_at: str
    gyro_bias: dict[str, float]
    gyro_std: dict[str, float]
    accel_mean: dict[str, float]
    accel_offset_horizontal: dict[str, float]
    accel_std: dict[str, float]
    accel_norm_mean: float
    mag_mean: dict[str, float]
    mag_std: dict[str, float]
    mag_norm_mean: float
    east_reference_heading_deg: float
    notes: list[str]
    accel_bias: Optional[dict[str, float]] = None
    accel_scale: Optional[dict[str, float]] = None
    accel_six_face: Optional[dict[str, dict[str, float]]] = None
    mag_bias: Optional[dict[str, float]] = None
    mag_scale: Optional[dict[str, float]] = None
    mag_min: Optional[dict[str, float]] = None
    mag_max: Optional[dict[str, float]] = None


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
class FaceValidationResult:
    ok: bool
    message: str


def calc_xor_checksum(data: bytes) -> int:
    chk = 0
    for byte in data:
        chk ^= byte
    return chk


def parse_frame(frame: bytes) -> tuple[int, str]:
    min_len = 2 + 1 + 4 + 1 + 2
    if len(frame) < min_len:
        raise ValueError("frame too short")

    if not frame.startswith(FRAME_HEADER):
        raise ValueError("invalid header")

    if not frame.endswith(FRAME_TAIL):
        raise ValueError("invalid tail")

    payload_len = frame[2]
    expected_len = 2 + 1 + 4 + payload_len + 1 + 2
    if len(frame) != expected_len:
        raise ValueError(
            f"length mismatch: expected {expected_len}, got {len(frame)}"
        )

    frame_id = struct.unpack("<I", frame[3:7])[0]
    payload = frame[7:7 + payload_len]
    recv_chk = frame[7 + payload_len]
    calc_chk = calc_xor_checksum(payload)
    if recv_chk != calc_chk:
        raise ValueError(
            f"checksum mismatch: expected 0x{calc_chk:02X}, got 0x{recv_chk:02X}"
        )

    try:
        payload_text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("payload is not valid UTF-8 text") from exc

    return frame_id, payload_text


def parse_payload(frame_id: int, payload_text: str) -> SensorSample:
    parts = payload_text.strip().split(",")
    if len(parts) not in (10, 11):
        raise ValueError(f"expected 10 or 11 CSV fields, got {len(parts)}")

    values = [float(part) for part in parts]
    payload_frame_id = int(values[0])
    timestamp_ms: Optional[int] = None

    if len(parts) == 11:
        timestamp_ms = int(values[1])
        base = 2
    else:
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
    buffer = bytearray(getattr(ser, "_imu_rx_buffer", b""))
    while True:
        if len(buffer) < 3:
            chunk = ser.read(max(1, ser.in_waiting or 1))
            if not chunk:
                setattr(ser, "_imu_rx_buffer", bytes(buffer))
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
                setattr(ser, "_imu_rx_buffer", bytes(buffer))
                return None
            buffer.extend(chunk)

        frame = bytes(buffer[:frame_len])
        del buffer[:frame_len]
        setattr(ser, "_imu_rx_buffer", bytes(buffer))
        return frame


def read_one_sample(ser: serial.Serial) -> Optional[SensorSample]:
    frame = read_one_frame(ser)
    if frame is None:
        return None

    frame_id, payload_text = parse_frame(frame)
    sample = parse_payload(frame_id, payload_text)
    return sample


def collect_samples(
    ser: serial.Serial,
    sample_count: int,
    progress_callback: Optional[Callable[[int, int], None]] = None,
    error_callback: Optional[Callable[[str], None]] = None,
) -> list[SensorSample]:
    samples: list[SensorSample] = []
    started_at = time.monotonic()
    timeout_seconds = max(20.0, sample_count * 1.5)

    while len(samples) < sample_count:
        if time.monotonic() - started_at > timeout_seconds:
            raise TimeoutError(
                f"timed out after {timeout_seconds:.1f}s with {len(samples)} samples"
            )

        try:
            sample = read_one_sample(ser)
        except ValueError as exc:
            message = f"Bad frame skipped: {exc}"
            print(message, file=sys.stderr)
            if error_callback is not None:
                error_callback(message)
            continue

        if sample is None:
            continue

        samples.append(sample)
        if progress_callback is not None:
            progress_callback(len(samples), sample_count)
        if len(samples) % 25 == 0 or len(samples) == sample_count:
            print(f"Collected {len(samples)}/{sample_count} samples...")

    return samples


def validate_face_samples(
    face_name: str,
    avg: dict[str, float],
    expected_threshold: float = 0.75,
    cross_axis_limit: float = 0.35,
) -> FaceValidationResult:
    axis_name = face_name[1].lower()
    sign = 1.0 if face_name.startswith("+") else -1.0
    expected_value = avg[f"a{axis_name}"]
    other_axes = [axis for axis in ("x", "y", "z") if axis != axis_name]
    other_values = [abs(avg[f"a{axis}"]) for axis in other_axes]

    if sign * expected_value < expected_threshold:
        return FaceValidationResult(
            ok=False,
            message=(
                f"{face_name} 面主轴加速度不够像该姿态："
                f"a{axis_name}={expected_value:.4f}"
            ),
        )

    if max(other_values) > cross_axis_limit:
        return FaceValidationResult(
            ok=False,
            message=(
                f"{face_name} 面串轴过大："
                f"{other_axes[0]}={other_values[0]:.4f}, {other_axes[1]}={other_values[1]:.4f}"
            ),
        )

    gyro_norm = math.sqrt(avg["gx"] ** 2 + avg["gy"] ** 2 + avg["gz"] ** 2)
    if gyro_norm > 0.25:
        return FaceValidationResult(
            ok=False,
            message=f"{face_name} 面采样时设备疑似仍在转动：gyro_norm={gyro_norm:.4f}",
        )

    return FaceValidationResult(ok=True, message=f"{face_name} 面姿态检查通过")


def solve_magnetometer_calibration(
    samples: list[SensorSample],
) -> tuple[dict[str, float], dict[str, float], dict[str, float], dict[str, float]]:
    min_values = {
        "x": min(sample.mx for sample in samples),
        "y": min(sample.my for sample in samples),
        "z": min(sample.mz for sample in samples),
    }
    max_values = {
        "x": max(sample.mx for sample in samples),
        "y": max(sample.my for sample in samples),
        "z": max(sample.mz for sample in samples),
    }

    bias = {
        axis: (max_values[axis] + min_values[axis]) / 2.0
        for axis in ("x", "y", "z")
    }
    radius = {
        axis: (max_values[axis] - min_values[axis]) / 2.0
        for axis in ("x", "y", "z")
    }
    if any(value <= 1e-6 for value in radius.values()):
        raise ValueError("磁力计标定失败：某一轴的采样范围过小，请转动得更充分。")

    average_radius = mean(radius.values())
    scale = {
        axis: average_radius / radius[axis]
        for axis in ("x", "y", "z")
    }
    return bias, scale, min_values, max_values


def mean_std(values: list[float]) -> tuple[float, float]:
    avg = mean(values)
    std = pstdev(values) if len(values) > 1 else 0.0
    return avg, std


def build_calibration(samples: list[SensorSample]) -> CalibrationResult:
    ax_values = [sample.ax for sample in samples]
    ay_values = [sample.ay for sample in samples]
    az_values = [sample.az for sample in samples]
    gx_values = [sample.gx for sample in samples]
    gy_values = [sample.gy for sample in samples]
    gz_values = [sample.gz for sample in samples]
    mx_values = [sample.mx for sample in samples]
    my_values = [sample.my for sample in samples]
    mz_values = [sample.mz for sample in samples]

    ax_mean, ax_std = mean_std(ax_values)
    ay_mean, ay_std = mean_std(ay_values)
    az_mean, az_std = mean_std(az_values)
    gx_mean, gx_std = mean_std(gx_values)
    gy_mean, gy_std = mean_std(gy_values)
    gz_mean, gz_std = mean_std(gz_values)
    mx_mean, mx_std = mean_std(mx_values)
    my_mean, my_std = mean_std(my_values)
    mz_mean, mz_std = mean_std(mz_values)

    accel_norm_mean = mean(
        [math.sqrt(sample.ax**2 + sample.ay**2 + sample.az**2) for sample in samples]
    )
    mag_norm_mean = mean(
        [math.sqrt(sample.mx**2 + sample.my**2 + sample.mz**2) for sample in samples]
    )

    expected_az = 1.0 if az_mean >= 0 else -1.0
    accel_offset_horizontal = {
        "x": ax_mean,
        "y": ay_mean,
        "z": az_mean - expected_az,
    }

    east_reference_heading_deg = math.degrees(math.atan2(my_mean, mx_mean))
    notes = [
        "Gyroscope bias is computed from a stationary average on the current horizontal pose.",
        "Accelerometer offset assumes the board is stationary on a horizontal plane.",
        "Magnetometer result is a single-pose east-facing reference, not a full hard/soft-iron calibration.",
        "For full 9-axis calibration, add six-face accelerometer calibration and multi-orientation magnetometer calibration later.",
    ]

    return CalibrationResult(
        port=PORT,
        baudrate=BAUDRATE,
        sample_count=len(samples),
        created_at=time.strftime("%Y-%m-%d %H:%M:%S"),
        gyro_bias={"x": gx_mean, "y": gy_mean, "z": gz_mean},
        gyro_std={"x": gx_std, "y": gy_std, "z": gz_std},
        accel_mean={"x": ax_mean, "y": ay_mean, "z": az_mean},
        accel_offset_horizontal=accel_offset_horizontal,
        accel_std={"x": ax_std, "y": ay_std, "z": az_std},
        accel_norm_mean=accel_norm_mean,
        mag_mean={"x": mx_mean, "y": my_mean, "z": mz_mean},
        mag_std={"x": mx_std, "y": my_std, "z": mz_std},
        mag_norm_mean=mag_norm_mean,
        east_reference_heading_deg=east_reference_heading_deg,
        notes=notes,
    )


def save_calibration(result: CalibrationResult, output_path: Path) -> None:
    output_path.write_text(
        json.dumps(asdict(result), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def load_calibration(path: Path) -> CalibrationResult:
    raw = json.loads(path.read_text(encoding="utf-8"))
    return CalibrationResult(**raw)


def correct_sample(
    sample: SensorSample, calibration: CalibrationResult
) -> CorrectedSample:
    gyro_bias = calibration.gyro_bias
    accel_bias = calibration.accel_bias or calibration.accel_offset_horizontal
    accel_scale = calibration.accel_scale or {"x": 1.0, "y": 1.0, "z": 1.0}
    mag_bias = calibration.mag_bias or {"x": 0.0, "y": 0.0, "z": 0.0}
    mag_scale = calibration.mag_scale or {"x": 1.0, "y": 1.0, "z": 1.0}

    return CorrectedSample(
        ax=(sample.ax - accel_bias["x"]) * accel_scale["x"],
        ay=(sample.ay - accel_bias["y"]) * accel_scale["y"],
        az=(sample.az - accel_bias["z"]) * accel_scale["z"],
        gx=sample.gx - gyro_bias["x"],
        gy=sample.gy - gyro_bias["y"],
        gz=sample.gz - gyro_bias["z"],
        mx=(sample.mx - mag_bias["x"]) * mag_scale["x"],
        my=(sample.my - mag_bias["y"]) * mag_scale["y"],
        mz=(sample.mz - mag_bias["z"]) * mag_scale["z"],
    )


def format_sample_line(sample: SensorSample, corrected: CorrectedSample) -> str:
    timestamp = (
        str(sample.timestamp_ms) if sample.timestamp_ms is not None else "None"
    )
    return (
        f"frame={sample.frame_id} payload_frame={sample.payload_frame_id} "
        f"timestamp_ms={timestamp} "
        f"raw_accel=({sample.ax:.4f},{sample.ay:.4f},{sample.az:.4f}) "
        f"corr_accel=({corrected.ax:.4f},{corrected.ay:.4f},{corrected.az:.4f}) "
        f"raw_gyro=({sample.gx:.4f},{sample.gy:.4f},{sample.gz:.4f}) "
        f"corr_gyro=({corrected.gx:.4f},{corrected.gy:.4f},{corrected.gz:.4f}) "
        f"mag=({sample.mx:.2f},{sample.my:.2f},{sample.mz:.2f})"
    )


def safe_print(line: str) -> None:
    try:
        print(line)
    except OSError:
        raise KeyboardInterrupt


def print_summary(result: CalibrationResult) -> None:
    print()
    print("Calibration finished.")
    print(
        "Gyro bias:"
        f" x={result.gyro_bias['x']:.6f}"
        f" y={result.gyro_bias['y']:.6f}"
        f" z={result.gyro_bias['z']:.6f}"
    )
    print(
        "Gyro std :"
        f" x={result.gyro_std['x']:.6f}"
        f" y={result.gyro_std['y']:.6f}"
        f" z={result.gyro_std['z']:.6f}"
    )
    print(
        "Accel mean:"
        f" x={result.accel_mean['x']:.6f}"
        f" y={result.accel_mean['y']:.6f}"
        f" z={result.accel_mean['z']:.6f}"
    )
    print(
        "Accel horizontal offset:"
        f" x={result.accel_offset_horizontal['x']:.6f}"
        f" y={result.accel_offset_horizontal['y']:.6f}"
        f" z={result.accel_offset_horizontal['z']:.6f}"
    )
    if result.accel_bias and result.accel_scale:
        print(
            "Accel 6-face bias:"
            f" x={result.accel_bias['x']:.6f}"
            f" y={result.accel_bias['y']:.6f}"
            f" z={result.accel_bias['z']:.6f}"
        )
        print(
            "Accel 6-face scale:"
            f" x={result.accel_scale['x']:.6f}"
            f" y={result.accel_scale['y']:.6f}"
            f" z={result.accel_scale['z']:.6f}"
        )
    print(f"Accel norm mean: {result.accel_norm_mean:.6f}")
    print(
        "Mag mean:"
        f" x={result.mag_mean['x']:.6f}"
        f" y={result.mag_mean['y']:.6f}"
        f" z={result.mag_mean['z']:.6f}"
    )
    print(
        "East reference heading from magnetometer:"
        f" {result.east_reference_heading_deg:.3f} deg"
    )
    print(f"Saved to {CALIBRATION_FILE.resolve()}")


def average_sample(samples: list[SensorSample]) -> dict[str, float]:
    return {
        "ax": mean([sample.ax for sample in samples]),
        "ay": mean([sample.ay for sample in samples]),
        "az": mean([sample.az for sample in samples]),
        "gx": mean([sample.gx for sample in samples]),
        "gy": mean([sample.gy for sample in samples]),
        "gz": mean([sample.gz for sample in samples]),
        "mx": mean([sample.mx for sample in samples]),
        "my": mean([sample.my for sample in samples]),
        "mz": mean([sample.mz for sample in samples]),
    }


def collect_face_average(
    ser: serial.Serial, face_name: str, instruction: str
) -> dict[str, float]:
    print()
    print(f"[{face_name}] {instruction}")
    input("Press Enter when the IMU is stable on this face...")
    print("Settling for 2 seconds...")
    time.sleep(2.0)
    print(f"Collecting {FACE_SAMPLES} samples for {face_name}...")
    samples = collect_samples(ser, FACE_SAMPLES)
    avg = average_sample(samples)
    print(
        f"{face_name} mean accel="
        f"({avg['ax']:.5f}, {avg['ay']:.5f}, {avg['az']:.5f})"
    )
    return avg


def solve_accel_six_face(
    face_means: dict[str, dict[str, float]]
) -> tuple[dict[str, float], dict[str, float]]:
    x_pos = face_means["+X"]["ax"]
    x_neg = face_means["-X"]["ax"]
    y_pos = face_means["+Y"]["ay"]
    y_neg = face_means["-Y"]["ay"]
    z_pos = face_means["+Z"]["az"]
    z_neg = face_means["-Z"]["az"]

    bias = {
        "x": (x_pos + x_neg) / 2.0,
        "y": (y_pos + y_neg) / 2.0,
        "z": (z_pos + z_neg) / 2.0,
    }

    half_range = {
        "x": (x_pos - x_neg) / 2.0,
        "y": (y_pos - y_neg) / 2.0,
        "z": (z_pos - z_neg) / 2.0,
    }

    if any(abs(value) < 1e-9 for value in half_range.values()):
        raise ValueError("invalid six-face data: one axis half-range is too small")

    scale = {
        axis: 1.0 / half_range[axis]
        for axis in ("x", "y", "z")
    }
    return bias, scale


def run_accel_six_face_calibration() -> int:
    print(
        f"Connecting to {PORT} @ {BAUDRATE} for six-face accelerometer calibration."
    )
    print("Follow the prompts and keep the IMU still on each face during sampling.")

    face_means: dict[str, dict[str, float]] = {}
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=READ_TIMEOUT) as ser:
            print("Waiting for valid frames...")
            for face_name, instruction in FACE_ORDER:
                face_means[face_name] = collect_face_average(ser, face_name, instruction)
    except serial.SerialException as exc:
        print(f"Failed to open {PORT}: {exc}", file=sys.stderr)
        return 1
    except TimeoutError as exc:
        print(f"Six-face calibration failed: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0

    try:
        bias, scale = solve_accel_six_face(face_means)
    except ValueError as exc:
        print(f"Six-face calibration failed: {exc}", file=sys.stderr)
        return 1

    if CALIBRATION_FILE.exists():
        calibration = load_calibration(CALIBRATION_FILE)
    else:
        calibration = CalibrationResult(
            port=PORT,
            baudrate=BAUDRATE,
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

    six_face_note = (
        "Accelerometer six-face calibration computed bias and per-axis scale from +/-X, +/-Y, +/-Z static poses."
    )
    if six_face_note not in calibration.notes:
        calibration.notes.append(six_face_note)

    save_calibration(calibration, CALIBRATION_FILE)

    print()
    print("Six-face accelerometer calibration finished.")
    print(
        f"Accel bias : x={bias['x']:.6f} y={bias['y']:.6f} z={bias['z']:.6f}"
    )
    print(
        f"Accel scale: x={scale['x']:.6f} y={scale['y']:.6f} z={scale['z']:.6f}"
    )
    print(f"Saved to {CALIBRATION_FILE.resolve()}")
    return 0


def run_calibration() -> int:
    print(
        f"Connecting to {PORT} @ {BAUDRATE}. "
        f"Keep the IMU stationary on a horizontal plane with +Y facing east."
    )

    try:
        with serial.Serial(PORT, BAUDRATE, timeout=READ_TIMEOUT) as ser:
            print("Waiting for valid frames...")
            samples = collect_samples(ser, CALIBRATION_SAMPLES)
    except serial.SerialException as exc:
        print(f"Failed to open {PORT}: {exc}", file=sys.stderr)
        return 1
    except TimeoutError as exc:
        print(f"Calibration failed: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0

    result = build_calibration(samples)
    save_calibration(result, CALIBRATION_FILE)
    print_summary(result)
    return 0


def run_stream() -> int:
    if not CALIBRATION_FILE.exists():
        print(
            f"Calibration file not found: {CALIBRATION_FILE.resolve()}",
            file=sys.stderr,
        )
        print("Run with MODE=calibrate first.", file=sys.stderr)
        return 1

    calibration = load_calibration(CALIBRATION_FILE)
    print(
        f"Streaming corrected IMU data from {PORT} @ {BAUDRATE} "
        f"using {CALIBRATION_FILE.resolve()}"
    )

    try:
        with serial.Serial(PORT, BAUDRATE, timeout=READ_TIMEOUT) as ser:
            while True:
                try:
                    sample = read_one_sample(ser)
                except ValueError as exc:
                    print(f"Bad frame skipped: {exc}", file=sys.stderr)
                    continue

                if sample is None:
                    continue

                corrected = correct_sample(sample, calibration)
                safe_print(format_sample_line(sample, corrected))
    except serial.SerialException as exc:
        print(f"Failed to open {PORT}: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


def main() -> int:
    mode = os.getenv("MODE", DEFAULT_MODE).strip().lower()
    if mode == "calibrate":
        return run_calibration()
    if mode == "accel6":
        return run_accel_six_face_calibration()
    if mode == "stream":
        return run_stream()

    print(
        f"Unsupported MODE={mode!r}. Use 'stream', 'calibrate', or 'accel6'.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
