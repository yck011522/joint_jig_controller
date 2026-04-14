"""
jig_controller.py — Shared controller logic for the joint placement jig.

This module contains all hardware-interfacing, configuration parsing, and
assembly-workflow logic that is independent of the user interface.  Both the
CLI (controller_cli.py) and a future GUI can import from here.

Key responsibilities:
  - Parse settings.json into typed dataclasses.
  - Auto-detect or manually connect the Modbus RTU bus.
  - Instantiate DistanceSensor + ZDTEmmMotor with correct addresses/kinematics.
  - Reset the motor on startup (clear stall, disable, re-enable).
  - Provide a rotate-with-stall-detection routine that handles retries.
  - Logging helpers (JSONL event append, ISO timestamp).
  - Design file validation (schema, offset completeness).
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from pymodbus.client import ModbusSerialClient
import serial.tools.list_ports

from distance_sensor import DistanceSensor
from motor_comm import MotorCommError, ZDTEmmMotor, MotorKinematics


# ═══════════════════════════════════════════════════════════════
#  Small pure-Python utilities
# ═══════════════════════════════════════════════════════════════

def now_iso_local() -> str:
    """Return a local ISO-8601 timestamp with millisecond resolution."""
    t = time.time()
    lt = time.localtime(t)
    ms = int((t - int(t)) * 1000)
    return time.strftime("%Y-%m-%dT%H:%M:%S", lt) + f".{ms:03d}"


def load_json(path: Path) -> dict:
    """Read and parse a JSON file, returning a dict."""
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def safe_get(d: dict, keys: list[str], default=None):
    """Walk a nested dict by *keys*, returning *default* on any miss."""
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def append_event(log_path: Path, event_obj: dict) -> None:
    """Append a single JSON object as one line to the JSONL log file."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    line = json.dumps(event_obj, ensure_ascii=False)
    with log_path.open("a", encoding="utf-8") as f:
        f.write(line + "\n")
        f.flush()


def design_log_path(design_path: Path, settings: dict) -> Path:
    """Derive the log file path from the design JSON path and settings."""
    ext = safe_get(settings, ["logging", "log_extension"], default=".log.jsonl")
    return design_path.with_name(design_path.stem + ext)


# ═══════════════════════════════════════════════════════════════
#  Design-file helpers
# ═══════════════════════════════════════════════════════════════

def ensure_schema_version(design: dict, expected: int = 1) -> None:
    """Raise if the design file's schema_version doesn't match *expected*."""
    v = design.get("schema_version", None)
    if v != expected:
        raise RuntimeError(
            f"Unsupported design schema version: got {v}, expected {expected}."
        )


def build_offset_map(settings: dict) -> Dict[Tuple[str, str], float]:
    """Build a ``{(type, orientation): offset_mm}`` lookup from settings.

    Malformed entries are silently skipped.  If duplicates exist the last
    entry wins.
    """
    entries = safe_get(settings, ["linear", "joint_offsets"], default=[])
    offset_map: Dict[Tuple[str, str], float] = {}
    for entry in entries:
        t = str(entry.get("type", "")).strip()
        o = str(entry.get("ori", "")).strip()
        if t and o:
            offset_map[(t, o)] = float(entry.get("offset_mm", 0.0))
    return offset_map


def required_type_ori_pairs(tube: dict) -> set[Tuple[str, str]]:
    """Return the set of (type, orientation) pairs needed by *tube*."""
    pairs: set[Tuple[str, str]] = set()
    for joint in tube.get("joints", []) or []:
        t = str(joint.get("type", "")).strip()
        o = str(joint.get("ori", "")).strip()
        if t and o:
            pairs.add((t, o))
    return pairs


def validate_offsets_complete(
    tube: dict, offset_map: Dict[Tuple[str, str], float]
) -> None:
    """Raise ``RuntimeError`` if *tube* references joints without an offset."""
    missing = sorted(
        p for p in required_type_ori_pairs(tube) if p not in offset_map
    )
    if missing:
        msg = "Missing joint_offsets entries for (type, ori):\n" + "\n".join(
            f"  - {p}" for p in missing
        )
        raise RuntimeError(msg)


# ═══════════════════════════════════════════════════════════════
#  Typed settings
# ═══════════════════════════════════════════════════════════════

@dataclass
class SerialSettings:
    """Serial / Modbus connection parameters."""
    port: Optional[str]       # fixed COM port (None → auto-detect)
    baudrate: int = 115200
    timeout_s: float = 0.4
    parity: str = "N"
    stopbits: int = 1
    bytesize: int = 8


@dataclass
class DeviceAddresses:
    """Modbus slave addresses for the two devices on the bus."""
    sensor_addr: int = 1
    motor_addr: int = 2


@dataclass
class MotionSettings:
    """Kinematic and motion parameters."""
    gear_ratio: float = 30.0
    motor_steps_per_rev: int = 200
    microsteps: int = 16
    rot_speed_rpm: int = 300   # motor-side RPM for moves
    rot_acc: int = 100         # acceleration byte (0–255 scale)
    rot_tol_deg: float = 1.0   # tolerance for "reached" confirmation


@dataclass
class StallSettings:
    """Stall detection / protection thresholds."""
    enabled: bool = True
    speed_rpm: int = 100       # RPM below which stall is detected
    current_ma: int = 400      # current above which stall is detected
    time_ms: int = 50          # duration the condition must persist
    disabled_current_ma: int = 2000  # effectively disables detection


@dataclass
class LinearSettings:
    """Linear axis tolerances and offsets."""
    linear_tol_mm: float = 2.0
    sensor_global_offset_mm: float = 0.0


@dataclass
class JigSettings:
    """Top-level parsed settings for the entire jig controller."""
    serial: SerialSettings = field(default_factory=SerialSettings)
    devices: DeviceAddresses = field(default_factory=DeviceAddresses)
    motion: MotionSettings = field(default_factory=MotionSettings)
    stall: StallSettings = field(default_factory=StallSettings)
    linear: LinearSettings = field(default_factory=LinearSettings)
    probe_enabled: bool = True
    # raw settings dict kept around for anything not yet parsed
    _raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_dict(cls, d: dict) -> "JigSettings":
        """Construct a ``JigSettings`` from a parsed settings.json dict."""
        serial = SerialSettings(
            port=safe_get(d, ["serial", "port"]),
            baudrate=int(safe_get(d, ["serial", "baudrate"], default=115200)),
            timeout_s=float(safe_get(d, ["serial", "timeout_s"], default=0.4)),
            parity=str(safe_get(d, ["serial", "parity"], default="N")),
            stopbits=int(safe_get(d, ["serial", "stopbits"], default=1)),
            bytesize=int(safe_get(d, ["serial", "bytesize"], default=8)),
        )
        devices = DeviceAddresses(
            sensor_addr=int(safe_get(d, ["device_addresses", "sensor_addr"], default=1)),
            motor_addr=int(safe_get(d, ["device_addresses", "motor_addr"], default=2)),
        )
        motion = MotionSettings(
            gear_ratio=float(safe_get(d, ["motion", "gear_ratio"], default=30.0)),
            motor_steps_per_rev=int(safe_get(d, ["motion", "motor_steps_per_rev"], default=200)),
            microsteps=int(safe_get(d, ["motion", "microsteps"], default=16)),
            rot_speed_rpm=int(safe_get(d, ["motion", "rot_speed_rpm"], default=300)),
            rot_acc=int(safe_get(d, ["motion", "rot_acc"], default=100)),
            rot_tol_deg=float(safe_get(d, ["motion", "rot_tol_deg"], default=1.0)),
        )
        stall = StallSettings(
            enabled=bool(safe_get(d, ["stall", "enabled"], default=True)),
            speed_rpm=int(safe_get(d, ["stall", "speed_rpm"], default=100)),
            current_ma=int(safe_get(d, ["stall", "current_ma"], default=400)),
            time_ms=int(safe_get(d, ["stall", "time_ms"], default=50)),
            disabled_current_ma=int(safe_get(d, ["stall", "disabled_current_ma"], default=2000)),
        )
        linear = LinearSettings(
            linear_tol_mm=float(safe_get(d, ["linear", "linear_tol_mm"], default=2.0)),
            sensor_global_offset_mm=float(
                safe_get(d, ["linear", "sensor_global_offset_mm"], default=0.0)
            ),
        )
        return cls(
            serial=serial,
            devices=devices,
            motion=motion,
            stall=stall,
            linear=linear,
            probe_enabled=bool(safe_get(d, ["probe", "enabled"], default=True)),
            _raw=d,
        )


# ═══════════════════════════════════════════════════════════════
#  Hardware connection
# ═══════════════════════════════════════════════════════════════

@dataclass
class HardwareContext:
    """Holds the live hardware handles after a successful connection.

    Pass this around instead of loose (client, sensor, motor) tuples.
    """
    client: ModbusSerialClient
    sensor: DistanceSensor
    motor: ZDTEmmMotor
    port: str                # the COM port in use
    # Initial probe readings (may be None if probe was skipped)
    sensor_mm: Optional[int] = None
    motor_deg: Optional[float] = None

    def close(self) -> None:
        """Close the underlying Modbus client cleanly."""
        self.client.close()


def _list_available_ports() -> list:
    """Return serial port info objects sorted by device name."""
    return sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)


def _try_probe_port(
    port: str,
    cfg: JigSettings,
) -> Tuple[Optional[ModbusSerialClient], Optional[int], Optional[float], str]:
    """Open *port*, try to read sensor + motor.

    Returns ``(client, sensor_mm, motor_deg, status)`` where *status* is one
    of ``"both_ok"``, ``"sensor_only"``, ``"no_devices"``, ``"connect_fail"``.
    The *client* is returned open only on ``"both_ok"``; otherwise it's closed.
    """
    s = cfg.serial
    client = ModbusSerialClient(
        port=port, baudrate=s.baudrate, parity=s.parity,
        stopbits=s.stopbits, bytesize=s.bytesize, timeout=s.timeout_s,
    )
    if not client.connect():
        return None, None, None, "connect_fail"

    sensor = DistanceSensor(client, slave_id=cfg.devices.sensor_addr)
    motor = ZDTEmmMotor(client, slave_id=cfg.devices.motor_addr)

    sensor_mm: Optional[int] = None
    motor_deg: Optional[float] = None
    try:
        sensor_mm = sensor.read_mm()
    except Exception:
        pass
    try:
        motor_deg = motor.read_realtime_position_deg_output()
    except Exception:
        pass

    if sensor_mm is not None and motor_deg is not None:
        return client, sensor_mm, motor_deg, "both_ok"
    if sensor_mm is not None:
        client.close()
        return None, sensor_mm, None, "sensor_only"
    client.close()
    return None, None, None, "no_devices"


def connect_hardware(
    cfg: JigSettings,
    *,
    log: Callable[..., None] = print,
) -> HardwareContext:
    """Connect to the jig hardware, auto-detecting the COM port if enabled.

    Args:
        cfg: Parsed settings.
        log: A callable used for status messages (``print`` for CLI,
             could be a GUI status callback).

    Returns:
        A ``HardwareContext`` with a live client, sensor, motor, and the
        motor already reset (stall cleared, disabled, re-enabled).

    Raises:
        RuntimeError: if the hardware cannot be found or connected.
    """
    s = cfg.serial
    d = cfg.devices
    m = cfg.motion

    kinematics = MotorKinematics(
        motor_steps_per_rev=m.motor_steps_per_rev,
        microsteps=m.microsteps,
        gearbox_ratio=m.gear_ratio,
    )

    # ── Auto-detect or fixed connection ──────────────────────────
    if cfg.probe_enabled:
        log("[init] Auto-detecting COM port ...")
        hw = _auto_detect(cfg, log=log)
    else:
        if not s.port:
            raise RuntimeError(
                "Probe disabled but settings.serial.port is not set."
            )
        log(f"[init] Opening Modbus RTU on {s.port} @ {s.baudrate} ...")
        client = ModbusSerialClient(
            port=s.port, baudrate=s.baudrate, parity=s.parity,
            stopbits=s.stopbits, bytesize=s.bytesize, timeout=s.timeout_s,
        )
        if not client.connect():
            raise RuntimeError(
                f"Failed to connect on {s.port} @ {s.baudrate}"
            )
        sensor = DistanceSensor(client, slave_id=d.sensor_addr)
        motor = ZDTEmmMotor(client, slave_id=d.motor_addr, kinematics=kinematics)
        # Manual probe
        try:
            smm = sensor.read_mm()
            mdeg = motor.read_realtime_position_deg_output()
        except Exception as e:
            client.close()
            raise RuntimeError(f"Hardware probe failed: {e!r}")
        hw = HardwareContext(
            client=client, sensor=sensor, motor=motor,
            port=s.port, sensor_mm=smm, motor_deg=mdeg,
        )

    # Ensure kinematics are set (auto-detect creates motor with defaults)
    hw.motor.kin = kinematics

    log(f"[probe] Sensor OK: {hw.sensor_mm} mm | "
        f"Motor OK: {hw.motor_deg:.3f} deg (output)")

    # ── Reset motor state (clear leftover stall, cycle enable) ───
    log("[init] Resetting motor state ...")
    hw.motor.clear_stall_protection()
    time.sleep(0.3)
    hw.motor.set_enabled(False)
    time.sleep(0.3)
    hw.motor.set_enabled(True)
    time.sleep(0.3)
    log(f"[init] Motor reset complete. Enabled: {hw.motor.is_enabled()}")

    return hw


def _auto_detect(
    cfg: JigSettings,
    *,
    log: Callable[..., None] = print,
) -> HardwareContext:
    """Scan all COM ports for the jig hardware.  Raises on failure."""
    s = cfg.serial
    d = cfg.devices

    all_ports = _list_available_ports()
    if not all_ports:
        raise RuntimeError(
            "No COM ports found.\n"
            "  - Check USB-RS485 adapter is plugged in.\n"
            "  - On Windows: Device Manager → Ports (COM & LPT)."
        )

    log(f"[probe] Found {len(all_ports)} COM port(s): "
        + ", ".join(f"{p.device} ({p.description})" for p in all_ports))

    sensor_only_port: Optional[str] = None
    connect_fail_ports: list[str] = []

    for pi in all_ports:
        port = pi.device
        log(f"[probe] Trying {port} ...", end=" ", flush=True)

        client, smm, mdeg, status = _try_probe_port(port, cfg)

        if status == "both_ok":
            log(f"OK  (sensor={smm} mm, motor={mdeg:.3f} deg)")
            sensor_obj = DistanceSensor(client, slave_id=d.sensor_addr)
            motor_obj = ZDTEmmMotor(client, slave_id=d.motor_addr)
            return HardwareContext(
                client=client, sensor=sensor_obj, motor=motor_obj,
                port=port, sensor_mm=smm, motor_deg=mdeg,
            )
        elif status == "sensor_only":
            log(f"sensor OK ({smm} mm) but MOTOR NOT RESPONDING")
            sensor_only_port = port
        elif status == "connect_fail":
            log("cannot open (may be in use by another application)")
            connect_fail_ports.append(f"{port} ({pi.description or ''})")
        else:
            log("no Modbus devices responded")

    # Build helpful error message
    lines = ["Could not automatically detect the hardware."]
    if sensor_only_port:
        lines.append(
            f"\n  On {sensor_only_port} the distance sensor responded "
            f"but the motor did NOT."
        )
        lines.append("  - Check motor RS-485 wiring and power supply.")
        lines.append(f"  - Verify motor Modbus address (expected {d.motor_addr}).")
    if connect_fail_ports:
        lines.append("\n  Ports that could not be opened (possibly in use):")
        for p in connect_fail_ports:
            lines.append(f"    {p}")
        lines.append("  - Close other serial applications.")
    lines.append(f"\n  Baud: {s.baudrate}  "
                 f"Sensor addr: {d.sensor_addr}  Motor addr: {d.motor_addr}")
    raise RuntimeError("\n".join(lines))


# ═══════════════════════════════════════════════════════════════
#  Rotation with stall detection
# ═══════════════════════════════════════════════════════════════

@dataclass
class RotationResult:
    """Outcome of a single rotation command (possibly with stall retries)."""
    reached: bool                 # True if the motor reported position reached
    target_deg: float             # commanded target (output shaft)
    actual_deg: float             # readback after completion
    rotation_time_ms: int         # wall-clock time for the entire rotation step
    stall_events: List[dict]      # list of stall log-event dicts (empty if none)


def arm_stall_detection(motor: ZDTEmmMotor, stall: StallSettings) -> None:
    """Write the sensitive stall-detection thresholds so the motor will trip
    on a blocked shaft."""
    if not stall.enabled:
        return
    motor.write_stall_params(
        mode=0x01,
        speed_rpm=stall.speed_rpm,
        current_ma=stall.current_ma,
        time_ms=stall.time_ms,
        store=False,
    )


def disarm_stall_detection(motor: ZDTEmmMotor, stall: StallSettings) -> None:
    """Restore a high current threshold so stall detection won't trigger
    during holding or joint installation."""
    if not stall.enabled:
        return
    motor.write_stall_params(
        mode=0x01,
        speed_rpm=stall.speed_rpm,
        current_ma=stall.disabled_current_ma,
        time_ms=stall.time_ms,
        store=False,
    )


def _recover_from_stall(motor: ZDTEmmMotor) -> None:
    """Clear stall protection flags and re-enable the motor.

    Uses generous delays because the firmware needs time to reset its
    internal state before it will accept new motion commands.
    """
    motor.clear_stall_protection()
    time.sleep(0.3)
    motor.set_enabled(True)
    time.sleep(0.5)
    # Verify
    flags = motor.read_status_flags()
    if not (flags & motor.FLAG_ENABLED):
        # Retry once if the first attempt didn't take
        motor.clear_stall_protection()
        time.sleep(0.3)
        motor.set_enabled(True)
        time.sleep(0.5)


def _send_motion_with_retry(
    motor: ZDTEmmMotor,
    target_deg: float,
    speed_rpm: int,
    acc: int,
    max_attempts: int = 3,
) -> None:
    """Send an absolute-position command, retrying on transient Modbus errors.

    After stall recovery the motor may briefly reject writes (exception code 3).
    This wrapper retries up to *max_attempts* times with short delays.
    """
    for attempt in range(max_attempts):
        try:
            motor.move_absolute_deg_output(
                target_deg, speed_rpm=speed_rpm, acc=acc
            )
            return
        except Exception as e:
            if attempt < max_attempts - 1:
                time.sleep(0.5)
            else:
                raise


def rotate_with_stall_detection(
    motor: ZDTEmmMotor,
    target_deg: float,
    motion: MotionSettings,
    stall: StallSettings,
    *,
    on_stall: Optional[Callable[[float, float, int], None]] = None,
    timeout_s: float = 20.0,
) -> RotationResult:
    """Rotate the output shaft to *target_deg* with stall detection.

    This function:
      1. Arms stall detection (lowers the current threshold).
      2. Sends the motion command.
      3. Polls status flags until REACHED or STALL_PROTECTED.
      4. On stall: recovers, calls *on_stall* callback, and retries.
      5. After successful reach: disarms stall detection (raises threshold
         back to holding-safe level).

    Args:
        motor:      The motor handle.
        target_deg: Absolute output-shaft angle in degrees.
        motion:     Motion settings (speed, acc).
        stall:      Stall detection settings.
        on_stall:   Optional callback ``(actual_deg, target_deg, elapsed_ms)``
                    called each time a stall is detected.  The callback should
                    block until the operator is ready to retry (e.g. ``input()``
                    in CLI, or a dialog in GUI).
        timeout_s:  Per-attempt timeout for waiting on the reached flag.

    Returns:
        A ``RotationResult`` summarising the outcome.
    """
    rotation_start = time.perf_counter()
    stall_events: list[dict] = []

    # Arm stall detection
    arm_stall_detection(motor, stall)

    while True:
        attempt_start = time.perf_counter()

        # Send motion command (with retry on transient errors)
        _send_motion_with_retry(
            motor, target_deg, motion.rot_speed_rpm, motion.rot_acc
        )

        # Poll until reached or stall
        reached = False
        stall_occurred = False
        t0 = time.time()
        while True:
            flags = motor.read_status_flags()
            if flags & motor.FLAG_STALL_PROTECTED:
                stall_occurred = True
                break
            if flags & motor.FLAG_REACHED:
                reached = True
                break
            if (time.time() - t0) >= timeout_s:
                break
            time.sleep(motor.poll_period_s)

        if not stall_occurred:
            # Success (or timeout without stall)
            break

        # ── Stall detected ──────────────────────────────────────
        elapsed_ms = int((time.perf_counter() - attempt_start) * 1000)
        actual_deg = motor.read_realtime_position_deg_output()

        stall_events.append({
            "ts": now_iso_local(),
            "target_deg": target_deg,
            "actual_deg": actual_deg,
            "time_since_motion_start_ms": elapsed_ms,
        })

        # Recover motor
        _recover_from_stall(motor)

        # Notify the caller (blocking — e.g. operator presses Enter)
        if on_stall is not None:
            on_stall(actual_deg, target_deg, elapsed_ms)

        # Re-arm stall detection for the retry
        arm_stall_detection(motor, stall)

    # Disarm stall detection (restore high holding current)
    disarm_stall_detection(motor, stall)

    total_ms = int((time.perf_counter() - rotation_start) * 1000)
    actual_deg = motor.read_realtime_position_deg_output()

    return RotationResult(
        reached=reached,
        target_deg=target_deg,
        actual_deg=actual_deg,
        rotation_time_ms=total_ms,
        stall_events=stall_events,
    )
