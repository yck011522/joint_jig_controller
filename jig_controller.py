"""
jig_controller.py — Shared controller logic for the joint placement jig.

This module contains all hardware-interfacing, configuration parsing, and
assembly-workflow logic that is independent of the user interface.  Both the
CLI (controller_cli.py) and the GUI (controller_gui.py) import from here.

Key responsibilities:
  - Parse settings.json into typed dataclasses.
  - Auto-detect or manually connect the Modbus RTU bus.
  - Instantiate DistanceSensor + ZDTEmmMotor with correct addresses/kinematics.
  - Reset the motor on startup (clear stall, disable, re-enable).
  - Provide a rotate-with-stall-detection routine that handles retries.
  - Logging helpers (JSONL event append, ISO timestamp).
  - Design file validation (schema, offset completeness).
  - Log parsing: read past events to determine tube completion status.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from pymodbus.client import ModbusSerialClient
import serial.tools.list_ports

from distance_sensor import DistanceSensor
from motor_comm import MotorCommError, ZDTEmmMotor, MotorKinematics


# ═══════════════════════════════════════════════════════════════
#  Event name constants — used in both log writing and parsing
# ═══════════════════════════════════════════════════════════════

EVENT_TUBE_START = "tube_start"
EVENT_LINEAR_CONFIRM = "linear_confirm"
EVENT_ROTATION_REACHED = "rotation_reached"
EVENT_STALL_DETECTED = "stall_detected"
EVENT_INSTALL_CONFIRM = "install_confirm"
EVENT_TUBE_INSTALL_COMPLETE = "tube_install_complete"
EVENT_TUBE_ABANDONED = "tube_abandoned"


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
    """Derive the log file path from the design JSON path and settings.

    The log file always lives next to the design JSON file.
    """
    ext = safe_get(settings, ["logging", "log_extension"], default=".log.jsonl")
    return design_path.with_name(design_path.stem + ext)


# ═══════════════════════════════════════════════════════════════
#  Log parsing — read past events from a JSONL log file
# ═══════════════════════════════════════════════════════════════

def parse_log_events(log_path: Path) -> List[dict]:
    """Read all events from an existing JSONL log file.

    Returns an empty list if the file doesn't exist.  Malformed lines
    (non-JSON) are silently skipped.
    """
    if not log_path.exists():
        return []
    events: List[dict] = []
    with log_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue  # skip corrupted lines
    return events


@dataclass
class TubeCompletionInfo:
    """Summary of the most recent successful assembly of one tube."""
    tube_id: str
    completed_at: str          # ISO timestamp string
    tube_time_ms: int          # total assembly time in ms


def get_tube_completion_map(
    events: List[dict],
) -> Dict[str, TubeCompletionInfo]:
    """Scan log events and return the *last* completion record per tube_id.

    Looks for both the old ``"tube_complete"`` event name and the new
    ``"tube_install_complete"`` name so that existing logs still work.
    """
    completion_map: Dict[str, TubeCompletionInfo] = {}
    for ev in events:
        event_type = ev.get("event", "")
        # Accept both old and new event names
        if event_type not in (EVENT_TUBE_INSTALL_COMPLETE, "tube_complete"):
            continue
        tube_id = ev.get("tube_id", "")
        if not tube_id:
            continue
        completion_map[tube_id] = TubeCompletionInfo(
            tube_id=tube_id,
            completed_at=ev.get("ts", ""),
            tube_time_ms=int(ev.get("tube_time_ms", 0)),
        )
    return completion_map


def format_time_ago(iso_ts: str) -> str:
    """Format an ISO-8601 local timestamp as a human-friendly relative string.

    Examples:
      - "30s ago"
      - "5 min ago"
      - "2 hours ago"
      - "2026-04-14 16:19"  (if older than today)
    """
    try:
        # Parse the ISO timestamp (with optional milliseconds)
        fmt = "%Y-%m-%dT%H:%M:%S"
        ts_str = iso_ts[:19]  # trim milliseconds if present
        dt = datetime.strptime(ts_str, fmt)
    except (ValueError, IndexError):
        return iso_ts  # can't parse — return raw

    now = datetime.now()
    delta = now - dt

    total_seconds = int(delta.total_seconds())
    if total_seconds < 0:
        return iso_ts  # future timestamp — just show raw

    # If it's from a different calendar day, show full date
    if dt.date() != now.date():
        return dt.strftime("%Y-%m-%d %H:%M")

    # Same day — show relative
    if total_seconds < 60:
        return f"{total_seconds}s ago"
    minutes = total_seconds // 60
    if minutes < 60:
        return f"{minutes} min ago"
    hours = minutes // 60
    return f"{hours} hours ago"


def format_duration_ms(ms: int) -> str:
    """Format a duration in milliseconds as a concise human string.

    Examples: "45s", "2m 18s", "1h 5m"
    """
    seconds = ms // 1000
    if seconds < 60:
        return f"{seconds}s"
    minutes = seconds // 60
    secs = seconds % 60
    if minutes < 60:
        return f"{minutes}m {secs}s"
    hours = minutes // 60
    mins = minutes % 60
    return f"{hours}h {mins}m"


# ═══════════════════════════════════════════════════════════════
#  Settings persistence helpers
# ═══════════════════════════════════════════════════════════════

def save_settings(settings_path: Path, settings: dict) -> None:
    """Write settings dict back to disk (pretty-printed JSON)."""
    with settings_path.open("w", encoding="utf-8") as f:
        json.dump(settings, f, indent=4, ensure_ascii=False)
        f.write("\n")


def get_last_design_dir(settings: dict) -> Optional[str]:
    """Return the last used design file directory, or None."""
    return safe_get(settings, ["last_design_dir"])


def set_last_design_dir(settings: dict, dir_path: str) -> None:
    """Update the in-memory settings with the last used design directory."""
    settings["last_design_dir"] = dir_path


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


# ═══════════════════════════════════════════════════════════════
#  HardwareHub — centralised, thread-safe hardware interface
# ═══════════════════════════════════════════════════════════════

class HardwareHub:
    """Thread-safe hardware abstraction layer.

    A single daemon thread polls sensor + motor readings at a fixed
    interval and caches them.  Any thread may read cached values without
    blocking.  All *command* methods (move, zero, arm/disarm stall, etc.)
    acquire an internal lock so only one Modbus transaction is on the bus
    at a time.

    Usage::

        hub = HardwareHub(hw, poll_interval_s=0.5)
        # Read latest cached values (thread-safe, non-blocking)
        print(hub.sensor_mm, hub.motor_deg, hub.flags)
        # Send a motor command (acquires lock)
        hub.zero_here()
        hub.stop()          # stop the poller
    """

    def __init__(self, hw: HardwareContext, poll_interval_s: float = 0.5):
        self._hw = hw
        self._lock = threading.Lock()
        self._poll_interval = poll_interval_s
        self._stop_event = threading.Event()

        # Cached readings — written only by the poller thread.
        # Reading a Python float/int attribute is atomic on CPython
        # (GIL), so callers may read without taking the lock.
        self.sensor_mm: Optional[int] = None
        self.motor_deg: Optional[float] = None
        self.flags: int = 0
        self.enabled: bool = False
        self.reached: bool = False
        self.stall_detected: bool = False
        self.stall_protected: bool = False
        self.poll_error: Optional[str] = None
        self.port: str = hw.port

        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="hw-hub-poller",
        )
        self._thread.start()

    # ── Background poller ────────────────────────────────────────

    def _poll_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                with self._lock:
                    self.sensor_mm = self._hw.sensor.read_mm()
                    self.motor_deg = self._hw.motor.read_realtime_position_deg_output()
                    flags = self._hw.motor.read_status_flags()
                self.flags = flags
                self.enabled = bool(flags & self._hw.motor.FLAG_ENABLED)
                self.reached = bool(flags & self._hw.motor.FLAG_REACHED)
                self.stall_detected = bool(flags & self._hw.motor.FLAG_STALL_DETECTED)
                self.stall_protected = bool(flags & self._hw.motor.FLAG_STALL_PROTECTED)
                self.poll_error = None
            except Exception as e:
                self.poll_error = str(e)
            self._stop_event.wait(self._poll_interval)

    def set_poll_interval(self, interval_s: float) -> None:
        """Change how often the poller reads the bus."""
        self._poll_interval = interval_s

    # ── Synchronous read helpers (acquire lock, read, release) ───

    def read_sensor_mm_sync(self) -> int:
        """Read sensor distance right now (blocking)."""
        with self._lock:
            return self._hw.sensor.read_mm()

    def read_motor_deg_sync(self) -> float:
        """Read motor position right now (blocking)."""
        with self._lock:
            return self._hw.motor.read_realtime_position_deg_output()

    def read_flags_sync(self) -> int:
        """Read motor status flags right now (blocking)."""
        with self._lock:
            return self._hw.motor.read_status_flags()

    # ── Motor commands ───────────────────────────────────────────

    def zero_here(self) -> None:
        """Set the current motor position as zero."""
        with self._lock:
            self._hw.motor.zero_here()

    def arm_stall(self, stall: StallSettings) -> None:
        """Write sensitive stall-detection thresholds."""
        with self._lock:
            arm_stall_detection(self._hw.motor, stall)

    def disarm_stall(self, stall: StallSettings) -> None:
        """Restore safe holding-current stall thresholds."""
        with self._lock:
            disarm_stall_detection(self._hw.motor, stall)

    def recover_from_stall(self) -> None:
        """Clear stall protection and re-enable the motor.

        Must be called while the poller is still running — each sub-step
        acquires/releases the lock individually so the poller can
        interleave between the recovery delays.
        """
        with self._lock:
            self._hw.motor.clear_stall_protection()
        time.sleep(0.3)
        with self._lock:
            self._hw.motor.set_enabled(True)
        time.sleep(0.5)
        # Verify
        flags = self.read_flags_sync()
        if not (flags & self._hw.motor.FLAG_ENABLED):
            with self._lock:
                self._hw.motor.clear_stall_protection()
            time.sleep(0.3)
            with self._lock:
                self._hw.motor.set_enabled(True)
            time.sleep(0.5)

    def send_move_absolute(
        self, target_deg: float, speed_rpm: int, acc: int,
        max_attempts: int = 3,
    ) -> None:
        """Send an absolute-position command with retry on transient errors."""
        for attempt in range(max_attempts):
            try:
                with self._lock:
                    self._hw.motor.move_absolute_deg_output(
                        target_deg, speed_rpm=speed_rpm, acc=acc,
                    )
                return
            except Exception:
                if attempt < max_attempts - 1:
                    time.sleep(0.5)
                else:
                    raise

    # ── High-level rotation with stall detection + progress ──────

    def rotate_with_stall_detection(
        self,
        target_deg: float,
        motion: MotionSettings,
        stall: StallSettings,
        *,
        on_progress: Optional[Callable[[float, float], None]] = None,
        on_stall: Optional[Callable[[float, float, int], None]] = None,
        abort: Optional[threading.Event] = None,
        timeout_s: float = 20.0,
    ) -> RotationResult:
        """Rotate with stall detection, fine-grained locking, and progress.

        Unlike the module-level ``rotate_with_stall_detection`` function
        (which holds the lock for the entire duration), this method
        acquires the lock only for each individual Modbus transaction,
        allowing the background poller to keep updating cached readings.

        Args:
            target_deg: Absolute output-shaft angle in degrees.
            motion:     Motion settings (speed, acc).
            stall:      Stall detection settings.
            on_progress: ``(actual_deg, target_deg)`` called every poll
                         cycle so the GUI can update a progress bar.
            on_stall:   ``(actual_deg, target_deg, elapsed_ms)`` called on
                        stall; should block until operator is ready to retry.
            abort:      If set, the rotation is abandoned early.
            timeout_s:  Per-attempt timeout.

        Returns:
            A ``RotationResult``.
        """
        rotation_start = time.perf_counter()
        stall_events: list[dict] = []
        poll_s = self._hw.motor.poll_period_s

        self.arm_stall(stall)

        reached = False
        while True:
            if abort and abort.is_set():
                break

            attempt_start = time.perf_counter()
            self.send_move_absolute(
                target_deg, motion.rot_speed_rpm, motion.rot_acc,
            )

            # Poll flags until reached or stall (fine-grained locking)
            stall_occurred = False
            t0 = time.time()
            while True:
                if abort and abort.is_set():
                    break
                flags = self.read_flags_sync()
                actual = self.motor_deg  # use cached value for progress
                if on_progress and actual is not None:
                    on_progress(actual, target_deg)
                if flags & self._hw.motor.FLAG_STALL_PROTECTED:
                    stall_occurred = True
                    break
                if flags & self._hw.motor.FLAG_REACHED:
                    reached = True
                    break
                if (time.time() - t0) >= timeout_s:
                    break
                time.sleep(poll_s)

            if abort and abort.is_set():
                break
            if not stall_occurred:
                break

            # Stall detected
            elapsed_ms = int((time.perf_counter() - attempt_start) * 1000)
            actual_deg = self.read_motor_deg_sync()
            stall_events.append({
                "ts": now_iso_local(),
                "target_deg": target_deg,
                "actual_deg": actual_deg,
                "time_since_motion_start_ms": elapsed_ms,
            })
            self.recover_from_stall()
            if on_stall is not None:
                on_stall(actual_deg, target_deg, elapsed_ms)
            if abort and abort.is_set():
                break
            self.arm_stall(stall)

        self.disarm_stall(stall)
        total_ms = int((time.perf_counter() - rotation_start) * 1000)
        actual_deg = self.read_motor_deg_sync()

        return RotationResult(
            reached=reached,
            target_deg=target_deg,
            actual_deg=actual_deg,
            rotation_time_ms=total_ms,
            stall_events=stall_events,
        )

    # ── Lifecycle ────────────────────────────────────────────────

    def stop(self) -> None:
        """Stop the background poller thread."""
        self._stop_event.set()

    def close(self) -> None:
        """Stop the poller and close the underlying Modbus client."""
        self.stop()
        self._hw.close()


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
