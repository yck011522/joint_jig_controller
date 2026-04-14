# controller_cli.py
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, Tuple, Optional

from pymodbus.client import ModbusSerialClient
import serial.tools.list_ports

from distance_sensor import DistanceSensor
from motor_comm import ZDTEmmMotor, MotorKinematics


# ----------------------------
# Small utilities
# ----------------------------
def now_iso_local() -> str:
    # Keep it simple: local time with seconds; add ms for better timing.
    # (If you want timezone offsets later, we can add them.)
    t = time.time()
    lt = time.localtime(t)
    ms = int((t - int(t)) * 1000)
    return time.strftime("%Y-%m-%dT%H:%M:%S", lt) + f".{ms:03d}"


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def safe_get(d: dict, keys: list[str], default=None):
    """Walk a nested dict using *keys* and return the value or *default*.

    This is a tiny helper to avoid ``KeyError``s when the configuration has
    missing entries; it treats anything that isn't a dict as a dead end.
    """
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def ensure_schema_version(design: dict, expected: int = 1):
    v = design.get("schema_version", None)
    if v != expected:
        raise RuntimeError(
            f"Unsupported design schema version: got {v}, expected {expected}."
        )


def build_offset_map(settings: dict) -> Dict[Tuple[str, str], float]:
    """Read joint offsets from settings and return a lookup map.

    The returned dictionary maps ``(type, orientation)`` to a float offset in
    millimetres. Any malformed entries are simply skipped; if duplicates occur
    the later entry overwrites the earlier one.
    """
    joint_offset_entries = safe_get(settings, ["linear", "joint_offsets"], default=[])
    offset_map: Dict[Tuple[str, str], float] = {}
    for entry in joint_offset_entries:
        type_str = str(entry.get("type", "")).strip()
        orientation = str(entry.get("ori", "")).strip()
        offset_mm = float(entry.get("offset_mm", 0.0))
        if not type_str or not orientation:
            # skip incomplete records
            continue
        key = (type_str, orientation)
        # If duplicates exist, last one wins; we can tighten later if needed.
        offset_map[key] = offset_mm
    return offset_map


def required_type_ori_pairs(tube: dict) -> set[Tuple[str, str]]:
    """Return the set of (type, orientation) pairs required by a tube design.

        Used to verify that we have offsets for every joint described in the
    design.
    """
    required_pairs: set[Tuple[str, str]] = set()
    for joint in tube.get("joints", []) or []:
        type_str = str(joint.get("type", "")).strip()
        orientation = str(joint.get("ori", "")).strip()
        if type_str and orientation:
            required_pairs.add((type_str, orientation))
    return required_pairs


def validate_offsets_complete(
    tube: dict, offset_map: Dict[Tuple[str, str], float]
) -> None:
    """Raise if there are joints described in *tube* without an offset entry.

    The check is performed before any assembly work begins so that the user is
    alerted early rather than part-way through the workflow.
    """
    required_pairs = required_type_ori_pairs(tube)
    missing = sorted([pair for pair in required_pairs if pair not in offset_map])
    if missing:
        msg = "Missing joint_offsets entries for (type, ori):\n" + "\n".join(
            [f"  - {p}" for p in missing]
        )
        raise RuntimeError(msg)


def design_log_path(design_path: Path, settings: dict) -> Path:
    # Prefer settings.logging.log_extension; default ".log.jsonl"
    ext = safe_get(settings, ["logging", "log_extension"], default=".log.jsonl")
    # Use stem + ext (cleaner than "file.json.log.jsonl")
    return design_path.with_name(design_path.stem + ext)


def append_event(log_path: Path, event_obj: dict) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    line = json.dumps(event_obj, ensure_ascii=False)
    with log_path.open("a", encoding="utf-8") as f:
        f.write(line + "\n")
        f.flush()


# ----------------------------
# Non-blocking "Enter" detection (Windows)
# ----------------------------
def enter_pressed_nonblocking() -> bool:
    """
    Returns True if ENTER was pressed since last check.
    On Windows uses msvcrt; on other platforms returns False (fallback to blocking mode).
    """
    try:
        import msvcrt  # type: ignore
    except Exception:
        return False

    if msvcrt.kbhit():
        ch = msvcrt.getch()
        # ENTER can be '\r' (carriage return)
        if ch in (b"\r", b"\n"):
            return True
    return False


# ----------------------------
# COM port auto-detection
# ----------------------------
def _list_available_ports() -> list:
    """Return list of serial port info objects, sorted by device name."""
    return sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)


def _try_probe_devices(
    port: str,
    *,
    baudrate: int,
    timeout_s: float,
    parity: str,
    stopbits: int,
    bytesize: int,
    sensor_addr: int,
    motor_addr: int,
) -> Tuple[Optional[ModbusSerialClient], Optional[int], Optional[float], str]:
    """Try to connect to *port* and probe sensor + motor.

    Returns (client, sensor_mm, motor_deg, status_msg).
    *client* is the opened ModbusSerialClient on success, else None.
    status_msg is one of: "both_ok", "sensor_only", "no_devices", "connect_fail".
    """
    client = ModbusSerialClient(
        port=port,
        baudrate=baudrate,
        parity=parity,
        stopbits=stopbits,
        bytesize=bytesize,
        timeout=timeout_s,
    )
    if not client.connect():
        return None, None, None, "connect_fail"

    sensor = DistanceSensor(client, slave_id=sensor_addr)
    motor = ZDTEmmMotor(client, slave_id=motor_addr)

    sensor_mm: Optional[int] = None
    motor_deg: Optional[float] = None

    # Probe sensor
    try:
        sensor_mm = sensor.read_mm()
    except Exception:
        pass

    # Probe motor
    try:
        motor_deg = motor.read_realtime_position_deg_output()
    except Exception:
        pass

    if sensor_mm is not None and motor_deg is not None:
        return client, sensor_mm, motor_deg, "both_ok"

    if sensor_mm is not None and motor_deg is None:
        # Sensor responds but motor doesn't — correct bus, motor issue
        client.close()
        return None, sensor_mm, None, "sensor_only"

    # Neither device responded — wrong bus or unrelated serial device
    client.close()
    return None, None, None, "no_devices"


def auto_detect_port(
    settings: dict,
) -> Tuple[ModbusSerialClient, int, float]:
    """Scan available COM ports and return (client, sensor_mm, motor_deg).

    Uses settings for baud/parity/etc and device addresses.
    Raises RuntimeError with a diagnostic message if detection fails.
    """
    baudrate = int(safe_get(settings, ["serial", "baudrate"], default=115200))
    timeout_s = float(safe_get(settings, ["serial", "timeout_s"], default=0.4))
    parity = str(safe_get(settings, ["serial", "parity"], default="N"))
    stopbits = int(safe_get(settings, ["serial", "stopbits"], default=1))
    bytesize = int(safe_get(settings, ["serial", "bytesize"], default=8))
    sensor_addr = int(safe_get(settings, ["device_addresses", "sensor_addr"], default=1))
    motor_addr = int(safe_get(settings, ["device_addresses", "motor_addr"], default=2))

    all_ports = _list_available_ports()
    if not all_ports:
        raise RuntimeError(
            "No COM ports found on this system.\n"
            "  - Check that the USB-RS485 adapter is plugged in.\n"
            "  - On Windows, check Device Manager → Ports (COM & LPT)."
        )

    print(f"[probe] Found {len(all_ports)} COM port(s): "
          + ", ".join(f"{p.device} ({p.description})" for p in all_ports))

    sensor_only_port: Optional[str] = None
    connect_fail_ports: list[str] = []

    for port_info in all_ports:
        port = port_info.device
        print(f"[probe] Trying {port} ...", end=" ", flush=True)

        client, sensor_mm, motor_deg, status = _try_probe_devices(
            port,
            baudrate=baudrate,
            timeout_s=timeout_s,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            sensor_addr=sensor_addr,
            motor_addr=motor_addr,
        )

        if status == "both_ok":
            print(f"OK  (sensor={sensor_mm} mm, motor={motor_deg:.3f} deg)")
            return client, sensor_mm, motor_deg

        if status == "sensor_only":
            print(f"sensor OK ({sensor_mm} mm) but MOTOR NOT RESPONDING")
            sensor_only_port = port

        elif status == "connect_fail":
            desc = port_info.description or ""
            # Detect likely "port in use" vs genuinely unavailable
            print(f"cannot open (may be in use by another application)")
            connect_fail_ports.append(f"{port} ({desc})")

        else:  # no_devices
            print("no Modbus devices responded")

    # --- Build a helpful error message ---
    lines = ["Could not automatically detect the hardware."]

    if sensor_only_port:
        lines.append(
            f"\n  On {sensor_only_port} the distance sensor responded but the motor did NOT."
        )
        lines.append("  - Check the motor RS-485 wiring and power supply.")
        lines.append("  - Verify the motor Modbus address matches settings.json "
                      f"(expected slave {motor_addr}).")

    if connect_fail_ports:
        lines.append("\n  The following port(s) could not be opened (possibly in use):")
        for p in connect_fail_ports:
            lines.append(f"    {p}")
        lines.append("  - Close any other application that may hold the port "
                      "(e.g. motor tuning software, serial monitor).")

    lines.append(f"\n  Available ports: "
                 + ", ".join(f"{p.device}" for p in all_ports))
    lines.append("  Baud rate: " + str(baudrate))
    lines.append("  Expected sensor address: " + str(sensor_addr))
    lines.append("  Expected motor address: " + str(motor_addr))

    raise RuntimeError("\n".join(lines))


# ----------------------------
# CLI flow
# ----------------------------
def main():
    base_dir = Path(__file__).resolve().parent

    # --- 1) Load settings.json -------------------------------------------------
    # The CLI is entirely driven by a few JSON configuration files located in
    # the same directory as the script.  If settings.json is missing we can't
    # proceed, so fail fast.
    settings_path = base_dir / "settings.json"
    if not settings_path.exists():
        raise RuntimeError(f"settings.json not found next to script: {settings_path}")
    settings = load_json(settings_path)

    # --- 2) Connect Modbus client (auto-detect or fixed port) ---
    probe_enabled = bool(safe_get(settings, ["probe", "enabled"], default=True))
    fixed_port = safe_get(settings, ["serial", "port"], default=None)

    baudrate = int(safe_get(settings, ["serial", "baudrate"], default=115200))
    timeout_s = float(safe_get(settings, ["serial", "timeout_s"], default=0.4))
    parity = str(safe_get(settings, ["serial", "parity"], default="N"))
    stopbits = int(safe_get(settings, ["serial", "stopbits"], default=1))
    bytesize = int(safe_get(settings, ["serial", "bytesize"], default=8))

    # device address configuration for Modbus slaves
    motor_address = int(
        safe_get(settings, ["device_addresses", "motor_addr"], default=2)
    )
    sensor_address = int(
        safe_get(settings, ["device_addresses", "sensor_addr"], default=1)
    )

    if probe_enabled:
        print("[init] Auto-detecting COM port ...")
        client, sensor_reading_mm, motor_reading_deg = auto_detect_port(settings)
        port = client.comm_params.host  # retrieve the port that worked
        print(f"[init] Hardware found on {port} @ {baudrate}")
    else:
        # Fall back to fixed port from settings
        if not fixed_port:
            raise RuntimeError(
                "Probe disabled but settings.serial.port is not set. "
                "Either enable probe or specify a COM port."
            )
        port = fixed_port
        print(f"[init] Opening Modbus RTU on {port} @ {baudrate} (probe disabled) ...")
        client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=timeout_s,
        )
        if not client.connect():
            raise RuntimeError(
                f"Failed to connect ModbusSerialClient on {port} @ {baudrate}"
            )
        print("[init] Modbus connected.")
        sensor_reading_mm = None
        motor_reading_deg = None

    # --- 3) Instantiate DistanceSensor + ZDTEmmMotor ---

    # Motor kinematics from settings
    motor_steps_per_rev = int(
        safe_get(settings, ["motion", "motor_steps_per_rev"], default=200)
    )
    microsteps = int(safe_get(settings, ["motion", "microsteps"], default=16))
    gear_ratio = float(safe_get(settings, ["motion", "gear_ratio"], default=30.0))

    kinematics = MotorKinematics(
        motor_steps_per_rev=motor_steps_per_rev,
        microsteps=microsteps,
        gearbox_ratio=gear_ratio,
    )

    # create objects for interacting with the hardware
    sensor = DistanceSensor(client, slave_id=sensor_address)
    motor = ZDTEmmMotor(client, slave_id=motor_address, kinematics=kinematics)

    # If we used auto-detect, devices were already probed; otherwise probe now
    if sensor_reading_mm is not None and motor_reading_deg is not None:
        print(
            f"[probe] Sensor OK: {sensor_reading_mm} mm | Motor OK: {motor_reading_deg:.3f} deg (output)"
        )
    else:
        try:
            sensor_reading_mm = sensor.read_mm()
            motor_reading_deg = motor.read_realtime_position_deg_output()
            print(
                f"[probe] Sensor OK: {sensor_reading_mm} mm | Motor OK: {motor_reading_deg:.3f} deg (output)"
            )
        except Exception as e:
            client.close()
            raise RuntimeError(
                f"Hardware probe failed (sensor or motor not responding): {e!r}"
            )

    # --- 4) Load design JSON (hardcoded) ---
    design_path = base_dir / "design" / "dummy.json"
    if not design_path.exists():
        client.close()
        raise RuntimeError(f"Design file not found: {design_path}")

    design = load_json(design_path)
    ensure_schema_version(design, expected=1)

    tubes = design.get("tubes", [])
    if not isinstance(tubes, list) or len(tubes) == 0:
        client.close()
        raise RuntimeError("Design file contains no tubes[].")

    # --- 5) Choose tube in CLI ---
    # show available tubes from the design so operator can choose one
    print("\n=== Tube List ===")
    for idx, tube_entry in enumerate(tubes):
        display_id = tube_entry.get("id", f"tube_{idx}")
        length = tube_entry.get("length", "?")
        num_joints = len(tube_entry.get("joints", []) or [])
        print(f"  [{idx}] id={display_id}  length={length}mm  joints={num_joints}")

    sel_str = input("Select tube index and press Enter: ").strip()
    if sel_str == "":
        selected_index = 0
    else:
        selected_index = int(sel_str)

    if selected_index < 0 or selected_index >= len(tubes):
        client.close()
        raise RuntimeError(f"Invalid tube selection: {selected_index}")

    tube = tubes[selected_index]
    tube_id = str(tube.get("id", f"tube_{selected_index}"))
    tube_length = float(tube.get("length", 0.0))
    joints = list(tube.get("joints", []) or [])

    # Sort joints by position_mm (left->right)
    joints.sort(key=lambda j: float(j.get("position_mm", 0.0)))

    # --- Offset completeness enforcement (before starting assembly) ---
    offset_map = build_offset_map(settings)
    validate_offsets_complete(tube, offset_map)

    # Extract other settings
    linear_tol_mm = float(safe_get(settings, ["linear", "linear_tol_mm"], default=2.0))
    sensor_global_offset_mm = float(
        safe_get(settings, ["linear", "sensor_global_offset_mm"], default=0.0)
    )

    rot_speed_rpm = int(safe_get(settings, ["motion", "rot_speed_rpm"], default=300))
    rot_acc = int(safe_get(settings, ["motion", "rot_acc"], default=100))
    rot_tol_deg = float(safe_get(settings, ["motion", "rot_tol_deg"], default=1.0))

    log_path = design_log_path(design_path, settings)

    # summarise configuration and wait for user to mount first tube
    print("\n=== Ready ===")
    print(f"Tube: {tube_id}  length={tube_length}mm  joints={len(joints)}")
    print(f"Design file: {design_path}")
    print(f"Log file:    {log_path}")
    print(f"Tolerance:   linear ±{linear_tol_mm} mm | rotation ±{rot_tol_deg} deg")
    input("\nMount tube, then press Enter to START (will zero motor here)...")

    # --- FSM start ---
    # record start time for full-tube duration
    tube_start_time = time.perf_counter()

    # tube mounted → motor.zero_here()
    motor.zero_here()

    append_event(
        log_path,
        {
            "ts": now_iso_local(),
            "event": "tube_start",
            "design_file": design_path.name,
            "tube_id": tube_id,
            "joint_id": None,
            "joint_type": None,
            "ori": None,
            "joint_seq": None,
            "tube_length_mm": tube_length,
            "num_joints": len(joints),
        },
    )

    # For each joint
    for joint_index, joint_entry in enumerate(joints):
        joint_id = str(joint_entry.get("id", f"joint_{joint_index}"))
        joint_type = str(joint_entry.get("type", "")).strip()
        joint_orientation = str(joint_entry.get("ori", "")).strip()
        position_mm = float(joint_entry.get("position_mm", 0.0))
        rotation_deg = float(joint_entry.get("rotation_deg", 0.0))

        # Compute target position (design truth + offsets)
        joint_offset = offset_map[(joint_type, joint_orientation)]
        target_mm = position_mm + sensor_global_offset_mm + joint_offset

        print("\n" + "-" * 60)
        print(
            f"Joint {joint_index+1}/{len(joints)} | id={joint_id} | type={joint_type} ori={joint_orientation}"
        )
        print(
            f"Target: position_mm={position_mm:.1f}mm  -> target_mm={target_mm:.1f}mm (global+joint offsets applied)"
        )
        print(f"Target rotation: rotation_deg={rotation_deg:.1f}°")

        # POSITIONING_J1_LINEAR (start timing the linear adjustment)
        linear_start_time = time.perf_counter()

        # remember the last few values for potential debugging/logging
        last_raw_sensor_mm: Optional[int] = None
        last_measured_sensor_mm: Optional[float] = None
        last_error_mm: Optional[float] = None

        print(
            "\nMove carriage to target. Press ENTER to attempt confirm (non-blocking on Windows)."
        )
        print("Live: current_mm_raw | measured_mm | target_mm | error_mm")

        while True:
            # read current position from the distance sensor
            raw_sensor_mm = int(sensor.read_mm())
            measured_sensor_mm = (
                float(raw_sensor_mm) + sensor_global_offset_mm
            )  # measured = raw + global offset only
            error_mm = measured_sensor_mm - target_mm

            last_raw_sensor_mm = raw_sensor_mm
            last_measured_sensor_mm = measured_sensor_mm
            last_error_mm = error_mm

            # Print on one line
            line = (
                f"\r  raw={raw_sensor_mm:6d} mm | meas={measured_sensor_mm:8.2f} | "
                f"target={target_mm:8.2f} | err={error_mm:7.2f}   "
                f"(tol ±{linear_tol_mm})"
            )
            print(line, end="", flush=True)

            if enter_pressed_nonblocking():
                # attempt confirm
                if abs(error_mm) <= linear_tol_mm:
                    linear_time_ms = int(
                        (time.perf_counter() - linear_start_time) * 1000
                    )
                    print("\n[ok] Linear position confirmed.")
                    append_event(
                        log_path,
                        {
                            "ts": now_iso_local(),
                            "event": "linear_confirm",
                            "design_file": design_path.name,
                            "tube_id": tube_id,
                            "joint_id": joint_id,
                            "joint_type": joint_type,
                            "ori": joint_orientation,
                            "joint_seq": joint_index,
                            "target_mm": target_mm,
                            "measured_mm_raw": raw_sensor_mm,
                            "measured_mm": measured_sensor_mm,
                            "error_mm": error_mm,
                            "tol_mm": linear_tol_mm,
                            "linear_time_ms": linear_time_ms,
                        },
                    )
                    break
                else:
                    print(
                        "\n[warn] Outside tolerance. Keep adjusting and press ENTER again."
                    )

            time.sleep(0.1)

        # ROTATING_J2: command motor to move and wait until it reports arrival
        rotation_start_time = time.perf_counter()
        motor.move_absolute_deg_output(
            rotation_deg, speed_rpm=rot_speed_rpm, acc=rot_acc
        )

        ok = motor.wait_until_reached(timeout_s=20.0)
        rotation_time_ms = int((time.perf_counter() - rotation_start_time) * 1000)
        actual_rotation_deg = motor.read_realtime_position_deg_output()

        print(
            f"[rot] reached={ok} target={rotation_deg:.2f}° actual={actual_rotation_deg:.2f}° time={rotation_time_ms}ms"
        )

        append_event(
            log_path,
            {
                "ts": now_iso_local(),
                "event": "rotation_reached",
                "design_file": design_path.name,
                "tube_id": tube_id,
                "joint_id": joint_id,
                "joint_type": joint_type,
                "ori": joint_orientation,
                "joint_seq": joint_index,
                "target_deg": rotation_deg,
                "actual_deg": actual_rotation_deg,
                "tol_deg": rot_tol_deg,
                "rot_speed_rpm": rot_speed_rpm,
                "rot_acc": rot_acc,
                "rotation_time_ms": rotation_time_ms,
                "motor_reached_flag": bool(ok),
            },
        )

        # INSTALLING
        # INSTALLING: human operator steps in
        print(
            f"\nInstall joint now: type={joint_type} ori={joint_orientation}. Press ENTER when installed."
        )
        install_start_time = time.perf_counter()
        input()
        install_time_ms = int((time.perf_counter() - install_start_time) * 1000)

        append_event(
            log_path,
            {
                "ts": now_iso_local(),
                "event": "install_confirm",
                "design_file": design_path.name,
                "tube_id": tube_id,
                "joint_id": joint_id,
                "joint_type": joint_type,
                "ori": joint_orientation,
                "joint_seq": joint_index,
                "install_time_ms": install_time_ms,
            },
        )

    # DONE
    total_tube_time_ms = int((time.perf_counter() - tube_start_time) * 1000)
    append_event(
        log_path,
        {
            "ts": now_iso_local(),
            "event": "tube_complete",
            "design_file": design_path.name,
            "tube_id": tube_id,
            "joint_id": None,
            "joint_type": None,
            "ori": None,
            "joint_seq": None,
            "tube_time_ms": total_tube_time_ms,
            "completed": True,
        },
    )

    print("\n" + "=" * 60)
    print(f"Tube complete: {tube_id} | total time = {total_tube_time_ms/1000:.1f}s")
    print(f"Log written to: {log_path}")

    client.close()
    print("[exit] Serial port closed.")


if __name__ == "__main__":
    main()
