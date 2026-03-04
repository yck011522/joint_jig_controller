# controller_cli.py
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, Tuple, Optional

from pymodbus.client import ModbusSerialClient

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
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def ensure_schema_version(design: dict, expected: int = 1):
    v = design.get("schema_version", None)
    if v != expected:
        raise RuntimeError(f"Unsupported design schema version: got {v}, expected {expected}.")


def build_offset_map(settings: dict) -> Dict[Tuple[str, str], float]:
    offsets = safe_get(settings, ["linear", "joint_offsets"], default=[])
    m: Dict[Tuple[str, str], float] = {}
    for entry in offsets:
        t = str(entry.get("type", "")).strip()
        o = str(entry.get("ori", "")).strip()
        off = float(entry.get("offset_mm", 0.0))
        if not t or not o:
            continue
        key = (t, o)
        # If duplicates exist, last one wins; we can tighten later if needed.
        m[key] = off
    return m


def required_type_ori_pairs(tube: dict) -> set[Tuple[str, str]]:
    req: set[Tuple[str, str]] = set()
    for j in tube.get("joints", []) or []:
        t = str(j.get("type", "")).strip()
        o = str(j.get("ori", "")).strip()
        if t and o:
            req.add((t, o))
    return req


def validate_offsets_complete(tube: dict, offset_map: Dict[Tuple[str, str], float]) -> None:
    req = required_type_ori_pairs(tube)
    missing = sorted([p for p in req if p not in offset_map])
    if missing:
        msg = "Missing joint_offsets entries for (type, ori):\n" + "\n".join([f"  - {p}" for p in missing])
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
# CLI flow
# ----------------------------
def main():
    base_dir = Path(__file__).resolve().parent

    # --- 1) Load settings.json ---
    settings_path = base_dir / "settings.json"
    if not settings_path.exists():
        raise RuntimeError(f"settings.json not found next to script: {settings_path}")
    settings = load_json(settings_path)

    # --- 2) Connect Modbus client (fixed port for now) ---
    port = safe_get(settings, ["serial", "port"], default=None)
    if not port:
        raise RuntimeError("settings.serial.port is required for CLI v0 (fixed port).")

    baudrate = int(safe_get(settings, ["serial", "baudrate"], default=115200))
    timeout_s = float(safe_get(settings, ["serial", "timeout_s"], default=0.4))
    parity = str(safe_get(settings, ["serial", "parity"], default="N"))
    stopbits = int(safe_get(settings, ["serial", "stopbits"], default=1))
    bytesize = int(safe_get(settings, ["serial", "bytesize"], default=8))

    client = ModbusSerialClient(
        port=port,
        baudrate=baudrate,
        parity=parity,
        stopbits=stopbits,
        bytesize=bytesize,
        timeout=timeout_s,
    )

    print(f"[init] Opening Modbus RTU on {port} @ {baudrate} ...")
    if not client.connect():
        raise RuntimeError(f"Failed to connect ModbusSerialClient on {port} @ {baudrate}")
    print("[init] Modbus connected.")

    # --- 3) Instantiate DistanceSensor + ZDTEmmMotor ---
    motor_addr = int(safe_get(settings, ["device_addresses", "motor_addr"], default=2))
    sensor_addr = int(safe_get(settings, ["device_addresses", "sensor_addr"], default=1))

    # Motor kinematics from settings
    motor_steps_per_rev = int(safe_get(settings, ["motion", "motor_steps_per_rev"], default=200))
    microsteps = int(safe_get(settings, ["motion", "microsteps"], default=16))
    gear_ratio = float(safe_get(settings, ["motion", "gear_ratio"], default=30.0))

    kin = MotorKinematics(
        motor_steps_per_rev=motor_steps_per_rev,
        microsteps=microsteps,
        gearbox_ratio=gear_ratio,
    )

    sensor = DistanceSensor(client, slave_id=sensor_addr)
    motor = ZDTEmmMotor(client, slave_id=motor_addr, kinematics=kin)

    # Probe connectivity quickly
    try:
        _mm = sensor.read_mm()
        _deg = motor.read_realtime_position_deg_output()
        print(f"[probe] Sensor OK: {_mm} mm | Motor OK: {_deg:.3f} deg (output)")
    except Exception as e:
        client.close()
        raise RuntimeError(f"Hardware probe failed (sensor or motor not responding): {e!r}")

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
    print("\n=== Tube List ===")
    for i, t in enumerate(tubes):
        tid = t.get("id", f"tube_{i}")
        length = t.get("length", "?")
        n = len(t.get("joints", []) or [])
        print(f"  [{i}] id={tid}  length={length}mm  joints={n}")

    sel_str = input("Select tube index and press Enter: ").strip()
    if sel_str == "":
        sel = 0
    else:
        sel = int(sel_str)

    if sel < 0 or sel >= len(tubes):
        client.close()
        raise RuntimeError(f"Invalid tube selection: {sel}")

    tube = tubes[sel]
    tube_id = str(tube.get("id", f"tube_{sel}"))
    tube_length = float(tube.get("length", 0.0))
    joints = list(tube.get("joints", []) or [])

    # Sort joints by s_on_tube (left->right)
    joints.sort(key=lambda j: float(j.get("s_on_tube", 0.0)))

    # --- Offset completeness enforcement (before starting assembly) ---
    offset_map = build_offset_map(settings)
    validate_offsets_complete(tube, offset_map)

    # Extract other settings
    linear_tol_mm = float(safe_get(settings, ["linear", "linear_tol_mm"], default=2.0))
    sensor_global_offset_mm = float(safe_get(settings, ["linear", "sensor_global_offset_mm"], default=0.0))

    rot_speed_rpm = float(safe_get(settings, ["motion", "rot_speed_rpm"], default=60.0))
    rot_acc_rpm_s = float(safe_get(settings, ["motion", "rot_acc_rpm_s"], default=100.0))
    rot_tol_deg = float(safe_get(settings, ["motion", "rot_tol_deg"], default=1.0))

    log_path = design_log_path(design_path, settings)

    print("\n=== Ready ===")
    print(f"Tube: {tube_id}  length={tube_length}mm  joints={len(joints)}")
    print(f"Design file: {design_path}")
    print(f"Log file:    {log_path}")
    print(f"Tolerance:   linear ±{linear_tol_mm} mm | rotation ±{rot_tol_deg} deg")
    input("\nMount tube, then press Enter to START (will zero motor here)...")

    # --- FSM start ---
    tube_t0 = time.perf_counter()

    # tube mounted → motor.zero_here()
    motor.zero_here()

    append_event(log_path, {
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
    })

    # For each joint
    for joint_seq, j in enumerate(joints):
        joint_id = str(j.get("id", f"joint_{joint_seq}"))
        joint_type = str(j.get("type", "")).strip()
        ori = str(j.get("ori", "")).strip()
        s_on_tube = float(j.get("s_on_tube", 0.0))
        roll_deg = float(j.get("roll_deg", 0.0))

        # Compute target position (design truth + offsets)
        joint_offset = offset_map[(joint_type, ori)]
        target_mm = s_on_tube + sensor_global_offset_mm + joint_offset

        print("\n" + "-" * 60)
        print(f"Joint {joint_seq+1}/{len(joints)} | id={joint_id} | type={joint_type} ori={ori}")
        print(f"Target: s_on_tube={s_on_tube:.1f}mm  -> target_mm={target_mm:.1f}mm (global+joint offsets applied)")
        print(f"Target rotation: roll_deg={roll_deg:.1f}°")

        # POSITIONING_J1_LINEAR
        linear_t0 = time.perf_counter()

        last_raw_mm: Optional[int] = None
        last_measured_mm: Optional[float] = None
        last_error_mm: Optional[float] = None

        print("\nMove carriage to target. Press ENTER to attempt confirm (non-blocking on Windows).")
        print("Live: current_mm_raw | measured_mm | target_mm | error_mm")

        while True:
            raw_mm = int(sensor.read_mm())
            measured_mm = float(raw_mm) + sensor_global_offset_mm  # measured = raw + global offset only
            error_mm = measured_mm - target_mm

            last_raw_mm = raw_mm
            last_measured_mm = measured_mm
            last_error_mm = error_mm

            # Print on one line
            line = (
                f"\r  raw={raw_mm:6d} mm | meas={measured_mm:8.2f} | "
                f"target={target_mm:8.2f} | err={error_mm:7.2f}   "
                f"(tol ±{linear_tol_mm})"
            )
            print(line, end="", flush=True)

            if enter_pressed_nonblocking():
                # attempt confirm
                if abs(error_mm) <= linear_tol_mm:
                    linear_time_ms = int((time.perf_counter() - linear_t0) * 1000)
                    print("\n[ok] Linear position confirmed.")
                    append_event(log_path, {
                        "ts": now_iso_local(),
                        "event": "linear_confirm",
                        "design_file": design_path.name,
                        "tube_id": tube_id,
                        "joint_id": joint_id,
                        "joint_type": joint_type,
                        "ori": ori,
                        "joint_seq": joint_seq,
                        "target_mm": target_mm,
                        "measured_mm_raw": raw_mm,
                        "measured_mm": measured_mm,
                        "error_mm": error_mm,
                        "tol_mm": linear_tol_mm,
                        "linear_time_ms": linear_time_ms,
                    })
                    break
                else:
                    print("\n[warn] Outside tolerance. Keep adjusting and press ENTER again.")

            time.sleep(0.1)

        # ROTATING_J2
        rot_t0 = time.perf_counter()
        motor.move_absolute_deg_output(roll_deg, speed_rpm=200, acc=50)  # speed/acc can be moved to settings later

        ok = motor.wait_until_reached(timeout_s=20.0)
        rot_time_ms = int((time.perf_counter() - rot_t0) * 1000)
        actual_deg = motor.read_realtime_position_deg_output()

        print(f"[rot] reached={ok} target={roll_deg:.2f}° actual={actual_deg:.2f}° time={rot_time_ms}ms")

        append_event(log_path, {
            "ts": now_iso_local(),
            "event": "rotation_reached",
            "design_file": design_path.name,
            "tube_id": tube_id,
            "joint_id": joint_id,
            "joint_type": joint_type,
            "ori": ori,
            "joint_seq": joint_seq,
            "target_deg": roll_deg,
            "actual_deg": actual_deg,
            "tol_deg": rot_tol_deg,
            "rot_speed_rpm": rot_speed_rpm,
            "rot_acc_rpm_s": rot_acc_rpm_s,
            "rotation_time_ms": rot_time_ms,
            "motor_reached_flag": bool(ok),
        })

        # INSTALLING
        print(f"\nInstall joint now: type={joint_type} ori={ori}. Press ENTER when installed.")
        install_t0 = time.perf_counter()
        input()
        install_time_ms = int((time.perf_counter() - install_t0) * 1000)

        append_event(log_path, {
            "ts": now_iso_local(),
            "event": "install_confirm",
            "design_file": design_path.name,
            "tube_id": tube_id,
            "joint_id": joint_id,
            "joint_type": joint_type,
            "ori": ori,
            "joint_seq": joint_seq,
            "install_time_ms": install_time_ms,
        })

    # DONE
    tube_time_ms = int((time.perf_counter() - tube_t0) * 1000)
    append_event(log_path, {
        "ts": now_iso_local(),
        "event": "tube_complete",
        "design_file": design_path.name,
        "tube_id": tube_id,
        "joint_id": None,
        "joint_type": None,
        "ori": None,
        "joint_seq": None,
        "tube_time_ms": tube_time_ms,
        "completed": True,
    })

    print("\n" + "=" * 60)
    print(f"Tube complete: {tube_id} | total time = {tube_time_ms/1000:.1f}s")
    print(f"Log written to: {log_path}")

    client.close()
    print("[exit] Serial port closed.")


if __name__ == "__main__":
    main()