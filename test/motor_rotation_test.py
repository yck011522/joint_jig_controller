"""
motor_rotation_test.py — Minimal motor rotation test.

Moves the output shaft: 0° → 180° → 0° using settings from settings.json.

Usage:
  python test/motor_rotation_test.py
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pymodbus.client import ModbusSerialClient
import serial.tools.list_ports

from motor_comm import ZDTEmmMotor, MotorKinematics
from distance_sensor import DistanceSensor


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


def find_port(settings: dict) -> str:
    """Auto-detect or return fixed port from settings."""
    baudrate = int(safe_get(settings, ["serial", "baudrate"], default=115200))
    timeout_s = float(safe_get(settings, ["serial", "timeout_s"], default=0.4))
    sensor_addr = int(safe_get(settings, ["device_addresses", "sensor_addr"], default=1))
    motor_addr = int(safe_get(settings, ["device_addresses", "motor_addr"], default=2))

    probe_enabled = bool(safe_get(settings, ["probe", "enabled"], default=True))
    if not probe_enabled:
        port = safe_get(settings, ["serial", "port"], default=None)
        if not port:
            raise RuntimeError("Probe disabled and no serial.port configured.")
        return port

    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    if not ports:
        raise RuntimeError("No COM ports found.")

    for p in ports:
        client = ModbusSerialClient(
            port=p.device, baudrate=baudrate, parity="N",
            stopbits=1, bytesize=8, timeout=timeout_s,
        )
        if not client.connect():
            continue
        try:
            sensor = DistanceSensor(client, slave_id=sensor_addr)
            motor = ZDTEmmMotor(client, slave_id=motor_addr)
            sensor.read_mm()
            motor.read_realtime_position_deg_output()
            client.close()
            return p.device
        except Exception:
            client.close()

    raise RuntimeError("Could not find hardware on any COM port.")


def main() -> None:
    settings_path = PROJECT_ROOT / "settings.json"
    settings = load_json(settings_path)

    port = find_port(settings)
    baudrate = int(safe_get(settings, ["serial", "baudrate"], default=115200))
    timeout_s = float(safe_get(settings, ["serial", "timeout_s"], default=0.4))
    motor_addr = int(safe_get(settings, ["device_addresses", "motor_addr"], default=2))
    speed_rpm = int(safe_get(settings, ["motion", "rot_speed_rpm"], default=300))
    acc = int(safe_get(settings, ["motion", "rot_acc"], default=100))

    gear_ratio = float(safe_get(settings, ["motion", "gear_ratio"], default=30.0))
    steps = int(safe_get(settings, ["motion", "motor_steps_per_rev"], default=200))
    microsteps = int(safe_get(settings, ["motion", "microsteps"], default=16))

    client = ModbusSerialClient(
        port=port, baudrate=baudrate, parity="N",
        stopbits=1, bytesize=8, timeout=timeout_s,
    )
    if not client.connect():
        raise RuntimeError(f"Cannot connect to {port}")

    motor = ZDTEmmMotor(
        client, slave_id=motor_addr,
        kinematics=MotorKinematics(
            motor_steps_per_rev=steps, microsteps=microsteps, gearbox_ratio=gear_ratio,
        ),
    )

    print(f"Connected on {port}  speed={speed_rpm} RPM  acc={acc}")
    print(f"Enabled: {motor.is_enabled()}")

    try:
        # 1) Go to absolute 0°
        print("\n[1] Moving to 0° (output) ...")
        motor.move_absolute_deg_output(0.0, speed_rpm=speed_rpm, acc=acc)
        ok = motor.wait_until_reached(timeout_s=30.0)
        deg = motor.read_realtime_position_deg_output()
        print(f"    reached={ok}  position={deg:.3f}°")

        time.sleep(0.5)

        # 2) Go to 180°
        print("[2] Moving to 180° (output) ...")
        motor.move_absolute_deg_output(180.0, speed_rpm=speed_rpm, acc=acc)
        ok = motor.wait_until_reached(timeout_s=30.0)
        deg = motor.read_realtime_position_deg_output()
        print(f"    reached={ok}  position={deg:.3f}°")

        time.sleep(0.5)

        # 3) Return to 0°
        print("[3] Moving back to 0° (output) ...")
        motor.move_absolute_deg_output(0.0, speed_rpm=speed_rpm, acc=acc)
        ok = motor.wait_until_reached(timeout_s=30.0)
        deg = motor.read_realtime_position_deg_output()
        print(f"    reached={ok}  position={deg:.3f}°")

        print("\nTest complete.")

    except KeyboardInterrupt:
        print("\n[ABORT] Interrupted.")
    finally:
        client.close()
        print("[exit] Serial port closed.")


if __name__ == "__main__":
    main()
